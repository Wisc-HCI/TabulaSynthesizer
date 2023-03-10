#!/usr/bin/env python3

"""nl_parser.py: Parse functions for natural language text."""

import spacy
import nltk
import copy
import contractions
import string
from program import *
from recording import ParamFilled, ParamHole, Action
from recording import Domain
from entities import *
from nl_data import Interval
from nltk.parse.corenlp import CoreNLPParser

nlp = spacy.load("en_core_web_sm")


class NLParser:

    def __init__(self):
        self.action_data = Domain.get_instance()
        self.entity_data = EntityData.get_instance()
        self.vnet3 = nltk.corpus.util.LazyCorpusLoader('verbnet3', nltk.corpus.reader.verbnet.VerbnetCorpusReader,r'(?!\.).*\.xml')

        # corenlp
        self.stanford_parser = CoreNLPParser("http://localhost:9000")

    def preprocess_text(self, text):
        # convert everything to lowercase
        text = text.lower()

        # remove contractions
        text = contractions.fix(text)

        # col 2:
        #b_list = [] 
        table = str.maketrans('', '', string.punctuation)
        #for sent in a_list:
        words = text.split()
        #words = sent.split()
        stripped = [w.translate(table) for w in words]
        processed_sent = " ".join(stripped)
        #b_list.append(processed_sent.lower())

        #return b_list
        return processed_sent.lower()

    def is_noun(self, tag):
        if tag == "NN" or tag == "NNS" or tag == "NNP" or tag == "NNPS":
            return True
        return False

    def is_pronoun(self, tag):
        if tag == "PRP" or tag == "PRP$":
            return True
        return False

    def is_verb(self, tag):
        if tag == "VB" or tag == "VBD" or tag == "VBN" or tag == "VBG" or tag == "VBP" or tag == "VBZ":
            return True
        return False

    def tag_sentences(self, sentence):
        doc = nlp(sentence)
        sent_split = sentence.split()
        sentence = " ".join(sent_split)
        entities = self.entity_data.get_entities(sentence)
        doc = nlp(sentence)
        tags = [(str(token), token.tag_) for token in doc]
        nouns = [(i, tags[i][0], tags[i][1]) for i in range(len(tags))
                 if self.is_noun(tags[i][1])
                 and not any(ent[1].name == self.entity_data.stemmer.stem(tags[i][0]) for ent in entities)]
        pronouns = [(i, tags[i][0], tags[i][1]) for i in range(len(tags)) if self.is_pronoun(tags[i][1])]
        verbs = [(i, doc[i].lemma_, tags[i][1]) for i in range(len(tags)) if self.is_verb(tags[i][1])]
        return entities, nouns, pronouns, verbs

    def get_task_hints(self, verbs, entities, nouns, pronouns, classification):
        '''
        Collect (1) commands, (2) half-commands [commands with missing entities], and (3) constraints [entities without a command]
        Collect unparameterized commands (ucoms) by looking at each verb
        
        Structure of task_hints dataset:

        {"commands": [(c1, OR c2, ...)], "half-commands": [c1, OR c2, ...], "constraints": [c1, c2, ...]}
        '''

        def remove_duplicate_entities(remaining_entities, entity):
            idxs = []
            for i, ent in enumerate(remaining_entities):
                if ent[1] == entity[1]:
                    idxs.append(i)
            idxs.reverse()
            for idx in idxs:
                del remaining_entities[idx]

        def contains_more_entities(curr, best):
            if len(curr) > len(best):
                return True
            if len([c for c in curr if c != "HOLE"]) > len([b for b in best if b != "HOLE"]):
                return True
            return False

        def fit_entities_to_command(action_data, remaining_entities, curr_ordered_entities, best_ordered_entities):
            if len(curr_ordered_entities) > len(action_data["argtypes"]):
                return
            if len(remaining_entities) == 0 or len(curr_ordered_entities) == len(action_data["argtypes"]): #and len(curr_ordered_entities) > len(best_ordered_entities):
                # fill remaining entities with holes if necessary
                for i in range(len(curr_ordered_entities), len(action_data["argtypes"])):
                    curr_ordered_entities.append("HOLE")
                if contains_more_entities(curr_ordered_entities, best_ordered_entities):
                    best_ordered_entities.clear()
                    best_ordered_entities.extend(curr_ordered_entities)
            for i, entity in enumerate(remaining_entities):
                if len(curr_ordered_entities) >= len(action_data["argtypes"]):
                    break
                action_data_argtype = action_data["argtypes"][len(curr_ordered_entities)]
                _curr_ordered_entities = copy.copy(curr_ordered_entities)
                val = "HOLE"
                if action_data_argtype == entity[2]:
                    val = entity
                _curr_ordered_entities.append(val)
                _remaining_entities = copy.copy(remaining_entities)[:i]
                _remaining_entities.extend(remaining_entities[i + 1:])
                # remove_duplicate_entities(_remaining_entities, entity)
                fit_entities_to_command(action_data, _remaining_entities, _curr_ordered_entities, best_ordered_entities)

        task_hints = {"commands": [], "half-commands": [], "constraints": copy.copy(entities)}
        for verb_data in verbs:
            candidate_action_names = []
            for cmd_name, cmd_data in self.action_data.action_primitives.items():
                if classification == "conditional" and ("conditional" not in cmd_data["action_types"] and "trigger" not in cmd_data["action_types"]):
                    continue
                if classification == "command" and "command" not in cmd_data["action_types"]:
                    continue
                if classification == "conditional" and "trigger" in cmd_data["action_types"]:
                    classification = "trigger"
                verb_classes = self.vnet3.classids(verb_data[1])
                if any([cmd_verb in vc for vc in verb_classes for cmd_verb in cmd_data["verbnet"]]):
                    candidate_action_names.append(cmd_name)
            best_action_names = None
            best_entities = []
            for action_name in candidate_action_names:
                curr_entities = []
                entities_copy = copy.copy(entities)
                required_args = self.action_data.action_primitives[action_name]["argnames"]
                missing_args = len(required_args) - len(entities_copy)
                for i in range(max(0, missing_args)):
                    entities_copy.append((None, self.entity_data.get_null_entity(), "null"))
                fit_entities_to_command(self.action_data.action_primitives[action_name], entities_copy, [], curr_entities)
                if best_action_names is None or curr_entities.count("HOLE") < best_entities[0].count("HOLE"):
                    best_action_names = [action_name]
                    best_entities = [curr_entities]
                elif best_action_names is not None and curr_entities.count("HOLE") == best_entities[0].count("HOLE"):
                    best_action_names.append(action_name)
                    best_entities.append(curr_entities)
            for best_entity in best_entities:
                for ent in best_entity:
                    to_remove = [unused_ent for unused_ent in task_hints["constraints"] if unused_ent[0] == ent[0]]
                    task_hints["constraints"] = list(set(task_hints["constraints"]).difference(set(to_remove)))
            if best_action_names is None:
                continue
            commands = []
            for j, best_action_name in enumerate(best_action_names):
                args = {}
                for i, argname in enumerate(self.action_data.action_primitives[best_action_name]["argnames"]):
                    args[argname] = ParamFilled(best_entities[j][i][1]) if best_entities[j][i] != "HOLE" else ParamHole()
                command = Action(best_action_name, args, classification)
                commands.append(command)
            command_tup = tuple(commands)
            command_list = task_hints["commands"]
            if any(["HOLE" in best_entity for best_entity in best_entities]):
                command_list = task_hints["half-commands"]
            command_list.append(command_tup)
        return task_hints

    def convert_speech_simple(self, sentence):
        original_sentence = sentence.split()
        contains_speech = False
        for i, word in enumerate(original_sentence):
            if word == "say" or word == "ask":
                contains_speech = True
                break
        sentence = original_sentence[:i + 1]
        if contains_speech:
            sentence.append(self.entity_data.get_entity("content", "$speech").name)
        return " ".join(sentence), " ".join(original_sentence[i + 1:])

    def parse_from_raw_text(self, raw_text):
        # STEP 0: return empty if there is no text
        if raw_text == "":
            return []
        #task_hints = []

        # STEP 0: split into individual sentences. Go thru each individual sentence.
        sentence_text = nlp(raw_text)
        curr_char_pos = 0
        char_intervals = []
        for span in sentence_text.sents:
            text = span.text

            # STEP 1: parse input into individual commands and conditionals
            # mark the indices where an SBAR is
            parse = next(self.stanford_parser.raw_parse(text))
            parse.pretty_print()
            sbar_positions = []
            sbar_leaves = []
            self.traverse_tree(parse, sbar_positions, sbar_leaves, [0])

            # convert sbar positions to character invervals
            # character intervals DO NOT include spaces (i.e., s in "ab s" is position 2)
            in_sbar = False if 0 not in sbar_positions else True
            prev_interval_end = curr_char_pos
            in_progress_char_interval = None if not in_sbar else [0]
            for idx, leaf in enumerate(sbar_leaves):
                if not in_sbar and idx in sbar_positions:
                    in_sbar = True
                    in_progress_char_interval = [curr_char_pos]
                    char_intervals.append(Interval(prev_interval_end, curr_char_pos, "command"))
                elif in_sbar and idx not in sbar_positions:
                    in_sbar = False
                    prev_interval_end = curr_char_pos
                    in_progress_char_interval.append(prev_interval_end)
                    char_intervals.append(Interval(in_progress_char_interval[0], in_progress_char_interval[1], "conditional"))
                leaf_cc = len(leaf)
                curr_char_pos += leaf_cc
            char_intervals.append(Interval(prev_interval_end, curr_char_pos, "conditional" if in_sbar else "command"))

        # STEP 1.5: broadcast recording update containing command/conditional intervals
        # work around both sentences and intervals

        intervals_to_print = []
        for interval_data in char_intervals:
            sentence = self.preprocess_text(interval_data.get_text_from_interval(raw_text))
            if len(sentence) == 0:
                continue
            sentence, speech = self.convert_speech_simple(sentence)
            entities, nouns, pronouns, verbs = self.tag_sentences(sentence)
            task_hint = self.get_task_hints(verbs, entities, nouns, pronouns, interval_data.classification)
            # discard empty hints
            if len(task_hint["commands"]) == 0 and len(task_hint["half-commands"]) == 0 and len(task_hint["constraints"]) == 0:
                continue
            for commands in task_hint["commands"]:
                for command in commands:
                    if "speech" in command.args:
                        speech_ent = SpeechEntity("$speech", ["content"], speech)
                        self.entity_data.add_new_entity(speech_ent)
                        command.args["speech"] = ParamFilled(speech_ent)
            interval_data.set_task_hint(task_hint)
            intervals_to_print.append(interval_data)
        return char_intervals

    def traverse_tree(self, tree, sbar_positions, sbar_leaves, curr_position, in_sbar=False):
        for subtree in tree:
            if type(subtree) == nltk.tree.Tree:
                if subtree.label() == "SBAR" or in_sbar:
                    self.traverse_tree(subtree, sbar_positions, sbar_leaves, curr_position, True)
                else:
                    self.traverse_tree(subtree, sbar_positions, sbar_leaves, curr_position, False)
            else:
                sbar_leaves.append(subtree)
                if in_sbar:
                    sbar_positions.append(curr_position[0])
                curr_position[0] += 1
