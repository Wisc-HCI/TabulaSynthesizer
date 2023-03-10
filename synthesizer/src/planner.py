#!/usr/bin/env python3

# Author: David Porfirio <dporfirio@wisc.edu>

"""
planner.py: Invokes Astar to make a plan from raw NL and
            sketch input.
"""

import copy
import math
import itertools
from world import World
from program import *
from recording import Action, ParamFilled
from recording import Domain
from entities import EntityData, SpeechEntity


class Planner:

    # penalties
    PENALTY_CREATION = 1
    PENALTY_DUPLICATE_CREATION = 1
    PENALTY_NEW_ACTION = 1
    PENALTY_EMPTY_WAYPOINT = 1

    def __init__(self):
        self.world_st = World.get_instance()
        self.available_actions = Domain.get_instance()
        self.available_entities = EntityData.get_instance()

    def reload_penalties(self, penalty_dict):
        self.PENALTY_CREATION = penalty_dict["PENALTY_CREATION"]
        self.PENALTY_DUPLICATE_CREATION = penalty_dict["PENALTY_DUPLICATE_CREATION"]
        self.PENALTY_NEW_ACTION = penalty_dict["PENALTY_NEW_ACTION"]
        self.PENALTY_EMPTY_WAYPOINT = penalty_dict["PENALTY_EMPTY_WAYPOINT"]

    def plan(self, recording):
        # get a SORTED list of ALL traces with holes (avoid multiple repeats of loops)
        # we can trim prefixes before the first hole by enumerating all possible preconditions
        # 
        # :recording - the parsed nl and seq data
        holey_wp_traces = []
        init_wp = recording.partial_program.init_waypoint
        self.get_traces(init_wp, holey_wp_traces, [init_wp])
        holey_traces = []
        for holey_wp_trace in holey_wp_traces:
            holey_trace = []
            for wp in holey_wp_trace:
                temp = {"waypoint": wp, "actions": []}
                temp["actions"].append(wp.move_action)
                for action in wp.postmove_actions:
                    temp["actions"].append(action)
                holey_trace.append(temp)
            holey_traces.append(holey_trace)
        holey_traces.sort(key=len)  # from smallest to largest
        best_solution = None
        n = len(holey_traces)    # change to 1 if slow

        # find solution for n traces (avoiding previous bad solutions),
        # prioritizing the traces that have the MOST holes
        traces = holey_traces[:n]
        solution = self.solve(traces[0], recording.get_task_hints())
        solution_sat = True if solution is not None else False

        # if there is a solution, set best solution break!
        if solution_sat:
            best_solution = solution

        # otherwise, add the bad solution to the list of bad solutions, n += 1
        # TODO: if there is no solution, then the code will die
        else:
            best_solution = None

        return best_solution, holey_traces[-1]

    def contains_double_loop(self, trace):
        idx = -1
        if len(trace) < 2:
            return False, idx
        # test to see if the same loop exists twice
        contains = False
        for i in range(1, math.floor((len(trace)+1)/2)):
            test_loop = trace[len(trace) - i:]
            test_loop.reverse()
            candidate_loop = trace[len(trace) - i*2:len(trace) - i]
            candidate_loop.reverse()
            if test_loop == candidate_loop:
                contains = True
                idx = i
                break
        return contains, idx

    def get_traces(self, curr_wp, holey_traces, curr_trace):
        # analyze curr trace to see if it is a complete trace
        if len(curr_wp.if_execs) == 0:
            if curr_trace not in holey_traces:
                holey_traces.append(curr_trace)
            return
        contains, i = self.contains_double_loop(curr_trace)
        if contains:
            if curr_trace not in holey_traces:
                holey_traces.append(curr_trace)
            return
        # continue to add to the curr trace
        for if_exec in curr_wp.if_execs:
            next_wp = if_exec.waypoint
            updated_trace = copy.copy(curr_trace)
            updated_trace.append(next_wp)
            self.get_traces(next_wp, holey_traces, updated_trace)

    def solve(self, trace, hints):
        idx_to_hint = {}
        hint_seq = []
        for interval in hints:

            # TODO: handle triggers and conditionals inserted mid-way
            #   into a recording
            if interval.classification != "command":
                continue

            hint_dict = interval.get_task_hint()
            hint_seq.extend(hint_dict["commands"])
            hint_seq.extend(hint_dict["half-commands"])
        for i, hint in enumerate(hint_seq):
            idx_to_hint[i] = hint

        # An unattached entity (incomplete hint) MUST
        # be included in the solution SEPARATELY.
        idx_to_ent = {}
        detached_entities = []
        for interval in hints:
            hint_dict = interval.get_task_hint()
            if len(hint_dict["constraints"]) > 0:
                detached_entities.append(hint_dict["constraints"][0][1])
        for i, de in enumerate(detached_entities):
            idx_to_ent[i] = de
        solutions = self.solve_helper(trace, hint_seq, detached_entities)
        if len(solutions) == 0:
            return None
        return solutions[0]

    def solve_helper(self, trace, hint_list, detached_entities):
        ad = Domain.get_instance()
        act_seq = []
        for wp_dict in trace:
            wp = wp_dict["waypoint"]
            act_seq.append(wp.move_action)
            act_seq.extend(wp.postmove_actions)
        start = ((ad.get_idle(),), tuple(key for key in sorted(list(ad.init.keys()))),
                 tuple([ad.init[key] for key in sorted(list(ad.init.keys()))]), ())
        solutions = []
        operators = self.get_allowable_operators(hint_list, detached_entities, act_seq)
        self.astar(start, act_seq, hint_list, detached_entities, solutions, operators)
        return solutions

    # # # # # # # # # # # # # # # # #
    #                               #
    # BEST FIRST SEARCH             #
    # (astar algorithm)             #
    #                               #
    # # # # # # # # # # # # # # # # #
    def astar(self, start, act_seq, hint_list, detached_entities, solutions, operators):
        open_set = [start]
        came_from = {}
        g_score = {}  # if an element is not here, the val is inf
        g_score[start] = 0
        f_score = {}  # if an element is not here, the val is inf
        f_score[start] = self.heuristic(start, act_seq, hint_list, detached_entities)
        while len(open_set) > 0:
            current = open_set[0]
            goal_sat, trace_idxs, hint_idxs, ent_idxs = self.goal_satisfied(current, act_seq, hint_list, detached_entities)
            if goal_sat:
                solutions.append((self.reconstruct_path(came_from, current), trace_idxs, hint_idxs, ent_idxs, current[3]))
                if len(solutions) > 0:
                    return
            open_set.remove(current)
            # adhere to the length cap
            # SIMPLE: length cap = len(act_seq)*3 with a minimum of 10
            if len(current[0]) > max(10, len(act_seq) * 3):
                neighbors = []
            else:
                neighbors = self.get_current_neighbors(current, act_seq, hint_list, detached_entities, operators)
            
            for neighbor_data in neighbors:
                # vet the neighbor.
                # if there is already a solution that includes neighbor
                # discard the neighbor
                neighbor = neighbor_data[0]
                if self.solutions_contain_neighbor(neighbor, solutions):
                    continue
                obj_penalty = neighbor_data[1]
                tentative_g_score = g_score[current] + self.get_neighbor_cost(current, neighbor, obj_penalty)
                if (neighbor in g_score and tentative_g_score < g_score[neighbor]) or tentative_g_score < 100000:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, act_seq, hint_list, detached_entities)
                    
                    if neighbor not in open_set:
                        open_set.append(neighbor)
                        open_set.sort(key=lambda x: f_score[x])
        return

    def get_current_neighbors(self, current, act_seq, hint_list, detached_entities, operators):
        # paths with more than two (three) loops should be vetted
        if self.contains_triple_loop(current):
            return []

        neighbors = []
        curr_args = {}
        action_history = list(current[0])
        for i, arg in enumerate(current[1]):
            curr_args[arg] = current[2][i]
        created_object_history = list(current[3])
        ap = Domain.get_instance().action_primitives
        ed = EntityData.get_instance().entities
        for act_name, act_data in ap.items():
            if act_name not in [o.name for o in operators]:
                continue
            for action_type in act_data["action_types"]:

                # TODO: handle triggers and conditionals inserted mid-way
                #   into a recording
                #if action_type != "command":
                #    continue

                avail_args = []
                for argtype in act_data["argtypes"]:
                    avail_args.append(ed[argtype])
                arg_combos = list(itertools.product(*avail_args))
                for arg_combo in arg_combos:
                    args = {}
                    item_args = {}
                    for i, ent in enumerate(arg_combo):
                        argname = act_data["argnames"][i]
                        argval = ent
                        args[argname] = ParamFilled(argval)
                        if act_name != "moveTo" and "content" not in argval.categories and "narrative" not in argval.categories:
                            item_args[argval] = False

                    # candidate action
                    act = Action(act_name, args, action_type)

                    # make sure that the candidate action is a viable operator
                    if not any([op.is_match(act) for op in operators]):
                        continue

                    # VETTING!
                    # VET unparameterized speech
                    if act_name == "say":
                        if type(args["speech"].label) != SpeechEntity:
                            continue

                    # VET moveTo destinations that are:
                    # a) not in the set of waypoints provided by the trajectory AND
                    # b) not in the set of objects WITHIN each waypoint trajectory
                    # c) not entities in the speech provided by the developer AND
                    # d) not in the categories of any unparameterized holes in a command AND
                    # e) (TODO) not in any partial order plan generated by POP algorithm
                    if act_name == "moveTo":
                        in_waypoints = False  #1
                        in_object_in_waypoints = False  #2
                        in_speech_entities = False  #3
                        in_hole_category = False  #4
                        in_pop = False  #5

                        moveTo_destination = args["destination"].label.name
                        
                        # 1 and 2
                        for wp_act in act_seq:
                            #1
                            if wp_act.equals(act):
                                in_waypoints = True
                            #2
                            if self.world_st.is_entity_within_region(moveTo_destination, wp_act.args["destination"].label.name):
                                in_object_in_waypoints = True

                        # 3, part 1, and 4
                        for hint_tup in hint_list:
                            for hint in hint_tup:
                                for argname, argval in hint.args.items():
                                    if argval.filled:
                                        # 3, part 1
                                        if moveTo_destination == argval.label.name:
                                            in_speech_entities = True
                                    else:
                                        # 4
                                        hint_name = hint.name
                                        hint_primitive = self.available_actions.action_primitives[hint_name]
                                        arg_idx = hint_primitive["argnames"].index(argname)
                                        argtype = hint_primitive["argtypes"][arg_idx]
                                        if moveTo_destination in [ent.name for ent in self.available_entities.entities[argtype]]:
                                            in_hole_category = True

                        # 3, part 2
                        for detached_entity in detached_entities:
                            if detached_entity.name == moveTo_destination:
                                in_speech_entities = True

                        # 5
                        if not (in_waypoints or in_object_in_waypoints or in_speech_entities or in_hole_category or in_pop):
                            continue

                    # vet other commands that are also not in the partial order plan generated by POP
                    else:
                        pass

                    # Vet the NON-moveto action and, NON-content, NON-narrative arguments.
                    # if the robot is manipulating something that it is not
                    # NEAR or does not POSSESS, then the action-to-be is invalid
                    # start with what the robot possesses
                    if curr_args["carryingItem"] in [ent.name for ent in item_args]:
                        argval = [ent for ent in item_args if ent.name == curr_args["carryingItem"]][0]
                        item_args[argval] = True

                    # now do what the robot is near
                    act_hist_rev = copy.copy(action_history)
                    act_hist_rev.reverse()
                    try:
                        loc, _, _ = self.find_previous_moveto(act_hist_rev)
                        regstr = loc.args["destination"].label.name
                    except Exception:
                        regstr = "home base"
                    for argval, b in item_args.items():
                        if not b:
                            item_args[argval] = argval.name == regstr # self.world_st.is_entity_within_region(argval.name, regstr)
                    
                    # penalties:
                    # - there is a penalty to CREATE
                    # - there is a penalty if creation is NEEDLESS (there is already one in the world)
                    new_created_objects = []
                    creation_penalty = 0
                    for key, val in item_args.items():
                        if not val:
                            creation_penalty += self.PENALTY_CREATION
                            new_created_objects.append(key)
                    #creation_penalty = sum([1 for key, val in item_args.items() if not val])
                    duplication_penalty = sum([self.PENALTY_DUPLICATE_CREATION
                                               for key, val in item_args.items()
                                               if ((not val) and key.name in
                                                   self.world_st.world["manuallyAddedObjects"])]) # self.world_st.world["regions"])]
                    penalty = creation_penalty + duplication_penalty

                    all_preconds_sat = True
                    updated_args = {}
                    for cond_name, cond_val in curr_args.items():
                        post = act.postcondition._or[0]
                        pre = act.precondition._or[0]
                        if cond_name in pre and pre[cond_name] != cond_val:
                            all_preconds_sat = False
                        else:
                            if cond_name in post:
                                updated_args[cond_name] = post[cond_name]
                            elif cond_name in pre:
                                updated_args[cond_name] = pre[cond_name]
                            else:
                                updated_args[cond_name] = curr_args[cond_name]
                    if all_preconds_sat:
                        updated_act_hist = copy.copy(action_history)
                        if self.is_conflicting_action(action_history, act):
                            continue
                        updated_act_hist.append(act)
                        updated_act_hist = tuple(updated_act_hist)
                        all_updated_args = copy.copy(curr_args)
                        for updated_arg, updated_argval in updated_args.items():
                            all_updated_args[updated_arg] = updated_argval
                        updated_argvals = tuple([all_updated_args[key] for key in sorted(list(all_updated_args.keys()))])

                        # update the list of created objects per step
                        new_created_object_history = copy.copy(created_object_history)
                        new_created_object_history.append(tuple(new_created_objects))
                        new_created_object_history = tuple(new_created_object_history)
                        neighbors.append(((updated_act_hist, current[1], updated_argvals, new_created_object_history), penalty))
        return neighbors

    def reconstruct_path(self, came_from, current):
        total_path = [current[0][-1]]
        while current in list(came_from.keys()):
            current = came_from[current]
            total_path.insert(0, current[0][-1])
        # cap the trace
        act_history = copy.copy(total_path)
        act_history.reverse()
        curr_move_to, _, i = self.find_previous_moveto(act_history)
        other_move_to, cap_acts, _ = self.find_matching_moveto(act_history, curr_move_to, i)
        if other_move_to is not None:
            act_history = act_history[i:]
            cap_acts.extend(act_history)
        else:
            cap_acts = act_history
        cap_acts.reverse()
        return cap_acts

    def trace_satisfied_hints_helper(self, curr_act_idx, curr_hint_idx, act_history, hint_list, detached_tracker, sat_counter, scores):
        # recursive helper method
        # base case
        if curr_act_idx == len(act_history):
            scores.append(sum(sat_counter))
            if curr_hint_idx == len(hint_list) and all(list(detached_tracker.values())):
                return True
            return False

        # recursive case -- hints
        curr_act = act_history[curr_act_idx]
        result = False
        # -- see if current hint matches
        if curr_hint_idx < len(hint_list):
            curr_hint_tuple = hint_list[curr_hint_idx]
            for curr_hint in curr_hint_tuple:
                if curr_hint.is_superset(curr_act):
                    new_sat_counter = copy.copy(sat_counter)
                    new_sat_counter[0] += 1
                    sat = self.trace_satisfied_hints_helper(curr_act_idx + 1, curr_hint_idx + 1, act_history, hint_list, detached_tracker, new_sat_counter, scores)
                    result = result or sat

        # recursive case -- detached args
        args = list(curr_act.args.values())
        args = [argval.label for argval in args]
        for entity, val in detached_tracker.items():
            if not val:
                if any([(entity == arg or ("location" in entity.categories and self.world_st.is_entity_within_region(arg.name, entity.name))) for arg in args]):# entity in args:
                    detached_tracker_copy = copy.copy(detached_tracker)
                    detached_tracker_copy[entity] = True
                    new_sat_counter = copy.copy(sat_counter)
                    new_sat_counter[1] += 1
                    sat = self.trace_satisfied_hints_helper(curr_act_idx + 1, curr_hint_idx, act_history, hint_list, detached_tracker_copy, new_sat_counter, scores)
                    result = result or sat

        # recursive case -- no match
        sat = self.trace_satisfied_hints_helper(curr_act_idx + 1, curr_hint_idx, act_history, hint_list, detached_tracker, sat_counter, scores)
        result = result or sat
        return result

    def trace_satisfies_hints(self, act_history, hint_list, detached_entities):
        # recursively determine if the list of hints and detached entities are SEPARATELY
        # present in the act_history.
        detached_tracker = {}
        for ent in detached_entities:
            detached_tracker[ent] = False
        sat_counter = [0, 0]
        scores = []
        hint_idxs = []
        ent_idxs = []
        to_return = self.trace_satisfied_hints_helper(0, 0, act_history, hint_list, detached_tracker, sat_counter, scores)
        return to_return, max(scores), hint_idxs, ent_idxs

    def goal_satisfied(self, curr, act_seq, hint_list, detached_entities):
        # 1) is the sequence of actions present in the curr?
        # 2) is the hint_list present in the curr?
        # 3) are all incomplete hints in the sequence? Each must be separately present. 
        act_history = curr[0]
        act_seq_idx = 0
        act_seq_idxs = []
        for i, act in enumerate(act_history):
            if act_seq_idx < len(act_seq) and act_seq[act_seq_idx].is_superset(act):
                act_seq_idxs.append(i)
                act_seq_idx += 1
        hint_sat, _, hint_idxs, ent_idxs = self.trace_satisfies_hints(act_history, hint_list, detached_entities)
        if act_seq_idx == len(act_seq) and hint_sat:
            return True, act_seq_idxs, hint_idxs, ent_idxs
        return False, act_seq_idxs, hint_idxs, ent_idxs

    def cost(self, curr, init_cost=PENALTY_NEW_ACTION):
        '''
        Find the cost for adding a new action.
        Most of the time, this cost will be 1. 
        However, sometimes the cost will have an added wp penalty for wp's with empty
            action sequences.
        For wp penalties, DO NOT double penalize waypoints.
        '''
        cost = init_cost
        act_history = curr[0]
        if len(act_history) < 2:
            return cost
        mr_action = act_history[-1]
        sr_action = act_history[-2]
        if mr_action.name == "moveTo" and sr_action.name == "moveTo":  # possible penalty
            is_double = False
            for i, act in enumerate(act_history[:-2]):
                if act.equals(sr_action):
                    is_double = True
                    break
            if not is_double:
                cost += self.PENALTY_EMPTY_WAYPOINT
        return cost

    def heuristic(self, curr, act_seq, hint_list, detached_entities):
        # how far are we from including the act_seq, hint_list, and incomplete_hint_seq in curr?
        act_history = curr[0]
        act_seq_idx = 0
        act_seq_matches = []
        for i, act in enumerate(act_history):
            if act_seq_idx < len(act_seq) and act_seq[act_seq_idx].is_superset(act):
                act_seq_matches.append(act)
                act_seq_idx += 1
        # calculate penalty deductions
        penalty_deduction = 0
        for i in range(act_seq_idx, len(act_seq)):
            act = act_seq[i]
            if any([act.is_superset(historical_act) for historical_act in act_seq_matches]):
                penalty_deduction += 1
        # is there a remaining hint with a precondition that cannot be satisfied
        #    with the current conditions AND cannot be satisfied by the postconditions
        #    of any other hint or waypoint?
        hint_precond = 0
        _, hint_score, _, _ = self.trace_satisfies_hints(act_history, hint_list, detached_entities)
        val = ((len(act_seq) - act_seq_idx) - penalty_deduction) + ((len(hint_list) + len(detached_entities)) - hint_score) + hint_precond
        return val

    def solutions_contain_neighbor(self, neighbor, solutions):
        '''
        The purpose of this method is to make sure that any additional available solutions
        are not duplicates of the first solution found. I.e., if the solution below exists,

                   X - Y - Z - X - Y - Z

        then the following additional solution should be prevented:

                   X - Y - Z - X - Y - Z - X
        '''
        result = False
        for solution in solutions:
            temp_result = True
            n_act_seq = neighbor[0]
            act_seq = solution[0]
            if len(n_act_seq) < len(act_seq):
                continue
            for i, n_act in enumerate(n_act_seq):
                if i >= len(act_seq):
                    break
                if not n_act.equals(act_seq[i]):
                    temp_result = False
                    break
            result = (result or temp_result)
        return result

    def get_neighbor_cost(self, current, neighbor, obj_penalty):
        if self.is_repeat_action(list(current[0]), neighbor):
            return self.cost(neighbor, 0)
        else:
            return self.cost(neighbor) + obj_penalty

    # # # # # # # # # # # # # # # # #
    #                               #
    # UTILITY FUNCTIONS             #
    #                               #
    # # # # # # # # # # # # # # # # #
    def is_repeat_action(self, act_history, new_act):
        if len(act_history) == 1:
            return False

        # get current "moveTo" and actions after the "moveTo"
        if new_act[0][-1].name == "moveTo":
            curr_move_to = new_act[0][-1]
            i = 0
        else:
            curr_move_to = None
            act_history = copy.copy(act_history)
            act_history.reverse()
            for i, act in enumerate(act_history):
                if act.name == "moveTo":
                    curr_move_to = act
                    break
        if curr_move_to is None:
            assert curr_move_to is not None,\
                   "must have a moveTo action in sequence of actions"

        # see if there is a previous "moveTo", determine expected action
        other_move_to = None
        for j in range(i+1, len(act_history)):
            act = act_history[j]
            if act.equals(curr_move_to):
                other_move_to = act
                break

        if other_move_to is None:
            return False
        return True

    def find_previous_moveto(self, rev_act_history):
        curr_move_to = None
        post_actions = []
        for i, act in enumerate(rev_act_history):
            if act.name == "moveTo":
                curr_move_to = act
                break
            post_actions.append(act)
        if curr_move_to is None:
            return Exception
        return curr_move_to, post_actions, i

    def find_matching_moveto(self, rev_act_history, move_to, start_idx):
        other_move_to = None
        cap_acts = []
        for j in range(start_idx+1, len(rev_act_history)):
            act = rev_act_history[j]
            if act.equals(move_to):
                other_move_to = act
                break
            elif act.name == "moveTo":
                cap_acts = []
            else:
                cap_acts.append(act)
        return other_move_to, cap_acts, j

    def is_conflicting_action(self, act_history, new_act):
        if len(act_history) == 1 and new_act.name == "moveTo":
            return False
        elif len(act_history) ==1 and new_act.name != "moveTo":
            return True

        # get current "moveTo" and actions after the "moveTo"
        act_history = copy.copy(act_history)
        act_history.reverse()
        curr_move_to, post_actions, i = self.find_previous_moveto(act_history)
        other_move_to, _, j = self.find_matching_moveto(act_history, curr_move_to, i)

        if other_move_to is None:
            return False

        expected_action = act_history[j - len(post_actions) - 1]
        if not expected_action.equals(new_act):
            return True
        return False

    def contains_triple_loop(self, current):
        act_history = current[0]
        act_history = [str(ah) for ah in act_history]
        if len(act_history) < 3:
            return False
        i = len(act_history) - 1
        contains = False
        while True:
            slice_size = len(act_history) - i
            if len(act_history) < slice_size * 3:
                break
            first_slice = act_history[i:]
            second_slice = act_history[i-slice_size:i]
            third_slice = act_history[i-(slice_size*2):i-slice_size]
            if first_slice == second_slice == third_slice:
                contains = True
                break
            i -= 1
        return contains

    def get_allowable_operators(self, hint_list, detached_entities, act_seq):
        operators = [Operator("moveTo", "command", [None])]

        # add hints
        for hint_tup in hint_list:
            for hint in hint_tup:
                args = []
                for argname in hint.argnames:
                    arg = hint.args[argname]
                    if type(arg) != ParamFilled:
                        args.append(None)
                    elif type(arg.label) == SpeechEntity:
                        args.append(arg.label.speech)
                    else:
                        args.append(arg.label.name)
                operators.append(Operator(hint.name, hint._type, args))

        # add actions that meet the preconditions of hints
        preconditions = []
        for hint_tup in hint_list:
            for act in hint_tup:
                for precond_type, precond_val in act.precondition._or[0].items():
                    preconditions.append({"type": precond_type, "val": precond_val})
        for entity in detached_entities:
            for ap, ap_data in Domain.get_instance().action_primitives.items():
                if len(set(ap_data["argtypes"]).intersection(set(entity.categories))) > 0:
                    for at in ap_data["action_types"]:
                        operators.append(Operator(ap, at, [None for arg in ap_data["argnames"]]))
                    continue    
        for ap, ap_data in Domain.get_instance().action_primitives.items():
            for precondition in preconditions:
                if precondition["type"] in ap_data["postconditions"]:
                    if type(ap_data["postconditions"][precondition["type"]]) != int and \
                       precondition["val"] == ap_data["postconditions"][precondition["type"]]:
                        for at in ap_data["action_types"]:
                            operators.append(Operator(ap, at, [None for arg in ap_data["argnames"]]))
                        break
                    elif type(ap_data["postconditions"][precondition["type"]]) == int:
                        for at in ap_data["action_types"]:
                            operators.append(Operator(ap, at, [None for arg in ap_data["argnames"]]))
                        break
        operators = list(set(operators))
        return operators


class Operator:

    def __init__(self, name, action_type, args):
        self.name = name
        self.action_type = action_type
        self.args = args

    def is_match(self, action):
        if action.name != self.name:
            return False
        if action._type != self.action_type:
            return False
        for i, argname in enumerate(action.argnames):
            arg = action.args[argname]
            self_arg = self.args[i]
            if self_arg is not None:
                if type(arg.label) is SpeechEntity and self_arg != arg.label.speech:
                    return False
                elif type(arg.label) is not SpeechEntity and self_arg != arg.label.name:
                    return False
        return True

    def __str__(self):
        return "operator({}, {}, {})".format(self.name, self.action_type,
                                             str(["{}".format(arg)
                                                 for arg in self.args]))
