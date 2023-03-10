#!/usr/bin/env python3

# Author: David Porfirio <dporfirio@wisc.edu>

"""
pipeline.py: Executes the steps required to parse raw NL
             and sketch input.
"""

import argparse
from nl_parser import *
from planner import *
from world import *
from recording import *
from program import *


class Pipeline:

    '''
    Pipeline refers to "Analysis Pipeline".
    This should be what happens when data is received.
    It does the following:
      - stores raw input data (place inside of convenience classes)
      - runs level 1 (sketch) analysis
      - stores results of level 1 (sketch) analysis
      - runs level 2 (synthesis) analysis
      - stores results of level 2 (synthesis) analysis
    '''

    def __init__(self, world, local_load=False):

        # level 1 analysis
        self.nl_parser = NLParser()
        if local_load:
            self.nl_parser.entity_data.load_entities_from_file()
        else:
            self.nl_parser.entity_data.load_entities_from_file("non_object_entities")

        # level 1 data 
        self.task_hints = None  # contains commands, half commands, and constraints

        # level 2 analysis
        self.world_st = World(world, Program())
        self.planner = Planner()

        # load default domain
        self.domain = Domain.get_instance()

    def reload_penalties(self, penalty_dict):
        self.planner.reload_penalties(penalty_dict)

    def reload_domain(self, domain_file):
        self.domain.load_domain(domain_file)

    def reload_world(self, world):
        self.world_st.load_world(world)

    def convert_msg_action_to_user_seq_action(self, trace):
        traj = []
        for item in trace:
            argdict = {}
            for arg in item["args"]:
                argdict[arg["argname"]] = arg["argval"]
            traj.append({"name": item["cmd"],
                         "type": item["_type"],
                         "_id": item["_id"],
                         "args": argdict})
        return traj

    def add_recording(self, nl, traj):
        recording = Recording(nl, traj, self.planner, self.nl_parser)
        prog = self.world_st.get_program()
        recording.create_partial_program_from_user_sequence()
        recording.parse_raw_input()
        prog.add_recording(recording)
        return prog

    def update_available_entities(self, new_avail_ents):
        print("Updating available entities.")
        self.nl_parser.entity_data.update_available_entities(new_avail_ents)

    def get_world(self):
        return self.world_st

    def get_program(self):
        return self.world_st.get_program()

    def load_program(self, nl, traj):
        for indiv_nl, indiv_traj in zip(nl, traj):
            converted_traj = self.convert_msg_action_to_user_seq_action(indiv_traj)
            self.add_recording(indiv_nl, converted_traj)


if __name__ == "__main__":
    import test_parser
    parser = argparse.ArgumentParser(description='The test interface for tabula.')
    parser.add_argument('-f', '--file', type=str, required=True)
    parser.add_argument('-o', '--oracle', action='store_true', help='Replenish the oracle test cases with updated results', required=False)
    args = vars(parser.parse_args())
    nl, traj, world, penalties, domain_file = test_parser.parse(args["file"])
    pipeline = Pipeline(world, True)
    pipeline.reload_penalties(penalties)
    pipeline.reload_domain(domain_file)
    pipeline.load_program(nl, traj)
    if args["oracle"]:
        pipeline.world_st.get_program().write_result("test_files/oracle/{}.txt".format(args["file"][args["file"].rindex("/"):]))
    else:
        pipeline.world_st.get_program().write_result("test_files/temp/{}.txt".format(args["file"][args["file"].rindex("/"):]))
