#!/usr/bin/env python3

# Author: David Porfirio <dporfirio@wisc.edu>


"""
test_parser.py: Parser for interpreting test suite files.
"""

import json
import os


def parse(filename=None):
    nl = traj = world = None
    testname = filename if filename is not None else "groceries.json"
    dir_path = os.path.dirname(os.path.realpath(__file__))
    dir_path = dir_path[:dir_path.index("TabulaSynthesizer")] + "TabulaSynthesizer/synthesizer/test_cases/" + testname
    with open(dir_path, "r") as infile:
        data = json.load(infile)
        nl = data["nl"]
        traj = data["trajectories"]
        world = data["world"]
        penalties = data["penalties"]
        domain_file = data["domain_name"]
    return nl, traj, world, penalties, domain_file
