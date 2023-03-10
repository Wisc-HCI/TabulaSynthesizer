#!/usr/bin/env python3

# Author: David Porfirio <dporfirio@wisc.edu>

"""
program.py: Stores an executable robot program.
"""
import random


class RobotState:

    def __init__(self):
        self.init_state = {
            "carryingItem": False,
            "currentLocation": "home base",
            "canTravel": True
        }


class Program:

    def __init__(self):
        self.reset()

    def reset(self):
        self.ltl_properties = []
        self.recordings = []
        self.main_recording = None
        self.waypoints = {}

        # recording id counter
        self.recording_counter = 0

    def get_recordings(self):
        return self.recordings

    def add_recording(self, recording):
        '''
        Purpose of this method is to add a recording to the program.
        In doing so, the program itself is updated.
        '''
        if self.main_recording is None:
            print("Added first recording.")
            self.main_recording = recording
            self.waypoints[self.main_recording] = []
            for wp in self.main_recording.get_plan().waypoints:
                self.waypoints[self.main_recording].append(wp)
        else:
            print("Added another recording.")
            # combine with other recordings
            new_init_wp = recording.get_plan().init_waypoint
            # type is 'branch', 'else', or 'tap'
            # the default is TAP
            _type = "tap"
            # find a matching waypoint
            for old_recording in self.recordings:
                to_break = False
                print('in recordings...')
                for old_wp_label, old_wp in old_recording.get_plan().waypoints.items():
                    print("looking at wp labels...")
                    if new_init_wp.label == old_wp.label:
                        print("found label match...")
                        for act in old_wp.postmove_actions:
                            print("iterating through acts...")
                            if act._type == "conditional":
                                print("found conditional")
                                act.jump.append(recording)
                                new_init_wp._id = old_wp._id
                                to_break = True
                                break
                    if to_break:
                        break
                if to_break:
                    break

        recording._id = self.recording_counter
        self.recording_counter += 1
        self.recordings.append(recording)
        print(self)

    def get_init_wp(self):
        return self.main_recording.get_init_wp()

    def get_branching_waypoint(self, sensor_input, curr_possible_wps_names):

        def match_sensor_to_branch(si, recording, branch_wp_source, branch_wp_target_str):
            to_return = None
            if branch_wp_source in curr_possible_wps_names:
                print("source in names")
                print(si)
                print(recording.plan.branch_condition)
                if match_sensor_to_branch_helper(si, recording.plan.branch_condition):
                    to_return = {"rec": recording._id, "wp": recording.get_wp_from_str(branch_wp_target_str)}
            return to_return

        def match_sensor_to_branch_helper(si, bc):
            if bc is None:
                return False
            print(bc.get_executable_string())
            if bc.get_executable_string() == si:
                return True
            return False

        to_return = None
        for si in sensor_input:
            for recording in self.recordings:
                if recording == self.main_recording:
                    to_return = match_sensor_to_branch(si, recording,
                                                       recording.plan.init_waypoint.label,
                                                       recording.plan.init_waypoint.label)
                else:
                    to_return = match_sensor_to_branch(si, recording,
                                                       recording.starting_act.args["destination"].label.name,
                                                       recording.plan.branch_destination)
                if to_return:
                    break
            if to_return:
                break
        return to_return

    def get_next_waypoint(self, rec_id, curr_possible_wps_names):
        # first get the next waypoint within the current recording
        possible_wps = self.get_next_waypoint_helper(rec_id, curr_possible_wps_names)

        # get additional possible waypoints within other recordings
        possible_wps.extend(self.get_nonconditional_branching_waypoints(curr_possible_wps_names))

        # if no viable waypoints exist, see if ANY waypoints from other recordings match
        if len(possible_wps) < 0:
            for recording in self.recordings:
                possible_wps.extend(self.get_next_waypoint_helper(recording._id, curr_possible_wps_names))

        # return random choice
        if len(possible_wps) > 0:
            return random.choice(possible_wps)
        return None

    def get_next_waypoint_helper(self, rec_id, curr_possible_wps_names):
        possible_wps = []
        recording = None
        for rec in self.recordings:
            if rec._id == rec_id:
                recording = rec
                break
        for wp_id, wp in recording.plan.waypoints.items():
            if wp.label in curr_possible_wps_names:
                possible_wps.append({"rec": rec_id, "wp": wp.get_next_waypoint()})
        return possible_wps

    def get_nonconditional_branching_waypoints(self, curr_possible_wps_names):
        possible_wps = []
        for recording in self.recordings:
            if recording.plan.init_waypoint in curr_possible_wps_names and\
               recording.plan.branch_condition is None:
                possible_wps.append({"rec": recording._id,
                                     "wp": recording.plan.init_waypoint.get_next_waypoint()})
        return possible_wps

    def write_result(self, path):
        s = str(self)
        with open(path, "w") as outfile:
            outfile.write(s)

    def stringify(self):
        prog_dict = {"recordings": []}
        for recording in self.recordings:
            prog_dict["recordings"].append(recording.stringify(is_main=recording==self.main_recording))
        return prog_dict

    def __str__(self):
        s = ""
        for recording in self.recordings:
            sketch = recording.plan
            if recording == self.main_recording:
                s += "PROGRAM ({}, main)\ninit: {}\n".format(recording._id, sketch.init_waypoint.label)
            else:
                s += "PROGRAM ({}, branch)\nfrom: {}\n".format(recording._id, recording.starting_act.args["destination"].label.name)
            if sketch.branch_condition is not None:
                s += "branch condition: {}\n".format(sketch.branch_condition)
            if sketch.branch_condition is not None or recording != self.main_recording:
                s += "branch destination: {}\n".format(sketch.branch_destination)
            for _, wp in sketch.waypoints.items():
                s += "\n"
                s += str(wp)
        return s
