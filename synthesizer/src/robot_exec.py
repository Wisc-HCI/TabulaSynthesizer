#!/usr/bin/env python3

# Author: David Porfirio <dporfirio@wisc.edu>

"""
robot_exec.py: For sending goals to a robot.
"""


class Executor:

    def __init__(self, send_action_cb):
        self.program = None
        self.send_action_cb = send_action_cb

        # program state for execution includes:
        #  - current waypoint
        #  - current action # within waypoint
        #  - sensor input collected during current action
        self.current_recording = 0
        self.current_wp = None
        self.current_action_no = 0
        self.sensor_input = []
        self.waiting = False

    def load_program(self, program):
        self.program = program

    def run_program(self):
        self.current_wp = self.program.get_init_wp()
        self.exec_action()

    def exec_action(self):
        '''
        0: check if a conditional has occurred that matches the
           current wp's outgoing transitions. If so, change wp.
        1: check if the number of actions in the curent wp has been
           exhausted. If so, change wp.
        2: check if this is the first action in the wp. If so, move.
        3: run the action at idx-1
        '''
        branching_wp = self.sensor_input_matches_outgoing_trans()
        if branching_wp:
            print(branching_wp)
            self.current_wp = branching_wp["wp"]
            self.current_recording = branching_wp["rec"]
            self.current_action_no = 0
            self.waiting = False
            self.sensor_input.clear()
            self.exec_action()
        elif self.current_action_no - 1 >= len(self.current_wp.postmove_actions):
            status = self.get_next_waypoint()
            if status:
                self.current_action_no = 0
                self.exec_action()
            else:
                self.waiting = True
        elif self.current_action_no == 0:
            # run the moveTo command
            act = self.current_wp.move_action
            act_string = act.get_executable_string()
            self.send_action_cb(act_string)
            self.current_action_no += 1
        else:
            # run the action at idx-1
            act = self.current_wp.postmove_actions[self.current_action_no - 1]
            act_string = act.get_executable_string()
            self.send_action_cb(act_string)
            self.current_action_no += 1
        self.sensor_input.clear()

    def sensor_input_matches_outgoing_trans(self):
        branching_wp = self.program.get_branching_waypoint(self.sensor_input, [self.current_wp.label])
        return branching_wp

    def get_next_waypoint(self):
        next_wp = self.program.get_next_waypoint(self.current_recording, [self.current_wp.label])
        if next_wp["wp"]:
            self.current_recording = next_wp["rec"]
            self.current_wp = next_wp["wp"]
            return True
        print("No further actions are available. Current wp is {}.".format(self.current_wp.label))
        return False

    def robot_finished_action(self):
        self.exec_action()

    def add_sensor_input(self, sensor_input):
        self.sensor_input.append(sensor_input)
        print("Added to sensor input: {}".format(self.sensor_input))
        if self.waiting:
            self.exec_action()