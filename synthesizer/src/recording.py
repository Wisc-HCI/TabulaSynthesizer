#!/usr/bin/env python3

# Author: David Porfirio <dporfirio@wisc.edu>

"""
recording.py: Stores a recording provided by a user.
"""

import os
import json
import copy
import time
from nl_data import NLData
from world import World
from entities import *


class Recording:

    def __init__(self, content, raw_sequence, planner, nl_parser):
        self.nl_data = NLData(content, nl_parser)
        self.planner = planner

        self.user_sequence = self.make_user_sequence(raw_sequence)
        self.starting_act = self.user_sequence[0]
        del self.user_sequence[0]
        
        self.partial_program = None  # type Sketch
        self.plan = None             # also type Sketch

        self._id = -1

    def parse_raw_input(self):
        '''
        :from_branch - if this recording will be attached to another
        '''
        self.nl_data.parse_content()
        plan_data, act_seq = self.planner.plan(self)
        self.plan = Sketch()

        # TODO: assess whether nl_data needs to be passed into the sketch here.
        # Currently it is passed only to yank the first interval if it is conditional.
        self.plan.create_sketch_from_plan(plan_data, act_seq, self.nl_data)

    def create_partial_program_from_user_sequence(self):
        self.partial_program = Sketch()
        self.partial_program.create_partial_program_from_user_sequence(self.user_sequence)

    def add_plan(self, plan):
        self.plan = plan

    def make_user_sequence(self, raw_sequence):
        user_sequence = []
        entity_data = EntityData.get_instance()
        for item in raw_sequence:
            param_args = {}
            for argname, argval in item["args"].items():
                category = entity_data.obj2category[argval][0]
                entity = entity_data.get_entity(category, argval)
                param_args[argname] = ParamFilled(entity)
            act = Action(item["name"], param_args, item["type"], item["_id"])
            user_sequence.append(act)
        return user_sequence

    def get_user_sequence(self):
        return self.user_sequence

    def get_task_hints(self):
        return self.nl_data.get_task_hints()

    def get_plan(self):
        return self.plan

    def get_init_wp(self):
        return self.plan.init_waypoint

    def get_wp_from_str(self, wp_name):
        wps = self.plan.waypoints
        for wp_id, wp in wps.items():
            if wp.label == wp_name:
                return wp
        else:
            return None

    def stringify(self, is_main):
        recording_dict = {"_id": self._id, "text": {},
                          "is_main_or_branch": "main" if is_main else "branch",
                          "branch_from": self.starting_act.args["destination"].label.name,
                          "branch_from_id": self.starting_act._id,
                          "branch_condition": None,
                          "sketch": {"user_sequence": {"data": [], "plan": {"waypoints": []}}}}
        
        # fill text
        recording_dict["text"]["content"] = self.nl_data.content.text
        recording_dict["text"]["label_intervals"] = []
        for interval in self.nl_data.content.intervals:
            interval_data = {"start": interval.start, "end": interval.end, "label": {}}
            label_data = interval_data["label"]
            label_data["_type"] = interval.classification
            label_data["cmd"] = "unknown"
            label_data["args"] = []
            if interval.action is not None:
                label_data["cmd"] = interval.action.name
                for argname, argval in interval.action.args.items():
                    label_data["args"].append({"argname": argname, "argval": argval.label.name})

        # fill sketch
        ## add first point
        init_act = {
            "_id": self.starting_act._id,
            "cmd": self.starting_act.name,
            "_type": self.starting_act._type,
            "args": [] 
        }
        for argname, argval in self.starting_act.args.items():
            init_act["args"].append({"argname": argname, "argval": argval.label.name})
        recording_dict["sketch"]["user_sequence"]["data"].append(init_act)

        ## add the rest
        for act in self.user_sequence:
            act_dict = {
                "_id": act._id,
                "cmd": act.name,
                "_type": act._type,
                "args": [] 
            }
            for argname, argval in act.args.items():
                act_dict["args"].append({"argname": argname, "argval": argval.label.name})
            recording_dict["sketch"]["user_sequence"]["data"].append(act_dict)

        wp_list = recording_dict["sketch"]["user_sequence"]["plan"]["waypoints"]
        for wp_id, wp in self.plan.waypoints.items():
            wp_dict = {"_id": wp_id, "x": 1, "y": 1, "task_subsequence": []}
            act_seq = wp_dict["task_subsequence"]
            wp_dest = wp.move_action.args["destination"].label.name
            act_seq.append({"_id": wp.move_action._id, "cmd": "moveTo", "_type": "command", "args": [{"argname": "destination", "argval": wp_dest}]})
            for act_container in wp.postmove_actions:
                # TODO: FIX THIS
                if act_container.action is not None:
                    act = act_container.action
                    act_dict = {"_id": act._id, "cmd": act.name, "_type": act._type, "args": []}
                    for argname, argval in act.args.items():
                        act_dict["args"].append({"argname": argname, "argval": argval.label.get_value()})
                    act_seq.append(act_dict)
            wp_list.append(wp_dict)

        # handle branch conditions
        # insert them as the first action at the first waypoint
        bc = self.plan.branch_condition
        if bc is not None:
            branch_dict = {"_id": bc._id, "cmd": bc.name, "_type": bc._type, "args": []}
            for argname, argval in bc.args.items():
                branch_dict["args"].append({"argname": argname, "argval": argval.label.get_value()})
            wp_list[0]["task_subsequence"].insert(1, branch_dict)
            recording_dict["branch_condition"] = branch_dict
        return recording_dict


class Sketch:

    def __init__(self):
        self.waypoints = {}
        self.init_waypoint = None

        # only for non-main sketches
        self.branch_condition = None
        self.branch_destination = None

    def create_sketch_from_plan(self, plan_data, act_seq_data, nl_data):
        '''
        Purpose of this method is to add a recording to the program.
        In doing so, the program itself is updated.
        '''
        def action_or_conditional(action):
            if action._type == "conditional" or action._type == "trigger":
                return "conditional"
            return "command"

        # for adding new objects to the world based on the plan
        world_st = World.get_instance()

        plan = plan_data[0]
        act_seq_idxs = plan_data[1]
        created_objects = plan_data[4]
        id_counter = -1
        act_id_tracker = {}
        act_id_tracker_rev = {}
        act_seq = []
        for wp_data in act_seq_data:
            act_seq.append(wp_data["waypoint"].move_action)
            act_seq.extend(wp_data["waypoint"].postmove_actions)
        for act in act_seq:
            if id_counter <= act._id:
                id_counter = act._id + 1
        wp = None
        prev_wp_id = None
        curr_acts = []
        for i, act in enumerate(plan):
            if act.name == "moveTo":
                if self.branch_destination is None:
                    self.branch_destination = act.args["destination"].label.name
                if wp is not None:
                    wp.postmove_actions = copy.copy(curr_acts)
                act_label = act.args["destination"].label.name

                # unsure if the following will work
                _id = -1
                if i in act_seq_idxs and \
                    (act_seq[act_seq_idxs.index(i)]._id not in act_id_tracker_rev \
                        or act_id_tracker_rev[act_seq[act_seq_idxs.index(i)]._id] == act_label):
                    orig_act = act_seq[act_seq_idxs.index(i)]
                    _id = orig_act._id
                    act_id_tracker[act_label] = _id
                    act_id_tracker_rev[_id] = act_label

                    # add relevant objects to the world
                    if i < len(created_objects) and len(created_objects[i]) > 0:
                        for created_object in created_objects[i]:
                            world_st.add_entity_to_region(created_object.name, act_seq[act_seq_idxs.index(i)].args["destination"].label.name)

                elif act_label in act_id_tracker:
                    _id = act_id_tracker[act_label]
                else:
                    _id = id_counter
                    act_id_tracker[act_label] = _id
                    act_id_tracker_rev[_id] = act_label
                    id_counter += 1
                # # # # # # #

                self.add_waypoint_from_trace(act_label, _id, prev_wp_id)
                wp = self.waypoints[_id]
                prev_wp_id = _id
                curr_acts.clear()
            elif wp is None:
                continue
            elif i == len(plan) - 1:
                curr_acts.append(ActionContainer(action_or_conditional(act), act))
                wp.postmove_actions = copy.copy(curr_acts)
            else:
                curr_acts.append(ActionContainer(action_or_conditional(act), act))

        # if applicable, make the conditional/trigger corresponding to the first
        #    command as the branch condition
        intervals = nl_data.get_task_hints()
        if len(intervals) > 0:
            first_interval = intervals[0]
            if first_interval.classification != "command":
                # extract the appropriate action
                # NOTE: actions cannot be created from half-commands or constraints!
                if len(first_interval.task_hints["commands"]) > 0:
                    self.branch_condition = first_interval.task_hints["commands"][0][0]

        '''
        # if necessary, move the conditional in the init waypoint to the branch condition
        if len(self.init_waypoint.postmove_actions) > 0 and self.init_waypoint.postmove_actions[0]._type != "command":
            self.branch_condition = self.init_waypoint.postmove_actions[0].action
            del self.init_waypoint.postmove_actions[0]
        '''

    def add_waypoint_from_trace(self, wp_label, wp_id, prev_wp_id):
        # TODO: waypoint labels might not be unique. 
        if wp_id not in self.waypoints:
            self.waypoints[wp_id] = Waypoint(wp_label, wp_id)
        if self.init_waypoint is None:
            self.init_waypoint = self.waypoints[wp_id]
        if prev_wp_id is not None:
            wp = self.waypoints[wp_id]
            prev_wp = self.waypoints[prev_wp_id]
            if not any([if_exec.is_same_trans(wp) for if_exec in prev_wp.if_execs]):
                prev_wp.if_execs.append(IfExec(wp))

    def create_partial_program_from_user_sequence(self, user_sequence):
        prev_wp_id = None
        for item in user_sequence:
            wp_label = item.args["destination"].label.name
            wp_id = item._id
            self.add_waypoint_from_trace(wp_label, wp_id, prev_wp_id)
            ###prev_wp_id = wp_label
            prev_wp_id = wp_id


class Domain:

    '''
    Singleton Storage Class
    '''
    __instance = None

    @staticmethod
    def get_instance():
        if Domain.__instance is None:
            Domain()
        return Domain.__instance

    def __init__(self):
        self.action_primitives = None
        self.init = None
        if Domain.__instance is not None:
            raise Exception("Domain is a singleton!")
        else:
            Domain.__instance = self
        self.action_primitives = None
        self.load_domain("default_domain")

    def load_domain(self, domain_file):
        dir_path = os.path.dirname(os.path.realpath(__file__))
        with open("{}/domains/{}.json".format(dir_path, domain_file), "r") as infile:
            self.action_primitives = json.load(infile)
            self.init = copy.copy(self.action_primitives["init"])
            del self.action_primitives["init"]

    def get_idle(self):
        return Action("idle", {})


class Waypoint:

    def __init__(self, label, _id=-1):
        self._id = _id
        self.label = label
        self.precondition = Condition([{}])
        #self.calculate_trajectory_action = Action("calculateTrajectory", {"destination": label})
        entity_data = EntityData.get_instance()
        category = entity_data.obj2category[label][0]
        entity = entity_data.get_entity(category, label)
        self.move_action = Action("moveTo", {"destination": ParamFilled(entity)}, _id=self._id)
        self.postmove_actions = []
        self.if_execs = []

    def get_next_waypoint(self):
        next_wp = None
        for wp in [ie.waypoint for ie in self.if_execs]:
            next_wp = wp
            break
        return next_wp

    def __str__(self):
        s = "waypoint: {}\n".format(self.label)
        s += "  >> acts {}\n".format(" : ".join([str(act) for act in self.postmove_actions]))
        for ie in self.if_execs:
            s += "  goto >> {}".format(str(ie))
        return s


class ActionContainer:

    def __init__(self, _type, action_or_conditional):
        self._type = _type  # command or conditional

        # if is a single action = None
        self.action = action_or_conditional   # type = action

        # if is a conditional
        self.jump = []   # type = list of recording id's

    def add_jump(self, recording):
        if self._type == "action":
            print("ERROR: cannot add a jump to a non-conditional")
            exit()
        self.jump = recording

    def get_executable_string(self):
        if self._type == "command":
            return self.action.get_executable_string()
        else:
            return "?"

    def __str__(self):
        if self._type == "conditional":
            s = "\n          CONTINUE IF: {};\n          ELSE see jumps: ".format(str(self.action))
            for jmp in self.jump:
                s += "\n            {}".format(str(jmp._id))
            return s
        else:
            return str(self.action)


class Action:

    def __init__(self, name, args, _type="command", _id=-1):
        action_data = Domain.get_instance().action_primitives[name]
        self._id = _id
        self.name = name
        self.args = args
        self.argnames = sorted(list(self.args.keys()))
        self.precondition = self.create_action_conditions(action_data["preconditions"])
        self.postcondition = self.create_action_conditions(action_data["postconditions"])
        self.verbsets = action_data["verbnet"]
        self.synsets = action_data["synsets"]

        # determine if this action is a command, trigger, or conditional
        if len(action_data["action_types"]) == 1:
            self._type = action_data["action_types"][0]
        else:
            self._type = _type

    def create_action_conditions(self, raw_conditions):
        conditions = copy.copy(raw_conditions)
        for condition_name, condition_val in raw_conditions.items():
            # case 1: val is an int
            if type(condition_val) == int:
                argname = self.argnames[condition_val]
                argval = self.args[argname]
                conditions[condition_name] = argval.label.name if argval.filled else condition_val
        return Condition([conditions])

    def is_superset(self, other):
        '''
        One action is a superset of another action if:
            - they are equal
            - the first action has holes that the other can fill
            - a "destination" arg of the original action contains the "destination" of the other
        '''
        world_st = World.get_instance()
        if self.name != other.name:
            return False
        if self._type != other._type:
            return False
        for arg, argval in self.args.items():
            other_argval = other.args[arg]
            if argval.hole:
                continue
            if arg == "destination":
                if other_argval.label.name != argval.label.name and\
                   not world_st.is_entity_within_region(other_argval.label.name, argval.label.name):
                    return False
            elif not argval.label.equals(other_argval.label):
                return False
        return True

    def get_executable_string(self):
        s = self.name
        for arg, argval in self.args.items():
            s += " {}".format(argval.label)
        return s

    def __str__(self):
        s = "ACTION{}: {} : {}".format(" ({})".format(self._type) if self._type != "command" else "", self.name, [str(argval) for argname, argval in self.args.items()])
        return s

    def equals(self, other):
        if self.name != other.name:
            return False
        if self._type != other._type:
            return False
        same_args = True
        for argname, argval in self.args.items():
            if not argval.equals(other.args[argname]):
                same_args = False
                break
        return same_args


class Condition:

    def __init__(self, dnf):
        self._or = dnf

    def __str__(self):
        return str(self._or)


class ConditionHole:

    def __init__(self):
        pass


class Param:

    def __init__(self, is_filled):
        self.filled = is_filled
        self.hole = not self.filled

    def equals(self, other):
        if self.hole and other.hole:
            return True
        if self.label.equals(other.label):
            return True
        return False


class ParamFilled(Param):

    def __init__(self, label):
        super().__init__(is_filled=True)
        self.label = label

    def __str__(self):
        s = "Param:\n"
        s += str(self.label)
        return s


class ParamHole(Param):

    def __init__(self):
        super().__init__(is_filled=False)


class IfExec:

    def __init__(self, wp):
        self.conditional = ConditionHole()
        self.waypoint = wp

    def is_same_trans(self, other_wp):
        if self.waypoint == other_wp:
            return True
        return False

    def __str__(self):
        return "{}\n".format(self.waypoint.label)