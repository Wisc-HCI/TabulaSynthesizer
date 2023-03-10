#!/usr/bin/env python3

# Author: David Porfirio <dporfirio@wisc.edu>

"""ctrl.py: Communicating with a frontend via ROS."""

import rospy
import json
import time
import traceback
import threading
from tabula_msgs.msg import AvailableEntities, Update
from std_msgs.msg import Empty, String
from nav_msgs.msg import OccupancyGrid
from pipeline import Pipeline
from robot_exec import Executor
import test_parser

NO_CHANGE = 0
NEW_RECORDING = 1
DELETE_RECORDING = 2
RESYNTH = 3


class Controller:

    def __init__(self):
        rospy.init_node("synthesizer", anonymous=True)
        rospy.Subscriber("synthesizer/recording", Update, self.receive_update, ())
        self.update_pub = rospy.Publisher("synthesizer/program_update", Update, queue_size=10)
        self.avail_ent_pub = rospy.Publisher("synthesizer/available_entity_request", Empty, queue_size=10)
        rospy.Subscriber("synthesizer/available_entity_receive", AvailableEntities, self.receive_available_entities, ())

        # for sending and receiving maps
        self.curr_map = None
        self.map_lock = threading.Lock()
        rospy.Subscriber("/map", OccupancyGrid, self.receive_map_cb, ())
        rospy.Subscriber("/ctrl/map_request", Empty, self.map_request_cb, ())
        self.send_map_pub = rospy.Publisher("/ctrl/map", OccupancyGrid, queue_size=10)

        self.pipeline = Pipeline({"regions": {}, "manuallyAddedObjects": []})
        print("initialized synthesizer node")

        # for running the program
        rospy.Subscriber("UI/exec_program", Empty, self.exec_program)
        self.send_action_pub = rospy.Publisher("synthesizer/send_action", String, queue_size=10)
        rospy.Subscriber("robot/sensor_input", String, self.receive_robot_sensor_input)
        rospy.Subscriber("robot/finished_action", Empty, self.robot_finished_action)
        self.exec = Executor(self.send_robot_action)

        # for debugging
        rospy.Subscriber("load_program", String, self.load_program)

        # state
        self.entities_received = False
        self.entity_request_lock = threading.Lock()

        # begin waiting process for entities
        thread = threading.Thread(target=self.request_available_entities)
        thread.daemon = True        # Daemonize thread
        thread.start()

    def receive_map_cb(self, msg, args):
        print("received map")
        self.map_lock.acquire()
        self.curr_map = msg
        self.map_lock.release()

    def map_request_cb(self, msg, args):
        self.map_lock.acquire()
        self.send_map_pub.publish(self.curr_map)
        self.map_lock.release()

    def request_available_entities(self):
        while True:
            self.entity_request_lock.acquire()
            received = False
            if self.entities_received:
                received = True
            self.entity_request_lock.release()
            msg = Empty()
            self.avail_ent_pub.publish(msg)
            time.sleep(3)
            if received:
                break
        print("Available entities received.")

    def receive_available_entities(self, msg, args):
        print("...receiving entities...")
        self.entity_request_lock.acquire()
        self.entities_received = True
        self.entity_request_lock.release()
        new_avail_ents = {}
        for entity in msg.entities:
            name = entity.name
            categories = entity.categories
            _class = entity.entity_class
            new_avail_ents[name] = {"categories": categories, "class": _class}
        self.pipeline.update_available_entities(new_avail_ents)

    def receive_update(self, msg, args):
        msg_id = msg.msg_id
        update_type = msg.type
        update = json.loads(msg.update)

        # ALWAYS(?) update the world
        raw_world = update["world"]
        world = {"regions": {}, "manuallyAddedObjects": raw_world["manuallyAddedObjects"]}
        for region in raw_world["regions"]:
            name = region["name"]
            world["regions"][name] = {"name": name, "objects": []}

            self.create_world_dict_helper(region["objects"], world["regions"][name]["objects"])
        self.pipeline.reload_world(world)

        # act accordingly based on update type
        if update_type == NO_CHANGE:
            # check backend state against frontend state
            pass
        elif update_type == NEW_RECORDING:
            # the new recording will be the most recent recording
            recording = update["program"]["recordings"][-1]
            _id = recording["_id"]
            nl = recording["text"]["content"]
            trace = recording["sketch"]["user_sequence"]["data"]
            traj = []
            for item in trace:
                argdict = {}
                for arg in item["args"]:
                    argdict[arg["argname"]] = arg["argval"]
                traj.append({"name": item["cmd"],
                             "type": item["_type"],
                             "_id": item["_id"],
                             "args": argdict})
            self.pipeline.add_recording(nl, traj)
            world = self.pipeline.get_world()
            self.send_update(world, msg_id)

            # update the executor
            self.exec.load_program(self.pipeline.get_program())

        elif update_type == DELETE_RECORDING:
            # iterate through the existing recordings, see which one
            #    is deleted in the message
            world = self.pipeline.get_world()
            program = world.get_program()
            recordings = program.get_recordings()
            recordings_to_delete = []
            for recording in recordings:
                _id = recording._id
                # does this id exist in the update?
                exists = False
                for ui_recording in update["program"]["recordings"]:
                    ui_rec_id = ui_recording["_id"]
                    if ui_rec_id == _id:
                        exists = True
                        break
                if not exists:
                    recordings_to_delete.append(recording)
            for recording in recordings_to_delete:
                recordings.remove(recording)
                if recording == program.main_recording:
                    program.main_recording = None
                    del program.waypoints[recording]
            self.send_update(world, msg_id)

    def create_world_dict_helper(self, raw_objects, objects):
        for obj in raw_objects:
            obj_dict = {"name": obj['name'], "objects": []}
            self.create_world_dict_helper(obj["objects"], obj_dict["objects"])
            objects.append(obj_dict)

    def send_update(self, world, msg_id):
        update_json = {}
        update_json["world"] = world.stringify()
        update_json["program"] = world.get_program().stringify()
        update_json_str = json.dumps(update_json)
        update_msg = Update()
        update_msg.msg_id = msg_id
        update_msg.type = RESYNTH
        update_msg.update = update_json_str
        self.update_pub.publish(update_msg)

    def load_program(self, msg):
        path = msg.data
        try:
            nl, traj, world = test_parser.parse(path)
            self.pipeline = Pipeline(world, True)
            self.pipeline.load_program(nl, traj)
            self.exec.load_program(self.pipeline.get_program())
        except Exception:
            print(traceback.format_exc())
            print("Could not load program from {} due to the ".format(path) +
                  "exception above.")

    def exec_program(self, msg):
        self.exec.run_program()

    def send_robot_action(self, action_string):
        msg = String()
        msg.data = action_string
        self.send_action_pub.publish(msg)

        # debug
        #print("SENDING ACTION: {}".format(action_string))

    def receive_robot_sensor_input(self, msg):
        sensor_input = msg.data
        self.exec.add_sensor_input(sensor_input)

    def robot_finished_action(self, msg):
        #print("ACTION FINISHED.")
        self.exec.robot_finished_action()


if __name__ == "__main__":
    ctrl = Controller()
    rospy.spin()
