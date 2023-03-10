#!/usr/bin/env python3

# Author: David Porfirio <dporfirio@wisc.edu>

"""
world.py: Stores information about the world within which
          the robot operates.
"""

import copy


class World:

    '''
    Singleton Storage Class
    '''
    __instance = None

    @staticmethod
    def get_instance():
        if World.__instance is None:
            print("ERROR: world state not initialized.")
            exit()
        return World.__instance

    def __init__(self, world, program):
        self.load_world_and_program(world, program)
        World.__instance = self

    def load_world(self, world):
        self.world = world
        self.fill_world()

    def load_world_and_program(self, world, program):
        self.world = world
        self.create_manually_added_objects()
        self.fill_world()
        self.program = program

    def get_program(self):
        return self.program

    def fill_world(self):
        '''
        Each object in the world should store a reference to its parent.
        Each object in the world should be directly accessible.
        '''
        regdata_to_scan = copy.copy(self.world["regions"])
        for regstr, regdata in regdata_to_scan.items():
            regdata["parent"] = ""
            self.fill_world_helper(regdata)

    def fill_world_helper(self, obj_data, parent=None):
        if obj_data["name"] not in self.world["regions"]:
            self.world["regions"][obj_data["name"]] = obj_data
            obj_data["parent"] = parent

        if len(obj_data["objects"]) == 0:
            return

        for obj in obj_data["objects"]:
            self.fill_world_helper(obj, parent=obj_data["name"])

    def is_entity_within_region(self, entstr, regstr):
        if regstr not in self.world["regions"]:
            return False
        obj_data = self.world["regions"][regstr]["objects"]
        return self.find_nested_obj(obj_data, entstr)

    def add_entity_to_region(self, entstr, regstr):
        # search for the region in the world
        # then add the entity to the region
        # entstr: the new item to be added to regstr
        # regstr: any currently existing region or object within the world
        # 
        # Note: if duplicates of regstr exist, then the entity will be added
        #   to the first instance of regstr that is found.
        # Note: if regstr is not found, no changes to world occur.
        # Note: this method does not add entstr if it already exists in regstr
        reg_objects = self.add_entity_to_region_helper(regstr)
        if not self.find_nested_obj(reg_objects, entstr):
            reg_objects.append({"name": entstr, "objects": []})

    def add_entity_to_region_helper(self, regstr):
        # Helper method. Returns the object list contained
        #   by regstr.
        to_return = []
        for reg_name, region_data in self.world["regions"].items():
            objs = region_data["objects"]
            if regstr == reg_name:
                to_return = objs
                break
            result = self.add_entity_to_region_helper_helper(regstr, objs)
            if result is not None:
                to_return = result
                break

        return to_return

    def add_entity_to_region_helper_helper(self, regstr, obj_list):
        # Recursive helper method to the helper method
        result = None
        for obj_data in obj_list:
            if obj_data["name"] == regstr:
                result = obj_data
                break
            deeper_result = self.add_entity_to_region_helper_helper(regstr, obj_data["objects"])
            if deeper_result is not None:
                result = deeper_result
                break
        return result

    def find_nested_obj(self, obj_data_list, entstr):
        
        in_inner_obj = False
        for obj_data in obj_data_list:

            # base case
            if obj_data["name"] == entstr:
                return True

            in_inner_obj = in_inner_obj or self.find_nested_obj(obj_data["objects"], entstr)
        return in_inner_obj

    def stringify(self):
        # need to refactor world so it matches what unity needs:
        data = {"regions": []}
        for region_name, region_info in self.world["regions"].items():
            objs = []
            
            self.stringify_objects_helper(region_info["objects"], objs)

            data["regions"].append({"name": region_name, "objects": objs})
        return data

    def stringify_objects_helper(self, objects_out, objects_in):
        for obj in objects_out:
            obj_dict = {"name": obj['name'], "objects": []}
            self.create_world_dict_helper(obj["objects"], obj_dict["objects"])
            objects_in.append(obj_dict)

    def create_world_dict_helper(self, raw_objects, objects):
        for obj in raw_objects:
            obj_dict = {"name": obj['name'], "objects": []}
            self.create_world_dict_helper(obj["objects"], obj_dict["objects"])
            objects.append(obj_dict)

    # debug
    def create_manually_added_objects(self):
        self.world["manuallyAddedObjects"] = []
        for region_name, region in self.world["regions"].items():
            for obj in region["objects"]:
                self.create_manually_added_objects_helper(obj)

    def create_manually_added_objects_helper(self, obj):
        self.world["manuallyAddedObjects"].append(obj["name"])
        for nested_obj in obj["objects"]:
            self.create_manually_added_objects_helper(nested_obj)