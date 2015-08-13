from interface.hl_actions import *
from interface.hl_actions.move import Move
from interface.hl_actions.pick import Pick
from interface.hl_actions.place import Place

from interface.hl_param import *
from utils import *

import sys
sys.path.insert(0, "../")
from envs.world import World

class TwoBoxOpt(object):
    def __init__(self, env):
        # fake_env, target_loc_values = World().generate_boxes_env(num=1)
        goal = [3.5,3.5,0]
        rows = 3
        cols = 1
        self.robot = env.GetRobots()[0]
        self.hl_params = {\
                "robot_init_loc":RP("robot_init_loc", rows, cols, is_var=False, value=mat_to_base_pose(self.robot.GetTransform())),\
                "box1_init_loc":ObjLoc("box1_init_loc", rows, cols, is_var=False, value=mat_to_base_pose(env.GetKinBody("box1").GetTransform())),\
                "box2_init_loc":ObjLoc("box2_init_loc", rows, cols, is_var=False, value=mat_to_base_pose(env.GetKinBody("box1").GetTransform())),\
                "goal":ObjLoc("goal", rows, cols, is_var=False, value=goal),\
                "pick_box1":RP("pick_box1", rows, cols),\
                "place_box1": RP("place_box1", rows, cols),\
                "pick_box2":RP("pick_box2", rows, cols),\
                "place_box2": RP("place_box2", rows, cols),\
                "gp1":GP("gp1", rows, cols),\
                "gp2":GP("gp2", rows, cols),\
                "box1":Movable("box1"),\
                "box2":Movable("box1")}
        self.params_to_sample = []
        for name in ['gp1']:
            self.params_to_sample.append(self.hl_params[name])
        self.name = "twobox_world"
        # self.action_init_hierarchy = [["pick", "place"], ["move"]]
        # self.hl_param_init_hierarchy = ["pick_obj0", "place_obj0", "gp1"]
        

    def pick(self, lineno, pr, env, robot, pos_str, obj_str, loc_str, gp_str):
        d = self.hl_params
        pos = d[pos_str]
        obj = d[obj_str]
        obj_loc = d[loc_str]
        gp = d[gp_str]
        action = Pick(lineno, pr, env, robot, pos, obj, obj_loc, gp, name="pick"+str(lineno))
        return action
    
    def place(self, lineno, pr, env, robot, pos_str, obj_str, loc_str, gp_str):
        d = self.hl_params
        pos = d[pos_str]
        obj = d[obj_str]
        obj_loc = d[loc_str]
        gp = d[gp_str]
        action = Place(lineno, pr, env, robot, pos, obj, obj_loc, gp, name="place"+str(lineno))
        return action

    def move(self, lineno, pr, env, robot, start_str, end_str, obj_str=None, gp_str=None):
        d = self.hl_params
        start = d[start_str]
        end = d[end_str]
        action = Move(lineno, pr, env, robot, start_param=start, end_param=end, name="move"+str(lineno))
        return action

    def move_w_obj(self, lineno, pr, env, robot, start_str, end_str, obj_str, gp_str):
        d = self.hl_params
        start = d[start_str]
        end = d[end_str]
        obj = d[obj_str]
        gp = d[gp_str]
        action = Move(lineno, pr, env, robot, start_param=start, end_param=end, obj_param=obj, gp_param=gp, name="move"+str(lineno))
        return action




