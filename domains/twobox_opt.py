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
        # goal = [3.5,3.5,0]
        goal = np.array([[3,4],[3,5],[0,0]])
        # goal = np.array([[3,3,0],[4,5,0]])
        rows = 3
        cols = 1
        self.robot = env.GetRobots()[0]
        # env.GetKinBody('box1').SetTransform(base_pose_to_mat(np.array([[3.5],[4.5],[0]])))
        self.hl_params = {\
                "robot_init_loc":RP("robot_init_loc", rows, cols, is_var=False, value=mat_to_base_pose(self.robot.GetTransform())),\
                "box1_init_loc":ObjLoc("box1_init_loc", rows, cols, is_var=False, value=mat_to_base_pose(env.GetKinBody("box1").GetTransform())),\
                "box2_init_loc":ObjLoc("box2_init_loc", rows, cols, is_var=False, value=mat_to_base_pose(env.GetKinBody("box2").GetTransform())),\
                # "goal1":ObjLoc("goal1", rows, cols, is_var=True, value=mat_to_base_pose(env.GetKinBody("box1").GetTransform())),\
                # "goal":ObjLoc("goal", rows, cols, is_var=False, value=goal),\
                "goal1":ObjLoc("goal1", rows, cols, is_var=True, value=None, region=goal),\
                "goal2":ObjLoc("goal2", rows, cols, is_var=True, value=None, region=goal),\
                "pick_box1":RP("pick_box1", rows, cols),\
                "place_box1": RP("place_box1", rows, cols),\
                "pick_box2":RP("pick_box2", rows, cols),\
                "place_box2": RP("place_box2", rows, cols),\
                "gp1":GP("gp1", rows, cols),\
                "gp2":GP("gp2", rows, cols),\
                "box1":Movable("box1"),\
                "box2":Movable("box2")}
        self.params_to_sample = []
        # for name in ['gp1','goal']:
        for name in ['gp1','gp2','goal1', 'goal2']:
        # for name in ['gp2', 'goal2']:
            self.params_to_sample.append(self.hl_params[name])
        self.name = "twobox_world"
        self.place_objs = []
        self.place_locs = []
        # self.obj_locs = []
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
        # action = Place(lineno, pr, env, robot, pos, obj, obj_loc, gp, name="place"+str(lineno))
        action = Place(lineno, pr, env, robot, pos, obj, obj_loc, gp, name="place"+str(lineno), place_obj_params=self.place_objs[:], place_loc_params=self.place_locs[:])
        self.place_objs.append(obj)
        self.place_locs.append(obj_loc)
        return action

    def move(self, lineno, pr, env, robot, start_str, end_str, obj_str=None, gp_str=None):
        d = self.hl_params
        start = d[start_str]
        end = d[end_str]
        action = Move(lineno, pr, env, robot, start_param=start, end_param=end, name="move"+str(lineno), place_obj_params=self.place_objs[:], place_loc_params=self.place_locs[:])
        return action

    def move_w_obj(self, lineno, pr, env, robot, start_str, end_str, obj_str, obj_start_str, obj_end_str, gp_str):
        d = self.hl_params
        start = d[start_str]
        end = d[end_str]
        obj = d[obj_str]
        obj_start = d[obj_start_str]
        obj_end = d[obj_end_str]
        gp = d[gp_str]
        # self.place_objs.append(d['box1'])
        # self.place_locs.append(d['goal1'])
        action = Move(lineno, pr, env, robot, start_param=start, end_param=end, obj_param=obj, obj_start_param=obj_start, obj_end_param=obj_end, gp_param=gp, name="move"+str(lineno), place_obj_params=self.place_objs[:], place_loc_params=self.place_locs[:])
        return action




