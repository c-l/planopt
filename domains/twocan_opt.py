from interface.hl_actions import *
from interface.hl_actions.move import Move
from interface.hl_actions.pick import Pick
from interface.hl_actions.place import Place

from utils import *

import sys
sys.path.insert(0, "../")
from envs.world import World
from interface.hl_param import *

class TwoCanOpt(object):
    def __init__(self, env):
        # fake_env, target_loc_values = World().generate_canes_env(num=1)
        # goal = [3.5,3.5,0]
        goal = np.array([[3,4],[3,5],[0,0]])
        # goal = np.array([[3,3,0],[4,5,0]])
        rows = 3
        cols = 1
        robot = Robot(env.GetRobots()[0].GetName())
        can1 = Obj("can1")
        can2 = Obj("can2")
        self.name_to_hl_param_map = {\
                robot.name: robot, \
                "robot_init_loc":RP("robot_init_loc", rows, cols, is_var=False, value=robot.get_pose(env)),\
                "can1_init_loc":ObjLoc("can1_init_loc", rows, cols, is_var=False, value=can1.get_pose(env)),\
                "can2_init_loc":ObjLoc("can2_init_loc", rows, cols, is_var=False, value=can2.get_pose(env)), \
                "goal1":ObjLoc("goal1", rows, cols, is_var=False, value=np.array([[3.5], [4], [0]])),\
                # "goal":ObjLoc("goal", rows, cols, is_var=False, value=goal),\
                # "goal1":ObjLoc("goal1", rows, cols, is_var=True, value=None, region=goal),\
                "goal2":ObjLoc("goal2", rows, cols, is_var=True, value=None, region=goal),\
                "pick_can1":RP("pick_can1", rows, cols),\
                "place_can1": RP("place_can1", rows, cols),\
                "pick_can2":RP("pick_can2", rows, cols),\
                "place_can2": RP("place_can2", rows, cols),\
                "gp1":GP("gp1", rows, cols, is_resampled=True),\
                "gp2":GP("gp2", rows, cols),\
                can1.name:can1,\
                can2.name:can2}
        self.params_to_sample = []
        for name in ['gp1']:
        # for name in ['gp1','goal1']:
        # for name in ['gp1','gp2','goal1', 'goal2']:
        # for name in ['gp2', 'goal2']:
            self.params_to_sample.append(self.name_to_hl_param_map[name])
        self.name = "twocan_world"
        # self.place_objs = []
        # self.place_locs = []
        # self.obj_locs = []
        # self.action_init_hierarchy = [["pick", "place"], ["move"]]
        # self.hl_param_init_hierarchy = ["pick_obj0", "place_obj0", "gp1"]


    def pick(self, lineno, pr, env, robot_str, pos_str, obj_str, loc_str, gp_str):
        d = self.name_to_hl_param_map
        robot = d[robot_str]
        pos = d[pos_str]
        obj = d[obj_str]
        obj_loc = d[loc_str]
        gp = d[gp_str]
        action = Pick(lineno, pr, env, robot, pos, obj, obj_loc, gp)
        return action

    def place(self, lineno, pr, env, robot_str, pos_str, obj_str, loc_str, gp_str):
        d = self.name_to_hl_param_map
        robot = d[robot_str]
        pos = d[pos_str]
        obj = d[obj_str]
        obj_loc = d[loc_str]
        gp = d[gp_str]
        action = Place(lineno, pr, env, robot, pos, obj, obj_loc, gp)
        # action = Place(lineno, pr, env, robot, pos, obj, obj_loc, gp, name="place"+str(lineno), place_obj_params=self.place_objs[:], place_loc_params=self.place_locs[:])
        # self.place_objs.append(obj)
        # self.place_locs.append(obj_loc)
        return action

    def move(self, lineno, pr, env, robot_str, start_str, end_str, obj_str=None, gp_str=None):
        d = self.name_to_hl_param_map
        robot = d[robot_str]
        start = d[start_str]
        end = d[end_str]
        # action = Move(lineno, pr, env, robot, start_param=start, end_param=end, name="move"+str(lineno), place_obj_params=self.place_objs[:], place_loc_params=self.place_locs[:])
        action = Move(lineno, pr, env, robot, start, end)
        return action

    def move_w_obj(self, lineno, pr, env, robot_str, start_str, end_str, obj_str, obj_start_str, obj_end_str, gp_str):
        d = self.name_to_hl_param_map
        robot = d[robot_str]
        start = d[start_str]
        end = d[end_str]
        obj = d[obj_str]
        obj_start = d[obj_start_str]
        obj_end = d[obj_end_str]
        gp = d[gp_str]
        # self.place_objs.append(d['can1'])
        # self.place_locs.append(d['goal1'])
        # action = Move(lineno, pr, env, robot, start_param=start, end_param=end, obj_param=obj, obj_start_param=obj_start, obj_end_param=obj_end, gp_param=gp, name="move"+str(lineno), place_obj_params=self.place_objs[:], place_loc_params=self.place_locs[:])
        action = Move(lineno, pr, env, robot, start, end, obj, gp)
        return action
