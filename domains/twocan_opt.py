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
        goal = np.array([[3,4],[3,5],[0,0]])
        # goal = np.array([[3,3,0],[4,5,0]])
        rows = 3
        cols = 1
        self.movable_objects = {"can1", "can2"}
        self.param_map = {}
        self.add_body_params(env, self.param_map)
        self.body_params = self.param_map.copy()
        robot = self.param_map['robot']
        can1 = self.param_map['can1']
        can2 = self.param_map['can2']
        self.param_map.update({\
                "robot_init_loc":RP("robot_init_loc", rows, cols, is_var=False, value=robot.get_pose(env)),\
                "can1_init_loc":ObjLoc("can1_init_loc", rows, cols, is_var=False, value=can1.get_pose(env)),\
                "can2_init_loc":ObjLoc("can2_init_loc", rows, cols, is_var=False, value=can2.get_pose(env)), \
                "goal1":ObjLoc("goal1", rows, cols, is_var=False, value=np.array([[3.5], [3.5], [0]])),\
                "goal2":ObjLoc("goal1", rows, cols, is_var=False, value=np.array([[3.5], [4.5], [0]])),\

                # "goal2":ObjLoc("goal2", rows, cols, is_var=True, value=None, region=goal),\
                "pick_can1":RP("pick_can1", rows, cols),\
                "place_can1": RP("place_can1", rows, cols),\
                "pick_can2":RP("pick_can2", rows, cols),\
                "place_can2": RP("place_can2", rows, cols),\
                "gp1":GP("gp1", rows, cols, is_resampled=True),\
                "gp2":GP("gp2", rows, cols, is_resampled=True)})

        self.params_to_sample = []
        for name in ['gp1','gp2']:
            self.params_to_sample.append(self.param_map[name])
        self.name = "twocan_world"
        self.world_state = {self.param_map["can1"]: self.param_map["can1_init_loc"], \
                            self.param_map["can2"]: self.param_map["can2_init_loc"]}

    def get_all_but_params(self, params_to_delete):
        params = self.body_params.copy()
        for param in params_to_delete:
            del params[param.name]
        return params.values()

    def add_body_params(self, env, param_map):
        for body in env.GetBodies():
            name = body.GetName()
            if not body.IsRobot():
                param_map[name] = Obj(name)
            else:
                param_map[name] = Robot(name)

    def pick(self, lineno, pr, env, robot_str, pos_str, obj_str, loc_str, gp_str):
        d = self.param_map
        robot = d[robot_str]
        pos = d[pos_str]
        obj = d[obj_str]
        obj_loc = d[loc_str]
        gp = d[gp_str]
        action = Pick(lineno, pr, env, robot, pos, obj, obj_loc, gp)
        return action

    def place(self, lineno, pr, env, robot_str, pos_str, obj_str, loc_str, gp_str):
        d = self.param_map
        robot = d[robot_str]
        pos = d[pos_str]
        obj = d[obj_str]
        obj_loc = d[loc_str]
        gp = d[gp_str]
        self.world_state[obj] = obj_loc
        action = Place(lineno, pr, env, robot, pos, obj, obj_loc, gp)
        # action = Place(lineno, pr, env, robot, pos, obj, obj_loc, gp, name="place"+str(lineno), place_obj_params=self.place_objs[:], place_loc_params=self.place_locs[:])
        # self.place_objs.append(obj)
        # self.place_locs.append(obj_loc)
        return action

    def move(self, lineno, pr, env, robot_str, start_str, end_str, obj_str=None, gp_str=None):
        d = self.param_map
        robot = d[robot_str]
        start = d[start_str]
        end = d[end_str]
        # action = Move(lineno, pr, env, robot, start_param=start, end_param=end, name="move"+str(lineno), place_obj_params=self.place_objs[:], place_loc_params=self.place_locs[:])
        action = Move(lineno, pr, env, self.world_state, robot, start, end)
        return action

    def move_w_obj(self, lineno, pr, env, robot_str, start_str, end_str, obj_str, obj_start_str, obj_end_str, gp_str):
        d = self.param_map
        robot = d[robot_str]
        start = d[start_str]
        end = d[end_str]
        obj = d[obj_str]
        obj_start = d[obj_start_str]
        obj_end = d[obj_end_str]
        gp = d[gp_str]
        world_state = self.world_state.copy()
        # self.place_objs.append(d['can1'])
        # self.place_locs.append(d['goal1'])
        # action = Move(lineno, pr, env, robot, start_param=start, end_param=end, obj_param=obj, obj_start_param=obj_start, obj_end_param=obj_end, gp_param=gp, name="move"+str(lineno), place_obj_params=self.place_objs[:], place_loc_params=self.place_locs[:])
        action = Move(lineno, pr, env, self.world_state, robot, start, end, obj, gp)
        return action
