from interface.hl_actions import *
from interface.hl_actions.move import Move
from interface.hl_actions.pick import Pick
from interface.hl_actions.place import Place

from utils import *

import sys
sys.path.insert(0, "../")
from envs.world import World
from interface.hl_param import *
from IPython import embed as shell

class TwoCanOpt(object):
    def __init__(self, env):
        self.rows = 3
        self.cols = 1

        num_cans = max(int(o.GetName()[3:]) for o in env.GetBodies() if o.GetName().startswith("can"))
        self.movable_objects = {"can%d"%i for i in range(1, num_cans+1)}
        self.param_map = {}
        self.add_body_params(env, self.param_map)
        self.body_params = self.param_map.copy()
        robot = self.param_map['robot']
        self.param_map.update({"robotinitloc":RP("robotinitloc", (self.rows, self.cols), is_var=False, value=robot.get_pose(env)),
                               "goal1":ObjLoc("goal1", (self.rows, self.cols), is_var=False, value=np.array([[3.5], [5.5], [0]])),
                               "goal2":ObjLoc("goal2", (self.rows, self.cols), is_var=False, value=np.array([[3.5], [3.5], [0]]))})
        for i in range(1, num_cans+1):
            self.param_map["can%dinitloc"%i] = ObjLoc("can%dinitloc"%i, (self.rows, self.cols),
                                                      is_var=False, value=self.param_map["can%d"%i].get_pose(env))

        self.name = "twocan_world"
        self.world_state = {self.param_map["can%d"%i]: self.param_map["can%dinitloc"%i] for i in range(1, num_cans+1)}

    def get_sampled_params(self):
        params_to_sample = []
        for param in self.param_map.values():
            if param.is_resampled:
                params_to_sample.append(param)
        return params_to_sample

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

    def _get_param(self, param_name):
        if param_name not in self.param_map:
            if param_name.startswith("pdp"):
                obj_loc = self._get_param(param_name.split("_")[2])
                self.param_map[param_name] = RP(param_name, (self.rows, self.cols), obj_loc, is_resampled=True)
            elif param_name.startswith("gp"):
                obj = self.param_map[param_name.split("_")[1]]
                obj_loc = self.world_state[obj]
                self.param_map[param_name] = RP(param_name, (self.rows, self.cols), obj_loc, is_resampled=True)
            elif param_name.startswith("grasp"):
                self.param_map[param_name] = Grasp(param_name, (self.rows, self.cols), is_resampled=True)
            elif param_name.startswith("none"):
                return None
            elif "temploc" in param_name:
                # sampled ObjLoc (object location)
                self.param_map[param_name] = ObjLoc(param_name, (self.rows, self.cols), is_var=True,
                                                    is_resampled=True, region=(0.5, 6.5, -1.5, 2.5))
            else:
                import ipdb; ipdb.set_trace()
                print ('not a valid parameter name')

        return self.param_map[param_name]

    def pick(self, lineno, pr, env, robot_str, obj_str, loc_str, pos_str, gp_str):
        d = self.param_map
        robot = self._get_param(robot_str)
        pos = self._get_param(pos_str)
        obj = self._get_param(obj_str)
        obj_loc = self._get_param(loc_str)
        gp = self._get_param(gp_str)
        action = Pick(lineno, pr, env, robot, pos, obj, obj_loc, gp)
        return action

    def place(self, lineno, pr, env, robot_str, obj_str, loc_str, pos_str, gp_str):
        d = self.param_map
        robot = self._get_param(robot_str)
        pos = self._get_param(pos_str)
        obj = self._get_param(obj_str)
        obj_loc = self._get_param(loc_str)
        gp = self._get_param(gp_str)
        self.world_state[obj] = obj_loc
        action = Place(lineno, pr, env, robot, pos, obj, obj_loc, gp)
        # action = Place(lineno, pr, env, robot, pos, obj, obj_loc, gp, name="place"+str(lineno), place_obj_params=self.place_objs[:], place_loc_params=self.place_locs[:])
        # self.place_objs.append(obj)
        # self.place_locs.append(obj_loc)
        return action

    def move(self, lineno, pr, env, robot_str, start_str, end_str, obj_str=None, gp_str=None):
        d = self.param_map
        robot = self._get_param(robot_str)
        start = self._get_param(start_str)
        end = self._get_param(end_str)
        action = Move(lineno, pr, env, self.world_state.copy(), robot, start, end)
        return action

    def move_w_obj(self, lineno, pr, env, robot_str, start_str, end_str, obj_str, gp_str):
        d = self.param_map
        robot = self._get_param(robot_str)
        start = self._get_param(start_str)
        end = self._get_param(end_str)
        obj = self._get_param(obj_str)
        gp = self._get_param(gp_str)
        action = Move(lineno, pr, env, self.world_state.copy(), robot, start, end, obj, gp)
        return action
