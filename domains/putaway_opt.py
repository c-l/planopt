from twocan_opt import TwoCanOpt
from interface.hl_param import RP, ObjLoc

class PutAwayOpt(TwoCanOpt):
    def __init__(self, env):
        self.rows = 3
        self.cols = 1

        num_cans = max(int(o.GetName()[3:]) for o in env.GetBodies() if o.GetName().startswith("can"))
        self.movable_objects = {"can%d"%i for i in range(1, num_cans+1)}
        self.param_map = {}
        self.add_body_params(env, self.param_map)
        self.body_params = self.param_map.copy()
        robot = self.param_map['robot']
        self.param_map.update({"robotinitloc":RP("robotinitloc", self.rows, self.cols, is_var=False, value=robot.get_pose(env)),
                               "goal1":ObjLoc("goal1", self.rows, self.cols, is_var=True, is_resampled=True, region=(3.5, 3.5, 3.5, 5.5)),
                               "goal2":ObjLoc("goal2", self.rows, self.cols, is_var=True, is_resampled=True, region=(3.5, 3.5, 3.5, 5.5))})
        for i in range(1, num_cans+1):
            self.param_map["can%dinitloc"%i] = ObjLoc("can%dinitloc"%i, self.rows,self.cols,
                                                      is_var=False, value=self.param_map["can%d"%i].get_pose(env))

        self.name = "twocan_world"
        self.world_state = {self.param_map["can%d"%i]: self.param_map["can%dinitloc"%i] for i in range(1, num_cans+1)}
