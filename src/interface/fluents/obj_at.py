from fluent import Fluent
from opt.constraints import Constraints
import numpy as np

class ObjAt(Fluent):
    def __init__(self, env, hl_action, model, obj, loc, obj_traj, loc_param=None):
        super(ObjAt, self).__init__(env, hl_action, model)
        self.obj = obj
        self.loc = loc.grb_vars
        self.obj_traj = obj_traj.grb_vars

        self.name = "ObjAt"

        # self.in_region = False
        self.loc_param = loc_param
        # if loc_param is not None:
        #     if loc_param.in_region:
        #         self.in_region = True


    # TODO: K currently depends on the robot's degrees of freedom when it shouldn't
    def precondition(self):
        K = self.hl_action.K
        self.constraints.add_eq_cntr(self.obj_traj[:K], self.loc)
        return self.constraints

    def postcondition(self):
        K = self.hl_action.K
        self.constraints.add_eq_cntr(self.obj_traj[-K:], self.loc)

        # adding constraints that location must be in designed region
        if self.loc_param is not None and self.loc_param.in_region:
            ab = self.obj.ComputeAABB()
            dim_x = ab.extents()[0]
            dim_y = ab.extents()[1]

            min_x = self.loc_param.min_x + dim_x
            max_x = self.loc_param.max_x - dim_x
            min_y = self.loc_param.min_y + dim_y
            max_y = self.loc_param.max_y - dim_y
            z = 0
            min_xyz = np.array([[min_x],[min_y],[z]])
            max_xyz = np.array([[max_x],[max_y],[z]])

            K = self.hl_action.K
            self.constraints.add_geq_cntr(self.loc, min_xyz)
            self.constraints.add_leq_cntr(self.loc, max_xyz)
        return self.constraints
