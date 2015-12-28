from fluent import LinEqFluent
from aff_expr import AffExpr
import numpy as np

class ObjAt(LinEqFluent):
    def __init__(self, hl_action, priority, obj, loc, obj_traj):
        self.hl_action = hl_action
        self.priority = priority
        self.obj = obj
        self.loc = loc
        self.obj_traj = obj_traj

        self.name = "ObjAt(" + obj.name + ", " + loc.name + ")"

    def pre(self):
        # initialize obj_traj value
        # import ipdb; ipdb.set_trace()
        # self.obj_traj.value[:,0:1] = self.loc.value

        T = self.obj_traj.cols
        coeff = np.zeros((T, 1), dtype=np.float)
        coeff[0, 0] = 1.0
        self.rhs = AffExpr({self.obj_traj: coeff})
        self.lhs = AffExpr({self.loc: 1.0})

    def post(self):
        T = self.obj_traj.cols
        coeff = np.zeros((T, 1), dtype=np.float)
        coeff[0, 0] = 1.0
        self.rhs = AffExpr({self.obj_traj: coeff})
        self.lhs = AffExpr({self.loc: 1.0})

        # # adding constraints that location must be in designed region
        # if self.loc_param is not None and self.loc_param.in_region:
        #     ab = self.obj.ComputeAABB()
        #     dim_x = ab.extents()[0]
        #     dim_y = ab.extents()[1]
        #
        #     min_x = self.loc_param.min_x + dim_x
        #     max_x = self.loc_param.max_x - dim_x
        #     min_y = self.loc_param.min_y + dim_y
        #     max_y = self.loc_param.max_y - dim_y
        #     z = 0
        #     min_xyz = np.array([[min_x],[min_y],[z]])
        #     max_xyz = np.array([[max_x],[max_y],[z]])
        #
        #     self.constraints.add_geq_cntr(self.loc, min_xyz)
        #     self.constraints.add_leq_cntr(self.loc, max_xyz)
        # return self.constraints
