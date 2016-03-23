from fluent import AndFluent, LinEqFluent, LinLEFluent
from aff_expr import AffExpr
import numpy as np

class ObjAt(AndFluent):
    def __init__(self, hl_action, priority, obj, loc, obj_traj):
        self.hl_action = hl_action
        self.priority = priority
        self.obj = obj
        self.loc = loc
        self.obj_traj = obj_traj
        self.do_extension = True
        self.extension_params = [loc, obj_traj]

        self.name = "ObjAt(" + obj.name + ", " + loc.name + ")"

    def pre(self):
        T = self.obj_traj.num_timesteps()
        assert T == 1 # TODO: general case
        coeff = np.zeros((T, 1), dtype=np.float)
        coeff[0, 0] = 1.0
        rhs = AffExpr({self.obj_traj: coeff})
        lhs = AffExpr({self.loc: 1.0})
        traj_at_loc = LinEqFluent("traj_at_loc_" + self.name, self.priority, self.hl_action, lhs, rhs)
        self.fluents = [traj_at_loc]
        if self.loc.region is not None:
            A_lhs = np.vstack((np.eye(3), -np.eye(3)))
            lhs = AffExpr({self.loc: (A_lhs, 1.0)})
            ranges = np.array([[self.loc.max_x], [self.loc.max_y], [0],
                               [-self.loc.min_x], [-self.loc.min_y], [0]])
            rhs = AffExpr(constant=ranges)
            obj_in_region = LinLEFluent("obj_in_region_" + self.name, self.priority, self.hl_action, lhs, rhs)
            self.fluents.append(obj_in_region)

    def post(self):
        self.pre()
