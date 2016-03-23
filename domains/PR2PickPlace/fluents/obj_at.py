from interface.fluents.fluent import AndFluent
from interface.fluents.fluent import LinEqFluent
from interface.fluents.aff_expr import AffExpr
import numpy as np

class PR2ObjAt(AndFluent):
    def __init__(self, hl_action, priority, obj, loc, obj_traj):
        self.hl_action = hl_action
        self.priority = priority
        self.obj = obj
        self.loc = loc
        self.obj_traj = obj_traj

        self.name = "ObjAt(" + obj.name + ", " + loc.name + ")"

    def post(self):
        T = self.obj_traj.num_timesteps()
        coeff = np.zeros((T, 1), dtype=np.float)
        coeff[0, 0] = 1.0
        # A_lhs = np.array([[0,0,0,1,1,1]])
        A_lhs = np.hstack((np.zeros((3,3)), np.eye(3)))
        A_rhs = np.zeros((T, 1))
        A_rhs[T-1,0] = 1
        loc_lhs = AffExpr({self.obj_traj: (A_lhs, A_rhs)})
        loc_rhs = AffExpr({self.loc: 1.0})
        loc_fluent = LinEqFluent(self.name + '_loc', self.priority, self.hl_action, loc_lhs, loc_rhs)

        A_lhs = np.hstack((np.eye(3), np.zeros((3,3))))
        A_rhs = np.zeros((T, 1))
        A_rhs[T-1,0] = 1
        standing_lhs = AffExpr({self.obj_traj: (A_lhs, A_rhs)})
        standing_rhs = AffExpr(constant=np.zeros((3,1)))
        standing_fluent = LinEqFluent(self.name + '_standing', self.priority, self.hl_action, standing_lhs, standing_rhs)
        self.fluents = [loc_fluent, standing_fluent]
