import numpy as np
from fluent import LinEqFluent
from aff_expr import AffExpr


class RobotAt(LinEqFluent):
    def __init__(self, hl_action, priority, pos, traj):
        self.hl_action = hl_action
        self.priority = priority
        self.pos = pos
        self.traj = traj
        self.name = 'RobotAt(' + pos.name + ')'
        self.do_extension = True
        self.extension_params = [pos, traj]

    def pre(self):
        # import ipdb; ipdb.set_trace()
        # self.traj.value[:,0:1] = self.pos.value

        T = self.traj.num_timesteps()
        # first time step of traj must equal pos
        coeff = np.zeros((T, 1), dtype=np.float)
        coeff[0, 0] = 1.0
        self.rhs = AffExpr({self.traj: coeff})
        self.lhs = AffExpr({self.pos: 1.0})

    def post(self):
        # self.traj.value[:,-1:] = self.pos.value

        T = self.traj.num_timesteps()
        # last time step of traj must equal pos
        coeff = np.zeros((T, 1), dtype=np.float)
        coeff[T-1, 0] = 1.0
        self.rhs = AffExpr({self.traj: coeff})
        self.lhs = AffExpr({self.pos: 1.0})
