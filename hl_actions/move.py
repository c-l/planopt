import numpy as np
import cvxpy as cvx
from sqp_cvx import SQP
import time
import ipdb

from openravepy import *
from hl_action import HLAction

from fluents.is_mp import IsMP
from fluents.in_manip import InManip
from fluents.robot_at import RobotAt

from utils import *

class Move(HLAction):
    def __init__(self, hl_plan, env, robot, start_param, end_param, obj=None, gp_param=None):
        super(Move, self).__init__(hl_plan, env, robot)
        self.start, self.hl_start = start_param.new_hla_var(self)
        self.end, self.hl_end = end_param.new_hla_var(self)
        if obj is None:
            assert gp_param is None
            self.obj = obj
            self.gp = gp
        else:
            self.obj = obj
            self.gp, self.hl_gp = gp_param.new_hla_var(self)
        self.name = "move"

        self.T = 40
        self.K = 3
        T = self.T
        K = self.K
        KT = K*T

        self.traj_init = np.matrix([np.linspace(i,j,T) for i,j in zip(np.array(self.start.value), np.array(self.end.value))])
        self.traj = cvx.Variable(K*T,1)
        self.traj.value = np.reshape(self.traj_init, (self.K*self.T,1), order='F')

        if obj is not None:
            self.obj_traj = cvx.Variable(K*T,1)
            self.obj_init = np.matrix([np.linspace(i,j,T) for i,j in zip(np.array(self.gp.value + self.start.value), np.array(self.gp.value + self.end.value))])
            self.obj_traj.value = np.reshape(self.obj_init, (self.K*self.T,1), order='F')
        else:
            self.obj_traj = None

        self.preconditions = [RobotAt(self, self.start, self.traj)] 
        self.create_robot_clones()
        self.preconditions += [IsMP(self.env, self, robot, self.traj, self.obj, self.obj_traj)]

        if obj is not None:
            self.preconditions += [InManip(self, self.obj, self.gp, self.traj, self.obj_traj)]
        self.postconditions = [RobotAt(self, self.end, self.traj)]

        # setting trajopt objective
        v = -1*np.ones((KT-K,1))
        d = np.vstack((np.ones((KT-K,1)),np.zeros((K,1))))
        # [:,0] allows numpy to see v and d as one-dimensional so that numpy will create a diagonal matrix with v and d as a diagonal
        P = np.matrix(np.diag(v[:,0],K) + np.diag(d[:,0]) )
        Q = np.transpose(P)*P

        self.objective += cvx.quad_form(self.traj, Q)

        self.add_fluents_to_opt_prob()

    def solve_opt_prob(self):
        sqp = SQP()
        sqp.initial_trust_box_size = 0.1
        # sqp.initial_trust_box_size = 1
        sqp.min_trust_box_size=1e-2
        sqp.initial_penalty_coeff = 0.1
        # sqp.initial_penalty_coeff = 0.01
        sqp.min_approx_improve = 1e-2

        # sqp.g_use_numerical = False

        x, success = sqp.penalty_sqp(self.traj, self.traj.value, self.objective, self.constraints, self.f, self.g, self.h)

    def plot(self, handles=[]):
        self.handles = []
        # self.handles += super(Move, self).plot(handles)
        super(Move, self).plot()
        self.handles += handles
        self.handles += self.plot_traj_line(self.traj, colors=(0,0,0.5))

        if self.obj is not None:
            self.handles += self.plot_traj_line(self.obj_traj, colors=(0,0.5,0))

        if not np.allclose(self.start.value, self.hl_start.value):
            start = np.array(self.start.value)
            start[2] = 1
            hl_start = np.array(self.hl_start.value)
            hl_start[2] = 1
            self.handles += [self.hl_plan.env.drawarrow(p1=start, p2=hl_start, linewidth=0.01, color=(1,0,0))]

        if not np.allclose(self.end.value, self.hl_end.value):
            end = np.array(self.end.value)
            end[2] = 1
            hl_end = np.array(self.hl_end.value)
            hl_end[2] = 1
            self.handles += [self.hl_plan.env.drawarrow(p1=end, p2=hl_end, linewidth=0.01, color=(1,0,0))]


if __name__ == "__main__":
    move = Move()
    move.solve_opt_prob()
