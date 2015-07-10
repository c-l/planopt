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

class Pick(HLAction):
    def __init__(self, env, pos, obj, plan_gp):
        super(Pick, self).__init__()
        self.handles = []
        self.env = env
        self.pos = pos
        self.obj = obj
        self.plan_gp = plan_gp

        self.T = 1
        self.K = 3
        T = self.T
        K = self.K

        self.traj_init = np.zeros((3,1))
        self.traj = cvx.Variable(K*T,1)
        self.gp = cvx.Variable(K,1)
        self.objective = 0


        self.preconditions = [RobotAt(self, pos, self.traj)]
        self.postconditions = [InManip(self, self.obj, self.gp, self.traj)]
        self.add_fluents_to_opt_prob()

    def solve_opt_prob(self):
        sqp = SQP()
        # sqp.initial_trust_box_size = 0.1
        sqp.initial_trust_box_size = 1
        sqp.min_trust_box_size=1e-4
        sqp.initial_penalty_coeff = 0.1
        # sqp.min_approx_improve = 1e-2
        # sqp.g_use_numerical = False

        # x = cvx.reshape(self.traj, self.K, self.T)
        # x0 = np.reshape(self.traj_init, (self.K*self.T,1), order='F')
        x, success = sqp.penalty_sqp(self.traj, self.traj_init, self.objective, self.constraints, self.f, self.g, self.h)

    def plot_kinbodies(self):
        # traj = self.traj.value.reshape((self.K,self.T), order='F')
        # traj = self.traj.value
        pick_robot = self.create_robot_kinbody("pick", color=[0,0,0], transparency=0.5)

        xt = self.traj.value[:,0]
        transform = np.identity(4)
        transform[0,3] = xt[0]
        transform[1,3] = xt[1]
        rot = matrixFromAxisAngle([0,0,xt[2]])
        transform = np.dot(rot,transform)
        pick_robot.SetTransform(transform)

        return [pick_robot]


