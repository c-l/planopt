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

class Move(HLAction):
    def __init__(self, env, start, end):
        super(Move, self).__init__()

        self.handles = []
        self.env = env
        self.start = start
        self.end = end

        self.T = 40
        self.K = 3
        T = self.T
        K = self.K
        KT = K*T

        # self.traj_init = np.matrix([np.linspace(i,j,T) for i,j in zip(start, end)])
        # self.traj_init = np.matrix([np.linspace(i,j,T) for i,j in zip(start, end.value)])
        self.traj_init = np.matrix([np.linspace(i,j,T) for i,j in zip(start, start)])
        self.traj = cvx.Variable(K*T,1)

        # self.end = cvx.Variable(K,1)

        self.preconditions = [RobotAt(self, start, self.traj), \
                            IsMP(self.env, self, self.traj)]
        self.postconditions = [RobotAt(self, self.end, self.traj)]

        # testing pick
        # self.gp = cvx.Variable(K,1)
        # self.postconditions = [InManip(self, self.env.GetKinBody('obstacle'), self.gp, self.traj)]


        # setting trajopt objective
        v = -1*np.ones((KT-K,1))
        d = np.vstack((np.ones((KT-K,1)),np.zeros((K,1))))
        # [:,0] allows numpy to see v and d as one-dimensional so that numpy will create a diagonal matrix with v and d as a diagonal
        P = np.matrix(np.diag(v[:,0],K) + np.diag(d[:,0]) )
        Q = np.transpose(P)*P

        self.objective = cvx.quad_form(self.traj, Q)

        self.add_fluents_to_opt_prob()

    def solve_opt_prob(self):
        sqp = SQP()
        # sqp.initial_trust_box_size = 0.1
        sqp.initial_trust_box_size = 1
        # sqp.min_trust_box_size=1e-4
        # sqp.initial_penalty_coeff = 0.1
        # sqp.min_approx_improve = 1e-2
        sqp.g_use_numerical = False

        # x = cvx.reshape(self.traj, self.K, self.T)
        x0 = np.reshape(self.traj_init, (self.K*self.T,1), order='F')
        x, success = sqp.penalty_sqp(self.traj, x0, self.objective, self.constraints, self.f, self.g, self.h)

    def plot_kinbodies(self):
        clones = []
        traj = self.traj.value.reshape((self.K,self.T), order='F')
        # traj = self.traj.value
        for t in range(self.T):
            xt = traj[:,t]
            if t == self.T-1:
                clones.append(self.create_robot_kinbody( "{0}".format(t), color =[1,0,0]))
            else:
                color_prec = t * 1.0/(self.T-1)
                color = [color_prec, 0, 1 - color_prec]
                clones.append(self.create_robot_kinbody( "{0}".format(t), color=color))
            # self.env.AddKinBody(clones[t])

            transform = np.identity(4)
            transform[0,3] = xt[0]
            transform[1,3] = xt[1]
            rot = matrixFromAxisAngle([0,0,xt[2]])
            transform = np.dot(rot,transform)
            with self.env:
                clones[t].SetTransform(transform)

        return clones

if __name__ == "__main__":
    move = Move()
    move.solve_opt_prob()
