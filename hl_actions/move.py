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

        # self.traj_init = np.matrix(np.vstack((np.linspace(start[0],end[0],num=T),\
        #                                       np.linspace(start[1],end[1],num=T),\
        #                                       np.linspace(start[2],end[2],num=T))))
        self.traj_init = np.matrix([np.linspace(i,j,T) for i,j in zip(start, end)])
        self.x = cvx.Variable(K*T,1)
        self.traj = cvx.reshape(self.x, K, T)
        # self.traj = cvx.Variable(K,T)

        self.preconditions = [RobotAt(start, self.traj), \
                            IsMP(self.env, self.traj)]
        # self.postconditions = [RobotAt(end, self.traj)]

        # testing pick
        self.gp = cvx.Variable(K,1)
        self.postconditions = [InManip(self.env.GetKinBody('obstacle'), self.gp, self.traj)]


        # setting trajopt objective
        v = -1*np.ones((KT-K,1))
        d = np.vstack((np.ones((KT-K,1)),np.zeros((K,1))))
        # [:,0] allows numpy to see v and d as one-dimensional so that numpy will create a diagonal matrix with v and d as a diagonal
        P = np.matrix(np.diag(v[:,0],K) + np.diag(d[:,0]) )
        Q = np.transpose(P)*P

        self.f = lambda x: np.zeros((1,1))
        self.objective = cvx.quad_form(self.x, Q)

        self.add_fluents_to_opt_prob()

    def solve_opt_prob(self):
        sqp = SQP()
        sqp.initial_trust_box_size = 0.1
        sqp.min_approx_improve = 1e-2
        sqp.g_use_numerical = False

        # x = cvx.reshape(self.traj, self.K, self.T)
        x0 = np.reshape(self.traj_init, (self.K*self.T,1), order='F')
        x, success = sqp.penalty_sqp(self.x, x0, self.objective, self.constraints, self.f, self.g, self.h)

        env = self.env
        handles = []
        clones = []
        # traj = x.value.reshape((K,T), order='F')
        traj = self.traj.value
        for t in range(self.T):
            xt = traj[:,t]
            if t == self.T-1:
                clones.append(self.create_robot_kinbody( "{0}".format(t), xt, color =[1,0,0]))
            else:
                color_prec = t * 1.0/(self.T-1)
                color = [color_prec, 0, 1 - color_prec]
                clones.append(self.create_robot_kinbody( "{0}".format(t), xt, color=color))
            env.AddKinBody(clones[t])

            transform = np.identity(4)
            transform[0,3] = xt[0]
            transform[1,3] = xt[1]
            rot = matrixFromAxisAngle([0,0,xt[2]])
            transform = np.dot(rot,transform)
            with env:
                clones[t].SetTransform(transform)
        env.UpdatePublishedBodies()
        ipdb.set_trace()

    def create_robot_kinbody(self, name, xt, color=[0,0,1]):
        robot = self.create_cylinder(name, np.eye(4), [0.2,2.01], color=color)
        return robot

        # # create robot KinBody
        # env = self.env
        # box = KinBody.Link.GeometryInfo()
        # box._type = KinBody.Link.GeomType.Box
        # box._vGeomData = [0.2,0.1,1.01]
        # box._bVisible = True
        # box._fTransparency = 0
        # box._vDiffuseColor = [0,0,1]

        # robot = RaveCreateKinBody(env,'')
        # robot.InitFromGeometries([box])
        # robot.SetName(name)

        # return robot

    def create_cylinder(self, body_name, t, dims, color=[0,1,1]):
        infocylinder = KinBody.GeometryInfo()
        infocylinder._type = GeometryType.Cylinder
        infocylinder._vGeomData = dims
        infocylinder._bVisible = True
        infocylinder._vDiffuseColor = color
        infocylinder._fTransparency = 0.8
        # infocylinder._t[2, 3] = dims[1] / 2

        cylinder = RaveCreateKinBody(self.env, '')
        cylinder.InitFromGeometries([infocylinder])
        cylinder.SetName(body_name)
        cylinder.SetTransform(t)

        return cylinder


if __name__ == "__main__":
    move = Move()
    move.solve_opt_prob()
