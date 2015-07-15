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
    def __init__(self, env, robot, start, end, obj=None, gp=None):
        super(Move, self).__init__()

        self.handles = []
        self.env = env
        self.robot = robot
        self.start = start
        self.end = end
        self.obj = obj
        self.gp = gp

        self.T = 40
        self.K = 3
        T = self.T
        K = self.K
        KT = K*T

        # self.traj_init = np.matrix([np.linspace(i,j,T) for i,j in zip(start, end)])
        # self.traj_init = np.matrix([np.linspace(i,j,T) for i,j in zip(start, end.value)])
        self.traj_init = np.matrix([np.linspace(i,j,T) for i,j in zip(np.array(start.value), np.array(end.value))])
        self.traj = cvx.Variable(K*T,1)
        self.traj.value = np.reshape(self.traj_init, (self.K*self.T,1), order='F')

        if obj is not None:
            self.obj_traj = cvx.Variable(K*T,1)
            self.obj_init = np.matrix([np.linspace(i,j,T) for i,j in zip(np.array(self.gp.value + start.value), np.array(self.gp.value + end.value))])
            self.obj_traj.value = np.reshape(self.obj_init, (self.K*self.T,1), order='F')
        else:
            self.obj_traj = None

        self.preconditions = [RobotAt(self, start, self.traj)] 
        self.preconditions += [IsMP(self.env, self, self.traj, self.obj, self.obj_traj)]

        if obj is not None:
            self.preconditions += [InManip(self, self.obj, self.gp, self.traj, self.obj_traj)]
        self.postconditions = [RobotAt(self, self.end, self.traj)]

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
        sqp.initial_trust_box_size = 0.1
        # sqp.initial_trust_box_size = 1
        sqp.min_trust_box_size=1e-2
        # sqp.initial_penalty_coeff = 0.1
        sqp.initial_penalty_coeff = 0.01
        sqp.min_approx_improve = 1e-2

        sqp.g_use_numerical = False

        x, success = sqp.penalty_sqp(self.traj, self.traj.value, self.objective, self.constraints, self.f, self.g, self.h)

    def plot_kinbodies(self):
        clones = []
        traj = self.traj.value.reshape((self.K,self.T), order='F')
        if self.obj is not None:
            obj_traj = self.obj_traj.value.reshape((self.K,self.T), order='F')
        transparency = 0.8
        robot = self.env.GetKinBody('robot')

        # Need to remove obj and robot, sleep and then add them back in to clone them....
        if self.obj is not None:
            self.env.Remove(self.obj)
        self.env.Remove(robot)
        time.sleep(1.5)
        if self.obj is not None:
            self.env.Add(self.obj)
        self.env.Add(robot)

        for t in range(self.T):
            xt = traj[:,t]
            newrobot = RaveCreateRobot(self.env,robot.GetXMLId())
            newrobot.Clone(robot,0)
            newrobot.SetName("move_" + robot.GetName() + "_" + str(t))
            # newrobot.SetName(str(t))

            for link in newrobot.GetLinks():
                for geom in link.GetGeometries():
                    geom.SetTransparency(transparency)
                    geom.SetDiffuseColor([0,0,1])

            # for obj in grabbed_objs:
            if self.obj is not None:
                ot = obj_traj[:,t]
                if self.obj is not None:
                    newobj = RaveCreateKinBody(self.env, self.obj.GetXMLId())
                    newobj.Clone(self.obj, 0)
                    newobj.SetName("move_" + self.obj.GetName() + str(t))
                    
                    for link in newobj.GetLinks():
                        for geom in link.GetGeometries():
                            geom.SetTransparency(transparency)
                            geom.SetDiffuseColor([1,0,0])
                    newobj.SetTransform(base_pose_to_mat(ot))
                    clones.append(newobj)
            newrobot.SetTransform(base_pose_to_mat(xt))
            clones.append(newrobot)

        return clones

if __name__ == "__main__":
    move = Move()
    move.solve_opt_prob()
