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
from fluents.obj_at import ObjAt

from utils import *

class Pick(HLAction):
    def __init__(self, env, robot, pos, obj, loc, gp):
        super(Pick, self).__init__()
        self.handles = []
        self.env = env
        self.robot = robot
        self.pos = pos
        self.obj = obj
        self.loc = loc
        self.gp = gp

        self.T = 1
        self.K = 3
        T = self.T
        K = self.K

        self.traj_init = np.zeros((3,1))
        self.traj = cvx.Variable(K*T,1)
        self.traj.value = self.traj_init

        self.obj_init = np.zeros((3,1))
        self.obj_traj = cvx.Variable(K*T,1)
        self.obj_traj.value = self.obj_init

        self.objective = 0


        self.preconditions = [RobotAt(self, pos, self.traj)]
        self.preconditions += [ObjAt(self, obj, loc, self.obj_traj)] 
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
        x, success = sqp.penalty_sqp(self.traj, self.traj.value, self.objective, self.constraints, self.f, self.g, self.h)

    def plot_kinbodies(self):
        robot = self.robot
        transparency = 0

        # Need to remove obj and robot, sleep and then add them back in to clone them....
        self.env.Remove(self.obj)
        self.env.Remove(robot)
        time.sleep(1)
        self.env.Add(self.obj)
        self.env.Add(robot)

        pick_robot = RaveCreateRobot(self.env,robot.GetXMLId())
        pick_robot.Clone(robot,0)
        pick_robot.SetName("pick_" + robot.GetName())

        for link in pick_robot.GetLinks():
            for geom in link.GetGeometries():
                geom.SetTransparency(transparency)
                geom.SetDiffuseColor([1,1,0])

        newobj = RaveCreateKinBody(self.env, self.obj.GetXMLId())
        newobj.Clone(self.obj, 0)
        newobj.SetName("pick_" + self.obj.GetName())
        
        for link in newobj.GetLinks():
            for geom in link.GetGeometries():
                geom.SetTransparency(transparency)
                geom.SetDiffuseColor([1,0,1])
        ot = self.obj_traj.value[:,0]
        newobj.SetTransform(base_pose_to_mat(ot))

        xt = self.traj.value[:,0]
        pick_robot.SetTransform(base_pose_to_mat(xt))

        return [pick_robot, newobj]


