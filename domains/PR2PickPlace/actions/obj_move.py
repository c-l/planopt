
import numpy as np
from interface.fluents.for_all_not_obstructs import ForAllNotObstructs
from interface.fluents.robot_at import RobotAt
from interface.fluents.is_mp import IsMP
from interface.hl_param import Traj
from opt.function import QuadFn
from action import PR2HLAction

import sys
DOMAIN_PATH = "../domains/PR2PickPlace/"
sys.path.insert(0, DOMAIN_PATH)

# from fluents.in_manip import PR2InManip
from fluents.obj_in_manip import ObjInManip
from fluents.is_mp import PR2IsMP

class PR2ObjMove(PR2HLAction):
    def __init__(self, lineno, hl_plan, env, robot, start, end, traj, obj, gp):
        super(PR2ObjMove, self).__init__(lineno, hl_plan, env, robot)

        self.hl_plan = hl_plan
        self.start = start
        self.end = end
        self.obj = obj
        self.gp = gp

        # TODO: set this using optimization domain
        # self.T = 40
        # self.T = 10
        self.T = 2
        self.K = 7
        self.obj_K = 6
        T = self.T
        K = self.K
        KT = self.K*self.T

        self.name = "move" + str(lineno)
        # traj_value = np.tile(start.value, (1, self.T))
        self.traj = Traj(self, self.name + "_traj", self.K, self.T, is_var=False, value=traj)

        # Object trajectory
        obj_traj = np.array([[.707], [0.], [0.707], [0.], [0.], [0.]])
        obj_traj = np.tile(obj_traj, (1,T))
        self.obj_traj = Traj(self, self.name + "_obj_traj", self.obj_K, self.T, is_var=True, value=obj_traj)

        self.params = [start, end, self.traj, self.obj_traj]
        self.preconditions = [RobotAt(self, 0, start, self.traj)]
        # self.preconditions += [PR2InManip(env, self, robot, 0, self.obj, self.gp, self.traj, self.obj_traj)]
        self.preconditions += [ObjInManip(env, self, robot, 0, self.obj, self.gp, self.traj, self.obj_traj)]
        self.preconditions += [PR2IsMP(env, self, robot, 0, start, end, self.traj)]

        # self.preconditions += [NotObstructsPR2(env, self, robot, 1, self.traj, obj)]

        self.postconditions = []
        # self.postconditions += [RobotAt(self, 0, self.end, self.traj)]

        # setting trajopt objective
        v = -1 * np.ones((KT - K, 1))
        d = np.vstack((np.ones((KT - K, 1)), np.zeros((K, 1))))
        # [:,0] allows numpy to see v and d as one-dimensional so
        # that numpy will create a diagonal matrix with v and d as a diagonal
        P = np.diag(v[:, 0], K) + np.diag(d[:, 0])
        Q = 2 * np.dot(np.transpose(P), P)

        # self.cost = QuadFn(self.traj, Q)
        self.cost = 0.
        self.create_robot_clones()
