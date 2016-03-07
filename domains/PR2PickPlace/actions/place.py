import numpy as np
import sys
DOMAIN_PATH = "../domains/PR2PickPlace/"
sys.path.insert(0, DOMAIN_PATH)
from fluents.not_obstructs import NotObstructsPR2
# from fluents.is_gp import PR2IsGP
from fluents.is_mp import PR2IsMP
from fluents.fix_base import FixBase
from fluents.in_manip import PR2InManip
from fluents.obj_in_manip import ObjInManip
from fluents.obj_at import PR2ObjAt
from interface.fluents.robot_at import RobotAt
from actions.action import PR2HLAction
from interface.hl_param import HLParam, Traj, Robot, Obj, PR2

from opt.function import QuadFn

class PR2Place(PR2HLAction):

    def __init__(self, lineno, hl_plan, env, robot, start, end, obj, loc):
        super(PR2Place, self).__init__(lineno, hl_plan, env, robot)

        self.hl_plan = hl_plan
        self.start = start
        self.end = end
        self.obj = obj
        self.loc = loc

        # TODO: set this using optimization domain
        self.T = 2
        self.K = robot.dofs

        self.name = "place" + str(lineno)
        self.traj = Traj(self, self.name + "_traj", self.K, self.T, is_var=True)
        self.obj_traj = Traj(self, self.name + "_obj_traj", 6, self.T, is_var=True)

        self.params = [self.start, self.end, self.loc, self.traj, self.obj_traj]
        self.preconditions = [RobotAt(self, 0, self.start, self.traj)]
        self.preconditions += [NotObstructsPR2(self.env, self, robot, 1, self.traj, self.obj)]
        self.preconditions += [PR2IsMP(env, self, robot, 0, self.start, self.end, self.traj)]
        # gp is None currently because it is defined as the middle of the cylinder
        self.preconditions += [PR2InManip(env, self, robot, 0, self.obj, None, self.traj, self.obj_traj)]
        self.preconditions += [FixBase(self, robot, 0, self.traj)]

        self.postconditions = []
        self.postconditions += [RobotAt(self, 0, self.end, self.traj)]
        self.postconditions += [PR2ObjAt(self, 0, self.obj, self.loc, self.obj_traj)]


        # setting trajopt objective
        v = -1 * np.ones((self.K*self.T - self.K, 1))
        d = np.vstack((np.ones((self.K*self.T - self.K, 1)), np.zeros((self.K, 1))))
        # [:,0] allows numpy to see v and d as one-dimensional so
        # that numpy will create a diagonal matrix with v and d as a diagonal
        P = np.diag(v[:, 0], self.K) + np.diag(d[:, 0])
        Q = 2 * np.dot(np.transpose(P), P)

        self.cost = QuadFn(self.traj, Q)
        # self.cost = 0
        self.create_robot_clones()
