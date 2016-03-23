import numpy as np
from interface.fluents.for_all_not_obstructs import ForAllNotObstructs
from interface.fluents.robot_at import RobotAt
from interface.fluents.is_mp import IsMP
from interface.hl_param import Traj
from opt.function import QuadFn
from action import PR2HLAction

class PR2Move(PR2HLAction):

    def __init__(self, lineno, hl_plan, env, robot, start, end, obj, gp=None):
        super(PR2Move, self).__init__(lineno, hl_plan, env, robot)

        self.hl_plan = hl_plan
        self.start = start
        self.end = end
        self.obj = obj

        # TODO: set this using optimization domain
        # self.T = 40
        self.T = 10
        self.K = 7
        T = self.T
        K = self.K
        KT = self.K*self.T

        self.name = "move" + str(lineno)
        self.traj = Traj(self, self.name + "_traj", (self.K, self.T), is_var=True)

        self.params = [start, end, self.traj]
        self.preconditions = [RobotAt(self, 0, start, self.traj)]
        # self.preconditions += [IsMP(self, 0, start, end, self.traj)]

        # self.preconditions += [NotObstructsPR2(env, self, robot, 1, self.traj, obj)]

        self.postconditions = []
        self.postconditions += [RobotAt(self, 0, self.end, self.traj)]

        # setting trajopt objective
        v = -1 * np.ones((KT - K, 1))
        d = np.vstack((np.ones((KT - K, 1)), np.zeros((K, 1))))
        # [:,0] allows numpy to see v and d as one-dimensional so
        # that numpy will create a diagonal matrix with v and d as a diagonal
        P = np.diag(v[:, 0], K) + np.diag(d[:, 0])
        Q = 2 * np.dot(np.transpose(P), P)

        self.cost = QuadFn(self.traj, Q)
        self.create_robot_clones()
