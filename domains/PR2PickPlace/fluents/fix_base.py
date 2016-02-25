from interface.fluents.fluent import LinEqFluent
from interface.fluents.aff_expr import AffExpr
from utils import *
from openravepy import *
import ctrajoptpy
import numpy as np
import time

class FixBase(LinEqFluent):
    def __init__(self, hl_action, robot, priority, traj):
        self.hl_action = hl_action
        self.priority = priority
        self.robot = robot
        self.traj = traj
        self.name = "IsMP"

    def pre(self):
        # start = self.start.value
        # end = self.end.value
        # self.traj.value = np.array([np.linspace(i, j, self.traj.cols) for i, j in zip(np.array(start), np.array(end))])
        if "base" not in self.robot.active_bodyparts:
            self.rhs = AffExpr(constant=np.zeros((1,1)))
            self.lhs = AffExpr(constant=np.zeros((1,1)))
        else:
            K = self.hl_action.K
            T = self.hl_action.T
            inds = self.robot.bodyparts_to_inds["base"]

            # A_lhs selects the base trajectory from traj
            A_lhs = np.zeros((3,K))
            A_lhs[0, inds[0]] = 1
            A_lhs[1, inds[1]] = 1
            A_lhs[2, inds[2]] = 1

            A_rhs = np.vstack((np.ones((1,T-1)), -np.eye(T-1)))
            # import ipdb; ipdb.set_trace()
            # A_eq = np.zeros((K*T, 3*(T-1)))
            # col = 0
            # for i in inds:
            #     for t in range(1,T):
            #         A_eq[i, col] = 1
            #         A_eq[i+t*K, col] = -1
            #         col += 1
            #
            # import ipdb; ipdb.set_trace()
            self.lhs = AffExpr({self.traj: (A_lhs, A_rhs)})
            self.rhs = AffExpr(constant = np.zeros((3,K-1)))
