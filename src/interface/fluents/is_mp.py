from fluent import LinLEFluent
from aff_expr import AffExpr
from utils import *
from openravepy import *
import ctrajoptpy
import numpy as np
import time

class IsMP(LinLEFluent):
    def __init__(self, hl_action, traj):
        self.hl_action = hl_action
        self.traj = traj
        self.name = "IsMP"

    def pre(self):
        traj = self.traj
        K = self.hl_action.K
        T = self.hl_action.T
        # K,T = traj.size

        v = -1*np.ones((K*T-K,1))
        d = np.vstack((np.ones((K*T-K,1)),np.zeros((K,1))))
        # [:,0] allows numpy to see v and d as one-dimensional so that numpy will create a diagonal matrix with v and d as a diagonal
        # P = np.matrix(np.diag(v[:,0],K) + np.diag(d[:,0]) )
        P = np.diag(v[:,0],K) + np.diag(d[:,0])

        # positions between time steps are less than 0.2
        A_ineq = np.vstack((P, -P))
        b_ineq = 0.3*np.ones((2*K*T,1))
        # linear_constraints = [A_ineq * traj <= b_ineq]
        v = -1*np.ones(((T-1),1))
        P = np.eye(T) + np.diag(v[:,0],-1)
        P = P[:,:T-1]
        A_ineq = np.hstack((P, -P))
        b_ineq = 0.3*np.ones((K, (T-1)*2))
        self.lhs = AffExpr({self.traj: A_ineq})
        self.rhs = AffExpr(constant = b_ineq)
