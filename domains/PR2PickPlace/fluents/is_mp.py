from interface.fluents.fluent import AndFluent, LinLEFluent
from interface.fluents.aff_expr import AffExpr
from utils import *
from openravepy import *
import ctrajoptpy
import numpy as np

eps = .5

class PR2IsMP(AndFluent):
    def __init__(self, env, hl_action, robot, priority, start, end, traj):
        self.env = env
        self.hl_action = hl_action
        self.robot = robot
        self.priority = priority
        self.start = start
        self.end = end
        self.traj = traj
        self.name = "IsMP"

    def pre(self):
        disp_lhs, disp_rhs = self.displacement()
        disp = LinLEFluent("disp_"+self.name, self.priority, self.hl_action, disp_lhs, disp_rhs)

        ub_lhs, ub_rhs = self.upper_joint_limits()
        ub = LinLEFluent("upper_joint_limits_"+self.name, self.priority, self.hl_action, ub_lhs, ub_rhs)

        lb_lhs, lb_rhs = self.lower_joint_limits()
        lb = LinLEFluent("lower_joint_limits_"+self.name, self.priority, self.hl_action, lb_lhs, lb_rhs)

        self.fluents = [disp, ub, lb]
        # self.fluents = [disp, ub]
        # self.fluents = [disp]

    def displacement(self):
        K = self.hl_action.K
        T = self.hl_action.T
        # K,T = traj.size

        v = -1*np.ones((K*T-K,1))
        d = np.vstack((np.ones((K*T-K,1)),np.zeros((K,1))))
        # [:,0] allows numpy to see v and d as one-dimensional so that numpy will create a diagonal matrix with v and d as a diagonal
        # P = np.matrix(np.diag(v[:,0],K) + np.diag(d[:,0]) )
        P = np.diag(v[:,0],K) + np.diag(d[:,0])

        # positions between time steps are less than 0.3
        A_ineq = np.vstack((P, -P))
        b_ineq = eps*np.ones((2*K*T,1))
        # linear_constraints = [A_ineq * traj <= b_ineq]
        v = -1*np.ones(((T-1),1))
        P = np.eye(T) + np.diag(v[:,0],-1)
        P = P[:,:T-1]
        A_ineq = np.hstack((P, -P))
        b_ineq = eps*np.ones((K, (T-1)*2))
        lhs = AffExpr({self.traj: A_ineq})
        rhs = AffExpr(constant = b_ineq)
        return (lhs, rhs)

    def upper_joint_limits(self):
        K = self.hl_action.K
        T = self.hl_action.T
        robot_body = self.robot.get_env_body(self.env)
        indices = robot_body.GetActiveDOFIndices()
        assert len(indices) == K
        lb, ub = robot_body.GetDOFLimits()
        active_ub = ub[indices].reshape((K,1))
        ub_stack = np.tile(active_ub, (1,T))
        # import ipdb; ipdb.set_trace()
        lhs = AffExpr({self.traj: 1})
        rhs = AffExpr(constant = ub_stack)
        return (lhs, rhs)

    def lower_joint_limits(self):
        K = self.hl_action.K
        T = self.hl_action.T
        robot_body = self.robot.get_env_body(self.env)
        indices = robot_body.GetActiveDOFIndices()
        assert len(indices) == K
        lb, ub = robot_body.GetDOFLimits()
        active_lb = lb[indices].reshape((K,1))
        lb_stack = np.tile(active_lb, (1,T))
        lhs = AffExpr(constant = lb_stack)
        rhs = AffExpr({self.traj: 1})
        return (lhs, rhs)
