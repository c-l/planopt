from fluent import Fluent
import cvxpy as cvx
from numpy.linalg import norm
import numpy as np
from opt.constraints import Constraints
from opt.function import Function

import ctrajoptpy
from utils import *

class InManip(Fluent):
    def __init__(self, env, hl_action, model, robot, obj, gp, traj, obj_traj = None):
        super(InManip, self).__init__(env, hl_action, model)
        self.plotting_env = hl_action.hl_plan.env
        self.robot = robot
        self.obj = obj
        self.gp = gp.grb_vars
        self.traj = traj.grb_vars
        self.obj_traj = obj_traj.grb_vars
        self.name = "InManip"
        
    def precondition(self):
        K = self.hl_action.K
        T = self.hl_action.T
        for i in range(T):
            self.constraints.add_eq_cntr(self.traj[K*i:K*(i+1)], self.obj_traj[K*i:K*(i+1)])
        return self.constraints


