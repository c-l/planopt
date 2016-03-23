from fluent import Fluent
from numpy.linalg import norm
import numpy as np
from opt.function import Function
from fluent import LinEqFluent
from aff_expr import AffExpr

import ctrajoptpy
from utils import *

import gurobipy as grb

class InManip(LinEqFluent):
    def __init__(self, hl_action, priority, obj, gp, traj, obj_traj):
        self.hl_action = hl_action
        self.priority = priority
        self.obj = obj
        self.gp = gp
        self.traj = traj
        self.obj_traj = obj_traj
        self.name = "InManip"

    def pre(self):
        # self.obj_traj.set_value(self.traj.get_value() + self.gp.get_value())

        self.lhs = AffExpr({self.traj: 1.0, self.gp: np.ones((1, self.traj.num_timesteps()))})
        self.rhs = AffExpr({self.obj_traj: 1.0})

    def post(self):
        pass

    def grasp(self, x):
        pass
