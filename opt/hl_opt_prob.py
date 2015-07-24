import numpy as np
import cvxpy as cvx
from opt.opt_prob import OptProb
from opt.solver import Solver

class HLOptProb(object):
    def __init__(self, hl_params = None, hl_actions=None):
        if hl_params is None:
            self.hl_params = []
        else:
            self.hl_params = hl_params

        if hl_actions is None:
            self.hl_actions = []
        else:
            self.hl_actions = hl_actions

        self.solver = Solver()

    def add_hl_param(self, hl_param):
        self.hl_params.append(hl_param)

    def solve(self):
        opt_probs = []
        for hl_action in self.hl_actions:
            opt_probs.append(hl_action.opt_prob)
        # self.solver.admm_sqp(opt_probs, self.hl_params)
        self.solver.sqp_admm(opt_probs, self.hl_params)
        # self.solver.big_sqp(opt_probs, self.hl_params)

    def solve_(self):
        opt_probs = []
        for hl_action in self.hl_actions:
            opt_probs.append(hl_action.opt_prob)

        solver = self.solver
        solver.improve_ratio_threshold = .25
        # self.min_trust_box_size = 1e-4
        solver.min_trust_box_size = 1e-2
        # self.min_approx_improve = 1e-4
        # solver.min_approx_improve = 1e-2
        solver.min_approx_improve = 1e-1
        solver.max_iter = 50
        solver.trust_shrink_ratio = .1
        solver.trust_expand_ratio = 1.5
        solver.cnt_tolerance = 1e-4
        solver.max_merit_coeff_increases = 5
        solver.merit_coeff_increase_ratio = 10
        # self.initial_trust_box_size = 1
        # solver.initial_trust_box_size = 2
        self.solver.initial_trust_box_size = 8
        # self.initial_penalty_coeff = 1.
        # solver.initial_penalty_coeff = 0.1
        solver.initial_penalty_coeff = 0.01
        solver.max_penalty_iter = 20

        self.solver.big_sqp(opt_probs, self.hl_params)


