import numpy as np
import cvxpy as cvx
from opt.opt_prob import OptProb
from opt.solver import Solver

class LLPlan(object):
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
        self.sqp_convexify_admm(opt_probs)
        # self.sqp_admm(opt_probs)
        # self.admm_sqp(opt_probs)
        # self.big_sqp(opt_probs)

    def sqp_convexify_admm(self, opt_probs):
        for opt_prob in opt_probs:
            opt_prob.augment_lagrangian()
        solver = self.solver
        solver.improve_ratio_threshold = .25
        # solver.min_trust_box_size = 1e-5
        solver.min_trust_box_size = 1e-4
        # solver.min_trust_box_size = 1e-3
        # solver.min_trust_box_size = 1e-2
        # solver.min_approx_improve = 1e-5
        solver.min_approx_improve = 1e-4
        # solver.min_approx_improve = 1e-3
        # solver.min_approx_improve = 1e-2
        # solver.min_approx_improve = 3e-2
        # solver.min_approx_improve = 1e-1
        # solver.min_approx_improve = 3e-1
        solver.max_iter = 50
        solver.trust_shrink_ratio = .1
        solver.trust_expand_ratio = 1.5
        # solver.cnt_tolerance = 1e-2
        solver.cnt_tolerance = 1e-3
        # solver.cnt_tolerance = 1e-4
        # solver.max_merit_coeff_increases = 5
        # solver.max_merit_coeff_increases = 4
        solver.max_merit_coeff_increases = 3
        # solver.max_merit_coeff_increases = 2
        solver.merit_coeff_increase_ratio = 10
        # solver.initial_trust_box_size = 0.1
        solver.initial_trust_box_size = 0.3
        # solver.initial_trust_box_size = 1
        # solver.initial_trust_box_size = 2
        # solver.initial_trust_box_size = 3
        # solver.initial_trust_box_size = 10
        # solver.initial_penalty_coeff = 1.
        solver.initial_penalty_coeff = 10
        # solver.initial_penalty_coeff = 0.3
        # solver.initial_penalty_coeff = 0.2
        # solver.initial_penalty_coeff = 0.1
        # solver.initial_penalty_coeff = 0.03
        # solver.initial_penalty_coeff = 0.01
        solver.callback = []

        # solver.epsilon = 5e-3
        solver.epsilon = 1e-2
        # solver.epsilon = 2e-2
        # solver.epsilon = 2.5e-2
        # solver.epsilon = 3e-2
        # solver.epsilon = 1e-1
        solver.sqp_iters = 0
        self.solver.sqp_convexify_admm(opt_probs, self.hl_params)

    def sqp_admm(self, opt_probs):
        for opt_prob in opt_probs:
            opt_prob.augment_lagrangian()
        solver = self.solver
        solver.improve_ratio_threshold = .25
        # solver.min_trust_box_size = 1e-4
        solver.min_trust_box_size = 1e-3
        # solver.min_trust_box_size = 1e-2
        # solver.min_approx_improve = 1e-5
        solver.min_approx_improve = 1e-4
        # solver.min_approx_improve = 1e-3
        # solver.min_approx_improve = 1e-2
        # solver.min_approx_improve = 3e-2
        # solver.min_approx_improve = 1e-1
        # solver.min_approx_improve = 3e-1
        solver.max_iter = 50
        solver.trust_shrink_ratio = .1
        solver.trust_expand_ratio = 1.5
        solver.cnt_tolerance = 1e-2
        # solver.cnt_tolerance = 1e-4
        solver.max_merit_coeff_increases = 5
        # solver.max_merit_coeff_increases = 2
        solver.merit_coeff_increase_ratio = 10
        # solver.initial_trust_box_size = 1
        # solver.initial_trust_box_size = 2
        solver.initial_trust_box_size = 3
        # solver.initial_trust_box_size = 10
        # solver.initial_penalty_coeff = 1.
        # solver.initial_penalty_coeff = 10
        # solver.initial_penalty_coeff = 0.3
        # solver.initial_penalty_coeff = 0.2
        solver.initial_penalty_coeff = 0.1
        # solver.initial_penalty_coeff = 0.03
        # solver.initial_penalty_coeff = 0.01
        solver.callback = []

        # self.epsilon = 5e-3
        # solver.epsilon = 1e-2
        solver.epsilon = 2e-2
        # solver.epsilon = 3e-2
        # solver.epsilon = 1e-1
        solver.sqp_iters = 0
        self.solver.sqp_admm(opt_probs, self.hl_params)

    def admm_sqp(self, opt_probs):
        for opt_prob in opt_probs:
            opt_prob.augment_lagrangian()
        solver = self.solver
        solver.improve_ratio_threshold = .25
        # solver.min_trust_box_size = 1e-4
        solver.min_trust_box_size = 1e-2
        # solver.min_approx_improve = 1e-4
        # solver.min_approx_improve = 1e-2
        solver.min_approx_improve = 1e-1
        # solver.min_approx_improve = 3e-1
        solver.max_iter = 50
        solver.trust_shrink_ratio = .1
        solver.trust_expand_ratio = 1.5
        solver.cnt_tolerance = 1e-4
        solver.max_merit_coeff_increases = 5
        solver.merit_coeff_increase_ratio = 10
        # solver.initial_trust_box_size = 1
        # solver.initial_trust_box_size = 2
        solver.initial_trust_box_size = 3
        # solver.initial_trust_box_size = 10
        # solver.initial_penalty_coeff = 1.
        # solver.initial_penalty_coeff = 10
        # solver.initial_penalty_coeff = 0.3
        solver.initial_penalty_coeff = 0.1
        # solver.initial_penalty_coeff = 0.01
        # solver.max_penalty_iter = 4
        solver.max_penalty_iter = 8
        solver.callback = []

        # solver.epsilon = 5e-3
        solver.epsilon = 1e-2
        solver.sqp_iters = 0
        solver.solver.admm_sqp(opt_probs, self.hl_params)

    def big_sqp(self, opt_probs):
        # for opt_prob in opt_probs:
        #     opt_prob.primal()
        solver = self.solver
        solver.improve_ratio_threshold = .25
        # solver.min_trust_box_size = 1e-4
        solver.min_trust_box_size = 1e-2
        # solver.min_approx_improve = 1e-4
        # solver.min_approx_improve = 1e-2
        solver.min_approx_improve = 1e-1
        solver.max_iter = 50
        solver.trust_shrink_ratio = .1
        solver.trust_expand_ratio = 1.5
        solver.cnt_tolerance = 1e-4
        solver.max_merit_coeff_increases = 5
        solver.merit_coeff_increase_ratio = 10
        # solver.initial_trust_box_size = 1
        # solver.initial_trust_box_size = 2
        solver.initial_trust_box_size = 8
        # solver.initial_penalty_coeff = 1.
        # solver.initial_penalty_coeff = 0.1
        solver.initial_penalty_coeff = 0.01
        solver.max_penalty_iter = 20

        self.solver.big_sqp(opt_probs, self.hl_params)


