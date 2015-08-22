import numpy as np
import cvxpy as cvx
from opt.objective import Objective
from opt.constraints import Constraints

class OptProb(object):
    def __init__(self, hl_action=None, augment_lagrangian=False):
        self.hl_action = hl_action
        self.augmented_objective = augment_lagrangian
        self.constraints = Constraints()
        self.objective = Objective()

        self.xs = [] # variables associated with the objectives and constraints that need to be convexified
        self.saved_xs = []

        self.callbacks = []

    def save_state(self):
        self.saved_xs = [x.value for x in self.xs]

    def restore_state(self):
        for i in range(len(self.xs)):
            # x's value needs to be also reset so that the merit can be computed correctly
            self.xs[i].value = self.saved_xs[i]

    def add_callback(self, callback):
        self.callbacks.append(callback)

    def callback(self):
        for callback in self.callbacks:
            callback()

    def add_var(self, x):
        self.xs.append(x)

    def add_constraints(self, constraints):
        self.constraints.add_constraints(constraints)

    def add_objective(self, objective):
        self.objective.add_objective(objective)

    def augment_lagrangian(self):
        self.augmented_objective = True

    def make_primal(self):
        self.augmented_objective = False

    def add_dual_cost(self, var, dual, consensus=None, ro = 0.05):
        self.objective.add_dual_cost(var, dual, consensus, ro)

    def add_opt_prob(self, prob):
        self.add_objective(prob.objective)
        self.add_constraints(prob.constraints)
        self.xs += prob.xs

    def constraints_satisfied(self, tolerance):
        return self.constraints.constraints_satisfied(tolerance)

    def convexify(self, penalty_coeff, trust_box_size):
        objective = self.objective.convexify(self.augmented_objective)
        # print "convexify"
        # if objective != 0:
        #     print "objective value: ", objective.value
        penalty_objective, linear_constraints = self.constraints.convexify()
        # print "penalty objective value: ", penalty_coeff.value * penalty_objective.value
        objective += penalty_coeff*penalty_objective
        print "\tpenalty objective: ",penalty_coeff.value*penalty_objective.value

        trust_box_sum = 0
        for x in self.xs:
            trust_box_sum += cvx.norm(x-x.value,1)
        linear_constraints += [trust_box_sum <= trust_box_size]

        return objective, linear_constraints


