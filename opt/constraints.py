import numpy as np
import cvxpy as cvx
from copy import copy, deepcopy

class Constraints(object):
    def __init__(self, linear_constraints=None, g=None, h=None):
        if linear_constraints is None:
            self.linear_constraints = []
        else:
            self.linear_constraints = linear_constraints

        if g is None:
            self.gs = [] # non-linear inequality constraints
        else:
            self.gs = [g]

        if h is None:
            self.hs = [] # non-linear equality constraints
        else:
            self.hs = [h]

    # def add_qp_constraint(self, constraints):
    #     self.linear_constraints += constraints

    # def add_nonliner_ineq_constraint(self, g, x)
    #     self.gs.append((g,x))

    # def add_nonlinear_eq_constraint(self, h, x)
    #     self.hs.append((h,x))
    
    def add_constraints(self, constraints):
        self.linear_constraints += constraints.linear_constraints
        self.gs += constraints.gs
        self.hs += constraints.hs

    def linear_constraints_satisfied(self):
        for constraint in self.linear_constraints:
            if not np.all(constraint.value):
                return False
        return True
    # convexifies constraints around value of the current variable x which is saved in g and h
    # x is not necessarily the same between functions, it could be the traj from different hl actions
    def constraints_satisfied(self, tolerance):
        for g,x in self.gs:
            gval, gjac = g.val_and_grad(x.value)
            if any(gval > tolerance):
                return False

        for h,x in self.hs:
            hval, hjac = h.val_and_grad(x.value)
            if abs(hval) > tolerance:
                return False
        return True

    def convexify(self):
        penalty_objective = 0
        for g,x in self.gs:
            gval, gjac = g.val_and_grad(x.value)
            penalty_objective += cvx.sum_entries(cvx.pos(gval + gjac*(x-x.value)))
        for h,x in self.hs:
            hval, hjac = h.val_and_grad(x.value)
            penalty_objective += cvx.norm(hval + hjac*(x-x.value), 1)
        # import ipdb; ipdb.set_trace() # BREAKPOINT
        return penalty_objective, copy(self.linear_constraints)

