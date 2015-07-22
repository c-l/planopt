import numpy as np
import cvxpy as cvx
from copy import deepcopy

class Objective(object):
    def __init__(self, qp_objective=0, f = None):
        self.qp_objective = qp_objective
        if f is None:
            self.fs = []
        else:
            self.fs = [f]

    def add_qp_objective(self, obj):
        self.objective += obj

    def add_nonquad_objective(self, f):
        self.fs.append(f)

    def add_objective(self, obj):
        self.qp_objective += obj.qp_objective
        self.fs += obj.fs

    def convexify(self):
        convex_objective = deepcopy(self.qp_objective)
        for f,x in self.fs:
            fval, fgrad, fhess = f.val_grad_and_hess(x.cur_value)
            convex_objective += fval + fgrad*(x-x.cur_value) + cvx.quad_form(x-x.cur_value, fhess)
        return convex_objective

    def add_dual_cost(self, var, dual, consensus, ro):
        if consensus is None:
            self.qp_objective += dual.T*var # dual gradient ascent
        else:
            self.qp_objective += dual.T*var + ro/2 * cvx.square(cvx.norm(var-consensus))
