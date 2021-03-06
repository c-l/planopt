import numpy as np
from copy import copy, deepcopy

class Objective(object):
    def __init__(self, qp_objective=0, f = None):
        self.qp_objective = qp_objective
        self.dual_terms = 0
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

    def convexify(self, augment_lagrangian=False):
        convex_objective = 0
        if augment_lagrangian:
            convex_objective = copy(self.qp_objective) + copy(self.dual_terms)
            if self.qp_objective != 0:
                print "\tqp objective value: ", self.qp_objective.value

            print "\tdual terms value: ", self.dual_terms.value
            # convex_objective = self.qp_objective + self.dual_terms
        else:
            convex_objective = copy(self.qp_objective)
        for f,x in self.fs:
            fval, fgrad, fhess = f.val_grad_and_hess(x.value)
            convex_objective += fval + fgrad*(x-x.value) + cvx.quad_form(x-x.value, fhess)
            print "\tf function's value: ", fval
            print "\ttotal objective: ", convex_objective.value
        return convex_objective

    def add_dual_cost(self, var, dual, consensus, ro):
        # if consensus is None:
        #     self.qp_objective += dual.T*var # dual gradient ascent
        # else:
        #     self.qp_objective += dual.T*var + ro/2 * cvx.square(cvx.norm(var-consensus))
        if consensus is None:
            import ipdb; ipdb.set_trace() # BREAKPOINT
            self.dual_terms += dual.T*var # dual gradient ascent
        else:
            self.dual_terms += dual.T*var + ro/2 * cvx.square(cvx.norm(var-consensus))
