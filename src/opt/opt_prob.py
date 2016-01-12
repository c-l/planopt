import numpy as np
from numpy import square, dot
from numpy.linalg import norm
# import cvxpy as cvx
import gurobipy as grb
GRB = grb.GRB


class OptProb(object):

    def __init__(self, augment_lagrangian=False):
        self.augmented_objective = augment_lagrangian
        self.constraints = []

        self.model = grb.Model()
        self.model.params.OutputFlag = 0 # suppresses output
        # quadratic objective
        self.obj_quad = grb.QuadExpr()
        # sqp objective
        self.obj_sqp = None
        # objective functions that need to be convexified
        self.obj_fns = []
        self.vars = []

        self.hlas = []

        self.trust_region_cnt = None
        self.dual_terms = []
        self.trust_temp = []

        self.callbacks = []

        self.init_trust_region = True

    def get_model(self):
        return self.model

    def update_vars(self):
        for var in self.vars:
            var.update()

    def plot(self):
        for hla in self.hlas:
            hla.plot()

    def clear_plots(self):
        for hla in self.hlas:
            hla.clear_plots()

    def find_closest_feasible_point(self):
        if self.trust_region_cnt is not None:
            self.model.remove(self.trust_region_cnt)
        self.clean(self.trust_temp)

        for constraint in self.constraints:
            constraint.clean()

        obj = grb.QuadExpr()
        for var in self.vars:
            if var.value is not None:
                obj += self.l2_norm_diff_squared(self.model, var)

        self.model.setObjective(obj)
        self.model.update()
        self.model.optimize()
        self.update_vars()
        return True

    def optimize(self):
        self.model.setObjective(self.obj_sqp)
        self.model.update()
        self.model.optimize()
        self.update_vars()

    def clean(self, temp):
        for item in temp:
            self.model.remove(item)
        temp = []

    def inc_obj(self, quad_fn):
        self.obj_fns += [quad_fn]
        self.obj_quad += quad_fn.expr

    def val(self, penalty_coeff):
        val = 0
        for fn in self.obj_fns:
            val += fn.val()

        if self.augmented_objective:
            dual_val = 0
            for (var, dual, consensus, ro) in self.dual_terms:
                dual_val += np.dot(np.transpose(dual.value), var.value)[0, 0] + ro / 2 * square(norm(var.value - consensus.value, 2))
            val += dual_val

        penalty_cost = penalty_coeff * \
            sum([constraint.val() for constraint in self.constraints])
        val += penalty_cost
        return val

    def save(self):
        for var in self.vars:
            var.save()

    def restore(self):
        for var in self.vars:
            var.restore()

    def add_callback(self, callback):
        self.callbacks.append(callback)

    def callback(self):
        for callback in self.callbacks:
            callback()

    def add_hla(self, hla):
        self.hlas.append(hla)

    def add_var(self, var):
        self.vars.append(var)

    def add_constraints(self, constraints):
        self.constraints.append(constraints)

    def augment_lagrangian(self):
        self.augmented_objective = True

    def make_primal(self):
        self.augmented_objective = False

    def add_dual_cost(self, var, dual, consensus=None, ro=0.05):
        self.dual_terms.append((var, dual, consensus, ro))

    def add_dual_costs(self):
        self.model.update()
        if self.augmented_objective:
            for (var, dual, consensus, ro) in self.dual_terms:
                self.obj_sqp += np.dot(np.transpose(dual.value), var.grb_vars)[0, 0] + ro / 2 * self.l2_norm_squared(self.model, var, consensus.value)

    def constraints_satisfied(self, tolerance):
        for constraint in self.constraints:
            if not constraint.satisfied(tolerance):
                return False
        return True

    # @profile
    def convexify(self, penalty_coeff):
        for constraint in self.constraints:
            constraint.clean()

        penalty_obj = grb.quicksum([constraint.convexify(
            self.model, penalty_coeff) for constraint in self.constraints])
        # self.obj_sqp = self.obj + penalty_coeff * self.nonlinear_cnts.convexify(self.model, penalty_coeff)
        self.obj_sqp = self.obj_quad + penalty_obj
        self.add_dual_costs()

    # @profile
    def add_trust_region(self, trust_region_size):
        if self.trust_region_cnt is not None:
            self.model.remove(self.trust_region_cnt)
        self.clean(self.trust_temp)

        var_list = [
            grb_var for var in self.vars for grb_var in var.grb_vars.flatten()]
        val_list = [val for var in self.vars for val in var.value.flatten()]

        self.trust_region_cnt = self.add_trust_region_cnt(
            var_list, val_list, trust_region_size)

    # @profile
    def add_trust_region_cnt(self, x, xp, trust_box_size):
        # expr = grb.LinExpr()
        if self.init_trust_region:
            rows = len(x)
            # expr = grb.quicksum(addAbs(grb.LinExpr(x[i]-xp[i])))
            pos = []
            neg = []
            expr = []
            self.diffs = []
            for i in range(rows):
                pos.append(
                    self.model.addVar(lb=0, ub=GRB.INFINITY, name='pos' + str(i)))
                neg.append(
                    self.model.addVar(lb=0, ub=GRB.INFINITY, name='neg' + str(i)))
                # expr.append(pos-neg)
            # import ipdb; ipdb.set_trace() # BREAKPOINT
            coeffs = [1] * (2 * rows)
            expr = grb.LinExpr(coeffs, pos + neg)
            # expr = grb.quicksum(pos+neg)
            self.trust_temp.extend(pos + neg)

            # self.trust_cnts = []
            self.model.update()
            for i in range(rows):
                diff = grb.LinExpr(-1 * x[i])
                diff.addConstant(xp[i])
                abs = grb.LinExpr([1, -1], [pos[i], neg[i]])
                # self.trust_temp.append(self.model.addConstr(diff == pos[i] - neg[i]))
                self.trust_temp.append(
                    self.model.addConstr(diff, GRB.EQUAL, abs))
                self.diffs.append(abs)
            # self.model.update()

            # import ipdb; ipdb.set_trace() # BREAKPOINT
            # for i in range(rows):
            #     expr += abs(self.model, grb.LinExpr(x[i]-xp[i]), temp=self.trust_temp)
            # self.init_trust_region = False
            return self.model.addConstr(expr <= trust_box_size)
        else:
            rows = len(x)
            for i in range(rows):
                diff = grb.LinExpr(-1 * x[i])
                diff.addConstant(xp[i])
                self.trust_temp.append(
                    self.model.addConstr(diff, GRB.EQUAL, self.diffs[i]))

    def l2_norm_squared(self, model, var, consensus):
        obj = grb.QuadExpr()
        x = var.grb_vars
        # value = consensus.value
        value = consensus
        rows, cols = x.shape
        for i in range(rows):
            for j in range(cols):
                obj += x[i, j] * x[i, j] - 2 * value[i, j] * \
                    x[i, j] + value[i, j] * value[i, j]
        return obj

    def l2_norm_diff_squared(self, model, var):
        obj = grb.QuadExpr()
        x = var.grb_vars
        value = var.value
        rows, cols = x.shape
        for i in range(rows):
            for j in range(cols):
                obj += x[i, j] * x[i, j] - 2 * value[i, j] * \
                    x[i, j] + value[i, j] * value[i, j]
        return obj
