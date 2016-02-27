import numpy as np
from numpy import square, dot
from numpy.linalg import norm
# import cvxpy as cvx
import gurobipy as grb
from function import Function
GRB = grb.GRB
from IPython import embed as shell


class OptProb(object):

    def __init__(self, augment_lagrangian=False):
        self.augmented_objective = augment_lagrangian
        self.constraints = []

        self.model = grb.Model()
        # suppress output
        self.model.params.OutputFlag = 0
        # set method to primal simplex
        self.model.params.method = 0
        # increase optimality tolerance to maximum allowed
        self.model.params.OptimalityTol = 0.01
        # quadratic objective
        self.obj_quad = []
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

    def clear_handles(self):
        for hla in self.hlas:
            hla.clear_handles()

    def initialize_traj(self, mode):
        # mode can be "straight" for straight line or
        # "adapt" for adapted previous trajectories
        if self.trust_region_cnt is not None:
            self.model.remove(self.trust_region_cnt)
        self.clean(self.trust_temp)

        for constraint in self.constraints:
            constraint.clean()

        obj = grb.QuadExpr()
        for var in self.vars:
            if var.value is not None and var.recently_sampled:
                obj += 100 * self.l2_norm_diff_squared(self.model, var)
            elif var.value is not None and var.is_resampled:
                obj += 10 * self.l2_norm_diff_squared(self.model, var)
        if mode == "straight":
            obj += grb.quicksum(self.obj_quad)
        elif mode == "adapt":
            for var in self.vars:
                if var.hl_param.is_traj:
                    K = var.hl_param.rows
                    T = var.hl_param.cols
                    KT = K * T
                    v = -1 * np.ones((KT - K, 1))
                    d = np.vstack((np.ones((KT - K, 1)), np.zeros((K, 1))))
                    # [:,0] allows numpy to see v and d as one-dimensional so
                    # that numpy will create a diagonal matrix with v and d as a diagonal
                    P = np.diag(v[:, 0], K) + np.diag(d[:, 0])
                    # minimum-velocity finite difference
                    Q = np.dot(np.transpose(P), P)
                    obj += Function.quad_expr((var.get_grb_vars() - var.value).flatten(order="f"), Q)
        else:
            raise NotImplementedError

        self.model.setObjective(obj)
        self.model.update()
        self.model.optimize()
        self.update_vars()
        self.plot()
        return True

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

    def inc_obj(self, quad_fn):
        self.obj_fns += [quad_fn]
        self.obj_quad += [quad_fn.expr]

    def val(self, penalty_coeff):
        param_to_inds = {}
        val = []
        for i, fn in enumerate(self.obj_fns):
            val.append(fn.val())
            param_to_inds.setdefault(fn.param, set()).add(i)

        # if self.augmented_objective:
        #     dual_val = 0
        #     for (var, dual, consensus, ro) in self.dual_terms:
        #         dual_val += np.dot(np.transpose(dual.value), var.value)[0, 0] + ro / 2 * square(norm(var.value - consensus.value, 2))
        #     val += dual_val

        assert len(self.constraints) == 1
        arr = []
        for c in self.constraints:
            arr.extend(c.val_lst(start_ind=i+1, param_to_inds=param_to_inds))

        return np.hstack((val, penalty_coeff * np.array(arr))), param_to_inds

    def val_old(self, penalty_coeff):
        val = 0
        for fn in self.obj_fns:
            val += fn.val()

        penalty_cost = penalty_coeff * \
            sum([constraint.val_old() for constraint in self.constraints])
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

        self.convexified_constr = []
        for c in self.constraints:
            self.convexified_constr.extend(constraint.convexify(self.model, penalty_coeff))
        penalty_obj = grb.quicksum(self.convexified_constr)
        # self.obj_sqp = self.obj + penalty_coeff * self.nonlinear_cnts.convexify(self.model, penalty_coeff)
        self.obj_sqp = grb.quicksum(self.obj_quad) + penalty_obj
        self.add_dual_costs()

    # @profile
    def add_trust_region(self, trust_region_sizes):
        if self.trust_region_cnt is not None:
            self.model.remove(self.trust_region_cnt)
        self.clean(self.trust_temp)
        self.trust_temp = []

        # var_list = [grb_var for var in self.vars for grb_var in var.grb_vars.flatten()]
        # val_list = [val for var in self.vars for val in var.value.flatten()]

        # self.add_trust_region_cnt(var_list, val_list, trust_region_size)

        for var in self.vars:
            if var.hl_param in trust_region_sizes:
                self.add_trust_region_cnt(var.name, var.grb_vars.flatten(), var.value.flatten(), trust_region_sizes[var.hl_param])

    def add_trust_region_old(self, trust_region_size):
        if self.trust_region_cnt is not None:
            self.model.remove(self.trust_region_cnt)
        self.clean(self.trust_temp)
        self.trust_temp = []

        var_list = [
            grb_var for var in self.vars for grb_var in var.grb_vars.flatten()]
        val_list = [val for var in self.vars for val in var.value.flatten()]

        # self.trust_region_cnt = self.add_trust_region_cnt(
        self.add_trust_region_cnt_old(var_list, val_list, trust_region_size)

    # @profile
    def add_trust_region_cnt_old(self, x, xp, trust_box_size):
        if self.init_trust_region:
            rows = len(x)

            pos = []
            neg = []
            expr = []
            self.diffs = []
            for i in range(rows):
                pos.append(
                    self.model.addVar(lb=0, ub=GRB.INFINITY, name='pos' + str(i)))
                neg.append(
                    self.model.addVar(lb=0, ub=GRB.INFINITY, name='neg' + str(i)))

            self.model.update()
            for i in range(rows):
                # diff = grb.LinExpr(-1 * x[i])
                # diff.addConstant(xp[i])
                diff = grb.LinExpr(x[i])
                diff.addConstant(-1*xp[i])
                abs_diff = grb.LinExpr([1, -1], [pos[i], neg[i]])
                # self.trust_temp.append(self.model.addConstr(diff == pos[i] - neg[i]))
                self.trust_temp.append(
                    self.model.addConstr(diff, GRB.EQUAL, abs_diff))
                abs_val = grb.LinExpr(pos[i] + neg[i])
                self.trust_temp.append(self.model.addConstr(abs_val <= trust_box_size))
                self.diffs.append(abs)
        # not being used currently
        else:
            print "Is this being used?"
            import ipdb; ipdb.set_trace()
            rows = len(x)
            for i in range(rows):
                diff = grb.LinExpr(-1 * x[i])
                diff.addConstant(xp[i])
                self.trust_temp.append(
                    self.model.addConstr(diff, GRB.EQUAL, self.diffs[i]))

    # @profile
    def add_trust_region_cnt(self, var_name, x, xp, trust_box_size):
        if self.init_trust_region:
            rows = len(x)

            pos = []
            neg = []
            expr = []
            for i in range(rows):
                pos.append(
                    self.model.addVar(lb=0, ub=GRB.INFINITY, name='%s_pos_%d'%(var_name, i)))
                neg.append(
                    self.model.addVar(lb=0, ub=GRB.INFINITY, name='%s_neg_%d'%(var_name, i)))

            self.model.update()
            for i in range(rows):
                # diff = grb.LinExpr(-1 * x[i])
                # diff.addConstant(xp[i])
                diff = grb.LinExpr(x[i])
                diff.addConstant(-1*xp[i])
                abs_diff = grb.LinExpr([1, -1], [pos[i], neg[i]])
                # self.trust_temp.append(self.model.addConstr(diff == pos[i] - neg[i]))
                self.trust_temp.append(
                    self.model.addConstr(diff, GRB.EQUAL, abs_diff))
                abs_val = grb.LinExpr(pos[i] + neg[i])
                self.trust_temp.append(self.model.addConstr(abs_val <= trust_box_size))
        # not being used currently
        else:
            assert False
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
