import numpy as np
# import cvxpy as cvx
import gurobipy as grb
GRB = grb.GRB
from opt.objective import Objective
from opt.constraints import Constraints
from opt.ops import abs

class OptProb(object):
    def __init__(self, hl_action=None, augment_lagrangian=False):
        self.hl_action = hl_action
        self.augmented_objective = augment_lagrangian
        # self.constraints = Constraints()
        self.constraints = []
        # self.objective = Objective()

        # self.xs = [] # variables associated with the objectives and constraints that need to be convexified
        # self.saved_xs = []

        self.model = grb.Model()
        # self.model_sqp = None
        self.obj = grb.QuadExpr()
        # self.obj_wo_penaly = grb.QuadExpr()
        self.obj_sqp = None
        self.obj_fns = []
        self.vars = []

        self.dual_terms = []

        # self.contraints = Constraints()
        # self.nonlinear_cnts = NonlinCnts()

        self.trust_region_cnt = None

        self.callbacks = []

    def get_model(self):
        return self.model

    def update_vars(self):
        for var in self.vars:
            var.update()

    def find_closest_feasible_point(self):
        obj = grb.QuadExpr()
        for var in self.vars:
            obj += self.l2_norm_squared(self.model, var)

        self.model.setObjective(obj)
        self.model.update()
        self.model.optimize()
        self.update_vars()
        return True

    def optimize(self):
        self.model.setObjective(self.obj_sqp)
        # self.model_sqp.setObjective(self.obj_sqp)
        self.model.update()
        self.model.optimize()
        self.update_vars()

    def inc_obj(self, quad_fn):
        self.obj_fns += [quad_fn]
        self.obj += quad_fn.expr

    def val(self, penalty_coeff):
        val = 0
        for fn in self.obj_fns:
            val += fn.val()

        for (var, dual, consensus, ro) in dual_terms:
        penalty_cost = penalty_coeff * sum([constraint.val() for constraint in self.constraints])
        val += penalty_cost
        return val

    def save(self):
        for var in self.vars:
            var.save()

    def restore(self):
        for var in self.vars:
            var.restore()

    # def save_state(self):
    #     self.saved_xs = [x.value for x in self.xs]

    # def restore_state(self):
    #     for i in range(len(self.xs)):
    #         # x's value needs to be also reset so that the merit can be computed correctly
    #         self.xs[i].value = self.saved_xs[i]

    def add_callback(self, callback):
        self.callbacks.append(callback)

    def callback(self):
        for callback in self.callbacks:
            callback()

    def add_var(self, var):
        self.vars.append(var)
    # def add_var(self, x):
    #     self.xs.append(x)

    def add_constraints(self, constraints):
        self.constraints.append(constraints)

    # def add_objective(self, objective):
    #     self.objective.add_objective(objective)

    def augment_lagrangian(self):
        self.augmented_objective = True

    def make_primal(self):
        self.augmented_objective = False

    def add_dual_cost(self, var, dual, consensus=None, ro = 0.05):
        return

    # def add_dual_costs(self):
    #     if self.augmented_objective:
    #         for (var, dual, hl_param, ro) in dual_terms:
    #             self.obj_sqp += 
    # def add_dual_cost(self, var, dual, consensus=None, ro = 0.05):
    #     self.objective.add_dual_cost(var, dual, consensus, ro)

    # # def add_opt_prob(self, prob):
    # #     self.add_objective(prob.objective)
    # #     self.add_constraints(prob.constraints)
    # #     self.xs += prob.xs

    def constraints_satisfied(self, tolerance):
        for constraint in self.constraints:
            if not constraint.satisfied(tolerance):
                return False
        return True

    # # def constraints_satisfied(self, tolerance):
    # #     return self.constraints.constraints_satisfied(tolerance)

    def convexify(self, penalty_coeff):
        penalty_obj = grb.quicksum([constraint.convexify(self.model, penalty_coeff) for constraint in self.constraints])
        # self.obj_sqp = self.obj + penalty_coeff * self.nonlinear_cnts.convexify(self.model, penalty_coeff)
        self.obj_sqp = self.obj + penalty_obj

    # # def convexify(self, penalty_coeff, trust_box_size):
    # #     objective = self.objective.convexify(self.augmented_objective)
    #     # print "convexify"
    #     # if objective != 0:
    #     #     print "objective value: ", objective.value
    #     penalty_objective, linear_constraints = self.constraints.convexify()
    #     # print "penalty objective value: ", penalty_coeff.value * penalty_objective.value
    #     objective += penalty_coeff*penalty_objective
    #     print "\tpenalty objective: ",penalty_coeff.value*penalty_objective.value

    #     trust_box_sum = 0
    #     for x in self.xs:
    #         trust_box_sum += cvx.norm(x-x.value,1)
    #     linear_constraints += [trust_box_sum <= trust_box_size]

    #     return objective, linear_constraints


    # def add_eq_cntr(self, var, b):
    #     # x = var.grb_vars
    #     x = var
    #     for i in range(len(x)):
    #         self.model.addConstr(var[i] == b[i])

    # def add_lin_eq_cntr(self, var, A_eq, b_eq):
    #     rows, cols = A_eq.shape
    #     # x = var.grb_vars
    #     x = var
    #     for i in range(rows):
    #         expr = grb.LinExpr()
    #         for j in range(cols):
    #             if A_eq[i,j] != 0:
    #                 expr += A_eq[i,j]*x[j]
    #         self.model.addConstr(expr == b_eq[i])

    # def add_lin_geq_cntr(self, var, A_ineq, b_ineq):
    #     rows, cols = A_ineq.shape
    #     # x = var.grb_vars
    #     x = var
    #     for i in range(rows):
    #         expr = grb.LinExpr()
    #         for j in range(cols):
    #             if A_ineq[i,j] != 0:
    #                 expr += A_ineq[i,j]*x[j]
    #         self.model.addConstr(expr <= b_ineq[i])

    def add_trust_region(self, trust_region_size):
        if self.trust_region_cnt is not None:
            self.model.remove(self.trust_region_cnt)
        var_list = [grb_var for var in self.vars for grb_var in var.grb_vars.flatten()]        
        val_list = [val for var in self.vars for val in var.value.flatten()]        
        self.trust_region_cnt = self.add_trust_region_cnt(var_list, val_list, trust_region_size)

    def add_trust_region_cnt(self, x, xp, trust_box_size):
        expr = grb.LinExpr()
        rows = len(x)
        # expr = grb.quicksum(addAbs(grb.LinExpr(x[i]-xp[i])))
        for i in range(rows):
            expr += abs(self.model, grb.LinExpr(x[i]-xp[i]))
        return self.model.addConstr(expr <= trust_box_size)

    def l2_norm_squared(self, model, var):
        obj = grb.QuadExpr()
        x = var.grb_vars
        value = var.value
        rows, cols = x.shape
        for i in range(rows):
            for j in range(cols):
                obj += x[i,j]*x[i,j] - 2*value[i,j]*x[i,j] + value[i,j]*value[i,j]
        # size = len(x)
        # for i in range(size):
        #     obj += x[i]*x[i] - 2*value[i]*x[i] + value[i]*value[i]
        return obj
