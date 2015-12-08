import numpy as np
# import cvxpy as cvx
import gurobipy as grb
GRB = grb.GRB


class Constraints(object):

    def __init__(self, model, g=None, h=None):
        self.model = model
        # linear_constraints = []
        # if linear_constraints is None:
        #     self.linear_constraints = []
        # else:
        #     self.linear_constraints = linear_constraints

        if g is None:
            self.gs = []  # non-linear inequality constraints
        else:
            self.gs = [g]

        if h is None:
            self.hs = []  # non-linear equality constraints
        else:
            self.hs = [h]

        self.temp = []

    def clean(self):
        for item in self.temp:
            self.model.remove(item)

    def add_eq_cntr(self, var, b):
        # x = var.grb_vars
        constraints = []
        x = var
        rows, cols = var.shape
        for i in range(rows):
            for j in range(cols):
                constraints.append(self.model.addConstr(x[i, j] == b[i, j]))
        # x = var.flatten()
        # b = b.flatten()
        # for i in range(len(x)):
        #     self.model.addConstr(x[i] == b[i])
        return constraints

    def add_leq_cntr(self, var, b):
        # x = var.grb_vars
        constraints = []
        x = var
        rows, cols = var.shape
        for i in range(rows):
            for j in range(cols):
                constraints.append(self.model.addConstr(x[i, j] <= b[i, j]))
        return constraints

    def add_geq_cntr(self, var, b):
        # x = var.grb_vars
        constraints = []
        x = var
        rows, cols = var.shape
        for i in range(rows):
            for j in range(cols):
                constraints.append(self.model.addConstr(x[i, j] >= b[i, j]))
        return constraints

    def add_lin_eq_cntr(self, var, A_eq, b_eq):
        # x = var.grb_vars
        x = var
        var = np.dot(A_eq, x)
        constraints = self.add_eq_cntr(var, b_eq)
        return constraints

    def add_lin_leq_cntr(self, var, A_ineq, b_ineq):
        # x = var.grb_vars
        x = var
        var = np.dot(A_ineq, x)
        constraints = self.add_leq_cntr(var, b_ineq)
        return constraints

    def add_lin_geq_cntr(self, var, A_ineq, b_ineq):
        # x = var.grb_vars
        x = var
        var = np.dot(A_ineq, x)
        constraints = self.add_geq_cntr(var, b_ineq)
        return constraints

    def add_nonlinear_ineq_constraint(self, g):
        self.gs.append(g)

    def add_nonlinear_eq_constraint(self, h):
        self.hs.append(h)

    def satisfied(self, tolerance):
        for g in self.gs:
            if g.val() > tolerance:
                return False
        for h in self.hs:
            if np.absolute(h.val()) > tolerance:
                return False
        return True

    def val(self):
        val = 0
        for g in self.gs:
            val += max(g.val(), 0)
        for h in self.hs:
            val += np.absolute(h.val())
        return val

    def convexify(self, model, penalty_coeff):
        penalty_obj = grb.LinExpr()
        for g in self.gs:  # non-linear inequality constraints
            hinges = self.add_hinges(model, g.convexify())
            exprlist = grb.quicksum([penalty_coeff * expr for expr in hinges])
            penalty_obj += exprlist
            # exprlist = penalty_coeff * self.add_hinges(model, g.convexify())
            # penalty_obj += grb.quicksum(exprlist)
        for h in self.hs:  # non-linear equality constraints
            exprlist = self.l1_norm(model, h.convexify())
            penalty_obj += grb.quicksum([penalty_coeff *
                                         expr for expr in exprlist])

        return penalty_obj

    def add_hinges(self, model, affexprlist):
        exprlist = [self.add_hinge(model, affexpr, temp=self.temp)
                    for affexpr in affexprlist]
        return exprlist

    def add_hinge(self, model, affexpr, temp=None):
        hinge = model.addVar(lb=0, ub=GRB.INFINITY, name='hinge')
        model.update()  # this may be really slow, need to change this
        cntr = model.addConstr(affexpr <= hinge)
        expr = grb.LinExpr(hinge)
        if temp is not None:
            temp.extend([hinge, cntr])
        return expr

    def l1_norm(self, model, affexprlist):
        return [abs(model, affexpr, temp=self.temp) for affexpr in affexprlist]
