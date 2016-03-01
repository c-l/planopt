import numpy as np
import gurobipy as grb
from ipdb import set_trace
from opt.ops import abs
GRB = grb.GRB


class Constraints(object):

    def __init__(self, prob, g=None, h=None):
        self.prob = prob
        self.model = prob.get_model()
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
        self.fluents_gs = []
        self.fluents_hs = []

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
                constraints.append(self.model.addConstr(x[i, j], GRB.EQUAL, b[i, j]))
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
                constraints.append(self.model.addConstr(x[i, j], GRB.LESS_EQUAL, b[i, j]))
        return constraints

    def add_geq_cntr(self, var, b):
        # x = var.grb_vars
        constraints = []
        x = var
        rows, cols = var.shape
        for i in range(rows):
            for j in range(cols):
                constraints.append(self.model.addConstr(x[i, j], GRB.GREATER_EQUAL, b[i, j]))
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

    def add_nonlinear_ineq_constraint(self, fluent):
        self.fluents_gs.append(fluent)
        self.gs.append(fluent.fn)

    def add_nonlinear_eq_constraint(self, fluent):
        self.fluents_hs.append(fluent)
        self.hs.append(fluent.fn)

    def hinge_val(self, g):
        gval = g.val()
        return np.sum(np.maximum(np.zeros(gval.shape), gval))

    def abs_val(self, h):
        return np.sum(np.absolute(h.val()))

    def satisfied(self, tolerance):
        for g in self.gs:
            if self.hinge_val(g) > tolerance:
                return False
        for h in self.hs:
            if self.abs_val(h) > tolerance:
                return False
        return True

    def val_lst(self, start_ind, param_to_inds, constr_inds_to_params, constr_inds_to_fluent):
        val = []
        i = start_ind
        for fluent, g in zip(self.fluents_gs, self.gs):
            assert fluent.fn == g
            constr_inds_to_fluent[i] = fluent
            val.append(self.hinge_val(g))
            for param in g.params:
                if param.is_var:
                    param_to_inds.setdefault(param, set()).add(i)
                    constr_inds_to_params.setdefault(i, set()).add(param)
            i += 1
        for fluent, h in zip(self.fluents_hs, self.hs):
            assert fluent.fn == h
            constr_inds_to_fluent[i] = fluent
            val.append(self.abs_val(h))
            for param in h.params:
                if param.is_var:
                    param_to_inds.setdefault(param, set()).add(i)
                    constr_inds_to_params.setdefault(i, set()).add(param)
            i += 1
        return val

    def val_old(self):
        val = 0
        for g in self.gs:
            val += self.hinge_val(g)
        for h in self.hs:
            val += self.abs_val(h)
        return val

    # @profile
    def convexify(self, model, penalty_coeff, batch=True):
        all_exprs = []
        if not batch:
            for g in self.gs: # non-linear inequality constraints
                g_convexified = g.convexify()
                exprlist = self.add_hinges(model, g_convexified)
                all_exprs.append(penalty_coeff * grb.quicksum(exprlist))

        if batch:
            l_unhinged_affexprs_l = []
            for g in self.gs:
                l_unhinged_affexprs_l.append(g.convexify())
            # set_trace()
            l_hinge_affexpr_l = self.add_hinges_batch(model, l_unhinged_affexprs_l)
            for hinge_affexpr_l in l_hinge_affexpr_l:
                all_exprs.append(penalty_coeff * grb.quicksum(hinge_affexpr_l))
            # set_trace()

        for h in self.hs: # non-linear equality constraints
            exprlist = self.l1_norm(model, h.convexify())
            all_exprs.append(penalty_coeff * grb.quicksum(exprlist))
        return all_exprs

    # @profile
    def add_hinges_batch(self, model, l_affexpr_l):
        l_hinge_l = []
        for affexpr_l in l_affexpr_l:
            hinge_l = []
            for affexpr in affexpr_l:
                hinge_l.append(model.addVar(lb=0, ub=GRB.INFINITY, name='hinge'))
            l_hinge_l.append(hinge_l)
        model.update()

        l_expr_l = []
        for affexpr_l, hinge_l in zip(l_affexpr_l, l_hinge_l):
            expr_l = []
            for affexpr, hinge in zip(affexpr_l, hinge_l):
                cntr = model.addConstr(affexpr <= hinge)
                expr_l.append(grb.LinExpr(hinge))
                if self.temp is not None:
                    self.temp.extend([hinge, cntr])
            l_expr_l.append(expr_l)
        return l_expr_l

    # @profile
    def add_hinges(self, model, affexprlist):
        hinges = []
        for affexpr in affexprlist:
            hinges.append(model.addVar(lb=0, ub=GRB.INFINITY, name='hinge'))
        model.update()

        exprlist = []
        for affexpr, hinge in zip(affexprlist, hinges):
            cntr = model.addConstr(affexpr <= hinge)
            exprlist.append(grb.LinExpr(hinge))
            if self.temp is not None:
                self.temp.extend([hinge, cntr])
        return exprlist
        # exprlist = [self.add_hinge(model, affexpr, temp=self.temp)
        #             for affexpr in affexprlist]
        # return exprlist

    # @profile
    def add_hinge(self, model, affexpr, temp=None):
        hinge = model.addVar(lb=0, ub=GRB.INFINITY, name='hinge')
        model.update()  # this may be really slow, need to change this
        cntr = model.addConstr(affexpr <= hinge)
        expr = grb.LinExpr(hinge)
        if temp is not None:
            temp.extend([hinge, cntr])
        return expr

    # @profile
    def l1_norm(self, model, affexprlist):
        return [abs(model, affexpr, temp=self.temp) for affexpr in affexprlist]
