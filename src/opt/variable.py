# import cvxpy as cvx
import numpy as np
from numpy.linalg import norm
import gurobipy as grb
from ipdb import set_trace
GRB = grb.GRB

RO_INIT = 3

class Variable(object):

    def __init__(self, hl_param, name=None, recently_sampled=False):
        self.hl_param = hl_param
        self.is_resampled = hl_param.is_resampled
        self.rows = hl_param.rows
        self.cols = hl_param.cols
        self.set(hl_param.get_value())
        self.recently_sampled = recently_sampled
        if name is None:
            self.name = hl_param.name
        else:
            self.name = name
        self.set(hl_param.get_value())
        self.opt_prob_to_grb_var = {}

    def add_opt_prob(self, opt_prob):
        model = opt_prob.model
        grb_var = []
        for i in range(self.rows):
            row = [model.addVar(lb=-1 * GRB.INFINITY, ub=GRB.INFINITY,
                                name=self.name + str(i) + "," + str(j))
                   for j in range(self.cols)]
            grb_var.append(row)
        grb_var = np.array(grb_var)
        self.opt_prob_to_grb_var[opt_prob] = grb_var

    def get_grb_vars(self, opt_prob=None):
        assert opt_prob is not None
        return self.opt_prob_to_grb_var[opt_prob].copy()

    def _get_grb_val(self, grb_var):
        value = np.zeros((self.rows, self.cols), dtype=np.float)
        for i in range(self.rows):
            for j in range(self.cols):
                value[i,j] = grb_var[i,j].X
                assert value[i,j] == grb_var[i,j].X
        return value

    def set(self, value):
        if value is None:
            self.value = None
        else:
            assert value.shape == (self.rows, self.cols)
            self.value = value.copy()

    def update(self, prob):
        assert prob is not None
        assert len(self.opt_prob_to_grb_var) == 1

        # if gurobi solution is not optimal
        if prob.model.status != 2:
            print self.name, " didn't get updated."
            return
        value = np.zeros((self.rows, self.cols))
        grb_var = self.opt_prob_to_grb_var[prob]
        self.set(self._get_grb_val(grb_var))
        self.update_hl_param()

    def update_hl_param(self):
        assert self.value.shape == (self.rows, self.cols)
        self.hl_param.set_value(self.value)

    def save(self):
        self.saved_value = self.value.copy()

    def restore(self):
        self.value = self.saved_value.copy()

    def reset_dual_vars(self):
        pass
    # returning default residual values
    def dual_update(self):
        pass

    def get_dual_cost(self, opt_prob):
        if opt_prob not in self.opt_prob_to_grb_var:
            print "get dual cost should only be called on variables that are in opt_prob"
        return 0.0

# global variable in consensus ADMM
class GlobalVariable(Variable):
    def __init__(self, hl_param, ro=RO_INIT, name=None, recently_sampled=False):
        super(GlobalVariable, self).__init__(hl_param, name=name, recently_sampled=recently_sampled)
        assert self.cols == 1
        self.ro = ro
        self.opt_prob_to_dual_vals = {}

    def add_opt_prob(self, opt_prob):
        super(GlobalVariable, self).add_opt_prob(opt_prob)
        self.opt_prob_to_dual_vals[opt_prob] = np.zeros((self.rows, self.cols))

    # using equation in Boyd's ADMM paper on page 49 in consensus ADMM
    def dual_update(self):
        x_k_bar = self.value.copy()
        x_k1_bar = np.zeros((self.rows, self.cols))
        # computing z^k+1 for consensus ADMM
        for grb_var in self.opt_prob_to_grb_var.values():
            x_k1_bar += self._get_grb_val(grb_var)
        # value is z^(k+1)
        x_k1_bar = x_k1_bar / len(self.opt_prob_to_grb_var)

        # computing y^k+1 for consensus ADMM and r the primal residual
        primal_res = 0
        for opt_prob, grb_var  in self.opt_prob_to_grb_var.items():
            x_k1i = self._get_grb_val(grb_var)
            self.opt_prob_to_dual_vals[opt_prob] = self.opt_prob_to_dual_vals[opt_prob] + \
                                                  self.ro*(x_k1i - x_k1_bar)
            primal_res = primal_res + norm(x_k1i-x_k1_bar)**2

        # computer r the primal residual
        # compute s the dual residual
        dual_res = len(self.opt_prob_to_grb_var) * self.ro**2*norm(x_k1_bar-x_k_bar)**2
        self.set(x_k1_bar)

        return primal_res, dual_res

    def reset_dual_vars(self):
        self.ro = RO_INIT
        for prob in self.opt_prob_to_dual_vals.keys():
            self.opt_prob_to_dual_vals[prob] = np.zeros((self.rows, self.cols))

    def get_dual_cost(self, opt_prob):
        if opt_prob not in self.opt_prob_to_grb_var:
            print "get dual cost should only be called on variables that are in opt_prob"
            set_trace()
        dual_val = self.opt_prob_to_dual_vals[opt_prob]
        var = self.opt_prob_to_grb_var[opt_prob]
        return np.dot(dual_val.T, var)[0,0] + self.ro/2*opt_prob.l2_norm_squared(opt_prob.model, self, self.value.copy())

    def update(self, prob):
        pass
        # raise NotImplementedError

class Constant(object):

    def __init__(self, hl_param, name=None):
        assert hl_param.get_value() is not None
        self.hl_param = hl_param
        self.rows = hl_param.rows
        self.cols = hl_param.cols
        self.set(hl_param.get_value())
        # self.grb_vars = hl_param.get_value()
        self.name = name

    def set(self, value):
        assert value.shape == (self.rows, self.cols)
        self.value = value.copy()

    def get_grb_vars(self, opt_prob=None):
        return self.value.copy()

    def update_hl_param(self):
        pass

    def update(self, prob):
        pass

    def dual_update(self):
        pass

    def reset_dual_vars(self):
        pass

    def save(self):
        pass

    def restore(self):
        pass
