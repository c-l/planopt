# import cvxpy as cvx
import numpy as np
from numpy.linalg import norm
import gurobipy as grb
from ipdb import set_trace
import itertools
GRB = grb.GRB

RO_INIT = 3

class Variable(object):

    def __init__(self, hl_param, name=None, recently_sampled=False):
        self.hl_param = hl_param
        self.is_resampled = hl_param.is_resampled
        self.shape = hl_param.shape
        self.set_val(hl_param.get_value())
        self.recently_sampled = recently_sampled
        if name is None:
            self.name = hl_param.name
        else:
            self.name = name
        self.set_val(hl_param.get_value())
        self.opt_prob_to_grb_var = {}

    def add_opt_prob(self, opt_prob):
        model = opt_prob.model

        # Create a numpy array of gurobi variables with the proper shape
        indices = itertools.product(*map(xrange, self.shape)) # cross product
        num_cells = np.product(self.shape)
        def idx_to_gurobi_var(index, name, model):
            return model.addVar(lb=-1*GRB.INFINITY, ub=GRB.INFINITY,
                         name=name+str(index))
        grb_var = np.array(map(idx_to_gurobi_var,  
                               indices,
                               itertools.repeat(self.name, num_cells),
                               itertools.repeat(model, num_cells)
                              )
                          ).reshape(self.shape)

        self.opt_prob_to_grb_var[opt_prob] = grb_var

    def get_grb_vars(self, opt_prob):
        assert opt_prob is not None
        return self.opt_prob_to_grb_var[opt_prob].copy()

    def _get_grb_val(self, grb_var):
        value = np.vectorize(lambda x: x.X)(grb_var)
        return value

    def set_val(self, value):
        if value is None:
            self._value = None
        else:
            assert value.shape == self.shape
            self._value = value.copy()

    def get_val(self):
        return self._value

    def update(self, prob):
        assert prob is not None
        assert len(self.opt_prob_to_grb_var) == 1

        # if gurobi solution is not optimal
        if prob.model.status != 2:
            print self.name, " didn't get updated."
            return
        grb_var = self.opt_prob_to_grb_var[prob]
        self.set_val(self._get_grb_val(grb_var))
        self.update_hl_param()

    def update_hl_param(self):
        assert self._value.shape == self.shape
        self.hl_param.set_value(self._value)

    def save(self):
        self.saved_value = self._value.copy()

    def restore(self):
        self._value = self.saved_value.copy()

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
        assert self.shape[1] == 1
        self.ro = ro
        self.opt_prob_to_dual_vals = {}

    def add_opt_prob(self, opt_prob):
        super(GlobalVariable, self).add_opt_prob(opt_prob)
        self.opt_prob_to_dual_vals[opt_prob] = np.zeros(self.shape)

    # using equation in Boyd's ADMM paper on page 49 in consensus ADMM
    def dual_update(self):
        x_k_bar = self._value.copy()
        x_k1_bar = np.zeros(self.shape)
        # computing z^k+1 for consensus ADMM
        for grb_var in self.opt_prob_to_grb_var._values():
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
        self.set_val(x_k1_bar)

        return primal_res, dual_res

    def reset_dual_vars(self):
        self.ro = RO_INIT
        for prob in self.opt_prob_to_dual_vals.keys():
            self.opt_prob_to_dual_vals[prob] = np.zeros(self.shape)

    def get_dual_cost(self, opt_prob):
        if opt_prob not in self.opt_prob_to_grb_var:
            print "get dual cost should only be called on variables that are in opt_prob"
            set_trace()
        dual_val = self.opt_prob_to_dual_vals[opt_prob]
        var = self.opt_prob_to_grb_var[opt_prob]
        return np.dot(dual_val.T, var)[0,0] + self.ro/2*opt_prob.l2_norm_squared(opt_prob.model, self, self._value.copy())

    def update(self, prob):
        pass
        # raise NotImplementedError

class Constant(object):

    def __init__(self, hl_param, name=None):
        assert hl_param.get_value() is not None
        self.hl_param = hl_param
        self.shape = hl_param.shape
        self.set_val(hl_param.get_value())
        # self.grb_vars = hl_param.get_value()
        self.name = name

    def set_val(self, value):
        assert value.shape == self.shape
        self._value = value.copy()

    def get_val(self):
        return self._value

    def get_grb_vars(self, opt_prob=None):
        return self._value.copy()

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
