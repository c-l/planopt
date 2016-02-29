# import cvxpy as cvx
import numpy as np
import gurobipy as grb
from ipdb import set_trace
GRB = grb.GRB

RO = 10

class Variable(object):

    def __init__(self, hl_param, name=None, recently_sampled=False):
        self.hl_param = hl_param
        self.is_resampled = hl_param.is_resampled
        self.rows = hl_param.rows
        self.cols = hl_param.cols
        self.set(hl_param.value)
        self.recently_sampled = recently_sampled
        if name is None:
            self.name = hl_param.name
        else:
            self.name = name
        self.set(hl_param.value)
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
        value = np.zeros((self.rows, self.cols))
        grb_var = self.opt_prob_to_grb_var[prob]
        self.set(self._get_grb_val(grb_var))
        self.update_hl_param()

    def update_hl_param(self):
        assert self.value.shape == (self.rows, self.cols)
        self.hl_param.set(self.value.copy())

    def save(self):
        self.saved_value = self.value.copy()

    def restore(self):
        self.value = self.saved_value.copy()

# global variable in consensus ADMM
class GlobalVariable(Variable):
    def __init__(self, hl_param, ro=RO, name=None, recently_sampled=False):
        super(GlobalVariable, self).__init__(hl_param, name=name, recently_sampled=recently_sampled)
        self.ro = ro
        self.opt_prob_to_dual_vals = {}

    def add_opt_prob(self, opt_prob):
        super(GlobalVariable, self).add_opt_prob(opt_prob)
        self.opt_prob_to_dual_vals[opt_prob] = np.zeros((self.rows, self.cols))

    # using equation in Boyd's ADMM paper on page 49 in consensus ADMM
    def dual_update(self):
        value = np.zeros((self.rows, self.cols))
        # computing z^k+1 for consensus ADMM
        for opt_prob, grb_var in self.opt_prob_to_grb_var.keys():
            set_trace()
            value += _get_grb_val(grb_var) + opt_prob_to_dual_vals[opt_prob] / self.ro
        # value is z^(k+1)
        value = value / len(opt_prob_to_grb_var)

        # computing y^k+1 for consensus ADMM
        for opt_prob, grb_var  in self.opt_prob_to_grb_var.keys():
            set_trace()
            opt_prob_to_dual_vals[opt_prob] = opt_prob_to_dual_vals[opt_prob] + \
                                                  self.ro*(_get_grb_val(grb_var) - value)
        self.set(value)

    def get_dual_costs(self, opt_prob):
        if opt_prob not in opt_prob_to_grb_var:
            return 0
        dual_val = opt_prob_to_dual_vals[opt_prob]
        var = opt_prob_to_grb_var[opt_prob]
        return np.dot(dual_val.T, var)[0,0] + self.ro/2*opt_prob.l2_norm_squared(opt_prob.model, var, self.value)

    def update(self):
        raise NotImplementedError

class Constant(object):

    def __init__(self, hl_param, name=None):
        assert hl_param.value is not None
        self.hl_param = hl_param
        self.rows = hl_param.rows
        self.cols = hl_param.cols
        self.set(hl_param.value)
        # self.grb_vars = hl_param.value
        self.name = name

    def set(self, value):
        assert value.shape == (self.rows, self.cols)
        self.value = value.copy()

    def get_grb_vars(self, opt_prob=None):
        return self.value.copy()

    def update_hl_param(self):
        pass

    def update(self):
        pass

    def save(self):
        pass

    def restore(self):
        pass
