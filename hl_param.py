import numpy as np
import cvxpy as cvx
from numpy.linalg import norm

class HLParam(object):
    def __init__(self, var_name, rows, cols, ro=0.05):
        self.var_name = var_name
        self.action_vars = []
        self.dual_vars = []
        self.rows = rows
        self.cols = cols
        self.consensus = cvx.Parameter(rows, cols, value=np.zeros((rows,cols)))
        self.ro = ro
        
    def add_dual(self, hl_action, var):
        rows = self.rows
        cols = self.cols
        assert var.size == (rows, cols)
        dual_var = cvx.Parameter(rows, cols, value=np.zeros((rows,cols)))
        self.add_action_var(var, dual_var)
        hl_action.add_dual_cost(var, dual_var, self.consensus, ro=self.ro)

    def add_action_var(self, var, dual_var):
        self.action_vars.append(var)
        self.dual_vars.append(dual_var)

    def dual_update(self):
        z = 0
        action_vars = self.action_vars  
        num_vars = len(action_vars)
        for var in action_vars:
            z += var.value
        z = z/num_vars
        self.consensus.value = z

        diff = 0
        # import ipdb; ipdb.set_trace()
        # print "z = {0}".format(z)
        for i in range(num_vars):
            xi = self.action_vars[i].value
            diff += norm(z - xi, 1)
            self.dual_vars[i].value += self.ro*(xi - z)
            print("dual var {0}: {1}".format(i, self.dual_vars[i].value))

        return diff
            




