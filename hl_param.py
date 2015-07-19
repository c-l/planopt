import numpy as np
import cvxpy as cvx
from numpy.linalg import norm

class HLParam(object):
    def __init__(self, name, rows, cols, is_var=True, value=None, ro=0.05):
        self.name = name
        # hla stands for high level action
        self.hlas = []
        self.hla_vars = []
        self.hla_dual_vars = []
        self.rows = rows
        self.cols = cols

        self.is_var = is_var
        if is_var:
            if value is None:
                self.consensus = cvx.Parameter(rows, cols, name="hl_" + name, value=np.zeros((rows,cols)))
            else:
                self.consensus = cvx.Parameter(rows, cols, name="hl_" + name, value=value)
        else:
            assert value is not None
            self.consensus = cvx.Parameter(rows, cols, name="hl_" + name, value=value)
        self.ro = ro
        
    def new_hla_var(self, hl_action):
        if not self.is_var:
            return self.consensus, self.consensus

        rows = self.rows
        cols = self.cols

        hla_var = cvx.Variable(rows, cols, name=self.name)
        hla_var.value = self.consensus.value

        dual_var = cvx.Parameter(rows, cols, value=np.zeros((rows,cols)))

        self.hlas.append(hl_action)
        self.hla_vars.append(hla_var)
        self.hla_dual_vars.append(dual_var)

        hl_action.add_dual_cost(hla_var, dual_var, self.consensus, ro=self.ro)
        return hla_var, self.consensus

    # def add_dual(self, hl_action, var):
    #     rows = self.rows
    #     cols = self.cols
    #     assert var.size == (rows, cols)
    #     dual_var = cvx.Parameter(rows, cols, value=np.zeros((rows,cols)))
    #     self.add_action_var(hl_action.name, var, dual_var)
    #     hl_action.add_dual_cost(var, dual_var, self.consensus, ro=self.ro)

    # def add_action_var(self, hl_action_name, var, dual_var):
    #     self.hl_action_name.append(hl_action_name)
    #     self.hla_vars.append(var)
    #     self.hla_dual_vars.append(dual_var)

    # @profile
    def dual_update(self):
        z = 0
        hla_vars = self.hla_vars  
        num_vars = len(hla_vars)
        for var in hla_vars:
            z += var.value
        z = z/num_vars
        self.consensus.value = z

        diff = 0
        # import ipdb; ipdb.set_trace()
        # print "z = {0}".format(z)
        for i in range(num_vars):
            xi = self.hla_vars[i].value
            diff += norm(z - xi, 1)
            self.hla_dual_vars[i].value += self.ro*(xi - z)
            # print("dual var {0}: {1}".format(i, self.hla_dual_vars[i].value))
            # print("{0}'s {1} dual variable: {2}".format(self.hlas[i].name, self.name, self.hla_dual_vars[i].value))

        return diff
            



