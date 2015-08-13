import numpy as np
import cvxpy as cvx
from opt.variable import Variable
from opt.constraints import Constraints
from numpy.linalg import norm

from utils import *

class HLParam(object):
    def __init__(self, name, rows, cols, is_var=True, value=None, ro=2):
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
        # self.consensus_var = Variable(rows, cols, name="hlvar_"_name, cur_value=self.consensus)
        self.ro = ro
        self.gen = None
        
    def get_eq_constraints(self):
        hl_var = Variable(self.rows, self.cols, name="hlvar_" + self.name, value=self.consensus.value)
        eq_constraints = []
        for var in self.hla_vars:
            eq_constraints += [hl_var == var]
        return Constraints(eq_constraints)

    def new_hla_var(self, hl_action):
        if not self.is_var:
            return self.consensus, self.consensus

        rows = self.rows
        cols = self.cols

        hla_var = Variable(rows, cols, name=self.name)
        hla_var.value = self.consensus.value

        dual_var = cvx.Parameter(rows, cols, value=np.zeros((rows,cols)))

        self.hlas.append(hl_action)
        self.hla_vars.append(hla_var)
        self.hla_dual_vars.append(dual_var)

        if self.ro != 2:
            import ipdb; ipdb.set_trace() # BREAKPOINT
        hl_action.add_dual_cost(hla_var, dual_var, self.consensus, ro=self.ro)
        return hla_var, self.consensus

    def initialize_to_consensus(self):
        if self.is_var is False:
            return
        for var in self.hla_vars:
            # if not var.initialized:
            var.value = self.consensus.value

    # @profile
    def dual_update(self):
        if self.is_var is False:
            return 0.0
        z = 0
        hla_vars = self.hla_vars  
        num_vars = 0
        for var in hla_vars:
            if var.initialized:
                num_vars += 1
                z += var.value
            else:
                print var.name(), " hasn't been initialized yet"
        if num_vars == 0:
            return 0

        z = z/num_vars
        self.consensus.value = z

        diff = 0
        # import ipdb; ipdb.set_trace()
        # print "z = {0}".format(z)
        # for i in range(num_vars):
        for var, dual_var in zip(self.hla_vars, self.hla_dual_vars):
            # xi = self.hla_vars[i].value
            if var.initialized:
                xi = var.value
                diff += norm(z - xi, 1)
                dual_var.value += self.ro*(xi - z)
            # print("dual var {0}: {1}".format(i, self.hla_dual_vars[i].value))
            # print("{0}'s {1} dual variable: {2}".format(self.hlas[i].name, self.name, self.hla_dual_vars[i].value))

        return diff
            
    def resample(self):
        if self.gen is None:
            self.gen= self.generator()
        self.consensus.value = next(self.gen)
        self.initialize_to_consensus()

    def reset(self):
        for var in self.hla_dual_vars:
            var.value = np.zeros((self.rows, self.cols))

    # def generator(self):
    #     if not self.is_var:
    #         yield self.consensus.value

class GP(HLParam):
    # grasp pose
    def __init__(self, name, rows, cols, is_var=True, value=None, ro=2):
        super(GP, self).__init__(name, rows, cols, is_var, value, ro)

        # self.consensus.value = np.array([[0],[1],[0]])
        # self.consensus.value = np.array([[1],[0],[0]])
        # self.consensus.value = np.array([[-1],[0],[0]])
        # self.consensus.value = np.array([[0],[-1],[0]])
        # self.consensus.value = np.array([[-1],[1],[0]])


    def generator(self):
        yield np.array([[1],[0],[0]])
        yield np.array([[-1],[0],[0]])
        yield np.array([[0],[1],[0]])
        # self.consensus.value = np.array([[-1],[0],[0]])
        # self.consensus.value = np.array([[0],[-1],[0]])
        # self.consensus.value = np.array([[-1],[1],[0]])

class RP(HLParam):
    # def __init__(self, name, rows, cols, is_var=True, value=None, ro=0.05):
    #     super(RP, self).__init__(name, rows, cols, is_var, value, ro)
    pass

class ObjLoc(HLParam):
    # object location
    # def __init__(self, name, rows, cols, is_var=True, value=None, ro=0.05):
    #     super(ObjLoc, self).__init__(name, rows, cols, is_var, value, ro)
    pass

class Movable(HLParam):
    def __init__(self, name):
        self.name = name
        self.is_var = False

    def initialize_to_consensus(self):
        return

    def new_hla_var(self, hl_actions, env):
        return env.GetKinBody(self.name), None
    
    def reset(self):
        return

class Traj(HLParam):
    # do not add in dual costs because this variable is local to one high level action
    def new_hla_var(self, hl_actions):
        if not self.is_var:
            return self.consensus, self.consensus

        rows = self.rows
        cols = self.cols

        hla_var = Variable(rows, cols, name=self.name)
        hla_var.value = self.consensus.value

        dual_var = cvx.Parameter(rows, cols, value=np.zeros((rows,cols)))

        self.hlas.append(hl_action)
        self.hla_vars.append(hla_var)
        self.hla_dual_vars.append(dual_var)

        # hl_action.add_dual_cost(hla_var, dual_var, self.consensus, ro=self.ro)
        return hla_var, self.consensus
