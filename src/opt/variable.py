# import cvxpy as cvx
import numpy as np
import gurobipy as grb
GRB = grb.GRB

class Variable(object):
    def __init__(self, hl_param, model, rows, cols, value=None, name=None):
        self.hl_param = hl_param
        self.model = model
        # self.size = size
        self.rows = rows
        self.cols = cols
        self.value = value
        if name is None:
            self.name = ""
        else:
            self.name = name

        self.grb_vars = []
        for i in range(rows):
            row = [model.addVar(lb=-1*GRB.INFINITY, ub=GRB.INFINITY, name=self.name+str(i)+","+str(j)) for j in range(cols)]
            self.grb_vars.append(row)
        # self.grb_vars = [model.addVar(lb=-1*GRB.INFINITY, ub=GRB.INFINITY, name=self.name+str(i)) for i in range(size)]
        self.grb_vars = np.array(self.grb_vars)

        if value is None:
            self.value = np.zeros((rows,cols))
            self.initialized = False
        else:
            self.initialized = True

    def set(self, value):
        assert value.shape == (self.rows, self.cols)
        self.value = value.copy()

    def update(self):
        if self.value is None:
            self.value = []
            for i in range(self.rows):
                row = []
                for j in range(self.cols):
                    row.append(self.grb_vars[i,j].X)
                self.value.append(row)
            self.value = np.array(self.value, dtype=np.float)
            for i in range(self.rows):
                for j in range(self.cols):
                    assert self.value[i,j] == self.grb_vars[i,j].X
        else:
            for i in range(self.rows):
                for j in range(self.cols):
                    self.value[i,j] = self.grb_vars[i,j].X
            for i in range(self.rows):
                for j in range(self.cols):
                    assert self.value[i,j] == self.grb_vars[i,j].X


    def save(self):
        self.saved_value = self.value.copy()
    
    def restore(self):
        self.value = self.saved_value.copy()

class Constant(object):
    def __init__(self, hl_param, rows, cols, value=None, name=None):
        assert value is not None
        self.hl_param = hl_param
        self.rows = rows
        self.cols = cols
        self.set(value)
        self.grb_vars = value
        self.name = name
        self.initialized = True

    def set(self, value):
        assert value.shape == (self.rows, self.cols)
        self.value = value.copy()

    def update(self):
        pass

    def save(self):
        pass

    def restore(self):
        pass

    
