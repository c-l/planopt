# import cvxpy as cvx
import numpy as np
import gurobipy as grb
GRB = grb.GRB


class Variable(object):

    def __init__(self, model, hl_param, name=None):
        self.model = model
        self.hl_param = hl_param
        self.rows = hl_param.rows
        self.cols = hl_param.cols
        self.set(hl_param.value)
        if name is None:
            self.name = hl_param.name
        else:
            self.name = name

        self.grb_vars = []
        for i in range(self.rows):
            row = [model.addVar(lb=-1 * GRB.INFINITY, ub=GRB.INFINITY,
                                name=self.name + str(i) + "," + str(j))
                   for j in range(self.cols)]
            self.grb_vars.append(row)
        self.grb_vars = np.array(self.grb_vars)

    def get_grb_vars(self):
        return self.grb_vars.copy()

    def set(self, value):
        if value is None:
            self.value = None
        else:
            assert value.shape == (self.rows, self.cols)
            self.value = value.copy()

    def update(self):
        if self.value is None:
            self.value = []
            for i in range(self.rows):
                row = []
                for j in range(self.cols):
                    row.append(self.grb_vars[i, j].X)
                self.value.append(row)
            self.value = np.array(self.value, dtype=np.float)
            for i in range(self.rows):
                for j in range(self.cols):
                    assert self.value[i, j] == self.grb_vars[i, j].X
        else:
            for i in range(self.rows):
                for j in range(self.cols):
                    self.value[i, j] = self.grb_vars[i, j].X
            for i in range(self.rows):
                for j in range(self.cols):
                    assert self.value[i, j] == self.grb_vars[i, j].X

    def update_hl_param(self):
        self.hl_param.value = self.value.copy()

    def save(self):
        self.saved_value = self.value.copy()

    def restore(self):
        self.value = self.saved_value.copy()


class Constant(object):

    def __init__(self, hl_param, name=None):
        assert hl_param.value is not None
        self.hl_param = hl_param
        self.rows = hl_param.rows
        self.cols = hl_param.cols
        self.set(hl_param.value)
        self.grb_vars = hl_param.value
        self.name = name

    def set(self, value):
        assert value.shape == (self.rows, self.cols)
        self.value = value.copy()

    def update(self):
        pass

    def save(self):
        pass

    def restore(self):
        pass
