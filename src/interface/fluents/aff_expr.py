import numpy as np
from collections import Iterable
from opt.variable import Variable


class AffExpr(dict):

    """
    Affine Expression is represented as a dictionary where the keys are
    HLParams and the values are coefficients"
    """

    def __init__(self, e=None, constant=0, name=None):
        self.name = name
        self.constant = constant

        if e is None:
            e = {}
        if isinstance(e, dict):
            super(AffExpr, self).__init__(list(e.items()))
        elif isinstance(e, Iterable):
            super(AffExpr, self).__init__(e)

    def value(self):
        # creates a copy if self.constant is an numpy array and works with ints
        s = 0.0 + self.constant
        for v, x in self.items():
            if v.get_value() is None:
                return None
            s += np.dot(v.get_value(), x)
        return s

    def to_gurobi_expr(self, model):
        expr = 0.0 + self.constant
        variables = []

        for v, x in self.items():
            var = Variable(model, v)
            variables.append(var)
            expr += np.dot(var.get_grb_vars(), x)
        return expr, variables
