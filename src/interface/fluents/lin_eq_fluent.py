import numpy as np


class LinEqFluent(object):

    def __init__(self):
        self.name
        self.rhs = None
        self.lhs = None

    def satisfied(self):
        if self.rhs is None or self.lhs is None:
            return False
        if self.rhs.value is None or self.lhs.value is None:
            return False
        return np.isclose(self.rhs.value, self.lhs.value)
