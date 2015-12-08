import numpy as np
from interface.fluents.fluent import Fluent


class LinLEFluent(Fluent):

    def __init__(self, name, lhs, rhs):
        super(LinLEFluent, self).__init__(name)
        self.lhs = lhs
        self.rhs = rhs

    def satisfied(self):
        if self.rhs is None or self.lhs is None:
            return False
        if self.rhs.value() is None or self.lhs.value() is None:
            return False
        return np.all(self.lhs.value() <= self.rhs.value())
