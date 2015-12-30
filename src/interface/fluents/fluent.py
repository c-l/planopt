import numpy as np


class Fluent(object):
    tol = 3e-3

    def __init__(self, name, priority):
        self.name = name
        self.priority = priority

    def satisfied(self):
        raise NotImplementedError


class AndFluent(Fluent):

    def __init__(self, name, priority):
        super(AndFluent, self).__init__(name, prioriy)
        self.fluents = None

    def satisfied(self):
        for fluent in self.fluents:
            if not np.all(fluent.satisfied()):
                return False
        return True


class LinFluent(Fluent):
    def __init__(self, name, priority, lhs, rhs):
        super(LinFluent, self).__init__(name, priority)
        self.lhs = lhs
        self.rhs = rhs

    def satisfied(self):
        raise NotImplementedError


class LinEqFluent(LinFluent):

    def satisfied(self):
        if self.rhs is None or self.lhs is None:
            return False
        if self.rhs.value() is None or self.lhs.value() is None:
            return False
        return np.allclose(self.lhs.value(), self.rhs.value())


class LinLEFluent(LinFluent):

    def satisfied(self):
        if self.rhs is None or self.lhs is None:
            return False
        if self.rhs.value() is None or self.lhs.value() is None:
            return False
        return np.all(self.lhs.value() <= self.rhs.value())


class FnFluent(Fluent):

    def __init__(self, name, priority, fn = None):
        super(FnFluent, self).__init__(name, priority)
        self.fn = None

    def satisfied(self):
        raise NotImplementedError


class FnEQFluent(FnFluent):

    def satisfied(self, tol=Fluent.tol):
        return np.isclose(self.fn.val(), 0.0, atol=tol)


class FnLEFluent(FnFluent):

    def satisfied(self, tol=Fluent.tol):
        return np.all(self.fn.val() <= tol)
