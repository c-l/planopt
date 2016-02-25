import numpy as np


# TODO: should the fluent class be an exception?
class Fluent(Exception):
    tol = 3e-3

    do_extension = False
    def __init__(self, name, priority, hl_action):
        self.name = name
        self.priority = priority
        self.hl_action = hl_action

    def satisfied(self):
        raise NotImplementedError


class AndFluent(Fluent):

    def __init__(self, name, priority, hl_action):
        super(AndFluent, self).__init__(name, priority, hl_action)
        self.fluents = None

    def satisfied(self):
        for fluent in self.fluents:
            if not np.all(fluent.satisfied()):
                return False
        return True


class LinFluent(Fluent):
    def __init__(self, name, priority, hl_action, lhs, rhs):
        super(LinFluent, self).__init__(name, priority, hl_action)
        self.lhs = lhs
        self.rhs = rhs

    def satisfied(self):
        raise NotImplementedError


class LinEqFluent(LinFluent):

    def satisfied(self, tol=Fluent.tol):
        if self.rhs is None or self.lhs is None:
            return False
        if self.rhs.value() is None or self.lhs.value() is None:
            return False
        return np.allclose(self.lhs.value(), self.rhs.value(), atol=tol)


class LinLEFluent(LinFluent):

    def satisfied(self, tol=Fluent.tol):
        if self.rhs is None or self.lhs is None:
            return False
        if self.rhs.value() is None or self.lhs.value() is None:
            return False
        return np.all(self.lhs.value() <= self.rhs.value() + tol)


class FnFluent(Fluent):

    def __init__(self, name, priority, hl_action, fn = None):
        super(FnFluent, self).__init__(name, priority, hl_action)
        self.fn = None

    def satisfied(self):
        raise NotImplementedError


class FnEQFluent(FnFluent):

    def satisfied(self, tol=Fluent.tol):
        return np.isclose(self.fn.val(), 0.0, atol=tol)


class FnLEFluent(FnFluent):

    def satisfied(self, tol=Fluent.tol):
        return np.all(self.fn.val() <= tol)
