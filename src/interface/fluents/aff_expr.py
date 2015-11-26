from collections import Iterable


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
        s = self.constant
        for v, x in self.items():
            if v.value is None:
                return None
            s += v.value * x
        return s
