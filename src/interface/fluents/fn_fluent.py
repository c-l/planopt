import numpy as np
from interface.fluents.fluent import Fluent


class FnFluent(Fluent):

    def __init__(self, name):
        super(FnFluent, self).__init__(name)

    def satisfied(self):
        return self.fn.val() <= 0
