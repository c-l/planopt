import numpy as np

class HLAction(object):
    def __init__(self):
        # optimization sqp info
        self.constraints = []
        self.g = lambda x: np.zeros((1,1))
        self.h = lambda x: np.zeros((1,1))

        # list of variables
        # list of precondition fluents
        self.preconditions = []

        # list of effect fluents
        self.postconditions = []

    def add_fluents_to_opt_prob(self):
        for precondition in self.preconditions:
            self.add_precondition(precondition)
        for postcondition in self.postconditions:
            self.add_postcondition(postcondition)

    def add_postcondition(self, fluent):
        constraints, g, h = fluent.postcondition()
        self.add_opt_info(constraints, g, h)

    def add_precondition(self, fluent):
        constraints, g, h = fluent.precondition()
        self.add_opt_info(constraints, g, h)

    def add_opt_info(self, constraints, g, h):
        self.constraints += constraints

        # fix nested f g and hs? Need to think about how to write this
        # TODO: Fix current implementation
        if g is not None:
            # self.g = lambda x: np.vstack((self.g(x), g(x)))
            self.g = lambda x: g(x)
        if h is not None:
            self.h = lambda x: np.vstack((self.h(x), h(x)))
