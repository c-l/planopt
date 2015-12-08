class Fluent(object):

    def __init__(self, name):
        self.name = name

    def satisfied(self, tolerance=None):
        raise NotImplementedError
