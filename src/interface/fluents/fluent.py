import openravepy
import numpy as np
from opt.constraints import Constraints

class Fluent(object):
    def __init__(self, env, hl_action, model):
        self.env = env
        self.hl_action = hl_action
        self.constraints = Constraints(model)
        self.model = model
        # self.tolerance = 1e-3
        self.tolerance = 1e-2

    def satisfied(self):
        return self.constraints.satisfied(self.tolerance)
        # if self.constraints is None:
        #     return True
        # else:
        #     return self.constraints.constraints_satisfied(self.tolerance)

    def precondition(self):
        raise NotImplementedError

    def postcondition(self):
        return self.constraints

    @staticmethod
    def get_object_loc(obj_kinbody):
        transform = obj_kinbody.GetTransform()
        angle = openravepy.axisAngleFromRotationMatrix(transform[:3, :3])[2]
        print "double check angle is in radians"
        x = transform[0,3]
        y = transform[1,3]
        loc = np.matrix([[x], [y], [angle]])

        return loc



