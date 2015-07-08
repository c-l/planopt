import cvxpy as cvx
import openravepy
import numpy as np

class Fluent(object):
    def __init__(self, env):
        self.env = env

    def precondition(self):
        linear_constraints = []
        func_eq_constraint = None
        func_ineq_constraint = None
        return (linear_constraints, func_eq_constraint, func_ineq_constraint)

    def postcondition(self):
        linear_constraints = []
        func_eq_constraint = None
        func_ineq_constraint = None
        return (linear_constraints, func_eq_constraint, func_ineq_constraint)

    @staticmethod
    def get_object_loc(obj_kinbody):
        transform = obj_kinbody.GetTransform()
        angle = openravepy.axisAngleFromRotationMatrix(transform[:3, :3])[2]
        print "double check angle is in radians"
        x = transform[0,3]
        y = transform[1,3]
        loc = np.matrix([[x], [y], [angle]])

        return loc



