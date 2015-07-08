from fluent import Fluent
import cvxpy as cvx
from numpy.linalg import norm
import numpy as np

class InManip(Fluent):
    def __init__(self, obj, gp, traj):
        self.obj = obj
        self.gp = gp
        self.traj = traj

    def postcondition(self):
        obj_pos = Fluent.get_object_loc(self.obj)
        # linear_constraints = [self.traj[:,-1] - self.gp == obj_pos, cvx.norm(self.gp,2) == 1.26] 
        linear_constraints = [self.traj[:,-1] - self.gp == obj_pos]
        h = lambda x: np.matrix(norm(x[-3:]-obj_pos) - 1.26)
        return (linear_constraints, None, h)


