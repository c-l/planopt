from fluent import Fluent
import cvxpy as cvx
from numpy.linalg import norm
import numpy as np

class InManip(Fluent):
    def __init__(self, hl_action, obj, gp, traj, obj_traj = None):
        self.hl_action = hl_action
        self.obj = obj
        self.gp = gp
        self.traj = traj
        self.obj_traj = obj_traj

    def precondition(self):
        K = self.hl_action.K
        T = self.hl_action.T
        gp_all_timesteps = self.gp
        linear_constraints = []
        for i in range(T):
            # gp_all_timesteps = cvx.vstack(gp_all_timesteps, self.gp)
            linear_constraints += [self.traj[K*i:K*(i+1)] + self.gp == self.obj_traj[K*i:K*(i+1)]]
        # linear_constraints = [self.traj - gp_all_timesteps == self.obj_traj]
        return (linear_constraints, None, None)

    def postcondition(self):
        # obj_pos = Fluent.get_object_loc(self.obj)
        # linear_constraints = [self.traj[:,-1] - self.gp == obj_pos, cvx.norm(self.gp,2) == 1.26] 
        K = self.hl_action.K
        linear_constraints = [self.traj[-K:] + self.gp == self.obj_traj[-K:]]
        # h = lambda x: np.matrix(norm(x[-K:]-obj_pos) - .55)
        h = lambda x: np.matrix(norm(x[-K:]-self.obj_traj[-K:].value) - .55)
        return (linear_constraints, None, h)


