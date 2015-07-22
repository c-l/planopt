from fluent import Fluent
import cvxpy as cvx
from opt.constraints import Constraints

class ObjAt(Fluent):
    def __init__(self, hl_action, obj, loc, obj_traj):
        self.hl_action = hl_action
        self.obj = obj
        self.loc = loc
        self.obj_traj = obj_traj

    # TODO: K currently depends on the robot's degrees of freedom when it shouldn't
    def precondition(self):
        K = self.hl_action.K
        linear_constraints = [self.obj_traj[:K] == self.loc] 
        return Constraints(linear_constraints, None, None)

    def postcondition(self):
        K = self.hl_action.K
        linear_constraints = [self.obj_traj[-K:] == self.loc] 
        return Constraints(linear_constraints, None, None)
