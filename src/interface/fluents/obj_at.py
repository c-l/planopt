from fluent import Fluent
import cvxpy as cvx
from opt.constraints import Constraints

class ObjAt(Fluent):
    def __init__(self, env, hl_action, obj, loc, obj_traj):
        super(ObjAt, self).__init__(env, hl_action)
        self.obj = obj
        self.loc = loc
        self.obj_traj = obj_traj
        self.constraints = None
        self.name = "ObjAt"

    # TODO: K currently depends on the robot's degrees of freedom when it shouldn't
    def precondition(self):
        K = self.hl_action.K
        linear_constraints = [self.obj_traj[:K] == self.loc] 
        self.constraints = Constraints(linear_constraints, None, None)
        return self.constraints

    def postcondition(self):
        K = self.hl_action.K
        linear_constraints = [self.obj_traj[-K:] == self.loc] 
        self.constraints = Constraints(linear_constraints, None, None)
        return self.constraints
