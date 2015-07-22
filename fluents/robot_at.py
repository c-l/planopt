from fluent import Fluent
import cvxpy as cvx
from opt.constraints import Constraints

class RobotAt(Fluent):
    def __init__(self, hl_action, pos, traj):
        self.pos = pos
        self.traj = traj
        self.hl_action = hl_action

    def precondition(self):
        K = self.hl_action.K
        linear_constraints = [self.traj[:K] == self.pos] 
        return Constraints(linear_constraints, None, None)

    def postcondition(self):
        K = self.hl_action.K
        linear_constraints = [self.traj[-K:] == self.pos] 
        return Constraints(linear_constraints, None, None)
