from fluent import Fluent
import cvxpy as cvx
from opt.constraints import Constraints

class RobotAt(Fluent):
    def __init__(self, env, hl_action, pos, traj):
        super(RobotAt, self).__init__(env, hl_action)
        self.pos = pos
        self.traj = traj
        self.hl_action = hl_action
        self.constraints = None
        self.name = "RobotAt"

    def precondition(self):
        K = self.hl_action.K
        linear_constraints = [self.traj[:K] == self.pos] 
        self.constraints = Constraints(linear_constraints, None, None)
        return self.constraints

    def postcondition(self):
        K = self.hl_action.K
        linear_constraints = [self.traj[-K:] == self.pos] 
        self.constraints = Constraints(linear_constraints, None, None)
        return self.constraints
