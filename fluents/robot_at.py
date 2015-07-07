from fluent import Fluent
import cvxpy as cvx

class RobotAt(Fluent):
    def __init__(self, pos, traj):
        self.pos = pos
        self.traj = traj

    def precondition(self):
        linear_constraints = [self.traj[:,0] == self.pos] 
        return (linear_constraints, None, None)

    def postcondition(self):
        linear_constraints = [self.traj[:,-1] == self.pos] 
        return (linear_constraints, None, None)
