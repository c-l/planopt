from fluent import Fluent
from opt.constraints import Constraints

class RobotAt(Fluent):
    def __init__(self, env, hl_action, model, pos, traj):
        super(RobotAt, self).__init__(env, hl_action, model)
        self.pos = pos.grb_vars
        self.traj = traj.grb_vars
        self.hl_action = hl_action
        self.name = "RobotAt"

    def precondition(self):
        K = self.hl_action.K
        self.constraints.add_eq_cntr(self.traj[:K], self.pos)
        return self.constraints

    def postcondition(self):
        K = self.hl_action.K
        self.constraints.add_eq_cntr(self.traj[-K:], self.pos)
        return self.constraints
