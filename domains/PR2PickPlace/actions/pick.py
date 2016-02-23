import sys
DOMAIN_PATH = "../domains/PR2PickPlace/"
sys.path.insert(0, DOMAIN_PATH)
from fluents.not_obstructs import NotObstructsPR2
from fluents.is_gp import PR2IsGP
from interface.fluents.robot_at import RobotAt
from actions.action import PR2HLAction
from interface.hl_param import HLParam, Traj, Robot, Obj, PR2

class PR2Pick(PR2HLAction):

    def __init__(self, lineno, hl_plan, env, robot, pos, obj, loc=None, gp=None):
        super(PR2Pick, self).__init__(lineno, hl_plan, env, robot)

        self.hl_plan = hl_plan
        self.pos = pos
        self.obj = obj

        # TODO: set this using optimization domain
        self.T = 1
        # self.K = 8
        self.K = 8
        T = self.T
        K = self.K
        KT = self.K*self.T

        self.name = "pick" + str(lineno)
        self.traj = Traj(self, self.name + "_traj", self.K, self.T, is_var=True)

        self.params = [pos, self.traj]
        self.preconditions = [RobotAt(self, 0, pos, self.traj)]
        self.preconditions += [PR2IsGP(self.env, self, robot, 0, obj, self.traj)]
        self.preconditions += [NotObstructsPR2(self.env, self, robot, 1, self.traj, self.obj)]

        # self.preconditions += [NotObstructsPR2(env, self, robot, 1, self.traj, obj)]

        self.postconditions = []
        # self.postconditions += [RobotAt(self, 0, self.end, self.traj)]

        self.cost = 0
        self.create_robot_clones()

    def plot(self):
        self.plot_traj_robot_kinbodies()
