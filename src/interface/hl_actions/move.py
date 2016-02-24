import numpy as np
from opt.constraints import Constraints
from opt.function import QuadFn
from openravepy import *
from hl_action import HLAction
from interface.hl_param import Traj

# from interface.fluents.is_mp import IsMP
# from interface.fluents.in_manip import InManip
from interface.fluents.robot_at import RobotAt
# from interface.fluents.not_obstructs import NotObstructs
from interface.fluents.for_all_not_obstructs import ForAllNotObstructs
from interface.fluents.is_mp import IsMP
from interface.fluents.in_manip import InManip
from utils import *


class Move(HLAction):

    def __init__(self, lineno, hl_plan, env, world_state, robot, start, end, obj=None, gp=None):
        super(Move, self).__init__(lineno, hl_plan, env, robot)

        self.world_state = world_state
        self.start = start
        self.end = end
        self.obj = obj

        # TODO: set this using optimization domain
        self.T = 40
        self.K = 3
        T = self.T
        K = self.K
        KT = self.K*self.T

        self.name = "move" + str(lineno)
        self.traj = Traj(self, self.name + "_traj", 3, 40, is_var=True)

        self.params = [start, end, self.traj]
        self.preconditions = [RobotAt(self, 0, start, self.traj)]
        self.preconditions += [IsMP(self, 0, start, end, self.traj)]

        if obj is None:
            objs = hl_plan.get_all_but_params([self.robot])
            self.preconditions += [ForAllNotObstructs(env, world_state, self, robot, 1, self.traj, objs)]
        else:
            assert gp is not None
            objs = hl_plan.get_all_but_params([self.robot, self.obj])
            self.preconditions += [ForAllNotObstructs(env, world_state, self, robot, 1, self.traj, objs)]

            self.obj_traj = Traj(self, self.name + "_objtraj", 3, 40, is_var=True)
            self.preconditions += [ForAllNotObstructs(env, world_state, self, self.obj, 1, self.obj_traj, objs)]
            self.preconditions += [InManip(self, 0, obj, gp, self.traj, self.obj_traj)]
            self.params += [gp, self.obj_traj]
        # self.create_robot_clones()

        self.postconditions = []
        self.postconditions += [RobotAt(self, 0, self.end, self.traj)]

        # setting trajopt objective
        v = -1 * np.ones((KT - K, 1))
        d = np.vstack((np.ones((KT - K, 1)), np.zeros((K, 1))))
        # [:,0] allows numpy to see v and d as one-dimensional so
        # that numpy will create a diagonal matrix with v and d as a diagonal
        P = np.diag(v[:, 0], K) + np.diag(d[:, 0])
        Q = 2 * np.dot(np.transpose(P), P)

        self.cost = QuadFn(self.traj, Q)

    def plot_consensus_pos(self):
        start = np.array(self.start.value)
        start[2] = 1
        hl_start = np.array(self.hl_start.value)
        hl_start[2] = 1
        if not np.allclose(start, hl_start, atol=1e-3):
            self.handles += [self.hl_plan.env.drawarrow(
                p1=start, p2=hl_start, linewidth=0.01, color=(1, 0, 0))]

        end = np.array(self.end.value)
        end[2] = 1
        hl_end = np.array(self.hl_end.value)
        hl_end[2] = 1
        if not np.allclose(end, hl_end, atol=1e-3):
            self.handles += [self.hl_plan.env.drawarrow(
                p1=end, p2=hl_end, linewidth=0.01, color=(1, 0, 0))]

    def plot_consensus_place_obj(self):
        # if self.name == 'move6':
        # import ipdb; ipdb.set_trace() # BREAKPOINT
        for loc, hl_loc in zip(self.place_locs, self.hl_place_locs):
            plot_loc = np.array(loc.value)
            plot_hl_loc = np.array(hl_loc.value)
            plot_loc[2] = 1
            plot_hl_loc[2] = 1
            if not np.allclose(plot_loc, plot_hl_loc, atol=1e-3):
                self.handles += [self.hl_plan.env.drawarrow(
                    p1=plot_loc, p2=plot_hl_loc, linewidth=0.01, color=(1, 0, 0))]
            self.handles += [self.hl_plan.env.plot3(
                points=plot_hl_loc[:, 0], pointsize=10, colors=(1, 0, 0))]
