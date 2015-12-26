import numpy as np
from opt.variable import Variable
from opt.solver import Solver
import time
import ipdb

from openravepy import *
from hl_action import HLAction

from interface.fluents.is_mp import IsMP
from interface.fluents.in_manip import InManip
from interface.fluents.is_gp import IsGP as IsPDP
from interface.fluents.robot_at import RobotAt
from interface.fluents.obj_at import ObjAt
from interface.hl_param import Traj

from utils import *

class Place(HLAction):
    def __init__(self, lineno, hl_plan, env, robot, pos, obj, loc, gp):
        super(Place, self).__init__(lineno, hl_plan, env, robot)

        self.pos = pos
        self.obj = obj
        self.loc = loc
        self.gp = gp
        self.name = "place" + str(lineno)

        self.T = 1
        self.K = 3
        T = self.T
        K = self.K

        self.traj = Traj(self, self.name + "_traj", K, T, is_var=True)
        self.obj_traj = Traj(self, self.name + "_objtraj", K, T, is_var=True)
        self.params = [pos, loc, gp, self.traj, self.obj_traj]

        self.preconditions = [RobotAt(self, 0, pos, self.traj)]
        self.preconditions += [InManip(self, 0, obj, gp, self.traj, self.obj_traj)]
        self.preconditions += [IsPDP(self.env, self, robot, 0, obj, gp, self.traj, self.obj_traj)]

        self.postconditions = []
        self.postconditions += [ObjAt(self, 0, obj, loc, self.obj_traj)]

        self.cost = 0.0

    def plot_consensus_pos(self):
        if not np.allclose(self.pos.value, self.hl_pos.value):
            place_pos = np.array(self.pos.value)
            place_pos[2] = 1
            hl_pos = np.array(self.hl_pos.value)
            hl_pos[2] = 1
            if not np.allclose(place_pos, hl_pos, atol=1e-3):
                self.handles += [self.hl_plan.env.drawarrow(p1=place_pos, p2=hl_pos, linewidth=0.01, color=(1,0,0))]
            self.handles += [self.hl_plan.env.plot3(points=hl_pos[:, 0], pointsize=10, colors=(1,0,0))]

    def plot_consensus_obj_pos(self):
        if not np.allclose(self.loc.value, self.hl_loc.value):
            # hl_gp = np.array(self.hl_gp.value)

            # pick_pos = np.array(self.traj.value)
            # pick_pos[2] = 1

            hl_obj_pos = np.array(self.hl_loc.value)
            hl_obj_pos[2]=1
            pick_obj_pos = np.array(self.loc.value)
            pick_obj_pos[2] = 1

            if not np.allclose(pick_obj_pos, hl_obj_pos, atol=1e-3):
                self.handles += [self.hl_plan.env.drawarrow(p1=pick_obj_pos, p2=hl_obj_pos, linewidth=0.01, color=(1,0,0))]
            self.handles += [self.hl_plan.env.plot3(points=hl_obj_pos[:, 0], pointsize=10, colors=(1,0,0))]

    # def plot(self, handles=[]):
    #     # self.handles = []
    #     super(Place, self).plot()
    #
    #     self.handles += handles
    #     self.plot_consensus_pos()
    #     self.plot_consensus_obj_pos()


        # if not np.allclose(self.gp.value, self.hl_gp.value):
        #     hl_gp = np.array(self.hl_gp.value)
        #     # hl_gp[2] = 1

        #     place_pos = np.array(self.traj.value)
        #     place_pos[2] = 1

        #     hl_obj_pos = hl_gp + place_pos
        #     hl_obj_pos[2]=1
        #     place_obj_pos = np.array(self.obj_traj.value)
        #     place_obj_pos[2] = 1

        #     self.handles += [self.hl_plan.env.drawarrow(p1=place_obj_pos, p2=hl_obj_pos, linewidth=0.01, color=(1,0,0))]
        #     hl_points = np.transpose(np.hstack((place_pos, hl_obj_pos)))
        #     place_points = np.transpose(np.hstack((place_pos, place_obj_pos)))
        #     self.handles += [self.hl_plan.env.drawlinestrip(points=place_points, linewidth=10, colors=(0,0.5,0))]
        #     self.handles += [self.hl_plan.env.drawlinestrip(points=hl_points, linewidth=10, colors=(1,0,0))]

    def init_opt(self):
        # initialize trajectories
        self.traj.value = self.hl_loc.value - self.hl_gp.value
        self.pos.value = self.traj.value.copy()
        self.obj_traj.value = self.hl_loc.value

        # sqp = SQP()
        solver = Solver()
        solver.initial_trust_box_size = 0.1
        # solver.initial_trust_box_size = 1
        solver.min_trust_box_size=1e-4
        # solver.initial_penalty_coeff = 0.1
        solver.min_approx_improve = 1e-2
        # solver.g_use_numerical = False

        self.opt_prob.make_primal()
        self.opt_prob.init_trust_region = True

        success = solver.penalty_sqp(self.opt_prob)
        self.pos.initialized = True
        self.loc.initialized = True
        self.gp.initialized = True


    def reset(self):
        self.pos.initialized = False
        self.loc.initialized = False
        self.gp.initialized = False

    def create_opt_prob(self):
        super(Place, self).create_opt_prob()
        self.opt_prob.add_var(self.pos)
        self.opt_prob.add_var(self.loc)

        for place_loc in self.place_locs:
            self.opt_prob.add_var(place_loc)
