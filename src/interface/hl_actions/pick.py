import numpy as np
from opt.variable import Variable
from opt.solver import Solver
# from opt.sqp import SQP
# from sqp_cvx import SQP
import time
import ipdb

from openravepy import *
from hl_action import HLAction

from interface.fluents.is_mp import IsMP
from interface.fluents.in_manip import InManip
from interface.fluents.is_gp import IsGP
from interface.fluents.robot_at import RobotAt
from interface.fluents.obj_at import ObjAt

from utils import *

class Pick(HLAction):
    def __init__(self, lineno, hl_plan, env, robot, pos_param, obj_param, loc_param, gp_param, name="pick"):
        super(Pick, self).__init__(lineno, hl_plan, env, robot)

        self.pos, self.hl_pos = pos_param.new_hla_var(self)
        self.obj, _ = obj_param.new_hla_var(self, env)
        self.loc, self.hl_loc = loc_param.new_hla_var(self)
        self.gp, self.hl_gp = gp_param.new_hla_var(self)
        self.name = name

        self.T = 1
        self.K = 3
        T = self.T
        K = self.K

        self.traj = Variable(self.model, K*T, 1, name=self.name+"_traj")
        self.obj_traj = Variable(self.model, K*T, 1, name=self.name+'_obj_traj')

        self.preconditions = [RobotAt(self.env, self, self.model, self.pos, self.traj)]
        self.preconditions += [ObjAt(self.env, self, self.model, self.obj, self.loc, self.obj_traj)] 
        self.preconditions += [IsGP(self.env, self, self.model, robot, self.obj, self.gp, self.traj, self.obj_traj)]
        self.preconditions += [IsMP(self.env, self, self.model, robot, self.traj, self.obj, self.obj_traj, place_objs=None, place_locs=None)]

        self.postconditions = [InManip(self.env, self, self.model, robot, self.obj, self.gp, self.traj, self.obj_traj)]

        self.create_opt_prob()

    def plot_consensus_pos(self):
        if not np.allclose(self.pos.value, self.hl_pos.value, atol=1e-3):
            pick_pos = np.array(self.pos.value)
            pick_pos[2] = 1
            hl_pos = np.array(self.hl_pos.value)
            hl_pos[2] = 1
            self.handles += [self.hl_plan.env.drawarrow(p1=pick_pos, p2=hl_pos, linewidth=0.01, color=(1,0,0))]
            self.handles += [self.hl_plan.env.plot3(points=hl_pos[:, 0], pointsize=10, colors=(1,0,0))]

    def plot_consensus_obj_pos(self):
        if not np.allclose(self.loc.value, self.hl_loc.value, atol=1e-3):
            # hl_gp = np.array(self.hl_gp.value)

            # pick_pos = np.array(self.traj.value)
            # pick_pos[2] = 1

            hl_obj_pos = np.array(self.hl_loc.value)
            hl_obj_pos[2]=1
            pick_obj_pos = np.array(self.loc.value)
            pick_obj_pos[2] = 1

            self.handles += [self.hl_plan.env.drawarrow(p1=pick_obj_pos, p2=hl_obj_pos, linewidth=0.01, color=(1,0,0))]
            self.handles += [self.hl_plan.env.plot3(points=hl_obj_pos[:, 0], pointsize=10, colors=(1,0,0))]

    def plot(self, handles=[]):
        # self.handles = []
        self.handles += handles
        # del self.handles
        super(Pick, self).plot()
        self.plot_consensus_pos()
        self.plot_consensus_obj_pos()

        
        # if not np.allclose(self.gp.value, self.hl_gp.value):
        #     hl_gp = np.array(self.hl_gp.value)

        #     pick_pos = np.array(self.traj.value)
        #     pick_pos[2] = 1

        #     hl_obj_pos = hl_gp + pick_pos
        #     hl_obj_pos[2]=1
        #     pick_obj_pos = np.array(self.obj_traj.value)
        #     pick_obj_pos[2] = 1

        #     self.handles += [self.hl_plan.env.drawarrow(p1=pick_obj_pos, p2=hl_obj_pos, linewidth=0.01, color=(1,0,0))]
        #     hl_points = np.transpose(np.hstack((pick_pos, hl_obj_pos)))
        #     pick_points = np.transpose(np.hstack((pick_pos, pick_obj_pos)))
        #     self.handles += [self.hl_plan.env.drawlinestrip(points=pick_points, linewidth=10, colors=(0,0.5,0))]
        #     self.handles += [self.hl_plan.env.drawlinestrip(points=hl_points, linewidth=10, colors=(1,0,0))]

    def init_opt(self):
        # initialize trajectories
        self.traj.value = self.hl_loc.value - self.hl_gp.value
        self.pos.value = self.traj.value.copy()
        self.obj_traj.value = self.hl_loc.value

        # solve initial optimization problem
        solver = Solver()
        # sqp.initial_trust_box_size = 0.1
        solver.initial_trust_box_size = 1
        solver.min_trust_box_size=1e-4
        # sqp.initial_penalty_coeff = 0.1
        # solver.min_approx_improve = 1e-2
        solver.min_approx_improve = 1e-4
        # sqp.g_use_numerical = False

        self.opt_prob.make_primal()
        self.opt_prob.init_trust_region = True
        # import ipdb; ipdb.set_trace() # BREAKPOINT
        success = solver.penalty_sqp(self.opt_prob)

        self.pos.initialized = True
        self.loc.initialized = True
        self.gp.initialized = True

    def reset(self):
        self.pos.initialized = False
        self.loc.initialized = False
        self.gp.initialized = False

    def create_opt_prob(self):
        super(Pick, self).create_opt_prob()
        self.opt_prob.add_var(self.pos)
        self.opt_prob.add_var(self.loc)
