import numpy as np
import cvxpy as cvx
from opt.variable import Variable
from opt.solver import Solver
# from opt.sqp import SQP

# from sqp_cvx import SQP
import time
import ipdb

from openravepy import *
from hl_action import HLAction

from fluents.is_mp import IsMP
from fluents.in_manip import InManip
from fluents.robot_at import RobotAt
from fluents.obj_at import ObjAt

from utils import *

class Place(HLAction):
    def __init__(self, hl_plan, env, robot, pos_param, obj, loc_param, gp_param):
        super(Place, self).__init__(hl_plan, env, robot)

        self.pos, self.hl_pos = pos_param.new_hla_var(self)
        self.obj = obj
        self.loc, self.hl_loc = loc_param.new_hla_var(self)
        self.gp, self.hl_gp = gp_param.new_hla_var(self)
        self.name = "place"

        self.T = 1
        self.K = 3
        T = self.T
        K = self.K

        self.traj_init = np.zeros((3,1))
        self.traj = Variable(K*T,1, name=self.name+"_traj",cur_value=self.traj_init)
        # self.traj.value = self.traj_init

        self.obj_init = np.zeros((3,1))
        self.obj_traj = Variable(K*T,1, name=self.name+'_obj_traj', cur_value=self.traj_init)
        # self.obj_traj.value = self.obj_init

        self.preconditions = [RobotAt(self, self.pos, self.traj)]
        # self.preconditions += [InManip(self, self.obj, self.gp, self.traj, self.obj_traj)]
        self.postconditions = [ObjAt(self, obj, self.loc, self.obj_traj)] 
        self.postconditions += [InManip(self, self.obj, self.gp, self.traj, self.obj_traj)]
        self.create_opt_prob()

    def plot(self, handles=[]):
        self.handles = []
        super(Place, self).plot()

        if not np.allclose(self.pos.value, self.hl_pos.value):
            place_pos = np.array(self.pos.value)
            place_pos[2] = 1
            hl_pos = np.array(self.hl_pos.value)
            hl_pos[2] = 1
            self.handles += [self.hl_plan.env.drawarrow(p1=place_pos, p2=hl_pos, linewidth=0.01, color=(1,0,0))]
            self.handles += [self.hl_plan.env.plot3(points=hl_pos[:, 0], pointsize=10, colors=(1,0,0))]
        
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

    def solve_opt_prob(self):
        # sqp = SQP()
        sqp = Solver()
        # sqp.initial_trust_box_size = 0.1
        sqp.initial_trust_box_size = 1
        sqp.min_trust_box_size=1e-4
        # sqp.initial_penalty_coeff = 0.1
        # sqp.min_approx_improve = 1e-2
        # sqp.g_use_numerical = False

        # x = cvx.reshape(self.traj, self.K, self.T)
        # x0 = np.reshape(self.traj_init, (self.K*self.T,1), order='F')
        success = sqp.penalty_sqp(self.opt_prob)
        # x, success = sqp.penalty_sqp(self.traj, self.traj.value, self.objective, self.constraints, self.f, self.g, self.h)
