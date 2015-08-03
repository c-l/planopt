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

class Pick(HLAction):
    def __init__(self, hl_plan, env, robot, pos_param, obj, loc_param, gp_param, name="pick"):
        super(Pick, self).__init__(hl_plan, env, robot)

        self.pos, self.hl_pos = pos_param.new_hla_var(self)
        self.obj = obj
        self.loc, self.hl_loc = loc_param.new_hla_var(self)
        self.gp, self.hl_gp = gp_param.new_hla_var(self)
        self.name = "pick"

        self.T = 1
        self.K = 3
        T = self.T
        K = self.K

        # self.traj_init = np.zeros((3,1))
        # self.traj_init = np.array([[-1],[2],[0]]) # cool/strange wiggling into a good solution
        # self.traj_init = np.array([[0],[-2],[0]])
        # self.traj = Variable(K*T,1, name=self.name+"_traj",value=self.traj_init)
        self.traj = Variable(K*T,1, name=self.name+"_traj")
        # self.traj.value = self.traj_init

        # self.obj_init = np.zeros((3,1))
        # self.obj_traj = Variable(K*T,1, name=self.name+'_obj_traj', value=self.traj_init)
        self.obj_traj = Variable(K*T,1, name=self.name+'_obj_traj')
        # self.obj_traj.value = self.obj_init

        self.preconditions = [RobotAt(self, self.pos, self.traj)]
        self.preconditions += [ObjAt(self, self.obj, self.loc, self.obj_traj)] 
        # self.preconditions += [IsMP(self.env, self, robot, self.traj, self.obj, self.obj_traj)]
        self.postconditions = [InManip(self.env, self, robot, self.obj, self.gp, self.traj, self.obj_traj)]
        self.create_opt_prob()
        self.initialize_opt()

    def plot(self, handles=[]):
        self.handles = []
        # del self.handles
        super(Pick, self).plot()

        if not np.allclose(self.pos.value, self.hl_pos.value):
            pick_pos = np.array(self.pos.value)
            pick_pos[2] = 1
            hl_pos = np.array(self.hl_pos.value)
            hl_pos[2] = 1
            self.handles += [self.hl_plan.env.drawarrow(p1=pick_pos, p2=hl_pos, linewidth=0.01, color=(1,0,0))]
            self.handles += [self.hl_plan.env.plot3(points=hl_pos[:, 0], pointsize=10, colors=(1,0,0))]
        
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

    def initialize_opt(self):
        # initialize trajectories
        self.traj.value = self.hl_loc.value - self.hl_gp.value
        self.obj_traj.value = self.hl_loc.value

        # solve initial optimization problem
        solver = Solver()
        # sqp.initial_trust_box_size = 0.1
        solver.initial_trust_box_size = 1
        solver.min_trust_box_size=1e-4
        # sqp.initial_penalty_coeff = 0.1
        solver.min_approx_improve = 1e-2
        # sqp.g_use_numerical = False

        self.opt_prob.make_primal()
        success = solver.penalty_sqp(self.opt_prob)

    def create_opt_prob(self):
        super(Pick, self).create_opt_prob()
        self.opt_prob.add_var(self.pos)
        self.opt_prob.add_var(self.loc)
