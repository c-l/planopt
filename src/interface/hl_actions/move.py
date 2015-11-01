import numpy as np
from opt.variable import Variable
from opt.solver import Solver
from opt.constraints import Constraints
from opt.function import QuadFn
# from opt.sqp import SQP
# from sqp_cvx import SQP
import time
import ipdb

from openravepy import *
from hl_action import HLAction

from interface.fluents.is_mp import IsMP
from interface.fluents.in_manip import InManip
from interface.fluents.robot_at import RobotAt
from interface.fluents.obj_at import ObjAt

from utils import *

class Move(HLAction):
    def __init__(self, lineno, hl_plan, env, robot, start_param=None, end_param=None, obj_param=None, obj_start_param=None, obj_end_param=None, gp_param=None, name="move", place_obj_params=None, place_loc_params=None):
        super(Move, self).__init__(lineno, hl_plan, env, robot)

        assert start_param is not None
        assert end_param is not None
        self.start, self.hl_start = start_param.new_hla_var(self)
        self.end, self.hl_end = end_param.new_hla_var(self)
        if obj_param is None:
            assert obj_start_param is None
            assert obj_end_param is None
            assert gp_param is None
            self.obj = obj_param
            self.gp = gp_param
        else:
            self.obj, _ = obj_param.new_hla_var(self, env)
            self.obj_start, self.hl_obj_start = obj_start_param.new_hla_var(self)
            self.obj_end, self.hl_obj_end = obj_end_param.new_hla_var(self)
            self.gp, self.hl_gp = gp_param.new_hla_var(self)
        self.name = name

        self.place_objs = []

        if place_obj_params is not None:
            for param in place_obj_params:
                obj, _ = param.new_hla_var(self, self.env)
                self.place_objs.append(obj)
        
        self.place_locs = []
        self.hl_place_locs = []
        if place_loc_params is not None:
            for param in place_loc_params:
                loc, hl_loc = param.new_hla_var(self)
                self.place_locs.append(loc)
                self.hl_place_locs.append(hl_loc)

        self.T = 40
        self.K = 3
        T = self.T
        K = self.K
        KT = K*T


        self.traj = Variable(self.model,K*T,1,name=self.name+'_traj')

        if self.obj is not None:
            self.obj_traj = Variable(self.model,K*T,1,name=self.name+'_obj_traj')
        else:
            self.obj_traj = None

        self.preconditions = [RobotAt(self.env, self, self.model, self.start, self.traj)] 
        self.create_robot_clones()
        self.preconditions += [IsMP(self.env, self, self.model, robot, self.traj, self.obj, self.obj_traj, place_objs=self.place_objs, place_locs=self.place_locs)]

        self.postconditions = []
        if self.obj is not None:
            self.preconditions += [InManip(self.env, self, self.model, robot, self.obj, self.gp, self.traj, self.obj_traj)]
        self.postconditions += [RobotAt(self.env, self, self.model, self.end, self.traj)]

        # setting trajopt objective
        v = -1*np.ones((KT-K,1))
        d = np.vstack((np.ones((KT-K,1)),np.zeros((K,1))))
        # [:,0] allows numpy to see v and d as one-dimensional so that numpy will create a diagonal matrix with v and d as a diagonal
        P = np.diag(v[:,0],K) + np.diag(d[:,0])
        Q = 2*np.dot(np.transpose(P),P)

        self.model.update()
        self.cost = QuadFn(self.traj, Q)

        self.create_opt_prob()

    def initial_traj(self):
        waypoint = np.array([[3],[-1],[0]])
        # waypoint = np.array([[-2],[1],[0]])
        start = self.start.value
        end = self.end.value

        mid_time_step = self.T/2
        init_traj = np.hstack((np.array([np.linspace(i,j,mid_time_step) for i,j in zip(np.array(start), np.array(waypoint))]), \
                    np.array([np.linspace(i,j,self.T-mid_time_step) for i,j in zip(np.array(waypoint), np.array(end))])))
        return init_traj
                    

    def plot(self, handles=[]):
        # self.handles = []
        # self.handles += super(Move, self).plot(handles)
        super(Move, self).plot()
        self.handles += handles
        # self.handles += self.plot_traj_line(self.traj, colors=(0,0,0.5))

        # if self.obj is not None:
        #     self.handles += self.plot_traj_line(self.obj_traj, colors=(0,0.5,0))

        self.plot_consensus_pos()
        self.plot_consensus_place_obj()

    def plot_consensus_pos(self):
        start = np.array(self.start.value)
        start[2] = 1
        hl_start = np.array(self.hl_start.value)
        hl_start[2] = 1
        if not np.allclose(start, hl_start, atol=1e-3):
            self.handles += [self.hl_plan.env.drawarrow(p1=start, p2=hl_start, linewidth=0.01, color=(1,0,0))]

        end = np.array(self.end.value)
        end[2] = 1
        hl_end = np.array(self.hl_end.value)
        hl_end[2] = 1
        if not np.allclose(end, hl_end, atol=1e-3):
            self.handles += [self.hl_plan.env.drawarrow(p1=end, p2=hl_end, linewidth=0.01, color=(1,0,0))]


    def plot_consensus_place_obj(self):
        # if self.name == 'move6':
        #     import ipdb; ipdb.set_trace() # BREAKPOINT
        for loc, hl_loc in zip(self.place_locs, self.hl_place_locs):
            plot_loc = np.array(loc.value)
            plot_hl_loc = np.array(hl_loc.value)
            plot_loc[2] = 1
            plot_hl_loc[2] = 1
            if not np.allclose(plot_loc, plot_hl_loc, atol=1e-3):
                self.handles += [self.hl_plan.env.drawarrow(p1=plot_loc, p2=plot_hl_loc, linewidth=0.01, color=(1,0,0))]
            self.handles += [self.hl_plan.env.plot3(points=plot_hl_loc[:, 0], pointsize=10, colors=(1,0,0))]


    def init_opt(self):
        start = self.hl_start.value
        end = self.hl_end.value
        midpoint = (start + end)/2
        # waypoint = midpoint + np.array([[0],[-2],[0]])
        waypoint = end + np.array([[0],[-2],[0]])

        # traj_init = np.matrix([np.linspace(i,j,T) for i,j in zip(np.array(self.hl_start.value), np.array(self.hl_end.value))])
        mid_time_step = self.T/2
        init_traj = np.hstack((np.array([np.linspace(i,j,mid_time_step) for i,j in zip(np.array(start), np.array(waypoint))]), \
                    np.array([np.linspace(i,j,self.T-mid_time_step) for i,j in zip(np.array(waypoint), np.array(end))])))
        init_traj = np.reshape(init_traj, (self.K*self.T,1), order='F')
        self.traj.value = init_traj
        # self.traj_init = self.initial_traj()
        if self.obj is not None:
            self.obj_traj.value = self.traj.value + np.tile(self.hl_gp.value, (self.T,1))

        solver = Solver()
        # solver.initial_trust_box_size = 0.1
        # solver.initial_trust_box_size = 1
        solver.initial_trust_box_size = 3
        # solver.min_trust_box_size=1e-2
        solver.min_trust_box_size=1e-1
        solver.initial_penalty_coeff = 0.1
        # solver.initial_penalty_coeff = 0.01
        solver.min_approx_improve = 1e-2
        solver.max_merit_coeff_increases = 2
        self.opt_prob.make_primal()

        K = self.K
        constraints = Constraints(self.model)
        to_remove_cnts = constraints.add_eq_cntr(self.traj.grb_vars[:K], self.hl_start.value)
        to_remove_cnts += constraints.add_eq_cntr(self.traj.grb_vars[-K:], self.hl_end.value)
        if self.obj is not None:
            to_remove_cnts += constraints.add_eq_cntr(self.gp.grb_vars, self.hl_gp.value)

        success = solver.penalty_sqp(self.opt_prob)
        self.opt_prob.init_trust_region = True

        for constraint in to_remove_cnts:
            self.model.remove(constraint)

        self.start.initialized = True
        self.end.initialized = True
        if self.obj is not None:
            self.gp.initialized = True
            # self.obj_start.initialized = True
            # self.obj_end.initialized = True

        for place_loc in self.place_locs:
            place_loc.initialized = True
    
    def reset(self):
        self.start.initialized = False
        self.end.initialized = False
        if self.obj is not None:
            self.gp.initialized = False
            # self.obj_start.initialized = False
            # self.obj_end.initialized = False

        for place_loc in self.place_locs:
            place_loc.initialized = False

    def solve_opt_prob(self):
        sqp = SQP()
        # sqp.initial_trust_box_size = 0.1
        # sqp.initial_trust_box_size = 1
        sqp.initial_trust_box_size = 2
        sqp.min_trust_box_size=1e-2
        sqp.initial_penalty_coeff = 0.1
        # sqp.initial_penalty_coeff = 0.01
        # sqp.min_approx_improve = 1e-2
        sqp.min_approx_improve = 1e-1

        # sqp.g_use_numerical = False
        self.opt_prob.make_primal()
        success = sqp.penalty_sqp(self.opt_prob)
        # x, success = sqp.penalty_sqp(self.traj, self.traj.value, self.objective, self.constraints, self.f, self.g, self.h)

    def create_opt_prob(self):
        super(Move, self).create_opt_prob()
        self.opt_prob.add_var(self.start)
        self.opt_prob.add_var(self.end)
        for place_loc in self.place_locs:
            self.opt_prob.add_var(place_loc)


if __name__ == "__main__":
    move = Move()
    move.solve_opt_prob()
