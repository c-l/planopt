import numpy as np
from opt.constraints import Constraints
from opt.function import QuadFn
from openravepy import *
from hl_action import HLAction
from interface.hl_param import Traj

# from interface.fluents.is_mp import IsMP
# from interface.fluents.in_manip import InManip
from interface.fluents.robot_at import RobotAt
from interface.fluents.not_obstructs import NotObstructs
from interface.fluents.is_mp import IsMP
from interface.fluents.in_manip import InManip
from utils import *


class Move(HLAction):

    def __init__(self, lineno, hl_plan, env, robot, start, end, obj=None, gp=None):
        super(Move, self).__init__(lineno, hl_plan, env, robot)

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
            self.preconditions += [NotObstructs(env, self, robot, 1, self.traj)]
        else:
            assert gp is not None
            self.obj_traj = Traj(self, self.name + "_objtraj", 3, 40, is_var=True)
            self.preconditions += [NotObstructs(env, self, robot, 1, self.traj, obj, self.obj_traj)]
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
        # self.create_robot_clones()

    def straight_line_init(self):
        start = self.start.value
        end = self.end.value
        init_traj = np.array([np.linspace(i, j, self.T) for i, j in zip(np.array(start), np.array(end))])
        return init_traj

    def initial_traj(self):
        waypoint = np.array([[3], [-1], [0]])
        # waypoint = np.array([[-2],[1],[0]])
        start = self.start.value
        end = self.end.value

        mid_time_step = self.T / 2
        init_traj = np.hstack((np.array([np.linspace(i, j, mid_time_step) for i, j in zip(np.array(start), np.array(waypoint))]),
                               np.array([np.linspace(i, j, self.T - mid_time_step) for i, j in zip(np.array(waypoint), np.array(end))])))
        return init_traj

    def plot(self, handles=[]):
        self.clear_plots()
        # self.handles += super(Move, self).plot(handles)

        super(Move, self).plot()
        self.handles += handles
        # self.handles += self.plot_traj_line(self.traj, colors=(0,0,0.5))

        # if self.obj is not None:
        #     self.handles += self.plot_traj_line(self.obj_traj, colors=(0,0.5,0))

        # self.plot_consensus_pos()
        # self.plot_consensus_place_obj()

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

    def init_opt(self):
        start = self.hl_start.value
        end = self.hl_end.value
        midpoint = (start + end) / 2
        # waypoint = midpoint + np.array([[0],[-2],[0]])
        waypoint = end + np.array([[0], [-2], [0]])

        # traj_init = np.matrix([np.linspace(i,j,T) for i,j in zip(np.array(self.hl_start.value), np.array(self.hl_end.value))])
        mid_time_step = self.T / 2
        init_traj = np.hstack((np.array([np.linspace(i, j, mid_time_step) for i, j in zip(np.array(start), np.array(waypoint))]),
                               np.array([np.linspace(i, j, self.T - mid_time_step) for i, j in zip(np.array(waypoint), np.array(end))])))
        init_traj = np.reshape(init_traj, (self.K * self.T, 1), order='F')
        self.traj.value = init_traj
        # self.traj_init = self.initial_traj()
        if self.obj is not None:
            self.obj_traj.value = self.traj.value + \
                np.tile(self.hl_gp.value, (self.T, 1))

        solver = Solver()
        # solver.initial_trust_box_size = 0.1
        # solver.initial_trust_box_size = 1
        solver.initial_trust_box_size = 3
        # solver.min_trust_box_size=1e-2
        solver.min_trust_box_size = 1e-1
        solver.initial_penalty_coeff = 0.1
        # solver.initial_penalty_coeff = 0.01
        solver.min_approx_improve = 1e-2
        solver.max_merit_coeff_increases = 2
        self.opt_prob.make_primal()

        K = self.K
        constraints = Constraints(self.model)
        to_remove_cnts = constraints.add_eq_cntr(
            self.traj.grb_vars[:K], self.hl_start.value)
        to_remove_cnts += constraints.add_eq_cntr(
            self.traj.grb_vars[-K:], self.hl_end.value)
        if self.obj is not None:
            to_remove_cnts += constraints.add_eq_cntr(
                self.gp.grb_vars, self.hl_gp.value)

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
        sqp.min_trust_box_size = 1e-2
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
