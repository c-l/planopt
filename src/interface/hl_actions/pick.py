import numpy as np
import time
import ipdb

from openravepy import *
from hl_action import HLAction

from interface.fluents.is_mp import IsMP
from interface.fluents.in_manip import InManip
from interface.fluents.is_gp import IsGP
from interface.fluents.robot_at import RobotAt
from interface.fluents.obj_at import ObjAt
from interface.hl_param import Traj

from utils import *

class Pick(HLAction):
    def __init__(self, lineno, hl_plan, env, robot, pos, obj, loc, gp):
        super(Pick, self).__init__(lineno, hl_plan, env, robot)

        self.pos = pos
        self.obj = obj
        self.loc = loc
        self.gp = gp
        self.name = "pick" + str(lineno)

        self.T = 1
        self.K = 3
        T = self.T
        K = self.K

        self.traj = Traj(self, self.name + "_traj", (K, T), is_var=True)
        self.obj_traj = Traj(self, self.name + "_objtraj", (K, T), is_var=True)
        self.params = [pos, loc, gp, self.traj, self.obj_traj]

        self.preconditions = [RobotAt(self, 0, pos, self.traj)]
        self.preconditions += [ObjAt(self, 0, obj, loc, self.obj_traj)]
        self.preconditions += [IsGP(self.env, self, robot, 0, obj, gp, loc, self.traj, self.obj_traj)]

        self.cost = 0.0

        # self.postconditions = [InManip(self.env, self, self.model, robot, self.obj, self.gp, self.traj, self.obj_traj)]

    def plot_consensus_pos(self):
        if not np.allclose(self.pos.get_value(), self.hl_pos.get_value(), atol=1e-3):
            pick_pos = np.array(self.pos.get_value())
            pick_pos[2] = 1
            hl_pos = np.array(self.hl_pos.get_value())
            hl_pos[2] = 1
            self.handles += [self.hl_plan.env.drawarrow(p1=pick_pos, p2=hl_pos, linewidth=0.01, color=(1,0,0))]
            self.handles += [self.hl_plan.env.plot3(points=hl_pos[:, 0], pointsize=10, colors=(1,0,0))]

    def plot_consensus_obj_pos(self):
        if not np.allclose(self.loc.get_value(), self.hl_loc.get_value(), atol=1e-3):
            # hl_gp = np.array(self.hl_gp.get_value())

            # pick_pos = np.array(self.traj.get_value())
            # pick_pos[2] = 1

            hl_obj_pos = np.array(self.hl_loc.get_value())
            hl_obj_pos[2]=1
            pick_obj_pos = np.array(self.loc.get_value())
            pick_obj_pos[2] = 1

            self.handles += [self.hl_plan.env.drawarrow(p1=pick_obj_pos, p2=hl_obj_pos, linewidth=0.01, color=(1,0,0))]
            self.handles += [self.hl_plan.env.plot3(points=hl_obj_pos[:, 0], pointsize=10, colors=(1,0,0))]
