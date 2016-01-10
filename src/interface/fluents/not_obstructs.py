
from fluent import FnLEFluent
from opt.function import CollisionFn
from utils import *
from openravepy import *
import ctrajoptpy
import numpy as np
import time

class NotObstructs(FnLEFluent):
    # def __init__(self, env, hl_action, robot, priority, traj, obj, obj_loc, dsafe=0.05):
    def __init__(self, env, hl_action, robot, priority, traj, obj, dsafe=0.05):
        self.env = env
        self.plotting_env = hl_action.hl_plan.env
        self.hl_action = hl_action
        self.traj = traj
        self.obj = obj
        # self.obj_loc = obj_loc
        self.robot = robot
        self.priority = priority

        self.name = "NotObstructs"

        self.K = self.hl_action.K
        self.T = self.hl_action.T
        self.cc = ctrajoptpy.GetCollisionChecker(env)
        self.dsafe = dsafe

    def pre(self):
        traj = self.traj

        g = lambda x: self.collisions(x) # function inequality constraint g(x) <= 0
        self.fn = CollisionFn([self.traj], g)

    # TODO: compute collisions properly
    # @profile
    def collisions(self, traj):
        env = self.env

        # ensure that there's gradient information outside dsafe
        self.cc.SetContactDistance(self.dsafe + .1)

        handles = []

        val = np.zeros((self.T, 1))
        jac = np.zeros((self.T, traj.size))
        # self.obj.set_pose(env, self.obj_loc.value)

        for t in range(self.T):
            # xt = self.traj.value[K*t:K*(t+1)]
            xt = traj[:,t:t+1]
            self.robot.set_pose(env, xt)
            collisions = self.cc.BodyVsAll(self.robot.get_env_body(env))

            col_val, col_jac = self.calc_grad_and_val(xt, collisions)
            if col_jac is not None:
                val[t], jac[t, self.K*t:self.K*(t+1)] = col_val, col_jac

        self.plotting_env.UpdatePublishedBodies()
        handles = []

        return (val, jac)

    def calc_grad_and_val(self, xt, collisions):
        val = float("inf")
        grad = None
        for c in collisions:
            linkA = c.GetLinkAParentName()
            linkB = c.GetLinkBParentName()
            distance = c.GetDistance()
            normal = c.GetNormal()

            ptA = c.GetPtA()
            ptA[2] = 1.01
            ptB = c.GetPtB()
            ptB[2] = 1.01

            if linkA == self.robot.name and linkB == self.obj.name:
                ptRobot = ptA
                ptObj = ptB
            elif linkB == self.robot.name and linkA == self.obj.name:
                ptRobot = ptB
                ptObj = ptA
            else:
                continue

            # assert linkA == self.robot.name
            # if linkB != self.obj.name:
            #     continue

            # plotting
            self.plot_collision(ptRobot, ptObj, distance)

            # if there are multiple collisions, use the one with the greatest penetration distance
            if self.dsafe - distance < val:
                val = self.dsafe - distance

                grad = np.dot(-1 * normal[0:2], self.calc_jacobian(np.transpose(ptObj), xt))

        return val, grad

    def plot_collision(self, ptA, ptB, distance):
        handles = []
        if not np.allclose(ptA, ptB, atol=1e-3):
            if distance < 0:
                handles.append(self.plotting_env.drawarrow(p1=ptA, p2=ptB, linewidth=.01,color=(1,0,0)))
            else:
                handles.append(self.plotting_env.drawarrow(p1=ptA, p2=ptB, linewidth=.01,color=(0,0,0)))
        self.hl_action.add_plot_handles(handles)


    def calc_jacobian(self, pt, x0):
        jac = np.zeros((2,3))
        r = pt[0:2] - x0[0:2, 0]
        jac[0,0] = 1
        jac[1,1] = 1
        jac[0,2] = -r[1]
        jac[1,2] = r[0]
        # return np.matrix(jac)
        return jac
