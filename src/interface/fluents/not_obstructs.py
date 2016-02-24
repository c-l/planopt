from fluent import FnLEFluent
from opt.function import CollisionFn
from utils import *
from openravepy import *
import ctrajoptpy
import numpy as np
import time

# If True, verify that caching produces equivalent results to
# computation. If False, actually do caching (return cached value).
CACHING_DEBUG = False

class NotObstructs(FnLEFluent):
    # def __init__(self, env, hl_action, robot, priority, traj, obj, obj_loc, dsafe=0.05):
    def __init__(self, env, hl_action, robot, priority, traj, obj, obj_loc=None, dsafe=0.05):
        self.env = env
        self.plotting_env = hl_action.hl_plan.env
        self.hl_action = hl_action
        self.traj = traj
        self.obj = obj
        self.obj_loc = obj_loc
        self.robot = robot
        self.priority = priority
        self.name = "NotObstructs"
        self.K = self.hl_action.K
        self.T = self.hl_action.T
        self.cc = ctrajoptpy.GetCollisionChecker(env)
        self.dsafe = dsafe
        self.cache = {}

    def pre(self):
        traj = self.traj

        g = lambda x: self.collisions(x) # function inequality constraint g(x) <= 0
        obj_loc_list = []
        if self.obj_loc is not None:
            obj_loc_list = [self.obj_loc for i in range(self.T)]
        self.fn = CollisionFn([self.traj] + obj_loc_list, g)

    # TODO: compute collisions properly
    # @profile
    def collisions(self, traj):
        flattened = tuple(traj.round(5).flatten())
        v_check, j_check = None, None
        if flattened in self.cache:
            if CACHING_DEBUG:
                v_check, j_check = self.cache[flattened]
            else:
                return self.cache[flattened]

        env = self.env
        T = self.T
        K = self.K

        # ensure that there's gradient information outside dsafe
        self.cc.SetContactDistance(self.dsafe + .1)

        handles = []

        if self.obj_loc is not None:
            val = np.zeros((2*self.T, 1))
            jac = np.zeros((2*self.T, traj.size))
        else:
            val = np.zeros((self.T, 1))
            jac = np.zeros((self.T, traj.size))
        # if self.obj_loc is not None:
        #     self.obj.set_pose(env, self.obj_loc.value)

        for t in range(self.T):
            # xt = self.traj.value[K*t:K*(t+1)]
            # xt = traj[:,t:t+1]
            xt = traj[K*t:K*(t+1)]
            self.robot.set_pose(env, xt)
            ot = None
            if self.obj_loc is not None:
                ot = traj[K*(t+T):K*(t+1+T)]
                self.obj.set_pose(env, ot)
            collisions = self.cc.BodyVsAll(self.robot.get_env_body(env))

            col_val, robot_jac, obj_jac = self.calc_grad_and_val(xt, ot, collisions)
            if robot_jac is not None:
                val[t], jac[t, K*t:K*(t+1)] = col_val, robot_jac
                if self.obj_loc is not None:
                    val[t+T], jac[t+T, K*(t+T):K*(t+1+T)] = col_val, obj_jac
                    # cross terms
                    jac[t, K*(t+T):K*(t+1+T)] = obj_jac
                    jac[t+T, K*t:K*(t+1)] = robot_jac

        self.plotting_env.UpdatePublishedBodies()
        handles = []

        if CACHING_DEBUG:
            assert v_check is None or np.allclose(v_check, val, atol=1e-4)
            assert j_check is None or np.allclose(j_check, jac, atol=1e-4)
        self.cache[flattened] = (val, jac)
        return (val, jac)

    # @profile
    def calc_grad_and_val(self, xt, ot, collisions):
        val = -1*float("inf")
        robot_grad = None
        obj_grad = None
        for c in collisions:
            linkA = c.GetLinkAParentName()
            linkB = c.GetLinkBParentName()

            if linkA == self.robot.name and linkB == self.obj.name:
                ptRobot = c.GetPtA()
                ptObj = c.GetPtB()
            elif linkB == self.robot.name and linkA == self.obj.name:
                ptRobot = c.GetPtB()
                ptObj = c.GetPtA()
            else:
                continue

            ptRobot[2] = 1.01
            ptObj[2] = 1.01

            distance = c.GetDistance()
            normal = c.GetNormal()

            # plotting
            self.plot_collision(ptRobot, ptObj, distance)

            # if there are multiple collisions, use the one with the greatest penetration distance
            if self.dsafe - distance > val:
                val = self.dsafe - distance

                robot_grad = np.dot(-1 * normal[0:2], self.calc_jacobian(np.transpose(ptObj), xt))
                if ot is not None:
                    obj_grad = np.dot(normal[0:2], self.calc_jacobian(np.transpose(ptRobot), ot))

        return val, robot_grad, obj_grad

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
