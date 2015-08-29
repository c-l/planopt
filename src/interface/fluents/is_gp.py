from fluent import Fluent
import cvxpy as cvx
from numpy.linalg import norm
import numpy as np
from opt.constraints import Constraints
from opt.function import CollisionFn

import ctrajoptpy
from utils import *

class IsGP(Fluent):
    def __init__(self, env, hl_action, model, robot, obj, gp, traj, obj_traj = None):
        super(IsGP, self).__init__(env, hl_action, model)
        self.env = env
        self.hl_action = hl_action
        self.plotting_env = hl_action.hl_plan.env
        self.robot = robot
        self.obj = obj
        self.gp = gp.grb_vars
        self.traj = traj.grb_vars
        self.traj_var = traj
        if self.obj is not None:
            self.obj_traj = obj_traj.grb_vars
            self.obj_traj_var = obj_traj
        self.name = "IsGP"
        
        self.cc = ctrajoptpy.GetCollisionChecker(env)

    def precondition(self):
        # obj_pos = Fluent.get_object_loc(self.obj)
        # linear_constraints = [self.traj[:,-1] - self.gp == obj_pos, cvx.norm(self.gp,2) == 1.26] 
        K = self.hl_action.K
        T = self.hl_action.T

        h = lambda x: self.distance_from_obj(x, 0.06, (K,T)) # function inequality constraint g(x) <= 0
        h_func = CollisionFn(self.traj_var, h)
        self.constraints.add_nonlinear_eq_constraint(h_func)

        self.constraints.add_eq_cntr(self.traj[-K] + self.gp, self.obj_traj[-K:])

        return self.constraints

    def distance_from_obj(self, x, target_dist, traj_shape):
        env = self.env
        K, T = traj_shape
        val = np.zeros((T,1))
        jac = np.zeros((val.size, x.size))

        cc = self.cc

        handles = []
        robot = self.robot
        obj = self.obj
        collisions = []
        
        xt = x[-K:]
        robot.SetTransform(base_pose_to_mat(xt))
        ot = self.obj_traj_var.value[-K:]
        obj.SetTransform(base_pose_to_mat(ot))

        cc.SetContactDistance(np.infty)
        collisions = cc.BodyVsBody(robot, obj)

        t = T-1
        for c in collisions:
            distance = c.GetDistance()
            # print "distance: ", distance
            linkA = c.GetLinkAParentName()
            linkB = c.GetLinkBParentName()

            ptA = c.GetPtA()
            ptA[2] = 1.01
            ptB = c.GetPtB()
            ptB[2] = 1.01

            # plotting collision information
            # handles.append(self.plotting_env.plot3(points=ptB, pointsize=10,colors=(1,0,0)))
            # handles.append(self.plotting_env.plot3(points=ptA, pointsize=10,colors=(0,1,0)))
            if not np.allclose(ptA, ptB, atol=1e-3):
                if distance < 0:
                    handles.append(self.plotting_env.drawarrow(p1=ptA, p2=ptB, linewidth=.01,color=(1,0,0)))
                else:
                    handles.append(self.plotting_env.drawarrow(p1=ptA, p2=ptB, linewidth=.01,color=(0,0,0)))

            gradd = np.zeros((1,K))
            normal = np.matrix(c.GetNormal())
            
            # normalObsToRobot2 = -1 * np.sign(c.GetDistance())*normalize(ptB-ptA)

            ptB = np.matrix(ptB)[:, 0:2]
            # why is there a negative one?
            gradd = -1 * normal[:,0:2] * self.calcJacobian(np.transpose(ptB), xt)

            val[t] = (target_dist - c.GetDistance())
            jac[t, K*t:K*(t+1)] = gradd
            # val[t] = 3*(target_dist - c.GetDistance())
            # jac[t, K*t:K*(t+1)] = 3*gradd
            # print "distance collision = ", (target_dist - c.GetDistance())
            # print "distance = ", (self.grasp(x) / 3.0)

        self.hl_action.plot(handles)

        self.plotting_env.UpdatePublishedBodies()
        handles = []
        return (val, jac)

    def calcJacobian(self, pt, x0):
        jac = np.zeros((2,3))
        r = pt - x0[0:2]
        jac[0,0] = 1
        jac[1,1] = 1
        jac[0,2] = -r[1]
        jac[1,2] = r[0]
        return np.matrix(jac)
