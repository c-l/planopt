from fluent import AndFluent, FnEQFluent, LinEqFluent
from aff_expr import AffExpr
from numpy.linalg import norm
import numpy as np
from opt.constraints import Constraints
from opt.function import CollisionFn

import ctrajoptpy
from utils import *

class IsGP(AndFluent):
    def __init__(self, env, hl_action, robot, priority, obj, gp, loc, traj, obj_traj):
        self.env = env
        self.hl_action = hl_action
        self.plotting_env = hl_action.hl_plan.env
        self.priority = priority
        self.robot = robot
        self.obj = obj
        self.gp = gp
        self.loc = loc
        self.traj = traj
        self.obj_traj = obj_traj
        self.name = "IsGP(" + self.obj.name + ", " + self.gp.name + ')'
        self.do_extension = True
        self.extension_params = [gp, traj, obj_traj]

        self.cc = ctrajoptpy.GetCollisionChecker(env)

    def pre(self):
        # TODO: remove assumption that grasp is one time step
        # import ipdb; ipdb.set_trace()
        # self.traj.set_value(self.gp.get_value() + self.obj_traj.get_value())


        K = self.hl_action.K
        T = self.hl_action.T

        h = lambda x: self.distance_from_obj(x, 0.05, (K,T)) # function inequality constraint g(x) <= 0
        # h = lambda x: self.distance_from_obj(x, 0.0, (K,T)) # function inequality constraint g(x) <= 0
        h_func = CollisionFn([self.traj, self.loc], h)

        fneq_fluent = FnEQFluent('fneq_' + self.name, self.priority, self.hl_action)
        fneq_fluent.fn = h_func

        coeff = np.zeros((T, 1), dtype=np.float)
        coeff[0, 0] = 1.0
        lhs = AffExpr({self.obj_traj: coeff})
        rhs = AffExpr({self.traj: coeff, self.gp: 1.0})
        lineq_fluent = LinEqFluent('lineq_' + self.name, self.priority, self.hl_action, lhs, rhs)
        self.fluents = [fneq_fluent, lineq_fluent]

    def post(self):
        K = self.hl_action.K
        T = self.hl_action.T

        h = lambda x: self.distance_from_obj(x, 0.05, (K,T)) # function inequality constraint g(x) <= 0
        h_func = CollisionFn([self.traj, self.loc], h)

        fneq_fluent = FnEQFluent('fneq_' + self.name, self.priority, self.hl_action)
        fneq_fluent.fn = h_func

        coeff = np.zeros((T, 1), dtype=np.float)
        coeff[T-1, 0] = 1.0
        lhs = AffExpr({self.obj_traj: coeff})
        rhs = AffExpr({self.traj: coeff, self.gp: 1.0})
        lineq_fluent = LinEqFluent('lineq_' + self.name, self.priority, self.hl_action, lhs, rhs)
        self.fluents = [fneq_fluent, lineq_fluent]

    def distance_from_obj(self, x, target_dist, traj_shape):
        env = self.env
        K, T = traj_shape
        assert T == 1
        val = np.zeros((T,1))
        jac = np.zeros((val.size, x.size))

        cc = self.cc

        handles = []
        robot = self.robot
        obj = self.obj
        collisions = []

        xt = x[:K]
        robot.set_pose(env, xt)
        ot = x[-K:]
        obj.set_pose(env, ot)

        cc.SetContactDistance(np.infty)
        collisions = cc.BodyVsBody(robot.get_env_body(env), obj.get_env_body(env))

        t = T-1
        for c in collisions:
            distance = c.GetDistance()
            # print "distance: ", distance
            linkA = c.GetLinkAParentName()
            linkB = c.GetLinkBParentName()

            assert linkA == robot.name
            assert linkB == obj.name

            ptA = c.GetPtA()
            ptA[2] = 1.01
            ptB = c.GetPtB()
            ptB[2] = 1.01

            # plotting collision information
            # handles.append(self.plotting_env.plot3(points=ptB, pointsize=10,colors=(1,0,0)))
            # handles.append(self.plotting_env.plot3(points=ptA, pointsize=10,colors=(0,1,0)))
            # if not np.allclose(ptA, ptB, atol=1e-3):
            #     if distance < 0:
            #         handles.append(self.plotting_env.drawarrow(p1=ptA, p2=ptB, linewidth=.01,color=(1,0,0)))
            #     else:
            #         handles.append(self.plotting_env.drawarrow(p1=ptA, p2=ptB, linewidth=.01,color=(0,0,0)))

            gradd = np.zeros((1,K))
            normal = np.matrix(c.GetNormal())

            # print 'distance: ', distance
            # print 'normal: ', normal
            # print 'linkA: ', linkA
            # print 'linkB: ', linkB

            # normalObsToRobot2 = -1 * np.sign(c.GetDistance())*normalize(ptB-ptA)

            ptA = np.matrix(ptA)[:, 0:2]
            ptB = np.matrix(ptB)[:, 0:2]
            # why is there a negative one?

            val[t] = 1*(target_dist - c.GetDistance())

            # gradd = -1 * np.sign(val[t]) * normal[:,0:2] * self.calcJacobian(np.transpose(ptB), xt)
            # gradd = np.sign(val[t]) * normal[:,0:2] * self.calcJacobian(np.transpose(ptB), xt)
            gradd =  -1*normal[:,0:2] * self.calcJacobian(np.transpose(ptB), xt)
            obj_grad = normal[:,0:2] * self.calcJacobian(np.transpose(ptA), ot)
            jac[t, K*t:K*(t+1)] = gradd
            jac[t, K*T+K*t:K*T+K*(t+1)] = obj_grad
            # print "normal: ", normal[:, 0:2]
            # print "val: ", val[t]
            # print "gradd: ", gradd
            # val[t] = 3*(target_dist - c.GetDistance())
            # jac[t, K*t:K*(t+1)] = 3*gradd
            # print "distance collision = ", (target_dist - c.GetDistance())
            # print "distance = ", (self.grasp(x) / 3.0)

        # self.hl_action.add_plot_handles(handles)

        # self.plotting_env.UpdatePublishedBodies()
        # handles = []
        # print 'val: ', val
        # print 'jac: ', jac
        return (val, jac)

    def calcJacobian(self, pt, x0):
        jac = np.zeros((2,3))
        r = pt - x0[0:2]
        jac[0,0] = 1
        jac[1,1] = 1
        jac[0,2] = -r[1]
        jac[1,2] = r[0]
        return np.matrix(jac)

    def params_to_resample(self):
        # resample Grasp, ObjLoc, and Robot Pose
        return filter(lambda p: p is not None and p.is_resampled, [self.gp, self.loc, self.hl_action.pos])
