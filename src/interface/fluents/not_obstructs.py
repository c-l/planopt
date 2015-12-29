
from fluent import FnLEFluent
from opt.function import CollisionFn
from utils import *
from openravepy import *
import ctrajoptpy
import numpy as np
import time

class NotObstructs(FnLEFluent):
    def __init__(self, env, hl_action, robot, priority, traj, obj=None, obj_traj=None, place_objs=None, place_locs=None):
        self.env = env
        self.plotting_env = hl_action.hl_plan.env
        self.hl_action = hl_action
        self.traj = traj
        self.obj = obj
        if self.obj is not None:
            self.obj_traj = obj_traj
        self.robot = robot
        self.priority = priority

        self.name = "NotObstructs"
        # self.tolerance = 1e-2

        if place_objs == None:
            self.place_objs = []
            self.place_locs = []
        else:
            self.place_objs = place_objs
            self.place_locs = place_locs

        self.cc = ctrajoptpy.GetCollisionChecker(env)

    def pre(self):
        traj = self.traj
        K = self.hl_action.K
        T = self.hl_action.T

        self.obj_names = []
        # precompute index to object mapping
        assert len(self.place_objs) == len(self.place_locs)
        self.num_objs = len(self.place_objs)
        self.obj_names = [obj.name for obj in self.place_objs]
        self.name_to_index = {}
        for obj, i in zip(self.place_objs, range(self.num_objs)):
            if self.obj is not None:
                self.name_to_index[obj.name] = i+2*T
            else:
                self.name_to_index[obj.name] = i+T

        # TODO: fix this hack to work when place locations and trajectory have different dimensions (perhaps zero pad place locations or trajectory?)
        # x = place_locs + [self.traj]

        # x = [self.traj] + self.place_locs
        # x = cvx.vstack(*x)

        # g = Function(lambda x: self.collisions(x, 0.05, (K,T)), use_numerical=False) # function inequality constraint g(x) <= 0
        g = lambda x: self.collisions(x, 0.05, (K,T)) # function inequality constraint g(x) <= 0
        if self.obj is not None:
            g_func = CollisionFn([self.traj, self.obj_traj] + self.place_locs, g)
        else:
            g_func = CollisionFn([self.traj] + self.place_locs, g)
        # h = None # function equality constraint h(x) ==0
        # self.constraints = Constraints(linear_constraints, (g, self.traj), h)
        # self.constraints = Constraints(linear_constraints, (g, x), h)
        self.fn = g_func

    # TODO: compute collisions properly
    # @profile
    def collisions(self, x, dsafe, traj_shape):
        # print "in ", self.hl_action.name, "'s collision method"
        env = self.env

        # setting placed objects at consensus locations
        if self.num_objs > 0:
            for obj, loc in zip(self.place_objs, self.place_locs):
                obj.set_pose(env, loc.value)

        K, T = traj_shape

        # traj = x[:K*T,:].reshape(traj_shape, order='F')

        traj = x[:K, :]

        obj_traj = None
        if self.obj is not None:
            obj_traj = x[K:2*K, :]
            # obj_traj = x[K*T:2*K*T,:].reshape(traj_shape, order='F')
        # val = np.zeros((len(obstacles)*T,1))
        if self.obj is not None:
            val = np.zeros((2*T+self.num_objs, 1))
        else:
            val = np.zeros((T+self.num_objs, 1))
        jac = np.zeros((val.size, x.size))

        cc = self.cc
        # collisions outside of dsafe aren't detected
        # cc.SetContactDistance(dsafe)
        cc.SetContactDistance(dsafe + .1)

        handles = []
        timesteps = []

        robot = self.robot
        obj = self.obj
        collisions = []
        # distances = np.infty * np.ones(T)
        if self.obj is not None:
            distances = np.infty * np.ones(2*T+self.num_objs)
        else:
            distances = np.infty * np.ones(T+self.num_objs)
        for t in range(T):
            # xt = self.traj.value[K*t:K*(t+1)]
            xt = traj[:,t:t+1]
            robot.set_pose(env, xt)
            if obj is not None:
                # ot = self.obj_traj.value[K*t:K*(t+1)]
                ot = obj_traj[:,t:t+1]
                # ot = self.obj_traj_var.value[K*t:K*(t+1)]
                obj.set_pose(env, ot)
            bodies = []
            if obj is not None:
                bodies = [robot.get_env_body(env), obj.get_env_body(env)]
            else:
                bodies = [robot.get_env_body(env)]
            num_bodies = len(bodies)
            # for body in bodies:
            for i in range(num_bodies):
                collisions = cc.BodyVsAll(bodies[i])

                for c in collisions:
                    distance = c.GetDistance()
                    linkA = c.GetLinkAParentName()
                    linkB = c.GetLinkBParentName()
                    if obj is not None:
                        if linkA == robot.name and linkB == obj.name:
                            continue
                        elif linkB == robot.name and linkA == obj.name:
                            continue

                    assert linkA == robot.name or linkA == obj.name

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

                    if not (robot.name == linkA or obj.name == linkA):
                        self.hl_action.plot(handles)
                        self.plotting_env.UpdatePublishedBodies()
                        import ipdb; ipdb.set_trace() # BREAKPOINT

                    gradd = np.zeros((1,K))
                    # import ipdb; ipdb.set_trace() # BREAKPOINT
                    # normal = np.matrix(c.GetNormal())
                    normal = c.GetNormal()
                    # print 'distance: ', distance
                    # print 'normal: ', normal
                    # print 'linkA: ', linkA
                    # print 'linkB: ', linkB

                    # normalObsToRobot2 = -1 * np.sign(c.GetDistance())*normalize(ptB-ptA)

                    assert linkA not in self.obj_names

                    body_index = i*T + t
                    placed_obj_index = None
                    include_placed_obj = linkB in self.obj_names
                    if include_placed_obj:
                        placed_obj_index = self.name_to_index[linkB]
                        # if distance <= distances[index]:
                        # import ipdb; ipdb.set_trace() # BREAKPOINT
                        distances[placed_obj_index] = distance
                        # need to flip the sign from the place objs point of reference
                        loc = self.place_locs[placed_obj_index-num_bodies*T]
                        ptA = ptA[0:2]
                        # gradd = np.dot(10*normal[0:2], self.calcJacobian(np.transpose(ptA), loc.value[:,0]))
                        gradd = np.dot(normal[0:2], self.calcJacobian(np.transpose(ptA), loc.value[:,0]))
                        val[placed_obj_index] += dsafe - c.GetDistance()
                        jac[placed_obj_index, K*placed_obj_index:K*(placed_obj_index+1)] += gradd
                        jac[body_index, K*placed_obj_index:K*(placed_obj_index+1)] += gradd

                    ptB = ptB[0:2]
                    # why is there a negative one?
                    # robot gradient
                    if i == 0:
                        gradd = np.dot(-1 * normal[0:2], self.calcJacobian(np.transpose(ptB), traj[:,t]))
                    # object gradient
                    elif i == 1:
                        gradd = np.dot(-1 * normal[0:2], self.calcJacobian(np.transpose(ptB), obj_traj[:,t]))
                    else:
                        print "this shouldn't happen we should only be iterating over the object and robot"
                        assert False

                    val[body_index] += dsafe - distance
                    jac[body_index, K*body_index:K*(body_index+1)] += gradd
                    if include_placed_obj:
                        jac[placed_obj_index, K*body_index:K*(body_index+1)] += gradd

        self.hl_action.add_plot_handles(handles)
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
        # return np.matrix(jac)
        return jac
