from fluent import Fluent
import cvxpy as cvx
from opt.constraints import Constraints
from opt.function import Function
# from trajopt_cvx import Trajopt
from utils import *
from openravepy import *
import ctrajoptpy
import numpy as np
import time

class IsMP(Fluent):
    def __init__(self, env, hl_action, robot, traj, obj, obj_traj, place_objs=None, place_locs=None):
        super(IsMP, self).__init__(env, hl_action)
        self.plotting_env = hl_action.hl_plan.env
        self.hl_action = hl_action
        self.traj = traj
        self.obj = obj
        self.obj_traj = obj_traj
        self.robot = robot
        self.constraints = None
        self.name = "IsMP"
        self.tolerance = 1e-2

        self.place_objs = place_objs
        self.place_locs = place_locs
        self.cc = ctrajoptpy.GetCollisionChecker(env)

    def precondition(self):
        traj = self.traj
        K = self.hl_action.K
        T = self.hl_action.T
        # K,T = traj.size

        v = -1*np.ones((K*T-K,1))
        d = np.vstack((np.ones((K*T-K,1)),np.zeros((K,1))))
        # [:,0] allows numpy to see v and d as one-dimensional so that numpy will create a diagonal matrix with v and d as a diagonal
        P = np.matrix(np.diag(v[:,0],K) + np.diag(d[:,0]) )

        # positions between time steps are less than 0.2
        A_ineq = np.vstack((P, -P))
        b_ineq = 0.3*np.ones((2*K*T,1))
        linear_constraints = [A_ineq * traj <= b_ineq]

        # precompute index to object mapping
        assert len(self.place_objs) == len(self.place_locs)
        self.num_objs = len(self.place_objs)
        self.obj_names = [obj.GetName() for obj in self.place_objs]
        self.name_to_index = {}
        for obj, i in zip(self.place_objs, range(self.num_objs)):
            self.name_to_index[obj.GetName()] = i+T

        # TODO: fix this hack to work when place locations and trajectory have different dimensions (perhaps zero pad place locations or trajectory?)
        # x = place_locs + [self.traj]
        x = [self.traj] + self.place_locs
        x = cvx.vstack(*x)
        g = Function(lambda x: self.collisions(x, 0.05, (K,T)), use_numerical=False) # function inequality constraint g(x) <= 0
        h = None # function equality constraint h(x) ==0
        # self.constraints = Constraints(linear_constraints, (g, self.traj), h)
        self.constraints = Constraints(linear_constraints, (g, x), h)
        return self.constraints

    # TODO: compute collisions properly
    # @profile
    def collisions(self, x, dsafe, traj_shape):
        # print "in ", self.hl_action.name, "'s collision method"
        env = self.env

        # setting placed objects at consensus locations
        if self.num_objs > 0:
            for obj, loc in zip(self.place_objs, self.place_locs):
                obj.SetTransform(base_pose_to_mat(loc.value))

        K, T = traj_shape

        traj = x[:K*T,:].reshape(traj_shape, order='F')
        # val = np.zeros((len(obstacles)*T,1))
        val = np.zeros((T+self.num_objs, 1))
        jac = np.zeros((val.size, x.size))

        cc = self.cc
        cc.SetContactDistance(dsafe)

        handles = []
        timesteps = []

        robot = self.robot
        obj = self.obj
        collisions = []
        # distances = np.infty * np.ones(T)
        distances = np.infty * np.ones(T+self.num_objs)
        for t in range(T):
            # xt = self.traj.value[K*t:K*(t+1)]
            xt = traj[:,t]
            robot.SetTransform(base_pose_to_mat(xt))
            if obj is not None:
                # ot = self.obj_traj.value[K*t:K*(t+1)]
                ot = self.obj_traj.value[K*t:K*(t+1)]
                obj.SetTransform(base_pose_to_mat(ot))
            bodies = []
            if obj is not None:
                bodies = [robot, obj]
            else:
                bodies = [robot]
            for body in bodies:
                collisions = cc.BodyVsAll(body)

                for c in collisions:
                    distance = c.GetDistance()
                    linkA = c.GetLinkAParentName()
                    linkB = c.GetLinkBParentName()
                    if obj is not None:
                        if linkA == robot.GetName() and linkB == obj.GetName():
                            continue
                        elif linkB == robot.GetName() and linkA == obj.GetName():
                            continue



                    # print "collision dist of ", distance, " between ", linkA, " ", linkB
                    # if distance > distances[t]:
                    #     continue
                    # else:
                    #     distances[t] = distance

                    ptA = c.GetPtA()
                    ptA[2] = 1.01
                    ptB = c.GetPtB()
                    ptB[2] = 1.01

                    # plotting collision information
                    # handles.append(self.plotting_env.plot3(points=ptB, pointsize=10,colors=(1,0,0)))
                    # handles.append(self.plotting_env.plot3(points=ptA, pointsize=10,colors=(0,1,0)))
                    if not np.all(ptA == ptB):
                        if distance < 0:
                            handles.append(self.plotting_env.drawarrow(p1=ptA, p2=ptB, linewidth=.01,color=(1,0,0)))
                        else:
                            handles.append(self.plotting_env.drawarrow(p1=ptA, p2=ptB, linewidth=.01,color=(0,0,0)))

                    if not (robot.GetName() == linkA or obj.GetName() == linkA):
                        self.hl_action.plot(handles)
                        self.plotting_env.UpdatePublishedBodies()
                        import ipdb; ipdb.set_trace() # BREAKPOINT

                    gradd = np.zeros((1,K))
                    normal = np.matrix(c.GetNormal())
                    
                    # normalObsToRobot2 = -1 * np.sign(c.GetDistance())*normalize(ptB-ptA)


                    if linkB in self.obj_names:
                        index = self.name_to_index[linkB]
                        if distance <= distances[index]:
                            # import ipdb; ipdb.set_trace() # BREAKPOINT
                            distances[index] = distance
                            # need to flip the sign from the place objs point of reference
                            loc = self.place_locs[index-T]
                            ptA = np.matrix(ptA)[:, 0:2]
                            gradd = 10*normal[:,0:2] * self.calcJacobian(np.transpose(ptA), loc.value)
                            val[index] = dsafe - c.GetDistance()
                            jac[index, K*index:K*(index+1)] = gradd
                    if linkA in self.obj_names:
                        print "we shouldn't be here, making assumption that linkA is the obj or robot"
                        import ipdb; ipdb.set_trace() # BREAKPOINT

                    if distance > distances[t]:
                        continue
                    else:
                        distances[t] = distance

                    ptB = np.matrix(ptB)[:, 0:2]
                    # why is there a negative one?
                    gradd = -1 * normal[:,0:2] * self.calcJacobian(np.transpose(ptB), traj[:,t])

                    val[t] = dsafe - c.GetDistance()
                    jac[t, K*t:K*(t+1)] = gradd

        self.hl_action.plot(handles)
        self.plotting_env.UpdatePublishedBodies()
        handles = []

        return (val, jac)

    
        
    # @profile
    def create_clones(self):
        self.clones = []
        robot = self.robot

        self.env.Remove(robot)
        time.sleep(1.5)
        self.env.Add(robot)

        transparency = 0.8
        for t in range(self.hl_action.T):
            newrobot = RaveCreateRobot(self.env,robot.GetXMLId())
            newrobot.Clone(robot,0)
            newrobot.SetName("move_" + robot.GetName() + "_" + str(t))
        
            for link in newrobot.GetLinks():
                for geom in link.GetGeometries():
                    geom.SetTransparency(transparency)
                    geom.SetDiffuseColor([0,0,1])

            self.clones.append(newrobot)

    def calcJacobian(self, pt, x0):
        jac = np.zeros((2,3))
        r = pt - x0[0:2]
        jac[0,0] = 1
        jac[1,1] = 1
        jac[0,2] = -r[1]
        jac[1,2] = r[0]
        return np.matrix(jac)

