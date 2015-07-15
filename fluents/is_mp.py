from fluent import Fluent
import cvxpy as cvx
from trajopt_cvx import Trajopt
from utils import *
from openravepy import *
import ctrajoptpy
import numpy as np
import time

class IsMP(Fluent):
    def __init__(self, env, hl_action, traj, obj, obj_traj):
        self.env = env
        self.hl_action = hl_action
        self.traj = traj
        self.obj = obj
        self.obj_traj = obj_traj

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
        b_ineq = 0.2*np.ones((2*K*T,1))
        constraints = [A_ineq * traj <= b_ineq]

        g = lambda x: self.collisions(x, 0.05, (K,T)) # function inequality constraint g(x) <= 0
        h = None # function equality constraint h(x) ==0
        return constraints, g, h

    # TODO: compute collisions properly
    def collisions(self, x, dsafe, traj_shape):
        env = self.env
        traj = x.reshape(traj_shape, order='F')
        K, T = traj_shape
        # val = np.zeros((len(obstacles)*T,1))
        val = np.zeros((T,1))
        jac = np.zeros((val.size, x.size))

        icontact = 1
        cc = ctrajoptpy.GetCollisionChecker(env)

        handles = []
        ptAs = []
        ptBs = []
        timesteps = []

        # collisions = cc.BodyVsAll(self.obstacle_kinbody)
        # robot = self.create_robot_kinbody("robot", color=[1,0,0])
        robot = self.env.GetRobots()[0]
        obj = self.obj
        collisions = []
        distances = -1 * np.infty * np.ones(T)
        for t in range(T):
            xt = self.traj.value[K*t:K*(t+1)]
            robot.SetTransform(base_pose_to_mat(xt))
            ot = self.obj_traj.value[K*t:K*(t+1)]
            obj.SetTransform(base_pose_to_mat(ot))
            # robot.Grab(obj, robot.GetLink('base'))
            # robot.Release(obj)
            for body in [robot, obj]:
                collisions = cc.BodyVsAll(body)

                for c in collisions:
                    # if c.GetDistance() > 0:
                    distance = c.GetDistance()
                    # print "normal: ", c.GetNormal()
                    # print "ptA: ", c.GetPtA()
                    # print "ptB: ", c.GetPtB()
                    linkA = c.GetLinkAParentName()
                    linkB = c.GetLinkBParentName()
                    if linkA == robot.GetName() and linkB == obj.GetName():
                        continue
                    elif linkB == robot.GetName() and linkA == obj.GetName():
                        continue

                    # print "distance: ", distance
                    if distance < distances[t]:
                        continue

                    # print "link A: ", linkA
                    # print "link B: ", linkB
                    ptA = c.GetPtA()
                    # ptA[2] = 1.01
                    # ptAs.append(ptA)
                    ptB = c.GetPtB()
                    # ptB[2] = 1.01
                    # ptBs.append(ptB)
                    # timesteps.append(int(c.GetLinkBParentName()))
                    # t = int(c.GetLinkBParentName())
                    # print "computed normal: ", normalize(ptB-ptA)

                    handles.append(env.plot3(points=ptB, pointsize=10,colors=(1,0,0)))
                    handles.append(env.plot3(points=ptA, pointsize=10,colors=(0,1,0)))
                    # if np.all(ptA == ptB):
                    #     import ipdb; ipdb.set_trace() # BREAKPOINT
                    handles.append(env.drawarrow(p1=ptA, p2=ptB, linewidth=.01,color=(0,1,0)))

                    gradd = np.zeros((1,K))
                    normal = np.matrix(c.GetNormal())
                    
                    # normalObsToRobot2 = -1 * np.sign(c.GetDistance())*normalize(ptB-ptA)

                    ptB = np.matrix(ptB)[:, 0:2]
                    # why is there a negative one?
                    gradd = -1 * normal[:,0:2] * self.calcJacobian(np.transpose(ptB), traj[:,t])

                    val[t] = dsafe - c.GetDistance()
                    jac[t, K*t:K*(t+1)] = gradd
                    # import ipdb; ipdb.set_trace() # BREAKPOINT

        clones = []
        transparency = 0.8
        self.env.Remove(robot)
        time.sleep(1.5)
        self.env.Add(robot)

        for t in range(T):
            xt = traj[:,t]

            newrobot = RaveCreateRobot(self.env,robot.GetXMLId())
            newrobot.Clone(robot,0)
            newrobot.SetName("move_" + robot.GetName() + "_" + str(t))
            # newrobot.SetName(str(t))

            for link in newrobot.GetLinks():
                for geom in link.GetGeometries():
                    geom.SetTransparency(transparency)
                    geom.SetDiffuseColor([0,0,1])

            newrobot.SetTransform(base_pose_to_mat(xt))
            env.Add(newrobot)
            clones.append(newrobot)


        time.sleep(0.5)
        env.UpdatePublishedBodies()
        for t in range(T):
            # env.RemoveKinBody(clones[t])
            env.Remove(clones[t])
        return (val, jac)

    
        
    def calcJacobian(self, pt, x0):
        jac = np.zeros((2,3))
        r = pt - x0[0:2]
        jac[0,0] = 1
        jac[1,1] = 1
        jac[0,2] = -r[1]
        jac[1,2] = r[0]
        return np.matrix(jac)

