from fluent import Fluent
import cvxpy as cvx
from trajopt_cvx import Trajopt
from openravepy import *
import ctrajoptpy
import numpy as np
import time

class IsMP(Fluent):
    def __init__(self, env, traj):
        self.env = env
        self.traj = traj
        self.obstacle_kinbody = env.GetKinBody("obstacle")

    def precondition(self):
        traj = self.traj
        K,T = traj.size

        v = -1*np.ones((K*T-K,1))
        d = np.vstack((np.ones((K*T-K,1)),np.zeros((K,1))))
        # [:,0] allows numpy to see v and d as one-dimensional so that numpy will create a diagonal matrix with v and d as a diagonal
        P = np.matrix(np.diag(v[:,0],K) + np.diag(d[:,0]) )

        # positions between time steps are less than 0.2
        A_ineq = np.vstack((P, -P))
        b_ineq = 0.2*np.ones((2*K*T,1))
        constraints = [A_ineq * cvx.reshape(traj, K*T,1) <= b_ineq]

        g = lambda x: self.collisions(x, 0.05, (K,T)) # function inequality constraint g(x) <= 0
        h = None # function equality constraint h(x) ==0
        return constraints, g, h

    def collisions(self, x, dsafe, traj_shape):
        # import ipdb; ipdb.set_trace()
        env = self.env
        traj = x.reshape(traj_shape, order='F')
        K, T = traj_shape
        # val = np.zeros((len(obstacles)*T,1))
        val = np.zeros((T,1))
        jac = np.zeros((val.size, x.size))

        icontact = 1

        handles = []
        clones = []
        for t in range(T):
            xt = traj[:,t]
            clones.append(self.create_robot_kinbody( "{0}".format(t), xt))
            env.AddKinBody(clones[t])

            transform = np.identity(4)
            transform[0,3] = xt[0]
            transform[1,3] = xt[1]
            rot = matrixFromAxisAngle([0,0,xt[2]])
            transform = np.dot(rot,transform)
            with env:
                clones[t].SetTransform(transform)
        env.UpdatePublishedBodies()

        cc = ctrajoptpy.GetCollisionChecker(env)

        handles = []
        ptAs = []
        ptBs = []
        timesteps = []

        collisions = cc.BodyVsAll(self.obstacle_kinbody)
        for c in collisions:
            # if c.GetDistance() > 0:
            # print "distance: ", c.GetDistance()
            # print "normal: ", c.GetNormal()
            # print "ptA: ", c.GetPtA()
            # print "ptB: ", c.GetPtB()
            # print "link A: ", c.GetLinkAParentName()
            # print "link B: ", c.GetLinkBParentName()
            ptA = c.GetPtA()
            ptA[2] = 1.01
            ptAs.append(ptA)
            ptB = c.GetPtB()
            ptB[2] = 1.01
            ptBs.append(ptB)
            # timesteps.append(int(c.GetLinkBParentName()))
            t = int(c.GetLinkBParentName())
            # print "computed normal: ", normalize(ptB-ptA)

            handles.append(env.plot3(points=ptB, pointsize=10,colors=(1,0,0)))
            handles.append(env.plot3(points=ptA, pointsize=10,colors=(0,1,0)))
            handles.append(env.drawarrow(p1=ptB, p2=ptA, linewidth=.01,color=(0,1,0)))

            gradd = np.zeros((1,K))
            normal = np.matrix(c.GetNormal())
            
            # normalObsToRobot2 = -1 * np.sign(c.GetDistance())*normalize(ptB-ptA)
            # ipdb.set_trace()

            ptB = np.matrix(ptB)[:, 0:2]
            gradd = normal[:,0:2] * self.calcJacobian(np.transpose(ptB), traj[:,t])

            val[t] = dsafe - c.GetDistance()
            jac[t, K*t:K*(t+1)] = gradd

        time.sleep(.5)
        # ipdb.set_trace()
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

    def create_robot_kinbody(self, name, xt):
        # create robot KinBody
        env = self.env
        box = KinBody.Link.GeometryInfo()
        box._type = KinBody.Link.GeomType.Box
        box._vGeomData = [0.2,0.1,1.01]
        box._bVisible = True
        box._fTransparency = 0
        box._vDiffuseColor = [0,0,1]

        robot = RaveCreateKinBody(env,'')
        robot.InitFromGeometries([box])
        robot.SetName(name)

        return robot

