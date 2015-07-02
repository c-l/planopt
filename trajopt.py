import numpy as np
import cvxpy as cvx
from sqp import SQP
import time
from sklearn.preprocessing import normalize
import ipdb

from openravepy import *
import ctrajoptpy
# import bulletsimpy

class Trajopt(object):
    def __init__(self):
        self.T = 40
        self.K = 3
        T = self.T
        self.traj_init = np.matrix(np.vstack((np.linspace(-2,2,num=T), np.zeros((1,T)), np.zeros((1,T)))))

    def g_collisions(self, x, dsafe, traj_shape):
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
            gradd = normal[:,0:2] * Trajopt.calcJacobian(np.transpose(ptB), traj[:,t])

            val[t] = dsafe - c.GetDistance()
            jac[t, K*t:K*(t+1)] = gradd



        # time.sleep(.5)
        # ipdb.set_trace()
        for t in range(T):
            # env.RemoveKinBody(clones[t])
            env.Remove(clones[t])
        return (val, jac)

    @staticmethod
    def calcJacobian(pt, x0):
        jac = np.zeros((2,3))
        r = pt - x0[0:2]
        jac[0,0] = 1
        jac[1,1] = 1
        jac[0,2] = -r[1]
        jac[1,2] = r[0]
        return np.matrix(jac)

    def init_openrave_test_env(self):
        env = Environment() # create openrave environment
        env.SetViewer('qtcoin') # attach viewer (optional)
        
        # create obstacle KinBody
        obstacles = np.matrix('-0.576036866359447, 0.918128654970760, 1;\
                        -0.806451612903226,-1.07017543859649, 1;\
                        1.01843317972350,-0.988304093567252, 1;\
                        0.640552995391705,0.906432748538011, 1;\
                        -0.576036866359447, 0.918128654970760, -1;\
                        -0.806451612903226,-1.07017543859649, -1;\
                        1.01843317972350,-0.988304093567252, -1;\
                        0.640552995391705,0.906432748538011, -1')

        body = RaveCreateKinBody(env,'')
        vertices = np.array(obstacles)
        indices = np.array([[0,1,2], [2,3,0], [4,5,6],[6,7,4],[0,4,5],[0,1,5],[1,2,5],[5,6,2],[2,3,6],[6,7,3],[0,3,7],[0,4,7]])
        body.InitFromTrimesh(trimesh=TriMesh(vertices,indices),draw=True)
        body.SetName('obstacle')
        env.AddKinBody(body) 
        self.obstacle_kinbody = body

        # create robot KinBody
        box = KinBody.Link.GeometryInfo()
        box._type = KinBody.Link.GeomType.Box
        box._vGeomData = [0.2,0.1,1.01]
        box._bVisible = True
        box._fTransparency = 0
        box._vDiffuseColor = [0,0,1]

        robot = RaveCreateKinBody(env,'')
        robot.InitFromGeometries([box])
        robot.SetName('box_robot')
        # env.AddKinBody(robot)
        self.robot_kinbody = robot

        return env

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

    def test(self):
        T = self.T
        K = self.K

        x0 = self.traj_init.reshape((K*T,1), order='F')


        dsafe = 0.05
        KT = K*T

        car_length = 0.4
        car_width = 0.2

        q = np.zeros((1,KT))

        v = -1*np.ones((KT-K,1))
        d = np.vstack((np.ones((KT-K,1)),np.zeros((K,1))))
        # [:,0] allows numpy to see v and d as one-dimensional so that numpy will create a diagonal matrix with v and d as a diagonal
        P = np.matrix(np.diag(v[:,0],K) + np.diag(d[:,0]) )
        Q = np.transpose(P)*P

        f = lambda x: np.zeros((1,1))
        g = lambda x: self.g_collisions(x, dsafe, (K,T))
        h = lambda x: np.zeros((1,1))

        A_ineq = np.vstack((P, -P))
        b_ineq = 0.2*np.ones((2*KT,1))


        d = np.vstack((np.ones((K,1)), np.zeros((KT-2*K,1)), np.ones((K,1))))
        A_eq = np.diag(d[:,0])

        b_eq = np.vstack((self.traj_init[:,0], np.zeros((KT-2*K,1)), self.traj_init[:,-1]))

        # ipdb.set_trace()
        # import ipdb; ipdb.set_trace()
        sqp = SQP()
        sqp.initial_trust_box_size =0.1
        sqp.min_approx_improve = 1e-2
        sqp.g_use_numerical = False


        self.env = self.init_openrave_test_env()
        # self.test_signedDistance(self.env, self.bullet_env, x0, (K,T))
        x, success = sqp.penalty_sqp(x0, Q, q, f, A_ineq, b_ineq, A_eq, b_eq, g, h)

        env = self.env
        handles = []
        clones = []
        traj = x.reshape((K,T), order='F')
        for t in range(T):
            xt = traj[:,t]
            clones.append(self.create_robot_kinbody( "clone{0}".format(t), xt))
            env.AddKinBody(clones[t])

            transform = np.identity(4)
            transform[0,3] = xt[0]
            transform[1,3] = xt[1]
            rot = matrixFromAxisAngle([0,0,xt[2]])
            transform = np.dot(rot,transform)
            with env:
                clones[t].SetTransform(transform)
        env.UpdatePublishedBodies()
        ipdb.set_trace()

if __name__ == "__main__":
    traj = Trajopt()
    traj.test()
