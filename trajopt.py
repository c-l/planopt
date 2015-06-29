import numpy as np
import cvxpy as cvx
from sqp import SQP
import time
# import shapely as sp
# import shapely.geometry as geo
# import shapely.affinity as aff
import convex_sets as cs
from sklearn.preprocessing import normalize

from openravepy import *
import bulletsimpy

class Trajopt(object):
    def __init__(self):
        self.T = 40
        self.K = 3
        T = self.T
        self.traj_init = np.matrix(np.vstack((np.linspace(-2,2,num=T), np.zeros((1,T)), np.zeros((1,T)))))
        self.obstacle_poly = np.matrix('-0.576036866359447, 0.918128654970760;\
                        -0.806451612903226,-1.07017543859649;\
                        1.11843317972350,-0.988304093567252;\
                        0.740552995391705,0.906432748538011')

    @staticmethod
    def boxToPolygon(x, y, length, width, theta):
        hw = length/2
        hh = width/2
        minx = x-hw
        maxx = x+hw
        miny = y-hh
        maxy = y+hh
        
        cot = np.cos(theta)
        sit = np.sin(theta)

        wc = hw*cot
        ws = hw*sit
        hc = hh*cot
        hs = hh*sit

        p = np.zeros((4,2))
        
        # p[point number, 0 for x and 1 for y]
        p[0,0] = x-wc+hs
        p[0,1] = y-ws-hc
        p[1,0] = x+wc+hs
        p[1,1] = y+ws-hc
        p[2,0] = x+wc-hs
        p[2,1] = y+ws+hc
        p[3,0] = x-wc-hs
        p[3,1] = y-ws+hc

        return np.matrix(p)

    # @staticmethod
    # def g_collisions(self, x, dsafe, traj_shape, make_robot_poly, obstacles):
    #     traj = x.reshape(traj_shape)
    #     K, T = traj_shape
    #     val = np.zeros((len(obstacles)*T,1))
    #     jac = np.zeros((val.size, x.size))

    #     icontact = 1

    #     for t in range(T):
    #         xt = traj[:,t]
    #         # one obstacle for now
    #         robot = make_robot_poly(xt)
    #         ptOnRobot, ptOnObs, dist = Trajopt.signedDistance(robot, obstacles)
    #         normalObsToRobot = -1 * np.sign(dist)*normalize(ptOnRobot-ptOnObs)
            
    #         gradd = np.transpose(normalObsToRobot) * Trajopt.calcJacobian(ptOnRobot, xt)

    #         val[t] = dsafe - dist
    #         jac[t, K*t:K*(t+1)] = gradd

        # return (val, jac)

    # @staticmethod
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
            clones.append(self.create_robot_kinbody( "clone{0}".format(t), xt))
            env.AddKinBody(clones[t])

            transform = np.identity(4)
            transform[0,3] = xt[0]
            transform[1,3] = xt[1]
            # rot = matrixFromAxisAngle([0,0,xt[2]])
            # transform = rot*transform
            with env:
                clones[t].SetTransform(transform)
        env.UpdatePublishedBodies()

        for t in range(T):
            xt = traj[:,t]
            # one obstacle for now
            # robot = self.make_robot_poly(xt)
            # ptOnRobot, ptOnObs, dist = self.signedDistance(robot, obstacles)
            ptOnRobot, ptOnObs, dist = self.signedDistance(xt, handles)

            gradd = np.zeros((1,K))
            if dist != None:
                normalObsToRobot = -1 * np.sign(dist)*normalize(ptOnRobot-ptOnObs)
                
                gradd = normalObsToRobot * Trajopt.calcJacobian(np.transpose(ptOnRobot), xt)
                # print 'normalObsToRobot: ', normalObsToRobot
                # print 'gradd: ', gradd
                # print 'd: ', dist
            else:
                dist = 0

            # val[t] = dsafe - dist
            val[t] = 0 - dist
            jac[t, K*t:K*(t+1)] = gradd

        # import ipdb; ipdb.set_trace()
        time.sleep(.1)
        for t in range(T):
            # env.RemoveKinBody(clones[t])
            env.Remove(clones[t])
        return (val, jac)

    def signedDistance(self, xt, handles):
        # handles = []
        return self.penetrationDepth(xt, handles)
        v1 = cs.ConvexHull(self.make_robot_poly(xt))
        v2 = cs.ConvexHull(self.obstacle_poly)
        # import ipdb; ipdb.set_trace()

        if cs.is_empty(cs.intersect(v1,v2)):
            return Trajopt.distance(v1, v2)
        else:
            return self.penetrationDepth(xt)

    @staticmethod
    def calcJacobian(pt, x0):
        jac = np.zeros((2,3))
        r = pt - x0[0:2]
        jac[0,0] = 1
        jac[1,1] = 1
        jac[0,2] = -r[1]
        jac[1,2] = r[0]
        return np.matrix(jac)
    
    # @staticmethod
    # def signedDistance(poly1, poly2):
    #     v1 = cs.ConvexHull(poly1.tolist())
    #     v2 = cs.ConvexHull(poly2.tolist())
    #     import ipdb; ipdb.set_trace()

    #     if cs.is_empty(cs.intersect(v1,v2)):
    #         return Trajopt.distance(v1, v2)
    #     else:
    #         return Trajopt.penetrationDepth(v1, v2)

    @staticmethod
    def distance(poly1, poly2):
        objective = cvx.Minimize(cvx.norm(poly1 - poly2, 2))
        prob = cvx.Problem(objective)
        prob.solve(solver='GUROBI')
        return (poly1.value, poly2.value, prob.value)


    # @staticmethod
    # def penetrationDepth(poly1, poly2):
    #     objective = cvx.Maximize(cvx.norm(poly1 - poly2, 2))
    #     prob = cvx.Problem(objective)
    #     prob.solve(solver='GUROBI')
    #     return (poly1.value, poly2.value, -1 * prob.value)

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
        env.AddKinBody(robot)
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


    # currently hardcoded objects (need to change)
    def init_bullet_test_env(self, env):
        print 'number of objects in environment:', len(env.GetBodies())
        print env.GetBodies()
        t0 = time.time()
        bullet_env = bulletsimpy.BulletEnvironment(env, ['box_robot','obstacle'])
        # bullet_env.SetContactDistance(10.0)
        bullet_env.SetGravity([0, 0, 0])

        bullet_objs = [bullet_env.GetObjectByName(b.GetName()) for b in env.GetBodies()]
        print 'bullet objs', [o.GetName() for o in bullet_objs]

        robot_obj = bullet_env.GetObjectByName('box_robot')
        robot_obj.UpdateBullet()

        return bullet_env

    def test_signedDistance(self, env, bullet_env, x, traj_shape):
        handles = []

        traj = x.reshape(traj_shape)
        import ipdb; ipdb.set_trace()
        K, T = traj_shape
        # val = np.zeros((len(obstacles)*T,1))
        # jac = np.zeros((val.size, x.size))

        robot = env.GetKinBody('box_robot')

        for t in range(T):
            xt = traj[:,t]
            print xt
            # ptOnRobot, ptOnObs, dist = Trajopt.penetrationDepth(robot)
            # self.penetrationDepth(xt)
            ptOnRobot, ptOnObs, dist = self.signedDistance(xt)
            # print ptOnRobot, ptOnObs, dist

    def penetrationDepth(self, xt, handles):
        env = self.env
        bullet_env = self.bullet_env
        robot = self.robot_kinbody

        # handles = []
        bullet_objs = [bullet_env.GetObjectByName(b.GetName()) for b in env.GetBodies()]
        # print 'bullet objs', [o.GetName() for o in bullet_objs]

        transform = np.identity(4)
        transform[0,3] = xt[0]
        transform[1,3] = xt[1]
        # rot = matrixFromAxisAngle([0,0,xt[2]])
        # transform = rot*transform
        with env:
            robot.SetTransform(transform)
        env.UpdatePublishedBodies()

        robot_obj = bullet_env.GetObjectByName('box_robot')
        obs = bullet_env.GetObjectByName('obstacle')
        robot_obj.UpdateBullet()
        bullet_env.Step(0.01, 100, 0.01)

        # print "Collisions:"
        collisions = bullet_env.ContactTest(obs)
        ptsOnRobot = [] # A is robot
        ptsOnObs = [] # B is obstacle
        distances = []

        for c in collisions:
            # print 'linkA:', c.linkA.GetParent().GetName(), c.linkA.GetName()
            # print 'linkB:', c.linkB.GetParent().GetName(), c.linkB.GetName()
            # print 'ptA:', c.ptA
            # print 'ptB:', c.ptB
            # print 'normalB2A:', c.normalB2A
            # print 'distance:', c.distance
            # print 'weight:', c.weight
            # with env:
            #     # handles.append(env.plot3(points=np.array((ptA,ptB)), pointsize=0.1,colors=np.array(((0,1,0),(0,0,0)))))
            #     handles.append(env.plot3(points=array((ptAs[0])), pointsize=0.1,colors=array(((0,1,0)))))
            ptA = c.ptA
            ptA[2] = 1.01
            ptsOnRobot.append(ptA)
            ptB = c.ptB
            ptB[2] = 1.01
            ptsOnObs.append(ptB)
            distances.append(c.distance)

        with env:
            # handles.append(env.plot3(points=np.array((ptA,ptB)), pointsize=0.1,colors=array(((0,1,0),(0,0,0)))))
            if len(ptsOnRobot) > 0:
                handles.append(env.plot3(points=ptsOnRobot[0], pointsize=10,colors=(1,0,0)))
                handles.append(env.plot3(points=ptsOnObs[0], pointsize=10,colors=(0,1,0)))
                handles.append(env.drawarrow(p1=ptsOnRobot[0], p2=ptsOnObs[0], linewidth=.01,color=(0,1,0)))

                # time.sleep(0.1)
                # import ipdb; ipdb.set_trace()
                return (np.matrix(ptsOnRobot[0][0:2]), np.matrix(ptsOnObs[0][0:2]), distances[0])

        # time.sleep(0.1)
        return (None, None, None)


    def test(self):
        T = self.T
        K = self.K

        x0 = self.traj_init.reshape((K*T,1), order='F')


        dsafe = 0.05
        KT = K*T

        car_length = 0.4
        car_width = 0.2

        # make robot polygon
        self.make_robot_poly = lambda x: Trajopt.boxToPolygon(x[0], x[1], car_length, car_width, x[2])

        q = np.zeros((1,KT))

        v = -1*np.ones((KT-K,1))
        d = np.vstack((np.ones((KT-K,1)),np.zeros((K,1))))
        # [:,0] allows numpy to see v and d as one-dimensional so that numpy will create a diagonal matrix with v and d as a diagonal
        P = np.matrix(np.diag(v[:,0],K) + np.diag(d[:,0]) )
        Q = np.transpose(P)*P

        f = lambda x: np.zeros((1,1))
        g = lambda x: self.g_collisions(x, dsafe, (K,T))
        h = lambda x: np.zeros((1,1))

        A_ineq = Q
        b_ineq = 0.04*np.ones((KT,1))


        d = np.vstack((np.ones((K,1)), np.zeros((KT-2*K,1)), np.ones((K,1))))
        A_eq = np.diag(d[:,0])

        b_eq = np.vstack((self.traj_init[:,0], np.zeros((KT-2*K,1)), self.traj_init[:,-1]))

        # import ipdb; ipdb.set_trace()
        sqp = SQP()
        sqp.initial_trust_box_size = 0.1
        sqp.min_approx_improve = 1e-2
        sqp.g_use_numerical = False


        self.env = self.init_openrave_test_env()
        self.bullet_env = self.init_bullet_test_env(self.env)
        # time.sleep(1)
        # import ipdb; ipdb.set_trace()
        # self.test_signedDistance(self.env, self.bullet_env, x0, (K,T))
        x = sqp.penalty_sqp(x0, Q, q, f, A_ineq, b_ineq, A_eq, b_eq, g, h)

        env = self.env
        handles = []
        clones = []
        traj = x[0].reshape((K,T), order='F')
        for t in range(T):
            xt = traj[:,t]
            clones.append(self.create_robot_kinbody( "clone{0}".format(t), xt))
            env.AddKinBody(clones[t])

            transform = np.identity(4)
            transform[0,3] = xt[0]
            transform[1,3] = xt[1]
            # rot = matrixFromAxisAngle([0,0,xt[2]])
            # transform = rot*transform
            with env:
                clones[t].SetTransform(transform)
        env.UpdatePublishedBodies()
        import ipdb; ipdb.set_trace()



if __name__ == "__main__":
    traj = Trajopt()
    traj.test()
