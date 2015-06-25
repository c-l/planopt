import numpy as np
import cvxpy as cvx
from sqp import SQP
# import shapely as sp
# import shapely.geometry as geo
# import shapely.affinity as aff
import convex_sets as cs
from sklearn.preprocessing import normalize

class Trajopt(object):
    def __init__(self):
        self.T = 40
        self.K = 3
        T = self.T
        self.traj_init = np.matrix(np.vstack((np.linspace(-2,2,num=T), np.zeros((1,T)), np.zeros((1,T)))))

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

    @staticmethod
    def g_collisions(x, dsafe, traj_shape, make_robot_poly, obstacles):
        traj = x.reshape(traj_shape)
        K, T = traj_shape
        val = np.zeros((len(obstacles)*T,1))
        jac = np.zeros((val.size, x.size))

        icontact = 1

        for t in range(T):
            xt = traj[:,t]
            # one obstacle for now
            robot = make_robot_poly(xt)
            ptOnRobot, ptOnObs, dist = Trajopt.signedDistance(robot, obstacles)
            normalObsToRobot = -1 * np.sign(dist)*normalize(ptOnRobot-ptOnObs)
            
            gradd = np.transpose(normalObsToRobot) * Trajopt.calcJacobian(ptOnRobot, xt)

            val[t] = dsafe - dist
            jac[t, K*t:K*(t+1)] = gradd

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
    
    @staticmethod
    def signedDistance(poly1, poly2):
        v1 = cs.ConvexHull(poly1.tolist())
        v2 = cs.ConvexHull(poly2.tolist())
        import ipdb; ipdb.set_trace()

        if cs.is_empty(cs.intersect(v1,v2)):
            return Trajopt.distance(v1, v2)
        else:
            return Trajopt.penetrationDepth(v1, v2)

    @staticmethod
    def distance(poly1, poly2):
        objective = cvx.Minimize(cvx.norm(poly1 - poly2, 2))
        prob = cvx.Problem(objective)
        prob.solve(solver='GUROBI')
        return (poly1.value, poly2.value, prob.value)


    @staticmethod
    def penetrationDepth(poly1, poly2):
        objective = cvx.Maximize(cvx.norm(poly1 - poly2, 2))
        prob = cvx.Problem(objective)
        prob.solve(solver='GUROBI')
        return (poly1.value, poly2.value, -1 * prob.value)

    def test(self):
        T = self.T
        K = self.K

        x0 = self.traj_init.reshape((K*T,1))

        obstacles = np.matrix('-0.576036866359447, 0.918128654970760;\
                                -0.806451612903226,-1.07017543859649;\
                                1.01843317972350,-0.988304093567252;\
                                0.640552995391705,0.906432748538011')

        dsafe = 0.05
        KT = K*T

        car_length = 0.4
        car_width = 0.2

        # make robot polygon
        make_robot_poly = lambda x: Trajopt.boxToPolygon(x[0], x[1], car_length, car_width, x[2])

        q = np.zeros((1,KT))

        v = -1*np.ones((KT-K,1))
        d = np.vstack((np.ones((KT-K,1)),np.zeros((K,1))))
        # [:,0] allows numpy to see v and d as one-dimensional so that numpy will create a diagonal matrix with v and d as a diagonal
        P = np.diag(v[:,0],K) + np.diag(d[:,0]) 
        Q = np.transpose(P)*P

        f = lambda x: np.zeros((1,1))
        g = lambda x: Trajopt.g_collisions(x, dsafe, (K,T), make_robot_poly, obstacles)
        h = lambda x: np.zeros((1,1))

        A_ineq = Q
        b_ineq = 0.04*np.ones((KT,1))


        d = np.vstack((np.ones((K,1)), np.zeros((KT-2*K,1)), np.ones((K,1))))
        A_eq = np.diag(d)

        b_eq = np.vstack((self.traj_init[:,0], np.zeros((KT-2*K,1)), self.traj_init[:,-1]))

        sqp = SQP()
        sqp.initial_trust_box_size = 0.1
        sqp.min_approx_improve = 1e-2
        sqp.g_use_numerical = False

        x = sqp.penalty_sqp(x0, Q, q, f, A_ineq, b_ineq, A_eq, b_eq, g, h)
        import ipdb; ipdb.set_trace()

if __name__ == "__main__":
    traj = Trajopt()
    traj.test()
