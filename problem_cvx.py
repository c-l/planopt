from sqp_cvx import SQP
import numpy as np
import cvxpy as cvx
import ipdb

class Problem(object):
    def __init__(self):
        zerofunc = lambda x: np.matrix(0)
        neginffunc = lambda x: np.matrix(-1e5)

        n = 2

        self.f = zerofunc
        self.g = neginffunc
        self.h = zerofunc
        self.objective = 0
        self.constraints = []

    def problem0(self):
        self.name = 'problem0'
        self.x0 = np.matrix('1;1', dtype='float64')
        self.x = cvx.Variable(self.x0.shape[0], self.x0.shape[1])
        self.f = lambda x: np.matrix(x[0]**2+x[1]**2)
        self.g = lambda x: np.matrix(3 - x[0] - x[1])
        self.xtrue = np.matrix('1.5;1.5', dtype='float64')

    def problem1(self):
        self.name = 'problem1'
        self.x0 = np.matrix('-2;1', dtype='float64')
        self.x = cvx.Variable(self.x0.shape[0], self.x0.shape[1])
        self.f = lambda x: (x[1]-x[0]**2)**2 + (1-x[0])**2
        self.g = lambda x: np.matrix(-1.5 - x[1], dtype='float64')
        self.xtrue = np.matrix('1;1', dtype='float64')

    def problem2(self):
        self.name = 'problem2'
        self.x0 = np.matrix('10;1', dtype='float64')
        self.x = cvx.Variable(self.x0.shape[0], self.x0.shape[1])
        self.f = lambda x: x[1] + 1e-5 + (x[1]-x[0])**2
        self.g = lambda x: -x[1]
        self.xtrue = np.matrix('0;0', dtype='float64')

    def problem3(self):
        self.name = 'problem3'
        self.x0 = np.matrix('10;1', dtype='float64')
        self.x = cvx.Variable(self.x0.shape[0], self.x0.shape[1])
        self.f = lambda x: (1-x[0])**2
        self.h = lambda x: 10*(x[1]-x[0]**2)
        self.xtrue = np.matrix('1;1', dtype='float64')

    def problem4(self):
        self.name = 'problem4'
        self.x0 = np.matrix('2;2', dtype='float64')
        self.x = cvx.Variable(self.x0.shape[0], self.x0.shape[1])
        self.f = lambda x: np.log(1+x[0]**2)-x[1]
        self.h = lambda x: (1+x[0]**2)**2+x[1]**2-4
        self.xtrue = np.matrix('0;{0}'.format(np.sqrt(3)))

    def problem5(self):
        self.name = 'problem5'
        angles = np.transpose(np.matrix(range(1,7)) * 2*np.pi/6)
        self.x0 = np.matrix('0;0', dtype='float64')
        self.x = cvx.Variable(self.x0.shape[0], self.x0.shape[1])

        self.A_ineq = np.hstack((np.cos(angles),np.sin(angles)))
        self.b_ineq = np.ones(angles.shape)
        self.q = -np.matrix([np.cos(np.pi/6), np.sin(np.pi/6)])
        self.constraints = [self.A_ineq * self.x <= self.b_ineq]
        self.objective = self.q*self.x
        self.xtrue = np.transpose(np.matrix([1, np.tan(np.pi/6)]))

    def problem6(self):
        self.name = 'problem6'
        angles = np.transpose(np.matrix(range(1,7)) * 2*np.pi/6)
        self.x0 = np.matrix('0;0', dtype='float64')
        self.x = cvx.Variable(self.x0.shape[0], self.x0.shape[1])
        self.Q = .1*np.identity(2)
        self.q = -np.matrix([np.cos(np.pi/6), np.sin(np.pi/6)])
        A_ineq = np.hstack((np.cos(angles),np.sin(angles)))
        b_ineq = np.ones(angles.shape)
        self.g = lambda x: .01*(A_ineq*x-b_ineq)
        # self.constraints = [A_ineq * self.x <= b_ineq]
        self.objective = 0.5 * cvx.quad_form(self.x, self.Q) + self.q*self.x
        self.xtrue = np.transpose(np.matrix([1, np.tan(np.pi/6)]))

    def problem7(self):
        self.name = 'problem7'
        self.x0 = np.matrix('0;0', dtype='float64')
        self.x = cvx.Variable(self.x0.shape[0], self.x0.shape[1])
        self.f = lambda x: x[0]**4 + x[1]**4
        self.g = lambda x: 3 - x[0] - x[1]
        self.h = lambda x: x[0] - 2*x[1]
        self.xtrue = np.matrix('2;1', dtype='float64')

    def problem8(self):
        self.name = 'problem8'
        self.x0 = np.matrix('5;5', dtype='float64')
        self.x = cvx.Variable(self.x0.shape[0], self.x0.shape[1])
        self.g = lambda x: np.vstack((\
            x[0]**2 + x[1]**2 - 4, \
            -((x[0]-1)**2 +(x[1]**2-1)**2 - 0.25), \
            -((x[0]+1)**2 +(x[1]**2-1)**2 - 0.25), \
            -((x[0])**2 + 7*(x[1]+1-x[0]**2/2)**2 - 0.8)))

        self.Q = np.identity(2)
        self.objective = 0.5 * cvx.quad_form(self.x, self.Q)
        self.xtrue = np.matrix('0;0', dtype='float64')

    def test(self):
        sqp = SQP()
        sqp.min_approx_improve = 1e-8
        sqp.min_trust_box_size = 1e-5
        # x = cvx.Variable(self.x0.shape[0], self.x0.shape[1])
        x, success = sqp.penalty_sqp(self.x, self.x0, self.objective, self.constraints, self.f, self.g, self.h)
        if success:
            print ('{0} succeeded'.format(self.name))
        else:
            print ('{0} failed'.format(self.name))
        if np.allclose(x.value, self.xtrue, atol=1e-3):
            print ('x = {1} and xtrue = {2} are close'.format(self.name, x.value, self.xtrue))
        else:
            print ('x = {1} and xtrue = {2} are not close'.format(self.name, x.value, self.xtrue))
            ipdb.set_trace()



