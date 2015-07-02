import cvxpy as cvx
import numpy as np
import numpy.linalg as linalg
import scipy.misc as sci
import ipdb

class SQP(object):
    def __init__(self):
        self.improve_ratio_threshold = .25
        self.min_trust_box_size = 1e-4
        # self.min_trust_box_size = 1e-5
        self.min_approx_improve = 1e-4
        # self.min_approx_improve = 1e-8
        self.max_iter = 50
        self.trust_shrink_ratio = .1
        self.trust_expand_ratio = 1.5
        self.cnt_tolerance = 1e-4
        self.max_merit_coeff_increases = 5
        self.merit_coeff_increase_ratio = 10
        self.initial_trust_box_size = 1
        self.initial_penalty_coeff = 1.
        self.max_penalty_iter = 4
        self.f_use_numerical = True
        self.g_use_numerical = True
        self.h_use_numerical = True
        self.full_hessian = True
        self.callback = []


    @staticmethod
    def numerical_jac(f,x):
        y = f(x)

        grad = np.matrix(np.zeros((y.size, x.size)))

        eps=1e-5
        xp = x.copy()

        for i in range(x.size):
            xp[i] = x[i] + eps/2
            yhi = f(xp)
            xp[i] = x[i] - eps/2
            ylo = f(xp)
            xp[i] = x[i]
            gradi = (yhi-ylo)/eps
            grad[:, i] = gradi.reshape((gradi.size,1))

        return grad

    # def numerical_grad_hess(f, x, full_hessian=True):
    @staticmethod
    def numerical_grad_hess(f, x):
        y = f(x)
        # assert length(y) == 1

        grad = np.zeros((1, x.size))
        hess = np.zeros((x.size, x.size))

        # eps=1e-5
        # xp = x.copy()

        grad = SQP.numerical_jac(f,x)
        hess = SQP.numerical_jac(lambda x: SQP.numerical_jac(f,x), x)
        hess = (hess + np.transpose(hess))/2

        w, v = linalg.eig(hess)
        mineig = np.min(w)
        if mineig < 0:
            print('    negative hessian detected, adjusting by ', - mineig)
            hess = hess + (-mineig) * np.identity(x.size)
        return (grad, hess)

    @staticmethod
    def f_linear(x):
        c = np.matrix([0.015273927029036, 0.746785676564429, 0.445096432287947])
        return c*x

    @staticmethod
    def f_quadratic(x):
        c = np.matrix('0.015273927029036; 0.746785676564429; 0.445096432287947')
        A = np.matrix('2.976991824249534   1.934366257364652  -0.410688884621065; 1.934366257364652   2.815433927071369   0.775017509184702; -0.410688884621065   0.775017509184702   1.522648051834486')
        return 0.5*np.transpose(x)*A*x+ np.transpose(c)*x

    @staticmethod
    def f_exponential(x):
        A = np.matrix('-0.4348    0.4694    0.5529    1.0184   -1.2344;\
                      -0.0793   -0.9036   -0.2037   -1.5804    0.2888; \
                      1.5352    0.0359   -2.0543   -0.0787   -0.4293; \
                      -0.6065   -0.6275    0.1326   -0.6817    0.0558; \
                      -1.3474    0.5354    1.5929   -1.0246   -0.3679')

        b = np.matrix('-3.0319; 1.8434; 1.5232; 5.0773; 1.7738')

        a = A*x + b
        return np.sum(np.exp(a));

    @staticmethod
    def hinge(x):
        return np.sum(np.maximum(x, np.zeros(x.shape)))

    @staticmethod
    def calc_merit(x, Q, q, f, g, h, penalty_coeff):
        return f(x) + q*x + 0.5*np.transpose(x)*(Q*x) + penalty_coeff*(SQP.hinge(g(x)) + linalg.norm(h(x),1))

    def penalty_sqp(self, x0, Q, q, f, A_ineq, b_ineq, A_eq, b_eq, g, h):
        trust_box_size = self.initial_trust_box_size
        penalty_coeff = self.initial_penalty_coeff
        # penalty_coeff = 100

        x, success = SQP.find_closest_feasible_point(x0, A_ineq, b_ineq, A_eq, b_eq)
        if not success:
            return (x, success)

        for i in range(self.max_merit_coeff_increases):
            x, trust_box_size, success = self.minimize_merit_function(x, Q, q, f, A_ineq, b_ineq, A_eq, b_eq, g, h, penalty_coeff, trust_box_size)
            print '\n'

            constraints_satisfied = True
            if self.g_use_numerical:
                gval = g(x)
                gjac = SQP.numerical_jac(g,x)
            else:
                gval, gjac = g(x)
            if abs(h(x)) > self.cnt_tolerance:
                constraints_satisfied = False
            elif any(gval > self.cnt_tolerance):
                constraints_satisfied = False

            if not constraints_satisfied:
                penalty_coeff = penalty_coeff*self.merit_coeff_increase_ratio
                trust_box_size = self.initial_trust_box_size
            else:
                return (x, success)
            # ipdb.set_trace()
        return (x, False)
    
    @staticmethod
    def find_closest_feasible_point(x0, A_ineq, b_ineq, A_eq, b_eq):
        success = True

        if any(A_ineq*x0 > b_ineq) or any(A_eq*x0 != b_eq):
            print("initialization doesn't satisfy linear constraints. Finding the closest feasible point")

            r, c = x0.shape
            xp = cvx.Variable(r,c)
            obj = cvx.norm(xp - x0,2)
            obj = cvx.Minimize(obj)
            
            constraints = [A_ineq*xp <= b_ineq, A_eq*xp == b_eq]

            prob = cvx.Problem(obj, constraints)
            # import ipdb; ipdb.set_trace()
            try:
                prob.solve(verbose=False, solver='GUROBI')
            except:
                print ("solver error")

            if prob.status != "optimal":
                success = False
                print("Couldn't find a point satisfying linear constraints")
            return (xp.value, success)
        return (x0, success)

    def minimize_merit_function(self, x, Q, q, f, A_ineq, b_ineq, A_eq, b_eq, g, h, penalty_coeff, trust_box_size):
        dim_x = x.size

        success = True
        sqp_iter = 1

        while True:
            # print("x: {0}".format(x))
            print("  sqp_iter: {0}".format(sqp_iter))

            # get gradients anad hess info for f,g and h
            #Use parameters for gradients and hess and trust box?
            #sci.derivative(f, x0, dx, n)
            fval = f(x)
            fgrad, fhess = SQP.numerical_grad_hess(f,x)
            if self.g_use_numerical:
                gval = g(x)
                gjac = SQP.numerical_jac(g,x)
            else:
                gval, gjac = g(x)
            hval = h(x)
            hjac = SQP.numerical_jac(h,x)

            merit = SQP.calc_merit(x, Q, q, f, lambda x: gval, h, penalty_coeff)

            while True:
                print("    trust region size: {0}".format(trust_box_size))

                xp = cvx.Variable(dim_x)

                #better name?
                penalty_obj = cvx.sum_entries(cvx.pos(gval + gjac*(xp-x)))
                penalty_obj += cvx.norm(hval + hjac*(xp-x), 1)

                obj = q*xp
                # ipdb.set_trace()
                if np.any(Q):
                    obj += 0.5 * cvx.quad_form(xp, Q)
                obj += fval
                obj += fgrad*(xp-x)
                if np.any(fhess):
                    obj += cvx.quad_form(xp-x, fhess)
                    # obj += np.transpose(xp-x)*fhess*(xp-x)
                obj += penalty_coeff * penalty_obj

                obj = cvx.Minimize(obj)

                constraints = [A_ineq*xp <= b_ineq, A_eq*xp == b_eq, cvx.norm(xp-x,2) <= trust_box_size]

                xp.value = x
                print "obj value: ", obj.value
                print "merit: ", merit

                if not np.isclose(obj.value, merit, atol=1e-3):
                    ipdb.set_trace()

                prob = cvx.Problem(obj, constraints)
                try:
                    prob.solve(verbose=False, solver='GUROBI')
                except:
                    print ("solver error")

                if prob.status != 'optimal':
                    print 'problem status: ', prob.status
                    print('Failed to solve QP subproblem.')
                    success = False
                    return (x, trust_box_size, success)

                # print("xp value: {0}".format(xp.value))

                model_merit = prob.value

                print 'evaluating for xp'
                if self.g_use_numerical:
                    gval2 = g(xp.value)
                    # gjach = SQP.numerical_jac(g,xp.value)
                else:
                    gval2, gjac2 = g(xp.value)
                new_merit = SQP.calc_merit(xp.value, Q, q, f, lambda x: gval2, h, penalty_coeff)
                approx_merit_improve = merit - model_merit
                exact_merit_improve = merit - new_merit
                merit_improve_ratio = exact_merit_improve / approx_merit_improve

                print("      approx_merit_improve: {0}. exact_merit_improve: {1}. merit_improve_ratio: {2}".format(approx_merit_improve, exact_merit_improve, merit_improve_ratio))

                if approx_merit_improve < -1e-5:
                    print("Approximate merit function got worse ({0})".format(approx_merit_improve))
                    print("Either convexification is wrong to zeroth order, or you're in numerical trouble.")
                    success = False
                    return (x, trust_box_size, success)
                elif approx_merit_improve < self.min_approx_improve:
                    print("Converged: y tolerance")
                    x = xp.value
                    #some sort of callback
                    return (xp.value, trust_box_size, success)
                elif (exact_merit_improve < 0) or (merit_improve_ratio < self.improve_ratio_threshold):
                    print("Shrinking trust region")
                    trust_box_size = trust_box_size * self.trust_shrink_ratio
                    ipdb.set_trace()
                else:
                    print("Growing trust region")
                    trust_box_size = trust_box_size * self.trust_expand_ratio
                    x = xp.value
                    break #from trust region loop

                if trust_box_size < self.min_trust_box_size:
                    print("Converged: x tolerance")
                    return (x, trust_box_size, success) 

            sqp_iter = sqp_iter + 1




# K = 2
# T = 10

# x0 = #initial trajectory

# v = -1 * np.ones((K*T-K,1))
# d = np.concatenate((np.ones((K*T,1)), np.zeros((K,1))))

# P = np.diagflat(v, K) + np.diagflat(d)
# Q = np.transpose(P)*P

def test_sqp():
    from problem import Problem
    prob = Problem()
    prob.problem8()
    prob.test()

def test_grad():
    x0 = np.transpose(np.matrix([0.418649467727506, 0.846221417824324, 0.525152496305172]))
    g1 = SQP.numerical_jac(SQP.f_linear, x0)
    g2, h2 = SQP.numerical_grad_hess(SQP.f_quadratic, x0)
    x_test = np.matrix('0.950129285147175;   0.231138513574288;   0.606842583541787;   0.485982468709300;   0.891298966148902');
    g3, h3 = SQP.numerical_grad_hess(SQP.f_exponential, x_test)

if __name__ == "__main__":
    test_sqp()
