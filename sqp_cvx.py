import cvxpy as cvx
import numpy as np
import numpy.linalg as linalg
import scipy.misc as sci
import ipdb

class SQP(object):
    """
    f is the function that needs to be approximately in the cost
    g is the inequality constraint
    h is the equality constraint
    """

    def __init__(self):
        self.improve_ratio_threshold = .25
        self.min_trust_box_size = 1e-4
        self.min_approx_improve = 1e-4
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

    def penalty_sqp(self, x, x0, objective, constraints, f, g, h):
        # trust_box_size = self.initial_trust_box_size
        # penalty_coeff = self.initial_penalty_coeff
        trust_box_size = cvx.Parameter(sign="positive", value = self.initial_trust_box_size)
        penalty_coeff = cvx.Parameter(sign="positive", value = self.initial_penalty_coeff)

        x, success = SQP.find_closest_feasible_point(x, x0, constraints)
        if not success:
            return (x, success)

        for i in range(self.max_merit_coeff_increases):
            x, trust_box_size, success = self.minimize_merit_function(x, objective, constraints, f, g, h, penalty_coeff, trust_box_size)
            print '\n'

            constraints_satisfied = True
            if self.g_use_numerical:
                gval = g(x.value)
                gjac = SQP.numerical_jac(g,x.value)
            else:
                gval, gjac = g(x.value)
            # TODO: need to account for case where h is not numerical
            if abs(h(x.value)) > self.cnt_tolerance:
                constraints_satisfied = False
            elif any(gval > self.cnt_tolerance):
                constraints_satisfied = False

            if not constraints_satisfied:
                penalty_coeff.value = penalty_coeff.value*self.merit_coeff_increase_ratio
                trust_box_size.value = self.initial_trust_box_size
            else:
                return (x, success)
        return (x, False)
    
    @staticmethod
    def find_closest_feasible_point(x, x0, constraints):
        success = True
        
        x.value = x0
        feasible = True
        for constraint in constraints:
            if not np.all(constraint.value):
                feasible = False
                break

        if feasible:
            return (x,success)
        else:
            obj = cvx.norm(x - x0,2)
            obj = cvx.Minimize(obj)

            prob = cvx.Problem(obj, constraints)
            # import ipdb; ipdb.set_trace()
            try:
                prob.solve(verbose=False, solver='GUROBI')
            except:
                print ("solver error")

            if prob.status != "optimal":
                success = False
                print("Couldn't find a point satisfying linear constraints")
            return (x, success)

    def update_fgh(self, x, f, g, h, fval, fgrad, fhess, gval, gjac, hval, hjac):
        fval.value, fgrad.value, fhess.value, gval.value, gjac.value, hval.value, hjac.value = self.convexify_fgh(x, f, g, h)

    def convexify_fgh(self, x, f, g, h):
        fval = f(x.value)
        fgrad, fhess = SQP.numerical_grad_hess(f,x.value)
        
        if self.g_use_numerical:
            gval = g(x.value)
            gjac = SQP.numerical_jac(g,x.value)
        else:
            gval, gjac = g(x.value)

        hval = h(x.value)
        hjac = SQP.numerical_jac(h,x.value)
        return (fval, fgrad, fhess, gval, gjac, hval, hjac)

    def minimize_merit_function(self, xp, objective, constraints_wo_trust_box, f, g, h, penalty_coeff, trust_box_size):
        objective_wo_fhess = objective

        success = True
        sqp_iter = 1

        # x is a cvxpy variable
        rows, cols = xp.size
        x = cvx.Parameter(rows, cols, value=xp.value)
        
        # computing convex approximations of f, g, h
        fval, fgrad, fhess, gval, gjac, hval, hjac = self.convexify_fgh(x, f, g, h)

        # setting up parameters for approximations of f,g and h
        # setting up parameters allows cvxpy to avoid setting up the problem every iteration
        fval = cvx.Parameter(fval.shape[0], fval.shape[1], value=fval)
        fgrad = cvx.Parameter(fgrad.shape[0], fgrad.shape[1], value=fgrad)
        fhess = cvx.Parameter(fhess.shape[0], fhess.shape[1], value=fhess)
        gval = cvx.Parameter(gval.shape[0], gval.shape[1], value=gval)
        gjac = cvx.Parameter(gjac.shape[0], gjac.shape[1], value=gjac)
        hval = cvx.Parameter(hval.shape[0], hval.shape[1], value=hval)
        hjac = cvx.Parameter(hjac.shape[0], hjac.shape[1], value=hjac)
 
        # set up objective and constraints
        penalty_objective = cvx.sum_entries(cvx.pos(gval + gjac*(xp-x)))
        penalty_objective += cvx.norm(hval + hjac*(xp-x), 1)

        objective_wo_fhess += fval
        objective_wo_fhess += fgrad*(xp-x)
        objective_wo_fhess += penalty_coeff * penalty_objective

        objective = objective_wo_fhess
        if np.any(fhess.value):
            objective += cvx.quad_form(xp-x, fhess)
            # obj += np.transpose(xp-x)*fhess*(xp-x)
        # objective = cvx.Minimize(objective)

        constraints = constraints_wo_trust_box + [cvx.norm(xp-x,2) <= trust_box_size]
        prob = cvx.Problem(cvx.Minimize(objective), constraints)

       
        while True:
            print("  sqp_iter: {0}".format(sqp_iter))
            merit = objective.value

            while True:
                print("    trust region size: {0}".format(trust_box_size.value))

                try:
                    objective = objective_wo_fhess
                    if np.any(fhess.value):
                        objective += cvx.quad_form(xp-x, fhess)
                    prob = cvx.Problem(cvx.Minimize(objective), constraints)
                    prob.solve(verbose=False, solver='GUROBI')
                except:
                    prob.solve(verbose=True, solver='CVXOPT')
                    print ("solver error")

                if prob.status != 'optimal':
                    if prob.status == None:
                        ipdb.set_trace()
                    else:
                        print 'problem status: ', prob.status
                        print('Failed to solve QP subproblem.')
                        success = False
                        return (xp, trust_box_size, success)

                model_merit = prob.value

                # saving current convexified approximations of f,g and h
                fval_cur = fval.value
                fgrad_cur = fgrad.value
                fhess_cur = fhess.value

                gval_cur = gval.value
                gjac_cur = gjac.value

                hval_cur = hval.value
                hjac_cur = hjac.value

                x_cur = x.value

                # ipdb.set_trace()
                self.update_fgh(xp, f, g, h, fval, fgrad, fhess, gval, gjac, hval, hjac)

                x.value = xp.value
                new_merit = objective.value

                approx_merit_improve = merit - model_merit
                exact_merit_improve = merit - new_merit
                merit_improve_ratio = exact_merit_improve / approx_merit_improve

                print("      approx_merit_improve: {0}. exact_merit_improve: {1}. merit_improve_ratio: {2}".format(approx_merit_improve, exact_merit_improve, merit_improve_ratio))

                if approx_merit_improve < -1e-5:
                    print("Approximate merit function got worse ({0})".format(approx_merit_improve))
                    print("Either convexification is wrong to zeroth order, or you're in numerical trouble.")
                    success = False
                    xp.value = x_cur
                    return (xp, trust_box_size, success)
                elif approx_merit_improve < self.min_approx_improve:
                    print("Converged: y tolerance")
                    #some sort of callback
                    # print "x: ", xp.value
                    return (xp, trust_box_size, success)
                elif (exact_merit_improve < 0) or (merit_improve_ratio < self.improve_ratio_threshold):
                    # reset convex approximations of f,g and h to their original values
                    fval.value = fval_cur
                    fgrad.value = fgrad_cur
                    fhess.value = fhess_cur
                    gval.value = gval_cur
                    gjac.value = gjac_cur
                    hval.value = hval_cur
                    hjac.value = hjac_cur

                    x.value = x_cur

                    print("Shrinking trust region")
                    trust_box_size.value = trust_box_size.value * self.trust_shrink_ratio
                else:
                    print("Growing trust region")
                    trust_box_size.value = trust_box_size.value * self.trust_expand_ratio
                    # print "x: ", xp.value
                    break #from trust region loop

                if trust_box_size.value < self.min_trust_box_size:
                    print("Converged: x tolerance")
                    return (xp, trust_box_size, success) 

            sqp_iter = sqp_iter + 1


def test_sqp():
    # from problem_cvx import Problem
    # prob = Problem()
    # prob.problem8()
    # prob.test()

    from problem_cvx import Problem
    for i in range(9):
        prob = Problem()
        getattr(prob, "problem{0}".format(i))()
        prob.test()

def test_grad():
    x0 = np.transpose(np.matrix([0.418649467727506, 0.846221417824324, 0.525152496305172]))
    g1 = SQP.numerical_jac(SQP.f_linear, x0)
    g2, h2 = SQP.numerical_grad_hess(SQP.f_quadratic, x0)
    x_test = np.matrix('0.950129285147175;   0.231138513574288;   0.606842583541787;   0.485982468709300;   0.891298966148902');
    g3, h3 = SQP.numerical_grad_hess(SQP.f_exponential, x_test)

if __name__ == "__main__":
    test_sqp()
