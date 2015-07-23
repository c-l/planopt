import cvxpy as cvx
from opt.variable import Variable
import numpy as np
import numpy.linalg as linalg
import scipy.misc as sci
import ipdb

class ADMM_SQP(object):
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
        self.full_hessian = True
        self.callback = []

    # @profile
    def penalty_sqp(self, opt_prob):
        # trust_box_size = self.initial_trust_box_size
        # penalty_coeff = self.initial_penalty_coeff
        trust_box_size = cvx.Parameter(sign="positive", value = self.initial_trust_box_size)
        penalty_coeff = cvx.Parameter(sign="positive", value = self.initial_penalty_coeff)

        success = ADMM_SQP.find_closest_feasible_point(opt_prob)
        if not success:
            return success

        for i in range(self.max_merit_coeff_increases):
            trust_box_size, success = self.minimize_merit_function(opt_prob, penalty_coeff, trust_box_size)
            print '\n'

            if not opt_prob.constraints_satisfied(self.cnt_tolerance):
                penalty_coeff.value = penalty_coeff.value*self.merit_coeff_increase_ratio
                trust_box_size.value = self.initial_trust_box_size
            else:
                return success
        return False
    
    @staticmethod
    def find_closest_feasible_point(opt_prob):
        success = True
        for x in opt_prob.xs:
            x.value = x.cur_value

        feasible = opt_prob.constraints.linear_constraints_satisfied()

        if feasible:
            return success
        else:
            obj = 0
            constraints = opt_prob.constraints.linear_constraints
            for x in opt_prob.xs:
                obj += cvx.norm(x - x.cur_value,2)

            gp = opt_prob.hl_action.hl_plan.hl_actions[0].gp.value

            prob = cvx.Problem(cvx.Minimize(obj), constraints)
            try:
                prob.solve(verbose=False, solver='GUROBI')
            except:
                import ipdb; ipdb.set_trace() # BREAKPOINT
                print ("solver error")

            if prob.status != "optimal":
                success = False
                print("Couldn't find a point satisfying linear constraints")
            else:
                for x in opt_prob.xs:
                    x.cur_value = x.value
            return success

    def minimize_merit_function(self, opt_prob, penalty_coeff, trust_box_size):
        success = True
        sqp_iter = 1

        x_cur = []
        objective, constraints = opt_prob.convexify(penalty_coeff, trust_box_size)
        prob = None

        while True:
            print("  sqp_iter: {0}".format(sqp_iter))
            merit = objective.value

            while True:
                print("    trust region size: {0}".format(trust_box_size.value))

                try:
                    prob = cvx.Problem(cvx.Minimize(objective), constraints)
                    prob.solve(verbose=False, solver='GUROBI')
                    # prob.solve(verbose=False, solver='ECOS')
                except:
                    import ipdb; ipdb.set_trace() # BREAKPOINT
                    # prob.solve(verbose=True, solver='GUROBI')
                    prob.solve(verbose=True, solver='ECOS')
                    print ("solver error")

                if prob.status != 'optimal':
                    if prob.status == None:
                        ipdb.set_trace()
                    else:
                        print 'problem status: ', prob.status
                        print('Failed to solve QP subproblem.')
                        success = False
                        return (trust_box_size, success)

                model_merit = prob.value

                # updating current x in which convex approximation is computed and saving previous current x value
                x_cur = []
                for x in opt_prob.xs:
                    x_cur.append(x.cur_value)
                    x.cur_value = x.value
                objective, constraints = opt_prob.convexify(penalty_coeff, trust_box_size)

                new_merit = objective.value

                approx_merit_improve = merit - model_merit
                exact_merit_improve = merit - new_merit
                merit_improve_ratio = exact_merit_improve / approx_merit_improve

                print("      approx_merit_improve: {0}. exact_merit_improve: {1}. merit_improve_ratio: {2}".format(approx_merit_improve, exact_merit_improve, merit_improve_ratio))

                if approx_merit_improve < -1e-5:
                    print("Approximate merit function got worse ({0})".format(approx_merit_improve))
                    print("Either convexification is wrong to zeroth order, or you're in numerical trouble.")
                    success = False
                    for i in range(len(opt_prob.xs)):
                        opt_prob.xs[i].cur_value = x_cur[i]
                        # x's value needs to be also reset so that the merit can be computed correctly
                        opt_prob.xs[i].value = x_cur[i]
                    return  (trust_box_size, success)
                elif approx_merit_improve < self.min_approx_improve:
                    print("Converged: y tolerance")
                    return (trust_box_size, success)
                elif (exact_merit_improve < 0) or (merit_improve_ratio < self.improve_ratio_threshold):
                    # reset convex approximations of optimization problem
                    for i in range(len(opt_prob.xs)):
                        opt_prob.xs[i].cur_value = x_cur[i]
                        # x's value needs to be also reset so that the merit can be computed correctly
                        opt_prob.xs[i].value = x_cur[i]

                    objective, constraints = opt_prob.convexify(penalty_coeff, trust_box_size)

                    print("Shrinking trust region")
                    trust_box_size.value = trust_box_size.value * self.trust_shrink_ratio

                else:
                    print("Growing trust region")
                    trust_box_size.value = trust_box_size.value * self.trust_expand_ratio
                    break #from trust region loop

                if trust_box_size.value < self.min_trust_box_size:
                    print("Converged: x tolerance")
                    return (trust_box_size, success) 

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
