from opt.opt_prob import OptProb
from opt.variable import Variable
import numpy as np
import numpy.linalg as linalg
import scipy.misc as sci
import ipdb

import time

class Solver(object):
    """
    f is the function that needs to be approximately in the cost
    g is the inequality constraint
    h is the equality constraint
    """

    def __init__(self):
        self.improve_ratio_threshold = .25
        # self.min_trust_box_size = 1e-4
        self.min_trust_box_size = 1e-2
        # self.min_approx_improve = 1e-4
        self.min_approx_improve = 1e-2
        # self.min_approx_improve = 3e-1
        # self.min_approx_improve = 1e-1
        self.trust_shrink_ratio = .1
        self.trust_expand_ratio = 1.5
        # self.cnt_tolerance = 1e-4
        self.cnt_tolerance = 1e-2
        self.max_merit_coeff_increases = 1
        self.merit_coeff_increase_ratio = 10 # doesn't matter if max_merit_coeff_increases = 1
        # self.initial_trust_box_size = 1e-4
        # self.initial_trust_box_size = .01
        # self.initial_trust_box_size = .03
        self.initial_trust_box_size = .1
        # self.initial_trust_box_size = 1
        # self.initial_trust_box_size = 2
        # self.initial_trust_box_size = 3
        # self.initial_trust_box_size = 10
        # self.initial_penalty_coeff = 1.
        self.initial_penalty_coeff = 10.
        # self.initial_penalty_coeff = 0.3
        # self.initial_penalty_coeff = 0.1
        self.callback = []

        # self.epsilon = 5e-3
        # self.epsilon = 1e-2
        # self.epsilon = 2e-2
        self.epsilon = 3e-2
        self.sqp_iters = 0

    def big_sqp(self, opt_probs, params):
        start = time.time()
        hl_opt_prob = OptProb()
        for prob in opt_probs:
            hl_opt_prob.add_opt_prob(prob)
        for param in params:
            hl_opt_prob.add_constraints(param.get_eq_constraints())
        success = self.penalty_sqp(hl_opt_prob)
        end = time.time()
        print "big_sqp time: ", end-start
        import ipdb; ipdb.set_trace() # BREAKPOINT
        return success


    # @profile
    def admm_sqp(self, opt_probs, params):
        start = time.time()
        epsilon = self.epsilon*len(params)
        while True:
            for opt_prob in opt_probs:
                successs = self.penalty_sqp(opt_prob)
                opt_prob.callback()
                # if not success:
                #     import ipdb; ipdb.set_trace() # BREAKPOINT
                #     return False

            diff = 0
            for param in params:
                diff += param.dual_update()

            print "diff: ", diff
            if diff < epsilon:
                break

        end = time.time()
        print "admm_sqp time: ", end-start
        print "sqp_iters: ", self.sqp_iters
        return True

    # @profile
    def sqp_convexify_admm(self, opt_probs, params):
        start = time.time()

        penalty_coeff = self.initial_penalty_coeff

        dual_updates = 0
        for opt_prob in opt_probs:
            # success = self.find_closest_feasible_point(opt_prob)
            success = opt_prob.find_closest_feasible_point()
            if not success:
                import ipdb; ipdb.set_trace() # BREAKPOINT
                return success

        penalty_increases = 0
        for i in range(self.max_merit_coeff_increases+1):
            num_probs = len(opt_probs)
            trust_box_sizes = []
            for i in range(num_probs):
                # trust_box_sizes.append(self.initial_trust_box_size/10.0)
                trust_box_sizes.append(self.initial_trust_box_size)

            for param in params:
                param.reset()

            success, updates = self.min_merit_fn_admm(opt_probs, params, penalty_coeff, trust_box_sizes)
            dual_updates += updates
            # import ipdb; ipdb.set_trace() # BREAKPOINT

            for opt_prob in opt_probs:
                if not opt_prob.constraints_satisfied(self.cnt_tolerance):
                    penalty_coeff = penalty_coeff*self.merit_coeff_increase_ratio
                    break
            else:
                # import ipdb; ipdb.set_trace() # BREAKPOINT
                success = True
                break

            penalty_increases += 1
            print penalty_increases, " penalty increase to ", penalty_coeff
        else:
            success = False

        end = time.time()
        print "sqp_admm time: ", end-start
        print "sqp_iters: ", self.sqp_iters
        print "penalty increases: ", penalty_increases
        import ipdb; ipdb.set_trace() # BREAKPOINT
        return success

    # @profile
    def penalty_sqp(self, prob):
        start = time.time()
        trust_box_size = self.initial_trust_box_size
        penalty_coeff = self.initial_penalty_coeff

        prob.find_closest_feasible_point()
        # x, success = SQP.find_closest_feasible_point(x, x0, constraints)
        # if not success:
        #     return (x, success)

        for i in range(self.max_merit_coeff_increases):
            trust_box_size, success = self.minimize_merit_function(prob, penalty_coeff, trust_box_size)
            print '\n'

            constraints_satisfied = prob.constraints_satisfied(self.cnt_tolerance)

            if not constraints_satisfied:
                penalty_coeff = penalty_coeff*self.merit_coeff_increase_ratio
                trust_box_size = self.initial_trust_box_size
            else:
                end = time.time()
                print "sqp time: ", end-start
                return success
        end = time.time()
        print "sqp time: ", end-start
        return False

    # @profile
    def min_merit_fn_admm(self, opt_probs, params, penalty_coeff, trust_box_sizes):
        success = True
        # sqp_iter = 1

        # approximation loop
        dual_updates = 0
        # trust_box_size = cvx.Parameter(sign="positive", value = self.initial_trust_box_size)
        converged = False
        while not converged and dual_updates <= 4:
            for prob, size, i in zip(opt_probs, trust_box_sizes, range(len(opt_probs))):
                sqp_iter = 1
                print
                print prob.hl_action.name
                print("  sqp_iter: {0}".format(sqp_iter))
                # trust_box_size.value = np.minimum(size, self.initial_trust_box_size*10)
                # trust_box_size.value = np.minimum(size, 3)
                # trust_box_size.value = size
                merit = prob.val(penalty_coeff)
                prob.convexify(penalty_coeff)
                prob.save()

                trust_box_size = self.initial_trust_box_size

                shrink = False
                # Trust region search
                while trust_box_size >= self.min_trust_box_size:
                    sqp_iter += 1
                    self.sqp_iters += 1
                    print("    trust region size: {0}".format(trust_box_size))

                    prob.add_trust_region(trust_box_size)
                    prob.optimize()

                    model_merit = prob.model.objVal
                    new_merit = prob.val(penalty_coeff)

                    approx_merit_improve = merit - model_merit
                    exact_merit_improve = merit - new_merit
                    merit_improve_ratio = exact_merit_improve / approx_merit_improve

                    print("      approx_merit_improve: {0}. exact_merit_improve: {1}. merit_improve_ratio: {2}".format(approx_merit_improve, exact_merit_improve, merit_improve_ratio))

                    if approx_merit_improve < -1e-5:
                        print("Approximate merit function got worse ({0})".format(approx_merit_improve))
                        print("Either convexification is wrong to zeroth order, or you're in numerical trouble.")
                        success = False
                        prob.restore()
                        # return  (trust_box_size, success)
                        import ipdb; ipdb.set_trace() # BREAKPOINT
                        break
                    elif approx_merit_improve < self.min_approx_improve:
                        print("Converged:because improvement was small ({0} < {1})".format(approx_merit_improve, self.min_approx_improve))
                        prob.callback()
                        prob.restore()
                        break
                        # return (trust_box_size, success)
                    elif (exact_merit_improve < 0) or (merit_improve_ratio < self.improve_ratio_threshold):
                        print("Shrinking trust region")
                        shrink = True
                        prob.restore()
                        trust_box_size = trust_box_size * self.trust_shrink_ratio

                    else:
                        print("Growing trust region")
                        prob.callback()
                        break #from trust region loop
                else:
                    print("Converged: x tolerance")
                # if prob.hl_action.name == "move2":
                #     import ipdb; ipdb.set_trace() # BREAKPOINT

            # sqp_iter = sqp_iter + 1

            diff = 0
            converged = True
            for param in params:
                diff = param.dual_update()
                if diff > self.epsilon:
                    converged = False
                print param.name, " disagree by ", diff
            dual_updates += 1
            # import ipdb; ipdb.set_trace() # BREAKPOINT
            # if penalty_coeff.value > 0.1:
        else:
            return True, dual_updates

    # @profile
    def minimize_merit_function(self, prob, penalty_coeff, trust_box_size):
        success = True
        sqp_iter = 1

        while True:
            print("  sqp_iter: {0}".format(sqp_iter))

            prob.convexify(penalty_coeff)
            merit = prob.val(penalty_coeff)
            prob.save()

            while True:
                print("    trust region size: {0}".format(trust_box_size))

                prob.add_trust_region(trust_box_size)
                # import ipdb; ipdb.set_trace()
                prob.clear_plots()
                prob.optimize()
                prob.plot()

                model_merit = prob.model.objVal
                new_merit = prob.val(penalty_coeff)

                approx_merit_improve = merit - model_merit
                exact_merit_improve = merit - new_merit
                merit_improve_ratio = exact_merit_improve / approx_merit_improve

                print("      approx_merit_improve: {0}. exact_merit_improve: {1}. merit_improve_ratio: {2}".format(approx_merit_improve, exact_merit_improve, merit_improve_ratio))

                if approx_merit_improve < -1e-5:
                    print("Approximate merit function got worse ({0})".format(approx_merit_improve))
                    print("Either convexification is wrong to zeroth order, or you're in numerical trouble.")
                    success = False
                    prob.restore()
                    return (trust_box_size, success)
                elif approx_merit_improve < self.min_approx_improve:
                    print("Converged: y tolerance")
                    # why do we restore if there is some improvement?
                    prob.restore()
                    return (trust_box_size, success)
                elif (exact_merit_improve < 0) or (merit_improve_ratio < self.improve_ratio_threshold):
                    # reset convex approximations of f,g and h to their original values
                    prob.restore()

                    print("Shrinking trust region")
                    trust_box_size = trust_box_size * self.trust_shrink_ratio
                else:
                    print("Growing trust region")
                    trust_box_size = trust_box_size * self.trust_expand_ratio
                    break #from trust region loop

                if trust_box_size < self.min_trust_box_size:
                    print("Converged: x tolerance")
                    return (trust_box_size, success)

            sqp_iter = sqp_iter + 1

if __name__ == "__main__":
    test_sqp()
