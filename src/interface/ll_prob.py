from opt.opt_prob import OptProb
from opt.variable import Variable, Constant
from opt.solver import Solver
from opt.constraints import Constraints
from interface.fluents.fluent import AndFluent, LinFluent, LinEqFluent, LinLEFluent, FnFluent, FnLEFluent, FnEQFluent
import numpy as np
from IPython import embed as shell
import settings

class LLProb(object):
    def __init__(self, hlas=None):
        if hlas is None:
            self.hlas = []
        else:
            self.hlas = hlas

    def add_fluent_to_constraints(self, constraints, fluent, param_to_var):
        if isinstance(fluent, AndFluent):
            for subfluent in fluent.fluents:
                self.add_fluent_to_constraints(constraints, subfluent, param_to_var)
        elif isinstance(fluent, FnFluent):
            fluent.fn.to_gurobi_fn(param_to_var)
            if isinstance(fluent, FnLEFluent):
                constraints.add_nonlinear_ineq_constraint(fluent.fn)
            elif isinstance(fluent, FnEQFluent):
                constraints.add_nonlinear_eq_constraint(fluent.fn)
        elif isinstance(fluent, LinFluent):
            lhs = self.to_gurobi_expr(fluent.lhs, param_to_var)
            rhs = self.to_gurobi_expr(fluent.rhs, param_to_var)
            if isinstance(fluent, LinLEFluent):
                constraints.add_leq_cntr(lhs, rhs)
            elif isinstance(fluent, LinEqFluent):
                constraints.add_eq_cntr(lhs, rhs)
        else:
            raw_input("shouldn't be here")
            import ipdb; ipdb.set_trace()


    def to_gurobi_expr(self, aff_expr, param_to_var):
        expr = 0.0 + aff_expr.constant

        for param, coeff in aff_expr.items():
            var = param_to_var[param]
            if type(coeff) is tuple:
                lhs = coeff[0]
                rhs = coeff[1]
                if isinstance(var, Constant):
                    expr = expr + np.dot(lhs, np.dot(param.get_value(), rhs))
                elif isinstance(var, Variable):
                    expr = expr + np.dot(lhs, np.dot(var.get_grb_vars(), rhs))
                else:
                    raw_input("shouldn't be here")
            else:
                if isinstance(var, Constant):
                    expr = expr + np.dot(param.get_value(), coeff)
                elif isinstance(var, Variable):
                    expr = expr + np.dot(var.get_grb_vars(), coeff)
                else:
                    raw_input("shouldn't be here")

        return expr

    def model_cnts_from_hla(self, model, param_to_var, priority):
        constraints = Constraints(model)
        for hla in self.hlas:
            for fluent in hla.preconditions:
                if fluent.priority <= priority:
                    fluent.pre()
            for fluent in hla.postconditions:
                if fluent.priority <= priority:
                    fluent.post()
            for fluent in hla.preconditions + hla.postconditions:
                if fluent.priority <= priority:
                    self.add_fluent_to_constraints(constraints, fluent, param_to_var)
        return constraints

    def solve_at_priority(self, priority, fix_sampled_params=False, recently_sampled=None):
        # initialize gurobi Model object
        prob = OptProb()
        model = prob.get_model()
        if recently_sampled is None:
            recently_sampled = []

        params = []
        for hla in self.hlas:
            params += hla.get_params()
            prob.add_hla(hla)

        param_to_var = {}
        for param in params:
            if param in param_to_var:
                continue
            if param.is_resampled and fix_sampled_params:
                print param.name, "is fixed."
                const = Constant(param)
                param_to_var[param] = const
            elif not param.is_var:
                print param.name, "is not a var."
                const = Constant(param)
                param_to_var[param] = const
            else: # param.is_var
                var = Variable(model, param, recently_sampled=(param in recently_sampled))
                param_to_var[param] = var
                prob.add_var(var)
        model.update()

        # max(priority, 0) because priority = -1 used for straight-line init
        constraints = self.model_cnts_from_hla(model, param_to_var, max(priority, 0))
        prob.add_constraints(constraints)
        for param, var in param_to_var.items():
            var.set(param.value)

        for hla in self.hlas:
            if hla.cost != 0.0:
                hla.cost.to_gurobi_fn(param_to_var)
                prob.inc_obj(hla.cost)

        solver = Solver()

        if fix_sampled_params or recently_sampled:
            solver.initial_trust_box_size = 1e5
            solver.max_merit_coeff_increases = 1
        if priority == -1:
            # initialize straight-line trajectories
            prob.initialize_traj(mode="straight")
        elif priority == 0:
            # initialize from adapted previous trajectories
            prob.initialize_traj(mode="adapt")
        else:
            solver.penalty_sqp(prob, do_early_converge=settings.DO_EARLY_CONVERGE)

        for param, var in param_to_var.items():
            var.update_hl_param()

        self.traj_cost = sum(prob.val(0)[0])
