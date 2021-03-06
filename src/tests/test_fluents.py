import ipdb
from interface.hl_param import HLParam
from interface.fluents.fluent import LinEqFluent, LinLEFluent
from interface.fluents.aff_expr import AffExpr
import numpy as np
from opt.opt_prob import OptProb
from opt.constraints import Constraints
from opt.variable import Variable, Constant
import gurobipy as grb
GRB = grb.GRB


class TestParam(HLParam):

    def generator(self):
        yield np.array([[1], [0], [0]], dtype=np.float)

def to_gurobi_expr(aff_expr, param_to_var):
    expr = 0.0 + aff_expr.constant

    for param, coeff in aff_expr.items():
        var = param_to_var[param]
        if isinstance(var, Constant):
            expr += np.dot(param.get_value(), coeff)
        elif isinstance(var, Variable):
            expr += np.dot(var.get_grb_vars(), coeff)
        else:
            raw_input("shouldn't be here")
            import ipdb; ipdb.set_trace()

    return expr

def test_aff_expr():
    one = HLParam("x", (1, 1), value=np.array([[1.0]]))
    two = HLParam("x", (1, 1), value=np.array([[2.0]]))
    expr = AffExpr({one: 3, two: 5}, constant=np.array([[0.5]]))
    assert expr.value() == 1.0 * 3 + 2.0 * 5 + 0.5


def test_fluent_zero_not_equal_to_e1():
    param = TestParam("test", 3, 1)
    param.resample()

    zero = AffExpr(constant=np.array([[0], [0], [0]], dtype=np.float))

    lhs = AffExpr({param: 1})

    fluent = LinEqFluent('test_eq', 0, lhs, zero)
    assert not fluent.satisfied()


def test_fluent_equals():
    # model = grb.Model()
    # obj = grb.QuadExpr()

    param = TestParam("test", 3, 1)
    param.resample()

    three = AffExpr(constant=np.array([[3], [0], [-2]], dtype=np.float))

    # var = Variable(model, param)
    lhs = AffExpr({param: 2}, constant=np.array([[1], [0], [-2]]))

    fluent = LinEqFluent('test_eq', 0, lhs, three)
    assert fluent.satisfied()


def test_fluent_equal_after_opt():
    # model = grb.Model()
    # obj = grb.QuadExpr()

    param = TestParam("test", 3, 1)
    param.resample()

    zero = AffExpr(constant=np.array([[0], [0], [0]], dtype=np.float))

    # var = Variable(model, param)
    lhs = AffExpr({param: 1})

    fluent = LinEqFluent('test_eq', 0, lhs, zero)
    assert not fluent.satisfied()


def test_eq_cnt_with_gurobi():
    model = grb.Model()
    obj = grb.QuadExpr()

    param = TestParam("test", 3, 1)
    param.resample()
    var = Variable(model, param)

    zero = AffExpr(constant=np.array([[0], [0], [0]], dtype=np.float))

    # var = Variable(model, param)
    lhs = AffExpr({param: 1})
    fluent = LinEqFluent('test_eq', 0, lhs, zero)

    variables = [Variable(model, param)]
    param_to_var = {param: variables[0]}

    constraints = Constraints(model)
    if isinstance(fluent, LinEqFluent):
        lhs = to_gurobi_expr(fluent.lhs, param_to_var)
        rhs = to_gurobi_expr(fluent.rhs, param_to_var)
        model.update()
        constraints.add_eq_cntr(lhs, rhs)
    assert not fluent.satisfied()

    for var in variables:
        for i in range(var.rows):
            for j in range(var.cols):
                v = var.grb_vars[i, j]
                obj += v*v - 2*v + 1

    model.setObjective(obj)
    model.update()
    model.optimize()

    var.update()
    var.update_hl_param()
    assert fluent.satisfied()


def test_le_cnt_with_gurobi():
    model = grb.Model()
    obj = grb.QuadExpr()

    param = TestParam("test", 3, 1)
    param.resample()
    var = Variable(model, param)

    twoe1 = AffExpr(constant=np.array([[2], [0], [-1]], dtype=np.float))

    # var = Variable(model, param)
    lhs = AffExpr({param: 1})
    fluent = LinLEFluent('test_eq', 0, lhs, twoe1)

    variables = [Variable(model, param)]
    param_to_var = {param: variables[0]}

    constraints = Constraints(model)
    if isinstance(fluent, LinLEFluent):
        lhs = to_gurobi_expr(fluent.lhs, param_to_var)
        rhs = to_gurobi_expr(fluent.rhs, param_to_var)
        model.update()
        constraints.add_leq_cntr(lhs, rhs)
    assert not fluent.satisfied()

    for var in variables:
        for i in range(var.rows):
            for j in range(var.cols):
                v = var.grb_vars[i, j]
                obj += v*v - 2*v + 1

    model.setObjective(obj)
    model.update()
    model.optimize()

    var.update()
    var.update_hl_param()
    assert fluent.satisfied()
    assert np.all(variables[0].get_val() == np.array([[1], [0], [-1]]))


def test_le_cnt_with_opt():
    prob = OptProb()
    model = prob.get_model()

    param = TestParam("test", 3, 1)
    param.resample()
    var = Variable(model, param)
    prob.add_var(var)

    twoe1 = AffExpr(constant=np.array([[2], [0], [-1]], dtype=np.float))

    # var = Variable(model, param)
    lhs = AffExpr({param: 1})
    fluent = LinLEFluent('test_eq', 0, lhs, twoe1)

    variables = [var]
    param_to_var = {param: var}

    constraints = Constraints(model)
    if isinstance(fluent, LinLEFluent):
        lhs = to_gurobi_expr(fluent.lhs, param_to_var)
        rhs = to_gurobi_expr(fluent.rhs, param_to_var)
        model.update()
        constraints.add_leq_cntr(lhs, rhs)
    assert not fluent.satisfied()

    prob.add_constraints(constraints)

    obj = grb.QuadExpr()
    for var in variables:
        for i in range(var.rows):
            for j in range(var.cols):
                v = var.grb_vars[i, j]
                obj += v*v - 2*v + 1

    prob.obj_sqp = obj
    prob.optimize()

    var.update_hl_param()
    assert fluent.satisfied()
    assert np.all(variables[0].get_val() == np.array([[1], [0], [-1]]))

test_aff_expr()
test_fluent_zero_not_equal_to_e1()
test_fluent_equals()
test_eq_cnt_with_gurobi()
test_le_cnt_with_gurobi()
test_le_cnt_with_opt()
