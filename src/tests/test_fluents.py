import ipdb
from interface.hl_param import HLParam
from interface.fluents.lin_eq_fluent import LinEqFluent
from interface.fluents.lin_le_fluent import LinLEFluent
from interface.fluents.aff_expr import AffExpr
import numpy as np
from opt.opt_prob import OptProb
from opt.constraints import Constraints
from opt.variable import Variable
import gurobipy as grb
GRB = grb.GRB


class TestParam(HLParam):

    def generator(self):
        yield np.array([[1], [0], [0]], dtype=np.float)


def test_aff_expr():
    one = HLParam("x", 1, 1, value=np.array([[1.0]]))
    two = HLParam("x", 1, 1, value=np.array([[2.0]]))
    expr = AffExpr({one: 3, two: 5}, constant=np.array([[0.5]]))
    assert expr.value() == 1.0 * 3 + 2.0 * 5 + 0.5


def test_fluent_zero_not_equal_to_e1():
    param = TestParam("test", 3, 1)
    param.resample()

    zero = AffExpr(constant=np.array([[0], [0], [0]], dtype=np.float))

    lhs = AffExpr({param: 1})

    fluent = LinEqFluent('test_eq', lhs, zero)
    assert not fluent.satisfied()


def test_fluent_equals():
    # model = grb.Model()
    # obj = grb.QuadExpr()

    param = TestParam("test", 3, 1)
    param.resample()

    three = AffExpr(constant=np.array([[3], [0], [-2]], dtype=np.float))

    # var = Variable(model, param)
    lhs = AffExpr({param: 2}, constant=np.array([[1], [0], [-2]]))

    fluent = LinEqFluent('test_eq', lhs, three)
    assert fluent.satisfied()


def test_fluent_equal_after_opt():
    # model = grb.Model()
    # obj = grb.QuadExpr()

    param = TestParam("test", 3, 1)
    param.resample()

    zero = AffExpr(constant=np.array([[0], [0], [0]], dtype=np.float))

    # var = Variable(model, param)
    lhs = AffExpr({param: 1})

    fluent = LinEqFluent('test_eq', lhs, zero)
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
    fluent = LinEqFluent('test_eq', lhs, zero)

    variables = [Variable(model, param)]
    param_to_var = {param: variables[0]}

    constraints = Constraints(model)
    if isinstance(fluent, LinEqFluent):
        lhs = fluent.lhs.to_gurobi_expr(model, param_to_var)
        rhs = fluent.rhs.to_gurobi_expr(model, param_to_var)
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
    fluent = LinLEFluent('test_eq', lhs, twoe1)

    variables = [Variable(model, param)]
    param_to_var = {param: variables[0]}

    constraints = Constraints(model)
    if isinstance(fluent, LinLEFluent):
        lhs = fluent.lhs.to_gurobi_expr(model, param_to_var)
        rhs = fluent.rhs.to_gurobi_expr(model, param_to_var)
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
    assert np.all(variables[0].value == np.array([[1], [0], [-1]]))


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
    fluent = LinLEFluent('test_eq', lhs, twoe1)

    variables = [var]
    param_to_var = {param: var}

    constraints = Constraints(model)
    if isinstance(fluent, LinLEFluent):
        lhs = fluent.lhs.to_gurobi_expr(model, param_to_var)
        rhs = fluent.rhs.to_gurobi_expr(model, param_to_var)
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
    assert np.all(variables[0].value == np.array([[1], [0], [-1]]))

test_aff_expr()
test_fluent_zero_not_equal_to_e1()
test_fluent_equals()
test_eq_cnt_with_gurobi()
test_le_cnt_with_gurobi()
test_le_cnt_with_opt()
