import gurobipy as grb
GRB = grb.GRB


def abs(model, affexpr):
    pos = model.addVar(lb=0, ub=GRB.INFINITY, name='pos')
    neg = model.addVar(lb=0, ub=GRB.INFINITY, name='neg')

    expr = grb.LinExpr(pos+neg)

    model.update()
    model.addConstr(affexpr == pos - neg)
    return expr

def diff(var, val):
    var = var.flatten()
    val = val.flatten()
    size = len(var)
    # var[i].X is the value of the current solution
    affexprlist = [grb.LinExpr(var[i]-val[i]) for i in range(size)]
    return affexprlist
