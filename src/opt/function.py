import numpy as np
import gurobipy as grb
from opt.ops import diff
GRB = grb.GRB


class Function(object):

    def __init__(self, f, var, use_numerical=True):
        self.f = f
        self.var = var
        self.use_numerical = use_numerical
        self.is_affine = False

    def eval(self, x):
        raise NotImplementedError

    def convexify(self, x):
        raise NotImplementedError

    @staticmethod
    def quad_expr(x, Q):
        rows, cols = Q.shape
        assert rows == cols
        obj = grb.QuadExpr()
        for i in range(rows):
            for j in range(cols):
                if Q[i, j] != 0:
                    obj += Q[i, j] * x[i] * x[j]
        return obj

    @staticmethod
    def aff_expr(x, A, b):
        rows, cols = A.shape
        rows_x = len(x)
        assert cols == rows_x
        exprlist = []
        for i in range(rows):
            expr = grb.LinExpr()
            for j in range(cols):
                if A[i, j] != 0:
                    expr += grb.LinExpr(A[i, j] * x[j])
            expr += b[i]
            exprlist.append(expr)
        return exprlist

    @staticmethod
    def lin_expr(x, A):
        rows, cols = A.shape
        rows_x, _ = x.shape
        assert cols == rows_x
        expr = grb.LinExpr()
        for i in range(rows):
            for j in range(cols):
                if A[i, j] != 0:
                    expr += A[i, j] * x[j]
        return expr

    def val(self, x):
        if self.use_numerical:
            return self.f(x)
        else:
            return self.f(x)

    def val_and_grad(self, x):
        if self.use_numerical:
            val = self.f(x)
            grad = Function.numerical_jac(self.f, x)
            return val, grad
        else:
            return self.f(x)

    def val_grad_and_hess(self, x):
        if self.use_numerical:
            val = self.f(x)
            grad, hess = Function.numerical_grad_hess(self.f, x)
            return val, grad, hess
        else:
            return self.f(x)

    @staticmethod
    def numerical_jac(f, x):
        y = f(x)

        # grad = np.matrix(np.zeros((y.size, x.size)))
        grad = np.zeros((y.size, x.size))

        eps = 1e-5
        xp = x.copy()

        for i in range(x.size):
            xp[i] = x[i] + eps / 2
            yhi = f(xp)
            xp[i] = x[i] - eps / 2
            ylo = f(xp)
            xp[i] = x[i]
            gradi = (yhi - ylo) / eps
            grad[:, i] = gradi.reshape((gradi.size, 1))

        return grad

    # def numerical_grad_hess(f, x, full_hessian=True):
    @staticmethod
    def numerical_grad_hess(f, x):
        # y = f(x)
        # assert length(y) == 1

        grad = np.zeros((1, x.size))
        hess = np.zeros((x.size, x.size))

        # eps=1e-5
        # xp = x.copy()

        grad = Function.numerical_jac(f, x)
        hess = Function.numerical_jac(
            lambda x: Function.numerical_jac(f, x), x)
        hess = (hess + np.transpose(hess)) / 2

        from numpy import linalg
        w, v = linalg.eig(hess)
        mineig = np.min(w)
        if mineig < 0:
            print('    negative hessian detected, adjusting by ', - mineig)
            hess = hess + (-mineig) * np.identity(x.size)
        return (grad, hess)


class AffineFn(Function):

    def __init__(self, var, A, b):
        self.var = var
        self.A = A
        self.b = b
        self.expr = Function.aff_expr(var, A, b)

    def eval(self, x):
        return np.dot(self.A, x) + self.b

    # def grad(self, x):
    #     return A


class QuadFn(Function):

    def __init__(self, param, Q):
        self.param = param
        self.Q = Q

    def val(self):
        # x = np.reshape(np.array(self.param.get_value()), (len(self.param.get_value()), 1))
        # val = np.dot(np.transpose(x), np.dot(self.Q, x))
        x = self.param.get_value().flatten(order='F')
        val = np.dot(x, np.dot(self.Q, x))
        return val
        # return val[0, 0]

    def to_gurobi_fn(self, param_to_var):
        var = param_to_var[self.param]
        # TODO: need to specify format
        self.expr = QuadFn.quad_expr(var.get_grb_vars().flatten(order='F'), self.Q)


    # def hess(self, x):
    #     return 2*Q

    # def grad(self, x):
    #     return 2*np.dot(Q,x)


class CollisionFn(Function):

    def __init__(self, params, f):
        self.f = f
        self.params = params

    def to_gurobi_fn(self, param_to_var):
        self.vs = []
        for param in self.params:
            self.vs.append(param_to_var[param])
        self.grb_vars = self.get_grb_vars()

    def get_values(self):
        values = [var.value.copy() for var in self.vs]
        values = np.vstack(values)
        return values

    def get_grb_vars(self):
        grb_vars = []
        for var in self.vs:
            grb_vars.extend(var.grb_vars.flatten(order='F'))
        grb_vars = np.vstack(grb_vars)
        return grb_vars

    def val(self):
        x = self.get_values()
        val, grad = self.f(x)
        return np.sum(val)

    def grad(self):
        x = self.get_values()
        val, grad = self.f(x)
        return grad

    def convexify(self):
        x = self.get_values()
        f_val, f_grad = self.f(x)
        affexprlist = diff(self.grb_vars, x.flatten(order='F'))
        affexprlist = self.aff_expr(affexprlist, f_grad, f_val)
        return affexprlist
