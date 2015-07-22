import numpy as np

class Function(object):
    def __init__(self, f, use_numerical=True):
        self.f = f
        self.use_numerical = use_numerical

    def val_and_grad(self, x):
        if self.use_numerical:
            val = self.f(x)
            grad = Function.numerical_jac(self.f,x)
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


