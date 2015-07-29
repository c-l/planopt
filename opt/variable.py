import cvxpy as cvx
import numpy as np

class Variable(cvx.Variable):
    def __init__(self, rows=1, cols=1, value=None, name=None):
        super(Variable, self).__init__(rows, cols, name)

        # if cur_value==None:
        #     self.cur_value = np.zeros((rows,cols))
        #     self.save_value(np.zeros((rows,cols)))
        # else:
        #     self.cur_value = self._validate_value(cur_value)
        #     self.save_value(cur_value)
        if value==None:
            self.save_value(np.zeros((rows,cols)))
        else:
            self.save_value(value)


    
