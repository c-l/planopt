from fluent import Fluent
from numpy.linalg import norm
import numpy as np
from opt.constraints import Constraints
from opt.function import Function

import ctrajoptpy
from utils import *

import gurobipy as grb

class InManip(Fluent):
    def __init__(self, env, hl_action, model, robot, obj, gp, traj, obj_traj = None):
        super(InManip, self).__init__(env, hl_action, model)
        self.plotting_env = hl_action.hl_plan.env
        self.robot = robot
        self.obj = obj
        # self.gp = gp.grb_vars
        # self.traj = traj.grb_vars
        # self.obj_traj = obj_traj.grb_vars
        self.gp = gp
        self.traj = traj
        self.obj_traj = obj_traj
        self.name = "InManip"
        
    def hl_params(self):
        return [self.gp.hl_param]

    def precondition(self):
        K = self.hl_action.K
        T = self.hl_action.T

        # equality constraints at each time step
        # x_obj- x_robot - x_gp*cos(theta_gp) + y_gp*sin(theta_gp) = 0
        # y_obj- y_robot - x_gp*sin(theta_gp) - y_gp*cos(theta_gp) = 0
        # theta_obj - theta_robot = theta_gp = 0
        traj_var = self.traj.grb_vars
        obj_traj_var = self.obj_traj.grb_vars
        gp_var = self.gp.grb_vars


        for i in range(T):
            self.constraints.add_eq_cntr(traj_var[K*i:K*(i+1)] + gp_var, obj_traj_var[K*i:K*(i+1)])
        # for i in range(T):
        #     self.constraints.add_eq_cntr(traj_var[K*i+2:K*i+3] + gp_var[2:3], obj_traj_var[K*i+2:K*i+3])
        # h_func = GraspFn(self.traj, self.obj_traj, self.gp, K, T)
        # self.constraints.add_nonlinear_eq_constraint(h_func)
        return self.constraints

    def postcondition(self):
        return self.precondition()

    def grasp(self, x):
        pass

class GraspFn(Function):
    def __init__(self, traj, obj_traj, gp, K, T):
        self.traj = traj
        self.obj_traj = obj_traj
        self.gp = gp

        self.K = K
        self.T = T
        self.weight= 3
    
    # def grad(self):
    #     pass

    def h_vals(self):
        traj_val = self.traj.value
        obj_traj_val = self.obj_traj.value
        gp_val = self.gp.value
        K = self.K

        x_gp = gp_val[0,0]
        y_gp = gp_val[1,0]
        theta_gp = gp_val[2,0]

        # h1_gp = -x_gp*np.cos(theta_gp) + y_gp*np.sin(theta_gp)
        # h2_gp = -x_gp*np.sin(theta_gp) - y_gp*np.cos(theta_gp)
        h1_gp = -x_gp*np.cos(theta_gp) - y_gp*np.sin(theta_gp)
        h2_gp = x_gp*np.sin(theta_gp) - y_gp*np.cos(theta_gp)

        h1_val = []
        h2_val = []
        max_theta_diff = 0
        for i in range(self.T):
            x_traj = traj_val[K*i,0]
            y_traj = traj_val[K*i+1,0]
            theta_traj = traj_val[K*i+2,0]
            x_obj = obj_traj_val[K*i,0]
            y_obj = obj_traj_val[K*i+1,0]
            theta_obj = obj_traj_val[K*i+2,0]
            h1_val.append(np.absolute(x_obj - x_traj + h1_gp))
            h2_val.append(np.absolute(y_obj - y_traj + h2_gp))
            max_theta_diff = max(max_theta_diff, np.absolute(theta_obj - theta_traj))
        print "max theta diff: ", max_theta_diff
        return self.weight*h1_val, self.weight*h2_val

    def val(self):
        h1_val, h2_val = self.h_vals()
        value = sum(h1_val+h2_val)
        print "in_manip values: ", value
        return value

    def convexify(self):
        T = self.T
        K = self.K

        traj_var = self.traj.grb_vars
        obj_traj_var = self.obj_traj.grb_vars
        gp_var = self.gp.grb_vars

        traj_val = self.traj.value
        obj_traj_val = self.obj_traj.value
        gp_val = self.gp.value

        x_gp = gp_val[0,0]
        y_gp = gp_val[1,0]
        theta_gp = gp_val[2,0]
        x_gp_coeff = -np.cos(theta_gp)
        # y_gp_coeff = np.sin(theta_gp)
        y_gp_coeff = -np.sin(theta_gp)
        # theta_gp_coeff = x_gp*np.sin(theta_gp)+y_gp*np.cos(theta_gp)
        theta_gp_coeff = x_gp*np.sin(theta_gp)-y_gp*np.cos(theta_gp)
        h1_gp_coeffs = [x_gp_coeff, y_gp_coeff, theta_gp_coeff]
        
        # ensure gradient is computed correctly
        # f = lambda x:-x[0]*np.cos(x[2]) + x[1]*np.sin(x[2])
        f = lambda x:-x[0]*np.cos(x[2]) - x[1]*np.sin(x[2])
        grad = Function.numerical_jac(f, np.array([[x_gp], [y_gp], [theta_gp]]))
        assert np.allclose(grad.flatten(), h1_gp_coeffs)

        h1_diff = x_gp*x_gp_coeff + y_gp*y_gp_coeff + theta_gp*theta_gp_coeff

        # x_gp_coeff = -np.sin(theta_gp)
        x_gp_coeff = np.sin(theta_gp)
        y_gp_coeff = -np.cos(theta_gp)
        # theta_gp_coeff = -x_gp*np.cos(theta_gp)+y_gp*np.sin(theta_gp)
        theta_gp_coeff = x_gp*np.cos(theta_gp)+y_gp*np.sin(theta_gp)
        h2_gp_coeffs = [x_gp_coeff, y_gp_coeff, theta_gp_coeff]

        # ensure gradient is computed correctly
        # f = lambda x: -x[0]*np.sin(x[2]) - x[1]*np.cos(x[2])
        f = lambda x: x[0]*np.sin(x[2]) - x[1]*np.cos(x[2])
        grad = Function.numerical_jac(f, np.array([[x_gp], [y_gp], [theta_gp]]))
        assert np.allclose(grad.flatten(), h2_gp_coeffs)

        h2_diff = x_gp*x_gp_coeff + y_gp*y_gp_coeff + theta_gp*theta_gp_coeff

        x_gp_var = gp_var[0,0]
        y_gp_var = gp_var[1,0]
        theta_gp_var = gp_var[2,0]

        h1_vals, h2_vals = self.h_vals()

        exprlist = [] 
        for i in range(T):
            x_traj_var = traj_var[K*i,0]
            y_traj_var = traj_var[K*i+1,0]
            x_obj_traj_var = obj_traj_var[K*i,0]
            y_obj_traj_var = obj_traj_var[K*i+1,0]

            h1_coeffs =[1, -1] + h1_gp_coeffs
            h2_coeffs =[1, -1] + h2_gp_coeffs

            h1_coeffs = [self.weight * coeff for coeff in h1_coeffs]
            h2_coeffs = [self.weight * coeff for coeff in h2_coeffs]

            h1_expr = grb.LinExpr(h1_coeffs, [x_obj_traj_var, x_traj_var, x_gp_var, y_gp_var, theta_gp_var])
            h2_expr = grb.LinExpr(h2_coeffs, [y_obj_traj_var, y_traj_var, x_gp_var, y_gp_var, theta_gp_var])

            x_traj_val = traj_val[K*i]
            y_traj_val = traj_val[K*i+1]
            x_obj_traj_val = obj_traj_val[K*i]
            y_obj_traj_val = obj_traj_val[K*i+1]

            h1_constant = self.weight*(-x_obj_traj_val + x_traj_val - h1_diff + h1_vals[i])
            h2_constant = self.weight*(-y_obj_traj_val + y_traj_val - h2_diff + h2_vals[i])

            h1_expr.addConstant(h1_constant)
            h2_expr.addConstant(h2_constant)
            exprlist.append(h1_expr)
            exprlist.append(h2_expr)
        
        return exprlist
            



    
        


