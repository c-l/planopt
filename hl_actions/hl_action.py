import numpy as np
import cvxpy as cvx
from openravepy import *
import time
from utils import *

class HLAction(object):
    def __init__(self, hl_plan, env, robot):
        self.hl_plan = hl_plan
        self.env = env
        self.robot = robot
        self.handles = []
        self.name = "hla"

        # optimization sqp info
        self.objective = 0
        self.constraints = []
        self.f = lambda x: np.zeros((1,1))
        self.g = lambda x: np.zeros((1,1))
        self.h = lambda x: np.zeros((1,1))

        # list of variables
        # list of precondition fluents
        self.preconditions = []

        # list of effect fluents
        self.postconditions = []

        self.obj = None
        self.traj = None
        self.obj_traj = None

        # for graphing
        self.robot_clones = None
        self.obj_clones = None

    def add_fluents_to_opt_prob(self):
        for precondition in self.preconditions:
            self.add_precondition(precondition)
        for postcondition in self.postconditions:
            self.add_postcondition(postcondition)

    def add_dual_cost(self, var, dual, consensus=None, ro = 0.05):
        # self.objective += dual.T*var # dual gradient ascent
        if consensus is None:
            self.objective += dual.T*var # dual gradient ascent
        else:
            self.objective += dual.T*var + ro/2 * cvx.square(cvx.norm(var-consensus))
            # self.objective += ro/2 * cvx.square(cvx.norm(var-consensus))


    def add_postcondition(self, fluent):
        constraints, g, h = fluent.postcondition()
        self.add_opt_info(constraints, g, h)

    def add_precondition(self, fluent):
        constraints, g, h = fluent.precondition()
        self.add_opt_info(constraints, g, h)

    def add_opt_info(self, constraints, g, h):
        self.constraints += constraints

        # fix nested f g and hs? Need to think about how to write this
        # TODO: Fix current implementation
        if g is not None:
            # self.g = lambda x: np.vstack((self.g(x), g(x)))
            self.g = lambda x: g(x)
        if h is not None:
            # self.h = lambda x: np.vstack((self.h(x), h(x)))
            self.h = lambda x: h(x)

    def plot_traj_line(self, traj, colors=(0,0,1)):
        handles = []
        env = self.hl_plan.env
        traj_points = np.reshape(traj.value.copy(), (self.T, self.K))
        traj_points[:,2] = np.ones((self.T, 1))
        handles.append(env.drawlinestrip(points=traj_points, linewidth=10.0, colors=colors))
        handles.append(env.plot3(points=traj_points[0,:],pointsize=20, colors=colors))
        handles.append(env.plot3(points=traj_points[-1,:],pointsize=20, colors=colors))

        return handles

    def create_robot_clones(self):
        self.robot_clones = []
        env = self.hl_plan.env
        robot = self.hl_plan.robot
        with env:
            with robot:
            
                transparency = 0.85
                traj = self.traj.value.reshape((self.K,self.T), order='F')
                for t in range(self.T):
                    xt = traj[:,t]
                    newrobot = self.create_robot_kinbody(name=self.name + "_" + robot.GetName() + str(t), transparency=transparency)
                    # newrobot = RaveCreateRobot(env,robot.GetXMLId())
                    # newrobot.Clone(self.robot,7)
                    # newrobot.SetName(self.name + "_" + robot.GetName() + str(t))
                    newrobot.SetTransform(base_pose_to_mat(xt))

                    for link in newrobot.GetLinks():
                        for geom in link.GetGeometries():
                            geom.SetTransparency(transparency)

                    self.robot_clones.append(newrobot)
                    env.Add(newrobot)
            env.UpdatePublishedBodies()


    def plot_traj_robot_kinbodies(self):
        traj = self.traj.value.reshape((self.K,self.T), order='F')
        if self.robot_clones is None:
            self.create_robot_clones()

        for t in range(self.T):
            xt = traj[:,t]
            self.robot_clones[t].SetTransform(base_pose_to_mat(xt))
        return self.robot_clones

    def create_obj_clones(self):
        self.obj_clones = []
        env = self.hl_plan.env
        obj = env.GetKinBody(self.obj.GetName())
        
        transparency = 0.85
        traj = self.obj_traj.value.reshape((self.K,self.T), order='F')
        with env:
            for t in range(self.T):
                xt = traj[:,t]
                newobj = self.create_obj_kinbody(name=self.name + "_" + obj.GetName() + str(t), transparency=transparency)
                # newobj = RaveCreateKinBody(env, obj.GetXMLId())
                # newobj.Clone(obj, 0)
                # newobj.SetName(self.name + "_" + obj.GetName() + str(t))

                for link in newobj.GetLinks():
                    for geom in link.GetGeometries():
                        geom.SetTransparency(transparency)
                        # geom.SetDiffuseColor([0,0,1])

                # for obj in grabbed_objs:
                self.obj_clones.append(newobj)
                env.Add(newobj)

    def plot_traj_obj_kinbodies(self):
        traj = self.obj_traj.value.reshape((self.K,self.T), order='F')
        if self.obj_clones is None:
            self.create_obj_clones()

        for t in range(self.T):
            xt = traj[:,t]
            self.obj_clones[t].SetTransform(base_pose_to_mat(xt))
        return self.obj_clones

    def plot(self):
        self.plot_traj_robot_kinbodies()
        if self.obj is not None:
            self.plot_traj_obj_kinbodies()
        # return handles

    def create_robot_kinbody(self, name, color=[0,0,1], transparency=0.8):
        robot = self.create_cylinder(name, np.eye(4), [0.2,2.01], color=color, transparency=transparency)
        return robot

    def create_obj_kinbody(self, name, color=[0,1,0], transparency=0.8):
        obj = self.create_cylinder(name, np.eye(4), [0.35,2.01], color=color, transparency=transparency)
        return obj

    def create_cylinder(self, body_name, t, dims, color=[0,1,1], transparency=0.8):
        infocylinder = KinBody.GeometryInfo()
        infocylinder._type = GeometryType.Cylinder
        infocylinder._vGeomData = dims
        infocylinder._bVisible = True
        infocylinder._vDiffuseColor = color
        infocylinder._fTransparency = transparency
        # infocylinder._t[2, 3] = dims[1] / 2

        cylinder = RaveCreateKinBody(self.hl_plan.env, '')
        cylinder.InitFromGeometries([infocylinder])
        cylinder.SetName(body_name)
        cylinder.SetTransform(t)

        return cylinder


