import numpy as np
import cvxpy as cvx
from openravepy import *

class HLAction(object):
    def __init__(self):
        # optimization sqp info
        self.constraints = []
        self.f = lambda x: np.zeros((1,1))
        self.g = lambda x: np.zeros((1,1))
        self.h = lambda x: np.zeros((1,1))

        # list of variables
        # list of precondition fluents
        self.preconditions = []

        # list of effect fluents
        self.postconditions = []

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

    def plot_kinbodies(self):
        clones = []
        traj = self.traj.value.reshape((self.K,self.T), order='F')
        if self.obj is not None:
            obj_traj = self.obj_traj.value.reshape((self.K,self.T), order='F')
        transparency = 0.8
        robot = self.env.GetKinBody('robot')

        # Need to remove obj and robot, sleep and then add them back in to clone them....
        if self.obj is not None:
            self.env.Remove(self.obj)
        self.env.Remove(robot)
        time.sleep(1.5)
        if self.obj is not None:
            self.env.Add(self.obj)
        self.env.Add(robot)

        for t in range(self.T):
            xt = traj[:,t]
            newrobot = RaveCreateRobot(self.env,robot.GetXMLId())
            newrobot.Clone(robot,0)
            newrobot.SetName(robot.GetName() + str(t))
            # newrobot.SetName(str(t))

            for link in newrobot.GetLinks():
                for geom in link.GetGeometries():
                    geom.SetTransparency(transparency)
                    geom.SetDiffuseColor([0,0,1])

            # for obj in grabbed_objs:
            if self.obj is not None:
                ot = obj_traj[:,t]
                if self.obj is not None:
                    newobj = RaveCreateKinBody(self.env, self.obj.GetXMLId())
                    newobj.Clone(self.obj, 0)
                    newobj.SetName(self.obj.GetName() + str(t))
                    
                    for link in newobj.GetLinks():
                        for geom in link.GetGeometries():
                            geom.SetTransparency(transparency)
                            # geom.SetDiffuseColor([0,0,1])
                    newobj.SetTransform(base_pose_to_mat(ot))
                    clones.append(newobj)
            newrobot.SetTransform(base_pose_to_mat(xt))
            clones.append(newrobot)

        return clones

    def create_robot_kinbody(self, name, color=[0,0,1], transparency=0.8):
        robot = self.create_cylinder(name, np.eye(4), [0.2,2.01], color=color, transparency=transparency)
        return robot

    def create_cylinder(self, body_name, t, dims, color=[0,1,1], transparency=0.8):
        infocylinder = KinBody.GeometryInfo()
        infocylinder._type = GeometryType.Cylinder
        infocylinder._vGeomData = dims
        infocylinder._bVisible = True
        infocylinder._vDiffuseColor = color
        infocylinder._fTransparency = transparency
        # infocylinder._t[2, 3] = dims[1] / 2

        cylinder = RaveCreateKinBody(self.env, '')
        cylinder.InitFromGeometries([infocylinder])
        cylinder.SetName(body_name)
        cylinder.SetTransform(t)

        return cylinder


