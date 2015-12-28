import numpy as np
from opt.opt_prob import OptProb
from openravepy import *
import time
from utils import *

class HLAction(object):
    def __init__(self, lineno, hl_plan, env, robot):
        self.lineno = lineno
        self.hl_plan = hl_plan
        self.env = env
        self.robot = robot
        self.handles = []
        self.name = "hla"

        # list of precondition fluents
        self.preconditions = []

        # list of effect fluents
        self.postconditions = []

        # for graphing
        self.robot_clones = None
        self.obj_clones = None
        self.params = []

    def get_params(self):
        return self.params

    def plot_traj_line(self, traj, colors=(0,0,1)):
        handles = []
        env = self.hl_plan.env
        traj_points = np.matrix(np.reshape(traj.value.copy(), (self.T, self.K)))
        traj_points[:,2] = np.ones((self.T,1))
        handles.append(env.drawlinestrip(points=traj_points, linewidth=10.0, colors=colors))
        handles.append(env.plot3(points=traj_points[0,:],pointsize=20, colors=colors))
        handles.append(env.plot3(points=traj_points[-1,:],pointsize=20, colors=colors))

        return handles

    def create_robot_clones(self):
        self.robot_clones = []
        env = self.hl_plan.env
        robot = self.robot.get_env_body(env)
        with env:
            with robot:

                transparency = 0.85
                # traj = self.traj.value.reshape((self.K,self.T), order='F')
                for t in range(self.T):
                    # xt = self.traj.value[self.K*t:self.K*(t+1)]
                    xt = self.traj.get_value()[:, t:t+1]
                    # env.Load(robot.GetXMLFilename())
                    newrobot = self.create_robot_kinbody(name=self.name + "_" + robot.GetName() + str(t), transparency=transparency)
                    # newrobot = RaveCreateRobot(env,robot.GetXMLId())
                    # newrobot.Clone(self.robot,0)
                    newrobot.SetName(self.name + "_" + robot.GetName() + str(t))

                    for link in newrobot.GetLinks():
                        for geom in link.GetGeometries():
                            geom.SetTransparency(transparency)

                    env.Add(newrobot, True)
                    newrobot.SetTransform(base_pose_to_mat(xt))
                    self.robot_clones.append(newrobot)
            env.UpdatePublishedBodies()


    def plot_traj_robot_kinbodies(self):
        # traj = self.traj.value.reshape((self.K,self.T), order='F')
        if self.robot_clones is None:
            self.create_robot_clones()

        for t in range(self.T):
            xt = self.traj.get_value()[:,t:t+1]
            # xt = self.traj.value[self.K*t:self.K*(t+1)]
            self.robot_clones[t].SetTransform(base_pose_to_mat(xt))
        return self.robot_clones

    def create_obj_clones(self):
        self.obj_clones = []
        env = self.hl_plan.env
        obj = self.obj.get_env_body(env)


        with env:
            with obj:

                transparency = 0.85
                # traj = self.obj_traj.value.reshape((self.K,self.T), order='F')
                for t in range(self.T):
                    xt = self.obj_traj.get_value()[:, t:t+1]
                    # xt = traj[:,t]
                    newobj = self.create_obj_kinbody(name=self.name + "_" + obj.GetName() + str(t), transparency=transparency)
                    # newobj = RaveCreateKinBody(env, obj.GetXMLId())
                    # newobj = RaveCreateRobot(env, obj.GetXMLId())
                    # newobj = RaveClone(obj, 4)
                    # newobj = RaveCreateKinBody(env, '')
                    # newobj = RaveCreateKinBody(env, self.name + "_" + obj.GetName() + str(t))
                    # newobj.Clone(obj, 0)
                    # newobj.SetName(self.name + "_" + obj.GetName() + str(t))

                    for link in newobj.GetLinks():
                        for geom in link.GetGeometries():
                            geom.SetTransparency(transparency)
                            # geom.SetDiffuseColor([0,0,1])

                    # for obj in grabbed_objs:
                    env.Add(newobj, True)
                    newobj.SetTransform(base_pose_to_mat(xt))
                    self.obj_clones.append(newobj)
            env.UpdatePublishedBodies()
            # time.sleep(3)

    def plot_traj_obj_kinbodies(self):
        # traj = self.obj_traj.value.reshape((self.K,self.T), order='F')
        if self.obj_clones is None:
            self.create_obj_clones()

        for t in range(self.T):
            xt = self.obj_traj.value[:, t:t+1]
            # xt = traj[:,t]
            self.obj_clones[t].SetTransform(base_pose_to_mat(xt))
        return self.obj_clones

    def plot(self):
        self.plot_traj_robot_kinbodies()
        if self.obj is not None:
           self.plot_traj_obj_kinbodies()
        # return handles

    def clear_plots(self):
        self.handles = []

    def add_plot_handles(self, handles):
        self.handles += handles

    def create_robot_kinbody(self, name, color=[1,0,0], transparency=0.8):
        robot = self.create_cylinder(name, np.eye(4), [0.2,2.01], color=color, transparency=transparency)
        return robot

    def create_obj_kinbody(self, name, color=[0,1,0], transparency=0.8):
        # obj = self.create_box(name, np.eye(4), [.35, .35, 1], transparency=transparency)
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

    # def create_box(self, env, body_name, t, dims, color=[0,0,1]):
    def create_box(self, name, transform, dims, color=[0,0,1], transparency=0.8):
        infobox = KinBody.Link.GeometryInfo()
        infobox._type = KinBody.Link.GeomType.Box
        # box._t[0,3] = -0.8
        # box._t[1,3] = 0.5
        # infobox._vGeomData = [0.2,0.1,.99]
        infobox._vGeomData = dims
        infobox._bVisible = True
        infobox._fTransparency = transparency
        infobox._vDiffuseColor = color

        box = RaveCreateKinBody(self.hl_plan.env,'')
        box.InitFromGeometries([infobox])

        # box.SetName('box')
        box.SetName(name)
        # transform = np.identity(4)
        # transform[0,3] = 0.9
        # transform[1,3] = 0.2
        box.SetTransform(transform)
        return box
