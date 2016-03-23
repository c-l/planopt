import numpy as np
from interface.hl_actions.hl_action import HLAction
import openravepy
import time
from utils import transform_from_obj_pose

class PR2HLAction(HLAction):

    def create_robot_clones(self):
        self.robot_clones = []
        env = self.hl_plan.env
        robot_body = self.robot.get_env_body(env)

        # active_dofs = np.ndarray(0)
        # active_dofs = np.r_[active_dofs, robot.GetManipulator('rightarm').GetArmIndices()]

        # import ipdb; ipdb.set_trace()
        with env:
            # with robot:

            transparency = 0.9
            # traj = self.traj.get_value().reshape((self.K,self.T), order='F')
            for link in robot_body.GetLinks():
                for geom in link.GetGeometries():
                    geom.SetTransparency(1)

            dof_values = robot_body.GetDOFValues()
            active_dof_inds = robot_body.GetActiveDOFIndices()
            for t in range(self.T):
                # newrobot = env.ReadRobotXMLFile("../models/pr2/pr2-head-kinect.xml")
                newrobot = env.ReadRobotXMLFile("../models/pr2/pr2.zae")

                for link in newrobot.GetLinks():
                    for geom in link.GetGeometries():
                        geom.SetTransparency(transparency)

                newrobot.SetName(self.name + "_" + robot_body.GetName() + str(t))

                env.Add(newrobot, True)
                newrobot.SetDOFValues(dof_values)
                if 'base' in self.robot.active_bodyparts:
                    newrobot.SetActiveDOFs(
                        active_dof_inds,
                        openravepy.DOFAffine.X + openravepy.DOFAffine.Y + openravepy.DOFAffine.RotationAxis,
                        [0, 0, 1])
                else:
                    newrobot.SetActiveDOFs(active_dof_inds)
                self.robot_clones.append(newrobot)
        env.UpdatePublishedBodies()
        print ('waiting...')
        # time.sleep(20)
        time.sleep(5)
        # import ipdb; ipdb.set_trace()
        # time.sleep(20)

    def create_obj_clones(self):
        self.obj_clones = []
        env = self.hl_plan.env
        obj = self.obj.get_env_body(env)
        obj_trans = obj.GetTransform()
        with env:
            transparency = 0.9
            for link in obj.GetLinks():
                for geom in link.GetGeometries():
                    geom.SetTransparency(1)

            dof_values = obj.GetDOFValues()
            active_dofs = obj.GetActiveDOFIndices()
            for t in range(self.T):
                name = self.name + "_" + obj.GetName() + str(t)
                newcyl = self.create_cylinder(env, name, obj_trans, (0.05, 0.25))

                for link in newcyl.GetLinks():
                    for geom in link.GetGeometries():
                        geom.SetTransparency(transparency)

                env.Add(newcyl, True)
                newcyl.SetDOFValues(dof_values)
                self.obj_clones.append(newcyl)
        env.UpdatePublishedBodies()

    def create_cylinder(self, env, body_name, t, dims, color=[0,1,1]):
        infocylinder = openravepy.KinBody.GeometryInfo()
        infocylinder._type = openravepy.GeometryType.Cylinder
        infocylinder._vGeomData = dims
        infocylinder._bVisible = True
        infocylinder._vDiffuseColor = color
        infocylinder._t[2, 3] = dims[1] / 2.0

        # cylinder = openravepy.RaveCreateKinBody(env, '')
        cylinder = openravepy.RaveCreateRobot(env, '')
        cylinder.InitFromGeometries([infocylinder])
        cylinder.SetName(body_name)
        cylinder.SetTransform(t)
        cylinder.SetActiveDOFs(np.ndarray(0), openravepy.DOFAffine.Transform)

        return cylinder

    def plot_traj_robot_kinbodies(self):
        if self.robot_clones is None:
            self.create_robot_clones()

        for t in range(self.T):
            xt = self.traj.get_value()[:,t:t+1]
            self.robot_clones[t].SetActiveDOFValues(xt.flatten())
        return self.robot_clones

    def plot_traj_obj_kinbodies(self):
        if self.obj_clones is None:
            self.create_obj_clones()

        for t in range(self.T):
            ot = self.obj_traj.get_value()[:,t:t+1]
            transform = transform_from_obj_pose(ot)
            self.obj_clones[t].SetTransform(transform)

        return self.obj_clones


    def plot(self):
        self.plot_traj_robot_kinbodies()
        self.plot_traj_obj_kinbodies()
        # import ipdb; ipdb.set_trace()
