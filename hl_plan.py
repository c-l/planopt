from hl_actions.move import Move
from openravepy import *
import numpy as np
import ipdb

class HLPlan(object):
    def __init__(self):
        self.hl_actions = []


    def add_hl_action(self, hl_action):
        self.hl_actions += [hl_action]

    def solve(self):
        for action in self.hl_actions:
            action.solve_opt_prob()

    def init_openrave_test_env(self):
        env = Environment() # create openrave environment
        env.SetViewer('qtcoin') # attach viewer (optional)
        

        # obstacles = np.matrix('-0.576036866359447, 0.918128654970760, 1;\
        #                 -0.806451612903226,-1.07017543859649, 1;\
        #                 1.01843317972350,-0.988304093567252, 1;\
        #                 0.640552995391705,0.906432748538011, 1;\
        #                 -0.576036866359447, 0.918128654970760, -1;\
        #                 -0.806451612903226,-1.07017543859649, -1;\
        #                 1.01843317972350,-0.988304093567252, -1;\
        #                 0.640552995391705,0.906432748538011, -1')

        # body = RaveCreateKinBody(env,'')
        # vertices = np.array(obstacles)
        # indices = np.array([[0,1,2], [2,3,0], [4,5,6],[6,7,4],[0,4,5],[0,1,5],[1,2,5],[5,6,2],[2,3,6],[6,7,3],[0,3,7],[0,4,7]])
        # body.InitFromTrimesh(trimesh=TriMesh(vertices,indices),draw=True)
        # body.SetName('obstacle')
        # env.AddKinBody(body) 

        # create cylindrical object
        transform = np.eye(4)
        transform[1,3] = -0.1
        body = self.create_cylinder(env, 'obstacle', np.eye(4), [1, 2])
        body.SetTransform(transform)
        env.AddKinBody(body) 

        return env

    def create_robot_kinbody(self, name, xt):
        # create robot KinBody
        env = self.env
        box = KinBody.Link.GeometryInfo()
        box._type = KinBody.Link.GeomType.Cylinder
        box._vGeomData = [0.2,2.01]
        box._bVisible = True
        box._fTransparency = 0
        box._vDiffuseColor = [0,0,1]

        robot = RaveCreateKinBody(env,'')
        robot.InitFromGeometries([box])
        robot.SetName(name)

        return robot

    def create_cylinder(self, env, body_name, t, dims, color=[0,1,1]):
      infocylinder = KinBody.GeometryInfo()
      infocylinder._type = GeometryType.Cylinder
      infocylinder._vGeomData = dims
      # ipdb.set_trace()
      infocylinder._bVisible = True
      infocylinder._vDiffuseColor = color
      # infocylinder._t[2, 3] = dims[1] / 2

      cylinder = RaveCreateKinBody(env, '')
      cylinder.InitFromGeometries([infocylinder])
      cylinder.SetName(body_name)
      cylinder.SetTransform(t)

      return cylinder

    def test(self):
        env = self.init_openrave_test_env()
        self.add_hl_action(Move(env, np.array((-2,0,0)),np.array((0,1,0))))
        self.solve()

if __name__ == "__main__":
    plan = HLPlan()
    plan.test()
