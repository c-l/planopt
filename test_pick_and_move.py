from hl_actions.pick_and_move import PickAndMove
# from hl_param import HLParam
from openravepy import *
import numpy as np
import cvxpy as cvx
import time

def init_openrave_test_env():
    env = Environment() # create openrave environment
    env.SetViewer('qtcoin') # attach viewer (optional)
    
    # create obstacle KinBody
    obstacles = np.matrix('-0.576036866359447, 0.918128654970760, .99;\
                    -0.806451612903226,-1.07017543859649, .99;\
                    1.01843317972350,-0.988304093567252, .99;\
                    0.640552995391705,0.906432748538011, .99;\
                    -0.576036866359447, 0.918128654970760, -.99;\
                    -0.806451612903226,-1.07017543859649, -.99;\
                    1.01843317972350,-0.988304093567252, -.99;\
                    0.640552995391705,0.906432748538011, -.99')

    body = RaveCreateKinBody(env,'')
    vertices = np.array(obstacles)
    indices = np.array([[0,1,2], [2,3,0], [4,5,6],[6,7,4],[0,4,5],[0,1,5],[1,2,5],[5,6,2],[2,3,6],[6,7,3],[0,3,7],[0,4,7]])
    body.InitFromTrimesh(trimesh=TriMesh(vertices,indices),draw=True)
    body.SetName('obstacle')
    env.AddKinBody(body) 
    # self.obstacle_kinbody = body

    # # create robot KinBody
    # box = KinBody.Link.GeometryInfo()
    # box._type = KinBody.Link.GeomType.Box
    # box._vGeomData = [0.2,0.1,1.01]
    # box._bVisible = True
    # box._fTransparency = 0
    # box._vDiffuseColor = [0,0,1]

    # robot = RaveCreateKinBody(env,'')
    # robot.InitFromGeometries([box])
    # robot.SetName('box_robot')
    # # env.AddKinBody(robot)
    # # self.robot_kinbody = robot

    transform = np.eye(4)
    transform[0,3] = -2.0
    body = create_cylinder(env, 'pick_obj', np.eye(4), [0.35, 2])
    body.SetTransform(transform)
    env.AddKinBody(body) 
    env.Load("robot.xml")

    return env

def create_cylinder(env, body_name, t, dims, color=[0,1,1]):
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


env = init_openrave_test_env()
start = cvx.Variable(3,1)
# start = cvx.Parameter(3,1, value=np.array((-2,0,0)))
start.value = np.array((-3,0,0))
end = cvx.Parameter(3,1, value=np.array((2,0,0)))
# end.value = np.array((2,0,0))

obj = env.GetKinBody('pick_obj')

gp = cvx.Parameter(3,1, value=np.array((-.55,0,0)))
# gp = cvx.Variable(3,1)
# gp.value = np.array((-.5,0,0))
pickmove = PickAndMove(env, start, end, obj, gp)
# move = Move(env, (-2,0,0),(2,0,0))
# move = Move(env, np.matrix("-2;0;0"),np.matrix("2;0;0"))
pickmove.solve_opt_prob()
robot = env.GetKinBody('robot')

plot_kinbodies = pickmove.plot_kinbodies()

for kinbody in plot_kinbodies:
    env.AddKinBody(kinbody)

env.UpdatePublishedBodies()
import ipdb; ipdb.set_trace() # BREAKPOINT
