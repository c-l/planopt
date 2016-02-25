# from openravepy import *
from openravepy import KinBody, Environment, RaveCreateKinBody, TriMesh, \
GeometryType, RaveCreateRobot, DOFAffine
import numpy as np
from utils import base_pose_to_mat

def create_cylinder(env, body_name, t, dims, color=[0, 1, 1]):
    infocylinder = KinBody.GeometryInfo()
    infocylinder._type = GeometryType.Cylinder
    infocylinder._vGeomData = dims
    # ipdb.set_trace()
    infocylinder._bVisible = True
    infocylinder._vDiffuseColor = color
    # infocylinder._t[2, 3] = dims[1] / 2

    # cylinder = RaveCreateKinBody(env, '')
    cylinder = RaveCreateRobot(env, '')
    cylinder.InitFromGeometries([infocylinder])
    cylinder.SetName(body_name)
    cylinder.SetTransform(t)

    return cylinder

def make_transparent(body, transparency=0.7):
    for link in body.GetLinks():
        for geom in link.GetGeometries():
            geom.SetTransparency(transparency)

def add_cyl_robot(env):
    env.Load("robot.xml")

    robot = env.GetRobots()[0]
    transform = np.eye(4)
    transform[0, 3] = -1
    robot.SetTransform(transform)
    make_transparent(robot)

def add_obstacle(env):
    obstacles = np.matrix('-0.576036866359447, 0.918128654970760, 1;\
                    -0.806451612903226,-1.07017543859649, 1;\
                    1.01843317972350,-0.988304093567252, 1;\
                    0.640552995391705,0.906432748538011, 1;\
                    -0.576036866359447, 0.918128654970760, -1;\
                    -0.806451612903226,-1.07017543859649, -1;\
                    1.01843317972350,-0.988304093567252, -1;\
                    0.640552995391705,0.906432748538011, -1')

    body = RaveCreateKinBody(env, '')
    vertices = np.array(obstacles)
    indices = np.array([[0, 1, 2], [2, 3, 0], [4, 5, 6], [6, 7, 4], [0, 4, 5],
                        [0, 1, 5], [1, 2, 5], [5, 6, 2], [2, 3, 6], [6, 7, 3],
                        [0, 3, 7], [0, 4, 7]])
    body.InitFromTrimesh(trimesh=TriMesh(vertices, indices), draw=True)
    body.SetName('obstacle')
    for link in body.GetLinks():
        for geom in link.GetGeometries():
            geom.SetDiffuseColor((.9, .9, .9))
    env.AddKinBody(body)
    make_transparent(body)

def add_obstacle2(env):
    obstacles = np.matrix('-0.576036866359447, 0.918128654970760, 1;\
                    -0.806451612903226,-1.07017543859649, 1;\
                    1.01843317972350,-0.988304093567252, 1;\
                    0.640552995391705,0.906432748538011, 1;\
                    -0.576036866359447, 0.918128654970760, -1;\
                    -0.806451612903226,-1.07017543859649, -1;\
                    1.01843317972350,-0.988304093567252, -1;\
                    0.640552995391705,0.906432748538011, -1')

    body = RaveCreateKinBody(env, '')
    vertices = np.array(obstacles)
    indices = np.array([[0, 1, 2], [2, 3, 0], [4, 5, 6], [6, 7, 4], [0, 4, 5],
                        [0, 1, 5], [1, 2, 5], [5, 6, 2], [2, 3, 6], [6, 7, 3],
                        [0, 3, 7], [0, 4, 7]])
    body.InitFromTrimesh(trimesh=TriMesh(vertices, indices), draw=True)
    body.SetName('obstacle2')
    for link in body.GetLinks():
        for geom in link.GetGeometries():
            geom.SetDiffuseColor((.9, .9, .9))
    env.AddKinBody(body)
    pose = np.array([[.1], [0.], [0.]])
    body.SetTransform(base_pose_to_mat(pose))
    make_transparent(body)

def add_obstacle3(env):
    obstacles = np.matrix('-0.576036866359447, 0.918128654970760, 1;\
                    -0.806451612903226,-1.07017543859649, 1;\
                    1.01843317972350,-0.988304093567252, 1;\
                    0.640552995391705,0.906432748538011, 1;\
                    -0.576036866359447, 0.918128654970760, -1;\
                    -0.806451612903226,-1.07017543859649, -1;\
                    1.01843317972350,-0.988304093567252, -1;\
                    0.640552995391705,0.906432748538011, -1')

    body = RaveCreateKinBody(env, '')
    vertices = np.array(obstacles)
    indices = np.array([[0, 1, 2], [2, 3, 0], [4, 5, 6], [6, 7, 4], [0, 4, 5],
                        [0, 1, 5], [1, 2, 5], [5, 6, 2], [2, 3, 6], [6, 7, 3],
                        [0, 3, 7], [0, 4, 7]])
    body.InitFromTrimesh(trimesh=TriMesh(vertices, indices), draw=True)
    body.SetName('obstacle3')
    for link in body.GetLinks():
        for geom in link.GetGeometries():
            geom.SetDiffuseColor((.9, .9, .9))
    env.AddKinBody(body)
    pose = np.array([[.0], [.2], [0.]])
    body.SetTransform(base_pose_to_mat(pose))
    import ipdb; ipdb.set_trace()

    make_transparent(body)
def add_object(env):
    # create cylindrical object
    transform = np.eye(4)
    transform[0, 3] = -2
    obj = create_cylinder(env, 'obj', np.eye(4), [.35, 2])
    obj.SetTransform(transform)
    # env.AddKinBody(obj)
    env.AddRobot(obj)
    make_transparent(obj)

def move_test_env():
    env = Environment()  # create openrave environment
    env.SetViewer('qtcoin')  # attach viewer (optional)

    add_cyl_robot(env)
    add_obstacle(env)
    add_obstacle2(env)
    add_obstacle3(env)
    raw_input('continue past warnings')

    return env

def pick_test_env():
    env = Environment()  # create openrave environment
    env.SetViewer('qtcoin')  # attach viewer (optional)

    add_cyl_robot(env)
    add_obstacle(env)
    add_object(env)
    raw_input('continue past warnings')

    return env

def add_pr2(env):
    robot = env.ReadRobotXMLFile("robots/pr2-beta-sim.robot.xml")
    env.Add(robot)

def pr2_env():
    env = Environment()
    env.SetViewer('qtcoin')

    add_pr2(env)

def cans_world_env():
    env = Environment()
    env.SetViewer('qtcoin')
    env.Load("../envs/can_world.dae")
    for body in env.GetBodies():
        if 'object' in body.GetName():
            body.SetActiveDOFs(np.ndarray(0), DOFAffine.Transform)
    return env

def pr2_can_test_env():
    env = cans_world_env()
    obj17 = env.GetKinBody('object17')
    tran = obj17.GetTransform()
    tran[0,3] = .49268
    # tran[1,3] = -.51415
    tran[1,3] = -.34
    obj17.SetTransform(tran)
    return env

class TestDomain(object):
    def __init__(self, env):
        self.env = env
