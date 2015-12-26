from openravepy import *
import numpy as np

def create_cylinder(env, body_name, t, dims, color=[0, 1, 1]):
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

def add_object(env):
    # create cylindrical object
    transform = np.eye(4)
    transform[0, 3] = -2
    obj = create_cylinder(env, 'obj', np.eye(4), [.35, 2])
    obj.SetTransform(transform)
    env.AddKinBody(obj)
    make_transparent(obj)

def move_test_env():
    env = Environment()  # create openrave environment
    env.SetViewer('qtcoin')  # attach viewer (optional)

    add_cyl_robot(env)
    add_obstacle(env)
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
