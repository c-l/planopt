"""Creates a box and then update the box's geometry dynamically.
"""
from openravepy import *
import openravepy
import bulletsimpy
import numpy as np
import time
from trajopt import Trajopt
env = Environment() # create openrave environment
env.SetViewer('qtcoin') # attach viewer (optional)

obstacles = np.matrix(
                        '-0.806451612903226,-1.07017543859649, 1;\
                        -0.576036866359447, 0.918128654970760, 1;\
                        0.040552995391705,0.906432748538011, 1;\
                        0.61843317972350,-0.988304093567252, 1;\
                        -0.806451612903226,-1.07017543859649, -1;\
                        -0.576036866359447, 0.918128654970760, -1;\
                        0.040552995391705,0.906432748538011, -1; \
                        0.61843317972350,-0.988304093567252, -1')

# obstacles = np.matrix(
#                         '-1.0,-1, 1;\
#                         -1, 1, 1;\
#                         1,1, 1;\
#                         1,-1, 1;\
#                         -1,-1, -1;\
#                         -1, 1, -1;\
#                         1,1, -1; \
#                         1,-1, -1')

# obstacles[2,0] = 0.5
# obstacles[6,0] = 0.5
# obstacles[3,0] = 0.9
# obstacles[7,0] = 0.9
# import ipdb; ipdb.set_trace()

def create_cylinder(env, body_name, t, dims, color=[0,1,1]):
  infocylinder = openravepy.KinBody.GeometryInfo()
  infocylinder._type = openravepy.GeometryType.Cylinder
  infocylinder._vGeomData = dims
  infocylinder._bVisible = True
  infocylinder._vDiffuseColor = color
  infocylinder._t[2, 3] = dims[1] / 2

  cylinder = openravepy.RaveCreateKinBody(env, '')
  cylinder.InitFromGeometries([infocylinder])
  cylinder.SetName(body_name)
  cylinder.SetTransform(t)

  return cylinder

with env:
    body = RaveCreateKinBody(env,'')
    vertices = np.array(obstacles)
    indices = np.array([[2,1,0], [2,0,3], [4,5,6],[6,7,4],[0,5,4],[0,1,5],[1,2,5],[5,2,6],[2,3,6],[6,3,7],[0,7,3],[0,4,7]])
    body.InitFromTrimesh(trimesh=TriMesh(vertices,indices),draw=True)
    body.SetName('obstacle')
    env.AddKinBody(body) 

    # body = RaveCreateKinBody(env,'')
    # body.InitFromSpheres(np.array([[-.1,.1,0,1.3]]))
    # # body.InitFromBoxes(np.array([[0,0,0,1, 1, 1]]))
    # body.SetName('obstacle')
    # env.AddKinBody(body) 
    # rot = matrixFromAxisAngle([0,0,.7])
    # transform = np.identity(4)
    # transform = np.dot(rot,transform)
    # body.SetTransform(transform)

    # body = create_cylinder(env, 'obstacle', np.identity(4), [1.5,1.5,2])
    # env.AddKinBody(body)

    box = KinBody.Link.GeometryInfo()
    box._type = KinBody.Link.GeomType.Box
    # box._t[0,3] = -0.8
    # box._t[1,3] = 0.5
    box._vGeomData = [0.2,0.1,.99]
    box._bVisible = True
    box._fTransparency = 0
    box._vDiffuseColor = [0,0,1]

    # vertices = np.array([[-.2, -0.1, 0],[-.2,0.1,0],[.2,.1,0], 
    robot = RaveCreateKinBody(env,'')
    robot.InitFromGeometries([box])
    # robot.InitFromBoxes(numpy.array([[0,0,0,0.2,0.1,0]]),True) # set geometry as one box of extents 0.1, 0.2, 0.3
    robot.SetName('box_robot')
    # robot._vDiffuseColor = [0,0,0]
    transform = np.identity(4)
    transform[0,3] = 0.9
    transform[1,3] = 0.2
    robot.SetTransform(transform)
    # env.AddKinBody(robot)
    # vertices = np.array([[-.2, -0.1, 0],[-.2,0.1,0],[.2,.1,0], 

# import ipdb; ipdb.set_trace()

clones = 50
ys = np.linspace(1.3, -1.3, num=clones)
xs = np.linspace(0.6, 1.1, num=clones)

bodies = ['box_robot', 'obstacle']
for i in range(clones):
    name = 'box_robot{0}'.format(i)
    bodies.append(name)
    robot = RaveCreateKinBody(env,'')
    robot.InitFromGeometries([box])
    robot.SetName(name)
    transform = np.identity(4)
    transform[0,3] = xs[i]
    transform[1,3] = ys[i]
    robot.SetTransform(transform)
    env.AddKinBody(robot)
    # vertices = np.array([[-.2, -0.1, 0],[-.2,0.1,0],[.2,.1,0], 


print 'number of objects in environment:', len(env.GetBodies())
print env.GetBodies()
t0 = time.time()

bullet_env = bulletsimpy.BulletEnvironment(env, bodies)
bullet_env.SetContactDistance(.15)

bullet_objs = [bullet_env.GetObjectByName(b.GetName()) for b in env.GetBodies()]
print 'bullet objs', [o.GetName() for o in bullet_objs]


for o in bullet_objs:
    print o.GetName()
    print o.GetTransform()
    o.UpdateRave()
env.UpdatePublishedBodies()

robot_obj = bullet_env.GetObjectByName('box_robot')
# robot_obj.UpdateBullet()
obs = bullet_env.GetObjectByName('obstacle')
# obs.UpdateBullet()
bullet_env.Step(0.01, 100, 0.01)


handles = []
print "Collisions:"
# collisions = bullet_env.DetectAllCollisions()
collisions = bullet_env.ContactTest(obs)
ptAs = []
ptBs = []
for c in collisions:
    print 'linkA:', c.linkA.GetParent().GetName(), c.linkA.GetName()
    print 'linkB:', c.linkB.GetParent().GetName(), c.linkB.GetName()
    print 'ptA:', c.ptA
    print 'ptB:', c.ptB
    print 'normalB2A:', c.normalB2A
    print 'distance:', c.distance
    print 'weight:', c.weight
    # with env:
    #     # handles.append(env.plot3(points=np.array((ptA,ptB)), pointsize=0.1,colors=np.array(((0,1,0),(0,0,0)))))
    #     handles.append(env.plot3(points=array((ptAs[0])), pointsize=0.1,colors=array(((0,1,0)))))
    ptA = c.ptA
    ptA[2] = 1.01
    ptAs.append(ptA)
    ptB = c.ptB
    ptB[2] = 1.01
    ptBs.append(ptB)

with env:
    time.sleep(0.5) # sleep 2 seconds
    # handles.append(env.plot3(points=np.array((ptA,ptB)), pointsize=0.1,colors=array(((0,1,0),(0,0,0)))))
    for (ptA, ptB) in zip(ptAs, ptBs):
        handles.append(env.plot3(points=ptA, pointsize=10,colors=(1,0,0)))
        handles.append(env.plot3(points=ptB, pointsize=10,colors=(0,1,0)))
        handles.append(env.drawarrow(p1=ptA, p2=ptB, linewidth=.01,color=(0,1,0)))
while True:
    time.sleep(0.5) # sleep 2 seconds
    import ipdb; ipdb.set_trace()
time.sleep(30) # sleep 2 seconds

