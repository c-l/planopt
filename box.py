"""Creates a box and then update the box's geometry dynamically.
"""
from openravepy import *
import bulletsimpy
import numpy as np
import time
from trajopt import Trajopt
env = Environment() # create openrave environment
env.SetViewer('qtcoin') # attach viewer (optional)

obstacles = np.matrix('-0.576036866359447, 0.918128654970760, 1;\
                        -0.806451612903226,-1.07017543859649, 1;\
                        1.01843317972350,-0.988304093567252, 1;\
                        0.640552995391705,0.906432748538011, 1;\
                        -0.576036866359447, 0.918128654970760, -1;\
                        -0.806451612903226,-1.07017543859649, -1;\
                        1.01843317972350,-0.988304093567252, -1;\
                        0.640552995391705,0.906432748538011, -1')


with env:
    body = RaveCreateKinBody(env,'')
    vertices = np.array(obstacles)
    indices = np.array([[0,1,2], [2,3,0], [4,5,6],[6,7,4],[0,4,5],[0,1,5],[1,2,5],[5,6,2],[2,3,6],[6,7,3],[0,3,7],[0,4,7]])
    body.InitFromTrimesh(trimesh=TriMesh(vertices,indices),draw=True)
    body.SetName('obstacle')
    env.AddKinBody(body) 

    box = KinBody.Link.GeometryInfo()
    box._type = KinBody.Link.GeomType.Box
    box._t[0,3] = 0.7
    box._t[1,3] = 0.5
    box._vGeomData = [0.2,0.1,1.01]
    box._bVisible = True
    box._fTransparency = 0
    box._vDiffuseColor = [0,0,1]

    # vertices = np.array([[-.2, -0.1, 0],[-.2,0.1,0],[.2,.1,0], 
    robot = RaveCreateKinBody(env,'')
    robot.InitFromGeometries([box])
    # robot.InitFromBoxes(numpy.array([[0,0,0,0.2,0.1,0]]),True) # set geometry as one box of extents 0.1, 0.2, 0.3
    robot.SetName('box_robot')
    # robot._vDiffuseColor = [0,0,0]
    # transform = np.identity(4)
    # transform[0,3] = -2
    # robot.SetTransform(transform)
    env.AddKinBody(robot)
    # vertices = np.array([[-.2, -0.1, 0],[-.2,0.1,0],[.2,.1,0], 

    # box._t[0,3] = -.25
    # box._vGeomData = [0.2,0.1,0]
    # box._bVisible = True
    # box._fTransparency = 0
    # box._vDiffuseColor = [1,0,0]

    # obs = RaveCreateKinBody(env,'')
    # obs.InitFromGeometries([box])
    # # robot.InitFromBoxes(numpy.array([[0,0,0,0.2,0.1,0]]),True) # set geometry as one box of extents 0.1, 0.2, 0.3
    # obs.SetName('obstacle')
    # # robot._vDiffuseColor = [0,0,0]
    # # transform = np.identity(4)
    # # transform[0,3] = -2
    # # robot.SetTransform(transform)
    # env.AddKinBody(obs)

# import ipdb; ipdb.set_trace()
print 'number of objects in environment:', len(env.GetBodies())
print env.GetBodies()
t0 = time.time()
bullet_env = bulletsimpy.BulletEnvironment(env, ['box_robot','obstacle'])
bullet_env.SetGravity([0, 0, 0])

bullet_objs = [bullet_env.GetObjectByName(b.GetName()) for b in env.GetBodies()]
print 'bullet objs', [o.GetName() for o in bullet_objs]

robot_obj = bullet_env.GetObjectByName('box_robot')
robot_obj.UpdateBullet()
bullet_env.Step(0.01, 100, 0.01)

handles = []
print "Collisions:"
collisions = bullet_env.DetectAllCollisions()
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
    # handles.append(env.plot3(points=np.array((ptA,ptB)), pointsize=0.1,colors=array(((0,1,0),(0,0,0)))))
    handles.append(env.plot3(points=ptAs[0], pointsize=10,colors=(0,1,0)))
    handles.append(env.plot3(points=ptBs[0], pointsize=10,colors=(0,1,0)))
    handles.append(env.drawarrow(p1=ptAs[0], p2=ptBs[0], linewidth=.01,color=(0,1,0)))
while True:
    time.sleep(0.5) # sleep 2 seconds
    import ipdb; ipdb.set_trace()
time.sleep(30) # sleep 2 seconds

