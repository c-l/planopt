from openravepy import *
import numpy as np
from utils import *


class World(object):
    def __init__(self):
        self.env = Environment() # create openrave environment
        return
        self.env.SetViewer('qtcoin') # attach viewer (optional)

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

    # def create_box(self, env, body_name, t, dims, color=[0,0,1]):
    def create_box(self, env, name, transform, dims, color=[0,0,1]):
        infobox = KinBody.Link.GeometryInfo()
        infobox._type = KinBody.Link.GeomType.Box
        # box._t[0,3] = -0.8
        # box._t[1,3] = 0.5
        # infobox._vGeomData = [0.2,0.1,.99]
        infobox._vGeomData = dims
        infobox._bVisible = True
        infobox._fTransparency = 0
        infobox._vDiffuseColor = color

        box = RaveCreateKinBody(env,'')
        box.InitFromGeometries([infobox])

        # box.SetName('box')
        box.SetName(name)
        # transform = np.identity(4)
        # transform[0,3] = 0.9
        # transform[1,3] = 0.2
        box.SetTransform(transform)
        return box


    def create_x_wall(self, env, name, length, start):
        transform = np.eye(4)
        transform[0,3] = start[0] + length/2
        transform[1,3] = start[1]

        dim_x = length/2
        dim_y = 0.05

        dims = [dim_x, dim_y, 1]

        wall = self.create_box(env, name, transform, dims, color=[.1,.1,.1]) 
        env.AddKinBody(wall)

    def create_y_wall(self, env, name, length, start):
        transform = np.eye(4)
        transform[0,3] = start[0]
        transform[1,3] = start[1] + length/2.0

        dim_x = 0.05
        dim_y = length/2.0

        dims = [dim_x, dim_y, 1]

        wall = self.create_box(env, name, transform, dims, color=[.1,.1,.1]) 
        env.AddKinBody(wall)
        
    def create_box_around(self, env, name, radius, transform):
        radius = radius + 0.11
        x = transform[0,3]
        y = transform[1,3]

        x_left = x - radius
        x_right = x + radius
        y_start = y - radius
        y_end = y + radius

        # self.create_x_wall(env, "wall1", 2, [x_left-2, y_start])
        self.create_y_wall(env, "left_" + name, radius*2.0, [x_left, y_start])
        self.create_x_wall(env, "top_" + name, radius*2.0, [x_left, y_end])
        self.create_y_wall(env, "right_" + name, radius*2.0, [x_right, y_start])
        # self.create_x_wall(env, "wall2", 2, [x_right, y_start])

    def generate_box_env(self):
        # box = self.create_box(self.env, "box", base_pose_to_mat([0,0,0]), dims=[.2, .1, .99])
        # self.env.AddKinBody(box)
        # self.create_x_wall(self.env, "wall1", 2, [-.5,1])
        # self.create_y_wall(self.env, "wall2", 1, [-.5,1])
        env = self.env
        transform = np.eye(4)
        # transform[0,3] = 2
        # transform[1,3] = 1
        transform[0,3] = 0.01
        transform[1,3] = 0
        # obj = self.create_box(env, 'obj', np.eye(4), [.35, .35, 1])
        obj = self.create_box(env, 'obj', np.eye(4), [.34, .34, 1])
        obj.SetTransform(transform)
        env.AddKinBody(obj) 

        target_transform = np.eye(4)
        target_transform[0,3] = 2
        target_transform[1,3] = 1
        self.create_box_around(env, "wall", 0.35, target_transform)

        env.Load("robot.xml")
        robot = env.GetRobots()[0]
        transform = np.eye(4)
        # transform[0,3] = -2
        transform[0,3] = -3
        robot.SetTransform(transform)

        transparency = 1
        # for body in [robot, body, obj]:
        for body in [robot, obj]:
            for link in body.GetLinks():
                for geom in link.GetGeometries():
                    geom.SetTransparency(transparency)
        return env

    def create_obj(self):
        transform = np.eye(4)
        # transform[0,3] = 2
        # transform[1,3] = 1
        transform[0,3] = 0.01
        transform[1,3] = 0
        # obj = self.create_box(env, 'obj', np.eye(4), [.35, .35, 1])
        obj = self.create_box(self.env, 'obj', np.eye(4), [.34, .34, 1])
        obj.SetTransform(transform)
        self.env.AddKinBody(obj) 

        self.make_transparent(obj)


    def create_robot(self):
        self.env.Load("robot.xml")
        robot = self.env.GetRobots()[0]
        transform = np.eye(4)
        # transform[0,3] = -2
        transform[0,3] = -3
        robot.SetTransform(transform)

        self.make_transparent(robot)

    def generate_boxes_env(self, num=1):
        env = self.env
        # self.create_obj()
        self.create_robot()
        target_locations = self.generate_target_locations(num)
        self.generate_boxes(num)

        return env, target_locations

    def make_transparent(self, body):
        transparency = 0.9
        for link in body.GetLinks():
            for geom in link.GetGeometries():
                geom.SetTransparency(transparency)

    def generate_boxes(self, num):
        transform = np.eye(4)
        # transform[0,3] = 2
        # transform[1,3] = 1
        transform[1,3] = 0

        for i in range(num):
            transform[0,3] = 0.01 + 4*i
            # obj = self.create_box(env, 'obj', np.eye(4), [.35, .35, 1])
            obj = self.create_box(self.env, 'obj'+str(i), transform, [.34, .34, 1])
            obj.SetTransform(transform)
            self.env.AddKinBody(obj) 

        self.make_transparent(obj)


    def generate_target_locations(self, num):
        target_locations = []
        transform = np.eye(4)
        transform[1,3] = 1
        for i in range(num):
            transform[0,3] = 4*(i) + 2
            target_locations.append(transform.copy())
            self.create_box_around(self.env, "wall"+str(i), 0.35, transform)
        return target_locations

    def generate_room_env(self):
        # box = self.create_box(self.env, "box", base_pose_to_mat([0,0,0]), dims=[.2, .1, .99])
        # self.env.AddKinBody(box)
        # self.create_x_wall(self.env, "wall1", 2, [-.5,1])
        # self.create_y_wall(self.env, "wall2", 1, [-.5,1])
        env = self.env
        transform = np.eye(4)
        # transform[0,3] = 2
        # transform[1,3] = 1
        transform[0,3] = 0.01
        transform[1,3] = 0
        obj = self.create_cylinder(env, 'obj', np.eye(4), [.35, 2])
        obj.SetTransform(transform)
        env.AddKinBody(obj) 

        target_transform = np.eye(4)
        target_transform[0,3] = 2
        target_transform[1,3] = 1
        self.create_box_around(env, "wall", 0.35, target_transform)

        env.Load("robot.xml")
        robot = env.GetRobots()[0]
        transform = np.eye(4)
        # transform[0,3] = -2
        transform[0,3] = -3
        robot.SetTransform(transform)

        transparency = 1
        # for body in [robot, body, obj]:
        for body in [robot, obj]:
            for link in body.GetLinks():
                for geom in link.GetGeometries():
                    geom.SetTransparency(transparency)
        return env

    def generate_test_env(self):
        env = self.env
        obstacles = np.matrix('-0.576036866359447, 0.918128654970760, 1;\
                        -0.806451612903226,-1.07017543859649, 1;\
                        1.01843317972350,-0.988304093567252, 1;\
                        0.640552995391705,0.906432748538011, 1;\
                        -0.576036866359447, 0.918128654970760, -1;\
                        -0.806451612903226,-1.07017543859649, -1;\
                        1.01843317972350,-0.988304093567252, -1;\
                        0.640552995391705,0.906432748538011, -1')

        body = RaveCreateKinBody(env,'')
        vertices = np.array(obstacles)
        indices = np.array([[0,1,2], [2,3,0], [4,5,6],[6,7,4],[0,4,5],[0,1,5],[1,2,5],[5,6,2],[2,3,6],[6,7,3],[0,3,7],[0,4,7]])
        body.InitFromTrimesh(trimesh=TriMesh(vertices,indices),draw=True)
        body.SetName('obstacle')
        for link in body.GetLinks():
            for geom in link.GetGeometries():
                geom.SetDiffuseColor((.9,.9,.9))
        env.AddKinBody(body) 

        # create cylindrical object
        transform = np.eye(4)
        transform[0,3] = -2
        obj = self.create_cylinder(env, 'obj', np.eye(4), [.35, 2])
        obj.SetTransform(transform)
        env.AddKinBody(obj) 



        # import ipdb; ipdb.set_trace() # BREAKPOINT
        env.Load("robot.xml")

        robot = env.GetRobots()[0]
        transform = np.eye(4)
        transform[0,3] = -1
        robot.SetTransform(transform)
        transparency = 0.7
        # for body in [robot, body, obj]:
        for body in [robot, body]:
            for link in body.GetLinks():
                for geom in link.GetGeometries():
                    geom.SetTransparency(transparency)
        return env

if __name__ == "__main__":
    world = World()
    env, target_locations = world.generate_boxes_env(1)
    env.Save("../envs/one_box_world.dae", Environment.SelectionOptions.Everything)
    import ipdb; ipdb.set_trace() # BREAKPOINT

