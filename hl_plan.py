from hl_actions.move import Move
from hl_actions.pick import Pick
from hl_actions.place import Place
from openravepy import *
from hl_param import HLParam
import numpy as np
import ipdb
import cvxpy as cvx
import time

from utils import *

from opt.hl_opt_prob import HLOptProb

class HLPlan(object):
    def __init__(self):
        self.hl_actions = []


    def add_hl_action(self, hl_action):
        self.hl_actions += [hl_action]
        return hl_action

    def solve(self):
        for action in self.hl_actions:
            action.solve_opt_prob()
            action.plot()

        self.env.UpdatePublishedBodies()

    def init_openrave_test_env(self):
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
        # import ctrajoptpy
        # cc = ctrajoptpy.GetCollisionChecker(env)
        # # cc.SetContactDistance(np.infty)
        # cc.SetContactDistance(np.infty)
        # collisions = cc.BodyVsBody(robot, obj)
        # for c in collisions:
        #     # if c.GetDistance() > 0:
        #     distance = c.GetDistance()
        #     print "distance: ", distance

        # import ipdb; ipdb.set_trace() # BREAKPOINT
        return env

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

    # def hl_var(self, name, init_value=None):
    #     param = self.params[name]
    #     rows = param.rows
    #     cols = param.cols

    #     var = Variable(rows, cols)
    #     if init_value is not None:
    #         var.value = init_value
    #     else:
    #         var.value = np.zeros((rows,cols))
    #     return var

    def create_local_envs(self, num_actions):
        hla_envs = []
        hla_robots = []
        hla_objs = []
        for i in range(num_actions):
            env = self.env.CloneSelf(1) # clones objects in the environment
            robot = env.GetRobots()[0]
            obj = env.GetKinBody('obj')
            hla_envs.append(env)
            hla_robots.append(robot)
            hla_objs.append(obj)
        return (hla_envs, hla_robots, hla_objs)
            

    # @profile
    def test_pick_move_and_place(self):
        self.env = self.init_openrave_test_env()
        self.robot = self.env.GetRobots()[0]
        # self.ro = 0.02
        # self.ro = 0.2
        self.ro = 2
        # self.ro = 10
        # self.ro = 20
        # self.ro = 300

        rp1 = HLParam("rp1", 3, 1, ro=self.ro)
        rp2 = HLParam("rp2", 3, 1, ro=self.ro)
        pick_obj = self.env.GetKinBody('obj')
        gp = HLParam("gp", 3, 1, ro=self.ro)
        obj_loc = HLParam("obj_loc", 3, 1, is_var=False, value=mat_to_base_pose(pick_obj.GetTransform()))
        target_loc = HLParam("target_loc", 3, 1, is_var=False, value=np.array((2,1,0)))

        envs, robots, objs = self.create_local_envs(num_actions=3)

        pick = Pick(self, envs[0], robots[0], rp1, objs[0], obj_loc, gp)
        self.add_hl_action(pick)

        place = Place(self, envs[1], robots[1], rp2, objs[1], target_loc, gp)
        self.add_hl_action(place)

        # for initialization
        pick.solve_opt_prob()
        rp1.dual_update()
        place.solve_opt_prob()
        rp2.dual_update()
        gp.dual_update()

        pick.plot()
        place.plot()
        # import ipdb; ipdb.set_trace() # BREAKPOINT
        move = Move(self, envs[2], robots[2], rp1, rp2, objs[2], gp)
        self.add_hl_action(move)


        params = [rp1, rp2, gp]
        hlprob = HLOptProb(params, self.hl_actions)
        hlprob.solve()

    # @profile
    def test_pick_and_move(self):
        self.env = self.init_openrave_test_env()
        self.robot = self.env.GetRobots()[0]
        # self.ro = 0.02
        # self.ro = 0.2
        # self.ro = 2
        self.ro = 20

        rp = HLParam("rp", 3, 1, ro=self.ro)
        end = HLParam("end", 3, 1, is_var=False, value=np.array((2,0,0)))
        pick_obj = self.env.GetKinBody('obj')
        gp = HLParam("gp", 3, 1, ro=self.ro)
        obj_loc = HLParam("obj_loc", 3, 1, is_var=False, value=mat_to_base_pose(pick_obj.GetTransform()))



        env = self.env.CloneSelf(1) # clones objects in the environment
        robot = env.GetRobots()[0]
        obj = env.GetKinBody('obj')

        pick = Pick(self, env, robot, rp, obj, obj_loc, gp)

        # must clone env, robot and obj before solve
        env = self.env.CloneSelf(1) # clones objects in the environment
        robot = env.GetRobots()[0]
        obj = env.GetKinBody('obj')

        self.add_hl_action(pick)
        pick.solve_opt_prob()
        rp.dual_update()
        gp.dual_update()

        move = Move(self, env, robot, rp, end, obj, gp)
        self.add_hl_action(move)

        params = [rp, gp]
        hlprob = HLOptProb(params, self.hl_actions)
        hlprob.solve()
        # epsilon = 5e-3*len(params)
        # while True:
        #     self.solve()
        #     diff = 0
        #     for param in params:
        #         diff += param.dual_update()
        #     print "diff: ", diff
        #     if diff < epsilon:
        #         break

    def test_pick(self):
        self.env = self.init_openrave_test_env()
        self.robot = self.env.GetRobots()[0]
        # self.ro = 0.02
        # self.ro = 0.2
        # self.ro = 2
        self.ro = 20

        start = HLParam("start", 3, 1, is_var=False, value=np.array((-2,0,0)))
        end = HLParam("end", 3, 1, is_var=False, value=np.array((2,0,0)))

        rp = HLParam("rp", 3, 1, ro=self.ro)
        # end = HLParam("end", 3, 1, is_var=False, value=np.array((2,0,0)))
        pick_obj = self.env.GetKinBody('obj')
        gp = HLParam("gp", 3, 1, ro=self.ro)
        obj_loc = HLParam("obj_loc", 3, 1, is_var=False, value=mat_to_base_pose(pick_obj.GetTransform()))

        env = self.env.CloneSelf(1) # clones objects in the environment
        robot = env.GetRobots()[0]
        obj = env.GetKinBody('obj')
        pick = Pick(self, env, robot, rp, obj, obj_loc, gp)
        self.add_hl_action(pick)
        pick.solve_opt_prob()
        import ipdb; ipdb.set_trace() # BREAKPOINT


    def test_move(self):
        self.env = self.init_openrave_test_env()
        self.robot = self.env.GetRobots()[0]
        # self.ro = 0.02
        # self.ro = 0.2
        # self.ro = 2
        self.ro = 20

        start = HLParam("start", 3, 1, is_var=False, value=np.array((-2,0,0)))
        end = HLParam("end", 3, 1, is_var=False, value=np.array((2,0,0)))

        env = self.env.CloneSelf(1) # clones objects in the environment
        robot = env.GetRobots()[0]
        # obj = env.GetKinBody('obj')
        move = Move(self, env, robot, start, end, None, None)
        self.add_hl_action(move)
        move.solve_opt_prob()

if __name__ == "__main__":
    plan = HLPlan()
    plan.test_pick_move_and_place()
    # plan.test_pick_and_move()
    # plan.test_move()
    # plan.test_pick()
