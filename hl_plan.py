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

class HLPlan(object):
    def __init__(self):
        self.hl_actions = []


    def add_hl_action(self, hl_action):
        self.hl_actions += [hl_action]
        return hl_action

    def solve(self):
        # plot_kinbodies = []
        # handles = []
        
        # action_num = 0
        for action in self.hl_actions:
            action.solve_opt_prob()
            # handles += action.plot(action_num)
            # handles += action.plot(handles)
            action.plot()
            # action_num += 1
            # plot_kinbodies += action.plot_kinbodies()

        # for kinbody in plot_kinbodies:
        #     self.env.AddKinBody(kinbody)

        self.env.UpdatePublishedBodies()
        import ipdb; ipdb.set_trace() # BREAKPOINT

        # for kinbody in plot_kinbodies:
        #     self.env.Remove(kinbody)
        # del plot_kinbodies

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
        transparency = 0.7
        for body in [robot, body, obj]:
            for link in body.GetLinks():
                for geom in link.GetGeometries():
                    geom.SetTransparency(transparency)
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

    #     var = cvx.Variable(rows, cols)
    #     if init_value is not None:
    #         var.value = init_value
    #     else:
    #         var.value = np.zeros((rows,cols))
    #     return var

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
        # consensus_rp = cvx.Parameter(3,1,value=np.zeros((3,1)))
        # consensus_gp = cvx.Parameter(3,1,value=np.zeros((3,1)))


        rp1 = HLParam("rp1", 3, 1, ro=self.ro)
        rp2 = HLParam("rp2", 3, 1, ro=self.ro)
        pick_obj = self.env.GetKinBody('obj')
        gp = HLParam("gp", 3, 1, ro=self.ro)
        obj_loc = HLParam("obj_loc", 3, 1, is_var=False, value=mat_to_base_pose(pick_obj.GetTransform()))
        target_loc = HLParam("target_loc", 3, 1, is_var=False, value=np.array((2,1,0)))

        env = self.env.CloneSelf(1) # clones objects in the environment
        robot = env.GetRobots()[0]
        obj = env.GetKinBody('obj')
        pick = Pick(self, env, robot, rp1, obj, obj_loc, gp)
        self.add_hl_action(pick)

        env = self.env.CloneSelf(1) # clones objects in the environment
        robot = env.GetRobots()[0]
        obj = env.GetKinBody('obj')
        place = Place(self, env, robot, rp2, obj, target_loc, gp)
        self.add_hl_action(place)

        # must clone env before solve and dual update?
        env = self.env.CloneSelf(1) # clones objects in the environment
        robot = env.GetRobots()[0]
        obj = env.GetKinBody('obj')

        pick.solve_opt_prob()
        rp1.dual_update()
        place.solve_opt_prob()
        rp2.dual_update()
        gp.dual_update()

        pick.plot()
        place.plot()
        # import ipdb; ipdb.set_trace() # BREAKPOINT
        move = Move(self, env, robot, rp1, rp2, obj, gp)
        self.add_hl_action(move)


        params = [rp1, rp2, gp]
        epsilon = 5e-3*len(params)
        while True:
            self.solve()
            diff = 0
            for param in params:
                diff += param.dual_update()
            print "diff: ", diff
            if diff < epsilon:
                break

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
        self.add_hl_action(pick)
        pick.solve_opt_prob()
        rp.dual_update()
        gp.dual_update()

        env = self.env.CloneSelf(1) # clones objects in the environment
        robot = env.GetRobots()[0]
        obj = env.GetKinBody('obj')
        move = Move(self, env, robot, rp, end, obj, gp)
        self.add_hl_action(move)

        params = [rp, gp]
        epsilon = 5e-3*len(params)
        while True:
            self.solve()
            diff = 0
            for param in params:
                diff += param.dual_update()
            print "diff: ", diff
            if diff < epsilon:
                break

if __name__ == "__main__":
    plan = HLPlan()
    plan.test_pick_move_and_place()
    # plan.test_pick_and_move()
