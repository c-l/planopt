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
        # self.ro = 2
        self.ro = 20
        # consensus_rp = cvx.Parameter(3,1,value=np.zeros((3,1)))
        # consensus_gp = cvx.Parameter(3,1,value=np.zeros((3,1)))
        self.params = {"rp": HLParam("rp", 3, 1, ro=self.ro), \
                "gp": HLParam("gp", 3, 1, ro=self.ro), \
                "rp2": HLParam("rp2", 3, 1, ro=self.ro)}
        # rp = HLParam("rp")
        rp = self.params["rp"]
        gp = self.params["gp"]
        rp2 = self.params["rp2"]

        pick_pos = self.hl_var("rp")
        obj = self.env.GetKinBody('obj')
        grasp = self.hl_var("gp")
        obj_loc = cvx.Parameter(3,1,value=mat_to_base_pose(obj.GetTransform()))
        pick = Pick(self.env, self.robot, pick_pos, obj, obj_loc, grasp)
        rp.add_dual(pick, pick_pos)
        gp.add_dual(pick, grasp)
        pick.solve_opt_prob()

        place_pos = self.hl_var("rp2")
        target_loc = cvx.Parameter(3,1,value=np.array((2,0,0)))
        grasp = self.hl_var("gp")
        place = Place(self.env, self.robot, place_pos, obj, target_loc, grasp)
        rp2.add_dual(place, place_pos)
        gp.add_dual(place, grasp)
        place.solve_opt_prob()

        rp.dual_update()
        rp2.dual_update()
        gp.dual_update()

        start = self.hl_var("rp", pick_pos.value)
        # end = cvx.Parameter(3,1,value=np.array((2,0,0)))
        end = self.hl_var("rp2", place_pos.value)
        grasp = self.hl_var("gp", grasp.value)

        # grasp = cvx.Variable(3,1)
        # grasp.value = np.zeros((3,1))
        # dual_grasp = cvx.Parameter(3,1,value=np.zeros((3,1)))
        # gp.add_action_var(grasp, dual_grasp)
        move2 = Move(self.env, self.robot, start, end, obj, grasp)
        rp.add_dual(move2, start)
        rp2.add_dual(move2, end)
        gp.add_dual(move2, grasp)

        # adding actions to hl plan
        self.add_hl_action(pick)
        self.add_hl_action(move2)
        self.add_hl_action(place)

        epsilon = 5e-3*len(self.params)
        while True:
            self.solve()
            diff = 0
            for param in self.params.itervalues():
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
    # plan.test_pick_move_and_place()
    plan.test_pick_and_move()
