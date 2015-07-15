from hl_actions.move import Move
from hl_actions.pick import Pick
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

    def solve(self):
        plot_kinbodies = []
        for action in self.hl_actions:
            action.solve_opt_prob()
            plot_kinbodies += action.plot_kinbodies()

        for kinbody in plot_kinbodies:
            self.env.AddKinBody(kinbody)

        self.env.UpdatePublishedBodies()
        import ipdb; ipdb.set_trace() # BREAKPOINT

        for kinbody in plot_kinbodies:
            self.env.Remove(kinbody)


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
        env.AddKinBody(body) 

        # create cylindrical object
        transform = np.eye(4)
        transform[0,3] = -2
        body = self.create_cylinder(env, 'pick_obj', np.eye(4), [.35, 2])
        body.SetTransform(transform)
        env.AddKinBody(body) 

        env.Load("robot.xml")

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

    def hl_var(self, name, init_value=None):
        param = self.params[name]
        rows = param.rows
        cols = param.cols

        var = cvx.Variable(rows, cols)
        if init_value is not None:
            var.value = init_value
        else:
            var.value = np.zeros((rows,cols))
        return var

    def test(self):
        self.env = self.init_openrave_test_env()
        self.robot = self.env.GetRobots()[0]
        # self.ro = 0.02
        self.ro = 0.2
        # self.ro = 20
        # consensus_rp = cvx.Parameter(3,1,value=np.zeros((3,1)))
        # consensus_gp = cvx.Parameter(3,1,value=np.zeros((3,1)))
        self.params = {"rp": HLParam("rp", 3, 1, ro=self.ro), \
                "gp": HLParam("gp", 3, 1, ro=self.ro)}
        # rp = HLParam("rp")
        rp = self.params["rp"]
        gp = self.params["gp"]
        # end = cvx.Variable(3,1)
        # end.value = np.array((-2,0,0))
        # dual_end = cvx.Parameter(3,1,value=np.zeros((3,1)))
        # rp.add_action_var(end, dual_end)
        # move1 = Move(self.env, np.array((-2,0,0)),end)
        # self.add_hl_action(move1)

        # pos = cvx.Variable(3,1)
        # dual_pos = cvx.Parameter(3,1,value=np.zeros((3,1)))
        # rp.add_action_var(pos, dual_pos)

        # grasp = cvx.Variable(3,1)
        # grasp.value = np.zeros((3,1))
        # dual_grasp = cvx.Parameter(3,1,value=np.zeros((3,1)))
        # gp.add_action_var(grasp, dual_grasp)

        pos = self.hl_var("rp")
        pick_obj = self.env.GetKinBody('pick_obj')
        grasp = self.hl_var("gp")
        obj_loc = cvx.Parameter(3,1,value=mat_to_base_pose(pick_obj.GetTransform()))
        pick = Pick(self.env, self.robot, pos, pick_obj, obj_loc, grasp)
        self.add_hl_action(pick)
        rp.add_dual(pick, pos)
        gp.add_dual(pick, grasp)
        pick.solve_opt_prob()
        rp.dual_update()
        gp.dual_update()

        # start = cvx.Variable(3,1)
        # start.value = np.array((-3,0,0))
        # dual_start = cvx.Parameter(3,1,value=np.zeros((3,1)))
        # rp.add_action_var(start, dual_start)

        start = self.hl_var("rp", pos.value)
        end = cvx.Parameter(3,1,value=np.array((2,0,0)))
        grasp = self.hl_var("gp", grasp.value)

        # grasp = cvx.Variable(3,1)
        # grasp.value = np.zeros((3,1))
        # dual_grasp = cvx.Parameter(3,1,value=np.zeros((3,1)))
        # gp.add_action_var(grasp, dual_grasp)
        move2 = Move(self.env, self.robot, start, end, pick_obj, grasp)
        self.add_hl_action(move2)
        rp.add_dual(move2, start)
        gp.add_dual(move2, grasp)
        

        # diff = self.params["rp"].dual_update()
        # move1.add_dual_cost(end, dual_end, consensus, ro=self.ro)
        # pick.add_dual_cost(pos, dual_pos, consensus_rp, ro=self.ro)
        # move2.add_dual_cost(start, dual_start, consensus_gp, ro=self.ro)
        # diff = self.params["rp"].dual_update()

        epsilon = 5e-3*len(self.params)
        while True:
            self.solve()
            diff = 0
            for param in self.params.itervalues():
                diff += param.dual_update()
            print "diff: ", diff
            if diff < epsilon:
                break

if __name__ == "__main__":
    plan = HLPlan()
    plan.test()
