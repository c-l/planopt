from hl_actions.move import Move
from hl_actions.pick import Pick
from openravepy import *
from hl_param import HLParam
import numpy as np
import ipdb
import cvxpy as cvx
import time

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
        time.sleep(0.5)

        for kinbody in plot_kinbodies:
            self.env.Remove(kinbody)


    def init_openrave_test_env(self):
        env = Environment() # create openrave environment
        env.SetViewer('qtcoin') # attach viewer (optional)
        

        # obstacles = np.matrix('-0.576036866359447, 0.918128654970760, 1;\
        #                 -0.806451612903226,-1.07017543859649, 1;\
        #                 1.01843317972350,-0.988304093567252, 1;\
        #                 0.640552995391705,0.906432748538011, 1;\
        #                 -0.576036866359447, 0.918128654970760, -1;\
        #                 -0.806451612903226,-1.07017543859649, -1;\
        #                 1.01843317972350,-0.988304093567252, -1;\
        #                 0.640552995391705,0.906432748538011, -1')

        # body = RaveCreateKinBody(env,'')
        # vertices = np.array(obstacles)
        # indices = np.array([[0,1,2], [2,3,0], [4,5,6],[6,7,4],[0,4,5],[0,1,5],[1,2,5],[5,6,2],[2,3,6],[6,7,3],[0,3,7],[0,4,7]])
        # body.InitFromTrimesh(trimesh=TriMesh(vertices,indices),draw=True)
        # body.SetName('obstacle')
        # env.AddKinBody(body) 

        # create cylindrical object
        transform = np.eye(4)
        transform[1,3] = -0.1
        body = self.create_cylinder(env, 'obstacle', np.eye(4), [1, 2])
        body.SetTransform(transform)
        env.AddKinBody(body) 

        return env

    def create_robot_kinbody(self, name, xt):
        # create robot KinBody
        env = self.env
        box = KinBody.Link.GeometryInfo()
        box._type = KinBody.Link.GeomType.Cylinder
        box._vGeomData = [0.2,2.01]
        box._bVisible = True
        box._fTransparency = 0
        box._vDiffuseColor = [0,0,1]

        robot = RaveCreateKinBody(env,'')
        robot.InitFromGeometries([box])
        robot.SetName(name)

        return robot

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

    def test(self):
        self.env = self.init_openrave_test_env()
        self.ro = 0.05
        consensus = cvx.Parameter(3,1,value=np.zeros((3,1)))
        self.params = {"rp": HLParam("rp", consensus, ro=self.ro)}
        # rp = HLParam("rp")
        rp = self.params["rp"]
        end = cvx.Variable(3,1)
        dual_end = cvx.Parameter(3,1,value=np.zeros((3,1)))
        rp.add_action_var(end, dual_end)
        move = Move(self.env, np.array((-2,0,0)),end)
        move.add_dual_cost(end, dual_end, consensus, ro=self.ro)
        self.add_hl_action(move)

        pos = cvx.Variable(3,1)
        dual_pos = cvx.Parameter(3,1,value=np.zeros((3,1)))
        rp.add_action_var(pos, dual_pos)
        pick = Pick(self.env, pos, self.env.GetKinBody('obstacle'), np.zeros((3,1)))
        pick.add_dual_cost(pos, dual_pos, consensus, ro=self.ro)
        self.add_hl_action(pick)

        epsilon = 1e-3
        while True:
            self.solve()
            diff = self.params["rp"].dual_update()
            print "diff: ", diff
            if diff < epsilon:
                break

if __name__ == "__main__":
    plan = HLPlan()
    plan.test()
