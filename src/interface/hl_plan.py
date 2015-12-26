from hl_actions.move import Move
from hl_actions.pick import Pick
from hl_actions.place import Place
from openravepy import *
# from hl_param import HLParam
from hl_param import *
import numpy as np
import ipdb
import time

# from envs.world import World

from utils import *

from ll_plan import LLPlan

class HLPlan(object):
    def __init__(self, env, robot):
        self.env = env
        self.robot = robot

class HLPlan2(object):
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
            # obj = env.GetKinBody('obj0')
            hla_envs.append(env)
            hla_robots.append(robot)
            hla_objs.append(obj)
        return (hla_envs, hla_robots, hla_objs)

    def clone_envs(self, num_actions):
        local_envs = []

        for i in range(num_actions):
            env = self.env.CloneSelf(1) # clones objects in the environment
            local_envs.append(env)
        return local_envs


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
        # target_loc = HLParam("target_loc", 3, 1, is_var=False, value=np.array((2,1,0)))
        target_loc = HLParam("target_loc", 3, 1, is_var=False, value=np.array((2,0.5,0)))

        envs, robots, objs = self.create_local_envs(num_actions=3)

        pick = self.add_hl_action(Pick(self, envs[0], robots[0], rp1, objs[0], obj_loc, gp))
        place = self.add_hl_action(Place(self, envs[1], robots[1], rp2, objs[1], target_loc, gp))

        # for initialization
        rp1.dual_update()
        rp2.dual_update()
        gp.dual_update()

        pick.plot()
        place.plot()
        # import ipdb; ipdb.set_trace() # BREAKPOINT
        move = self.add_hl_action(Move(self, envs[2], robots[2], rp1, rp2, objs[2], gp))


        params = [rp1, rp2, gp]
        llplan = LLPlan(params, self.hl_actions)
        llplan.solve()

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
        llplan = LLPlan(params, self.hl_actions)
        llplan.solve()
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

    # @profile
    def pick_in_container(self):
        self.env = World().generate_room_env()
        self.robot = self.env.GetRobots()[0]
        # self.ro = 0.02
        # self.ro = 0.2
        self.ro = 2
        # self.ro = 10
        # self.ro = 20
        # self.ro = 300

        # rp1 = HLParam("rp1", 3, 1, ro=self.ro)
        rp1 = HLParam("rp1", 3, 1, is_var=False, value=np.array((0,0,0)))
        rp2 = HLParam("rp2", 3, 1, ro=self.ro, value=np.array((2,-1,0)))
        pick_obj = self.env.GetKinBody('obj')
        gp = HLParam("gp", 3, 1, ro=self.ro)
        obj_loc = HLParam("obj_loc", 3, 1, is_var=False, value=mat_to_base_pose(pick_obj.GetTransform()))
        # target_loc = HLParam("target_loc", 3, 1, is_var=False, value=np.array((2,1,0)))
        # target_loc = HLParam("target_loc", 3, 1, is_var=False, value=np.array((2,0.5,0)))

        envs, robots, objs = self.create_local_envs(num_actions=2)

        pick = self.add_hl_action(Pick(self, envs[0], robots[0], rp2, objs[0], obj_loc, gp))

        # for initialization
        # rp1.dual_update()
        rp2.dual_update()
        gp.dual_update()

        import ipdb; ipdb.set_trace() # BREAKPOINT
        pick.plot()
        # move = self.add_hl_action(Move(self, envs[1], robots[1], rp1, rp2, objs[1], gp))
        move = self.add_hl_action(Move(self, envs[1], robots[1], rp1, rp2, None, None))


        params = [rp2, gp]
        llplan = LLPlan(params, self.hl_actions)
        llplan.solve()

    def place_in_container(self):
        self.env = World().generate_room_env()
        self.robot = self.env.GetRobots()[0]
        # self.ro = 0.02
        # self.ro = 0.2
        self.ro = 2
        # self.ro = 10
        # self.ro = 20
        # self.ro = 300

        # rp1 = HLParam("rp1", 3, 1, ro=self.ro)
        rp1 = HLParam("rp1", 3, 1, is_var=False, value=np.array((0,0,0)))
        # rp2 = HLParam("rp2", 3, 1, ro=self.ro, value=np.array((2,-1,0)))
        rp2 = HLParam("rp2", 3, 1, ro=self.ro, value=np.array((1,2,0)))
        obj = self.env.GetKinBody('obj')
        gp = HLParam("gp", 3, 1, ro=self.ro)
        obj_loc = HLParam("obj_loc", 3, 1, is_var=False, value=mat_to_base_pose(obj.GetTransform()))
        # target_loc = HLParam("target_loc", 3, 1, is_var=False, value=np.array((2,1,0)))
        # target_loc = HLParam("target_loc", 3, 1, is_var=False, value=np.array((2,0.5,0)))

        envs, robots, objs = self.create_local_envs(num_actions=2)

        place = self.add_hl_action(Place(self, envs[0], robots[0], rp2, objs[0], obj_loc, gp))

        # for initialization
        # rp1.dual_update()
        rp2.dual_update()
        gp.dual_update()

        import ipdb; ipdb.set_trace() # BREAKPOINT
        place.plot()
        move = self.add_hl_action(Move(self, envs[1], robots[1], rp1, rp2, objs[1], gp))


        params = [rp2, gp]
        llplan = LLPlan(params, self.hl_actions)
        llplan.solve()

    # @profile
    def pick_move_and_place_in_container(self):
        self.env = World().generate_room_env()
        self.robot = self.env.GetRobots()[0]
        self.num_actions = 3
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
        # target_loc = HLParam("target_loc", 3, 1, is_var=False, value=np.array((2,1,0)))
        target_loc = HLParam("target_loc", 3, 1, is_var=False, value=np.array((2,1,0)))

        envs, robots, objs = self.create_local_envs(num_actions=3)

        pick = self.add_hl_action(Pick(self, envs[0], robots[0], rp1, objs[0], obj_loc, gp))
        place = self.add_hl_action(Place(self, envs[1], robots[1], rp2, objs[1], target_loc, gp))

        # for initialization
        rp1.dual_update()
        rp2.dual_update()
        gp.dual_update()

        pick.plot()
        place.plot()
        import ipdb; ipdb.set_trace() # BREAKPOINT
        # envs[2].SetViewer('qtcoin')
        move = self.add_hl_action(Move(self, envs[2], robots[2], rp1, rp2, objs[2], gp))


        params = [rp1, rp2, gp]
        llplan = LLPlan(params, self.hl_actions)
        llplan.solve()

    def move_pick_and_place_in_container(self):
        # self.env = World().generate_room_env()
        self.env = World().generate_box_env()
        self.robot = self.env.GetRobots()[0]
        self.num_actions = 4
        # self.ro = 0.02
        # self.ro = 0.2
        self.ro = 2
        # self.ro = 10
        # self.ro = 20
        # self.ro = 300

        robot_init = HLParam("rp_init", 3, 1, is_var=False, value=mat_to_base_pose(self.robot.GetTransform()))
        rp1 = HLParam("rp1", 3, 1, ro=self.ro)
        rp2 = HLParam("rp2", 3, 1, ro=self.ro)
        pick_obj = self.env.GetKinBody('obj')
        gp = HLParam("gp", 3, 1, ro=self.ro)
        obj_loc = HLParam("obj_loc", 3, 1, is_var=False, value=mat_to_base_pose(pick_obj.GetTransform()))
        # target_loc = HLParam("target_loc", 3, 1, is_var=False, value=np.array((2,1,0)))
        target_loc = HLParam("target_loc", 3, 1, is_var=False, value=np.array((2,1,0)))

        envs, robots, objs = self.create_local_envs(num_actions=self.num_actions)

        pick = self.add_hl_action(Pick(self, envs[1], robots[1], rp1, objs[1], obj_loc, gp))
        place = self.add_hl_action(Place(self, envs[2], robots[2], rp2, objs[2], target_loc, gp))

        # for initialization
        rp1.dual_update()
        rp2.dual_update()
        gp.dual_update()

        pick.plot()
        place.plot()
        import ipdb; ipdb.set_trace() # BREAKPOINT
        move1 = self.add_hl_action(Move(self, envs[0], robots[0], "move1", robot_init, rp1))
        move2 = self.add_hl_action(Move(self, envs[3], robots[3], "move2", rp1, rp2, objs[3], gp))


        params = [rp1, rp2, gp]
        llplan = LLPlan(params, self.hl_actions)
        llplan.solve()

    def pick_place_boxes(self):
        # self.env = World().generate_room_env()
        num_boxes = 1
        self.env, target_loc_values = World().generate_boxes_env(num=num_boxes)
        target_locs = []
        i=0
        for loc in target_loc_values:
            target_locs.append(ObjLoc("target"+"_loc"+str(i), 3, 1, is_var=False, value=mat_to_base_pose(loc)))
            i += 1

        self.robot = self.env.GetRobots()[0]
        self.num_actions = 4*len(target_locs)
        # self.ro = 0.02
        # self.ro = 0.2
        self.ro = 2
        # self.ro = 10
        # self.ro = 20
        # self.ro = 300

        hla_envs = self.clone_envs(self.num_actions)

        rps = []
        gps = []
        objs = []
        obj_locs =[]
        for i in range(num_boxes):
            gps.append(GP("gp"+str(i), 3, 1, ro=self.ro))
            obj = self.env.GetKinBody('obj'+str(i))
            objs.append(obj)
            obj_locs.append(ObjLoc("obj"+str(i)+"_loc", 3, 1, is_var=False, value=mat_to_base_pose(obj.GetTransform())))

        robot_positions = 1 + 2*num_boxes
        for i in range(robot_positions):
            if i == 0:
                rps.append(RP("rp_init", 3, 1, is_var=False, value=mat_to_base_pose(self.robot.GetTransform())))
            else:
                rps.append(RP("rp"+str(i), 3, 1, ro=self.ro))

        rp_index=1
        for i in range(num_boxes):
            env = hla_envs[1+4*i]
            robot = env.GetRobots()[0]
            obj = env.GetKinBody(objs[i].GetName())
            pick = self.add_hl_action(Pick(self, env, robot, rps[rp_index], obj, obj_locs[i], gps[i], name="pick"+str(i)))
            env = hla_envs[2+4*i]
            robot = env.GetRobots()[0]
            obj = env.GetKinBody(objs[i].GetName())
            place = self.add_hl_action(Place(self, env, robot, rps[rp_index+1], obj, target_locs[i], gps[i], name="place"+str(i)))
            rp_index += 2

        for rp in rps:
            rp.dual_update()

        for gp in gps:
            gp.dual_update()

        for hl_action in self.hl_actions:
            hl_action.plot()

        # import ipdb; ipdb.set_trace() # BREAKPOINT
        for i in range(num_boxes):
            env = hla_envs[4*i]
            robot = env.GetRobots()[0]
            move1 = self.add_hl_action(Move(self, env, robot, "move"+str(2*i+1), rps[2*i], rps[2*i+1]))

            env = hla_envs[4*i+3]
            robot = env.GetRobots()[0]
            obj = env.GetKinBody(objs[i].GetName())
            move2 = self.add_hl_action(Move(self, env, robot, "move"+str(2*i+2), rps[2*i+1], rps[2*i+2], obj, gps[i]))

        params = rps + gps
        llplan = LLPlan(params, self.hl_actions)
        llplan.solve()

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
    # plan.test_pick_move_and_place()
    # plan.pick_in_container()
    # plan.place_in_container()
    # plan.pick_move_and_place_in_container()
    # plan.move_pick_and_place_in_container() # worm example
    plan.pick_place_boxes()
    # plan.test_pick_and_move()
    # plan.test_move()
    # plan.test_pick()
