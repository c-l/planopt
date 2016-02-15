from openravepy import KinBody, Environment, RaveCreateKinBody, TriMesh, \
GeometryType
import numpy as np
from test_utils import *
from interface.ll_prob import LLProb
from interface.hl_actions.hl_action import HLAction
from interface.hl_plan import HLPlan
from interface.hl_param import HLParam, Traj, Robot, Obj, PR2
import time


class TestDomain(object):
    def __init__(self, env):
        self.env = env

class TestAction(HLAction):
    def __init__(self, hl_plan, env, robot, pos, obj):
        super(TestAction, self).__init__(0, hl_plan, env, robot)

        self.name = "Test" + str(0)
        self.pos = pos

        self.K = 11
        self.T = 1

        self.precondition = NotObstructsPR2(env, self, robot, 1, self.pos, obj)


import ctrajoptpy
from opt.function import CollisionFn
from interface.fluents.fluent import FnLEFluent


class NotObstructsPR2(FnLEFluent):
    # def __init__(self, env, hl_action, robot, priority, traj, obj, obj_loc, dsafe=0.05):
    def __init__(self, env, hl_action, robot, priority, traj, obj, obj_loc=None, dsafe=0.01):
        self.env = env
        self.plotting_env = hl_action.hl_plan.env
        self.hl_action = hl_action
        self.traj = traj
        self.obj = obj
        self.obj_loc = obj_loc
        # self.obj_loc = obj_loc
        self.robot = robot
        self.priority = priority

        self.name = "NotObstructsPR2"

        self.robot_dofs = traj.rows
        self.timesteps = traj.cols
        if self.obj_loc is None:
            self.obj_dofs = 0
        else:
            self.obj_dofs = obj_loc.rows
        self.cc = ctrajoptpy.GetCollisionChecker(env)
        self.dsafe = dsafe

    def pre(self):
        traj = self.traj

        g = lambda x: self.collisions(x) # function inequality constraint g(x) <= 0
        obj_loc_list = []
        if self.obj_loc is not None:
            obj_loc_list = [self.obj_loc for i in range(self.timesteps)]
        self.fn = CollisionFn([self.traj] + obj_loc_list, g)

    # TODO: compute collisions properly
    # @profile
    def collisions(self, traj):
        env = self.env
        T = self.timesteps
        # K = self.K

        # ensure that there's gradient information outside dsafe
        self.cc.SetContactDistance(self.dsafe + .1)

        handles = []

        # if self.obj_loc is not None:
        #     val = np.zeros((2*T, 1))
        #     jac = np.zeros((2*T, traj.size))
        # else:
        val = np.zeros((T, 1))
        jac = np.zeros((T, traj.size))
        # if self.obj_loc is not None:
        #     self.obj.set_pose(env, self.obj_loc.value)

        # import ipdb; ipdb.set_trace()
        for t in range(self.timesteps):
            # xt = self.traj.value[K*t:K*(t+1)]
            # xt = traj[:,t:t+1]
            robot_start_ind = self.robot_dofs * t
            robot_end_ind = self.robot_dofs * (t+1)
            # obj_start_ind = self.robot_dofs * T + self.obj_dofs * t
            # obj_end_ind = self.robot_dofs * T + self.obj_dofs * (t+1)
            # xt = traj[self.robot_dofs*t:self.robot_dofs*(t+1)]
            xt = traj[robot_start_ind:robot_end_ind]
            self.robot.set_pose(env, xt)
            ot = None
            # if self.obj_loc is not None:
            #     # ot = traj[K*(t+T):K*(t+1+T)]
            #     ot = traj[obj_start_ind:obj_end_ind]
            #     self.obj.set_pose(env, ot)
            collisions = self.cc.BodyVsAll(self.robot.get_env_body(env))

            col_val, robot_jac, obj_jac = self.calc_grad_and_val(xt, ot, collisions)
            if robot_jac is not None:
                val[t], jac[t, robot_start_ind:robot_end_ind] = col_val, robot_jac
                # if self.obj_loc is not None:
                #     val[t+T], jac[t+T, obj_start_ind:obj_end_ind] = col_val, obj_jac
                #     # cross terms
                #     jac[t, obj_start_ind:obj_end_ind] = obj_jac
                #     jac[t+T, robot_start_ind:robot_end_ind] = robot_jac

        self.plotting_env.UpdatePublishedBodies()
        handles = []
        # import ipdb; ipdb.set_trace()

        return (val, jac)

    # @profile
    def calc_grad_and_val(self, xt, ot, collisions):
        val = -1*float("inf")
        robot = self.robot.get_env_body(self.env)
        obj = self.obj.get_env_body(self.env)
        robot_grad = None
        obj_grad = None
        for c in collisions:
            linkAParent = c.GetLinkAParentName()
            linkBParent = c.GetLinkBParentName()
            # print "linkAParent: ", linkAParent
            # print "linkBParent: ", linkBParent
            linkA = c.GetLinkAName()
            linkB = c.GetLinkBName()
            # print "linkA", linkA
            # print "linkB", linkB

            linkRobot = None
            linkObj = None
            sign = 1
            if linkAParent == self.robot.name and linkBParent == self.obj.name:
                ptRobot = c.GetPtA()
                linkRobot = linkA
                sign = -1
                ptObj = c.GetPtB()
                linkObj = linkB
            elif linkBParent == self.robot.name and linkAParent == self.obj.name:
                ptRobot = c.GetPtB()
                linkRobot = linkB
                sign = 1
                ptObj = c.GetPtA()
                linkObj = linkA
            else:
                continue

            distance = c.GetDistance()
            normal = c.GetNormal()

            # plotting
            self.plot_collision(ptRobot, ptObj, distance)
            # import ipdb; ipdb.set_trace()

            # if there are multiple collisions, use the one with the greatest penetration distance
            # import ipdb; ipdb.set_trace()
            if self.dsafe - distance > val:
                val = self.dsafe - distance

                # import ipdb; ipdb.set_trace()
                robot_link_ind = robot.GetLink(linkRobot).GetIndex()
                robot_jac = robot.CalculateActiveJacobian(robot_link_ind, ptRobot)
                robot_grad = np.dot(sign * normal, robot_jac)

                # if ot is not None:
                #     obj_link_ind = obj.GetLink(linkObj).GetIndex()
                #     obj_jac = obj.CalculateActiveJacobian(obj_link_ind, ptObj)
                #     import ipdb; ipdb.set_trace()
                #     obj_grad = np.dot(normal, obj_jac)

        return val, robot_grad, obj_grad

    def plot_collision(self, ptA, ptB, distance):
        handles = []
        if not np.allclose(ptA, ptB, atol=1e-3):
            if distance < 0:
                handles.append(self.plotting_env.drawarrow(p1=ptA, p2=ptB, linewidth=.01,color=(1,0,0)))
            else:
                handles.append(self.plotting_env.drawarrow(p1=ptA, p2=ptB, linewidth=.01,color=(0,0,0)))
        self.hl_action.add_plot_handles(handles)

from interface.fluents.for_all_not_obstructs import ForAllNotObstructs
from interface.fluents.robot_at import RobotAt
from interface.fluents.is_mp import IsMP
from opt.function import QuadFn

class PR2Move(HLAction):

    def __init__(self, lineno, hl_plan, env, robot, start, end, obj, gp=None):
        super(PR2Move, self).__init__(lineno, hl_plan, env, robot)

        self.hl_plan = hl_plan
        self.start = start
        self.end = end
        self.obj = obj

        # TODO: set this using optimization domain
        self.T = 40
        self.K = 7
        T = self.T
        K = self.K
        KT = self.K*self.T

        self.name = "move" + str(lineno)
        self.traj = Traj(self, self.name + "_traj", self.K, self.T, is_var=True)

        self.params = [start, end, self.traj]
        self.preconditions = [RobotAt(self, 0, start, self.traj)]
        self.preconditions += [IsMP(self, 0, start, end, self.traj)]

        self.preconditions += [NotObstructsPR2(env, self, robot, 1, self.traj, obj)]

        self.postconditions = []
        self.postconditions += [RobotAt(self, 0, self.end, self.traj)]

        # setting trajopt objective
        v = -1 * np.ones((KT - K, 1))
        d = np.vstack((np.ones((KT - K, 1)), np.zeros((K, 1))))
        # [:,0] allows numpy to see v and d as one-dimensional so
        # that numpy will create a diagonal matrix with v and d as a diagonal
        P = np.diag(v[:, 0], K) + np.diag(d[:, 0])
        Q = 2 * np.dot(np.transpose(P), P)

        self.cost = QuadFn(self.traj, Q)
        self.create_robot_clones()

    def create_robot_clones(self):
        self.robot_clones = []
        env = self.hl_plan.env
        robot = self.robot.get_env_body(env)

        # active_dofs = np.ndarray(0)
        # active_dofs = np.r_[active_dofs, robot.GetManipulator('rightarm').GetArmIndices()]

        # import ipdb; ipdb.set_trace()
        with env:
            # with robot:

            transparency = 0.9
            # traj = self.traj.value.reshape((self.K,self.T), order='F')
            for link in robot.GetLinks():
                for geom in link.GetGeometries():
                    geom.SetTransparency(1)

            dof_values = robot.GetDOFValues()
            active_dofs = robot.GetActiveDOFIndices()
            for t in range(self.T):
            # for t in range(1):
                # xt = self.traj.value[self.K*t:self.K*(t+1)]
                # xt = self.traj.get_value()[:, t:t+1]
                # env.Load(robot.GetXMLFilename())
                # newrobot = self.create_robot_kinbody(name=self.name + "_" + robot.GetName() + str(t), transparency=transparency)
                # newrobot = RaveCreateRobot(env,robot.GetXMLId())
                newrobot = env.ReadRobotXMLFile("../models/pr2/pr2-head-kinect.xml")
                # newrobot = RaveCreateRobot(env)
                # newrobot.Clone(robot,0)

                for link in newrobot.GetLinks():
                    for geom in link.GetGeometries():
                        geom.SetTransparency(transparency)

                newrobot.SetName(self.name + "_" + robot.GetName() + str(t))

                env.Add(newrobot, True)
                newrobot.SetDOFValues(dof_values)
                # newrobot.SetDOFValues([0.3],
                #         [robot.GetJoint("torso_lift_joint").GetDOFIndex()])
                newrobot.SetActiveDOFs(active_dofs)
                # import ipdb; ipdb.set_trace()
                # newrobot.SetActiveDOFValues(xt.flatten())
                self.robot_clones.append(newrobot)
        env.UpdatePublishedBodies()
        print ('waiting...')
        time.sleep(20)
        # import ipdb; ipdb.set_trace()
        # time.sleep(20)

    def plot_traj_robot_kinbodies(self):
        # traj = self.traj.value.reshape((self.K,self.T), order='F')
        if self.robot_clones is None:
            self.create_robot_clones()

        for t in range(self.T):
            xt = self.traj.get_value()[:,t:t+1]
            # xt = self.traj.value[self.K*t:self.K*(t+1)]
            # self.robot_clones[t].SetTransform(base_pose_to_mat(xt))
            self.robot_clones[t].SetActiveDOFValues(xt.flatten())
        return self.robot_clones

    def plot(self):
        self.plot_traj_robot_kinbodies()
        # import ipdb; ipdb.set_trace()
        print ''

def test_not_obstructs_pr2():
    env = cans_world_env()
    robot = env.GetRobots()[0]
    import ctrajoptpy
    cc = ctrajoptpy.GetCollisionChecker(env)
    collisions = cc.BodyVsAll(robot)
    import ipdb; ipdb.set_trace()
    handles = []
    for c in collisions:
        linkAParent = c.GetLinkAParentName()
        linkBParent = c.GetLinkBParentName()
        print linkAParent
        print linkBParent
        linkA = c.GetLinkAName()
        linkB = c.GetLinkBName()
        print linkA
        print linkB

        if linkAParent == robot.GetName():
            ptRobot = c.GetPtA()
            ptObj = c.GetPtB()
        elif linkBParent == robot.GetName():
            ptRobot = c.GetPtB()
            ptObj = c.GetPtA()
        else:
            continue

        distance = c.GetDistance()
        print distance
        handles += plot_collision(env, ptRobot, ptObj, distance)
    import ipdb; ipdb.set_trace()

def test_pr2():
    env = cans_world_env()
    robot = PR2('pr2')
    robot.tuck_arms(env)
    robot._set_active_dofs(env, ['rightarm', 'torso', 'base'])
    robot.set_pose(env, np.array([-0.023593, 1.10728, -1.5566882, -2.124408, -1.4175, \
        -1.8417,  0.21436,  0.,  0.2,  0., -0.]))
    import ipdb; ipdb.set_trace()
    print "end of test_pr2"

def test_not_obstructs_pr2_table():
    env = cans_world_env()
    robot = PR2('pr2')
    robot.tuck_arms(env)
    robot._set_active_dofs(env, ['rightarm', 'torso', 'base'])
    # pose = np.array([-0.023593, 1.10728, -1.5566882, -2.124408, -1.4175, \
    #     -1.8417,  0.21436,  0.,  0.2,  0., -0.])
    # robot.set_pose(env, pose)
    pose = robot.get_pose(env)

    hl_plan = HLPlan(env)
    obj = Obj('object9')
    # obj_pose = np.array([[0.],[0.],[0.]])
    # obj.set_pose(env, obj_pose)

    pos = HLParam("pos", 11, 1, is_var=False, value=robot.get_pose(env))
    obj_pos = HLParam("obj_pos", 3, 1, is_var=False, value=obj.get_pose(env))
    action = TestAction(hl_plan, env, robot, pos, obj)
    fluent = action.precondition
    fluent.pre()

    val, grad = fluent.collisions(pose)

def test_move_pr2_gradient():
    env = cans_world_env()
    obj17 = env.GetKinBody('object17')
    tran = obj17.GetTransform()
    tran[0,3] = .49268
    tran[1,3] = -.51415
    obj17.SetTransform(tran)

    hl_plan = TestDomain(env)
    move_env = env.CloneSelf(1) # clones objects in the environment
    move_pr2 = move_env.GetKinBody('pr2')
    move_pr2.SetActiveDOFs(move_pr2.GetManipulator('rightarm').GetArmIndices())
    move_pr2.SetDOFValues([0.3],
                    [move_pr2.GetJoint("torso_lift_joint").GetDOFIndex()])

    robot = PR2('pr2')
    robot.tuck_arms(env)
    robot._set_active_dofs(env, ['rightarm'])
    pose = robot.get_pose(env)

    hl_plan = HLPlan(env)
    obj = Obj('object17')
    K = 7
    # obj_pose = np.array([[0.],[0.],[0.]])
    # obj.set_pose(env, obj_pose)

    # start = HLParam("pos", self.K, 1, is_var=False, value=robot.get_pose(env))
    start_pose = np.array([[-1.57265580e+00], [5.27875957e-01], [-1.40000000e+00], \
        [-1.58244363e+00], [-1.64143184e+00], [-9.76938766e-02], [-5.78403045e-01]])
    end_pose = np.array([[-1.31066711e+00], [1.08996543e+00], [-1.40000000e+00],\
        [-2.10243169e+00], [2.93131497e+00], [-6.18669215e-01], [5.20253411e-01]])
    start = HLParam("start", K, 1, is_var=False, value=start_pose)
    end = HLParam("end", K, 1, is_var=False, value=end_pose)
    obj_pos = HLParam("obj_pos", 3, 1, is_var=False, value=obj.get_pose(env))

    move = PR2Move(0, hl_plan, move_env, robot, start, end, obj)
    hlas = [move]
    ll_prob = LLProb(hlas)
    ll_prob.solve()
    import ipdb; ipdb.set_trace()

def test_robot():
    env = pick_test_env()
    import ipdb; ipdb.set_trace()
    print ""

def plot_collision(env, ptA, ptB, distance):
    handles = []
    if not np.allclose(ptA, ptB, atol=1e-3):
        if distance < 0:
            handles.append(env.drawarrow(p1=ptA, p2=ptB, linewidth=.002,color=(1,0,0)))
        else:
            handles.append(env.drawarrow(p1=ptA, p2=ptB, linewidth=.002,color=(0,0,0)))
    return handles

if __name__ == "__main__":
    # test_not_obstructs_pr2()
    # test_pr2()
    # test_not_obstructs_pr2_table()
    test_move_pr2_gradient()
    # test_robot()
