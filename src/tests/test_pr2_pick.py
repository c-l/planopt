from openravepy import KinBody, Environment, RaveCreateKinBody, TriMesh, \
GeometryType, poseFromMatrix, axisAngleFromRotationMatrix
import numpy as np
from test_utils import *
from interface.ll_prob import LLProb
from interface.hl_actions.hl_action import HLAction
from interface.hl_plan import HLPlan
from interface.hl_param import HLParam, Traj, Robot, Obj, PR2
import time

from test_pr2 import NotObstructsPR2


class TestDomain(object):
    def __init__(self, env):
        self.env = env

from interface.fluents.robot_at import RobotAt
from opt.function import CollisionFn
from interface.fluents.fluent import FnEQFluent

class PR2IsGP(FnEQFluent):
    def __init__(self, env, hl_action, robot, priority, obj, traj):
        self.env = env
        self.hl_action = hl_action
        self.plotting_env = hl_action.hl_plan.env
        self.priority = priority
        self.robot = robot
        self.obj = obj
        self.traj = traj
        self.name = "IsGP(" + self.obj.name + ')'

        self.T = hl_action.T
        self.K = hl_action.K

    def pre(self):
        traj = self.traj

        # h = lambda x: self.error(x) # function inequality constraint g(x) <= 0
        h = lambda x: self.face_up(x) # function inequality constraint g(x) <= 0
        obj_loc_list = []
        self.fn = CollisionFn([self.traj], h)

    def error(self, traj):
        self.traj.value = traj

        # for t in range(self.T):
        t = 0
        xt = traj[self.K*t:self.K*(t+1)]
        self.robot.set_pose(self.env, xt)
        self.hl_action.plot()
        robot_body = self.robot.get_env_body(self.env)
        # self.plot_finger_pad(robot_body)
        rarm = robot_body.GetManipulator('rightarm')
        # ee_trans = rarm.GetEndEffector().GetTransform()
        ee_trans = rarm.GetEndEffectorTransform()
        ee_pose = poseFromMatrix(ee_trans)
        # print "ee pose: ", ee_pose
        rot_jac = rarm.CalculateRotationJacobian()
        trans_jac = rarm.CalculateJacobian()

        # robot_body.CalculateActiveJacobian()
        # rot_jac = robot_body.CalculateActiveRotationJacobian()
        # trans_jac = robot_body.CalculateActiveJacobian()
        # jac = np.vstack((rot_jac, trans_jac))
        # jac = trans_jac
        # jac = np.hstack((trans_jac, np.zeros((3,1))))
        # no jacobian on constaint and x axis because penalty only exists for deviations on the y and z axis
        # rot_jac[0,:] = 0
        # rot_jac[3,:] = 0
        # import ipdb; ipdb.set_trace()
        # jac = np.vstack((rot_jac, trans_jac))
        jac = np.vstack((rot_jac, np.zeros((3,7))))
        jac = np.hstack((jac, np.zeros((7,1)))) # gripper doesn't affect value
        # jac = np.vstack((np.zeros(3,2), rot_jac, trans_jac, np.zeros(3,1)))

        obj_body = self.obj.get_env_body(self.env)
        obj_trans = obj_body.GetTransform()
        # obj_trans[2,3] = obj_trans[2,3] + .125
        obj_trans[2,3] = obj_trans[2,3] + .325
        obj_pose = poseFromMatrix(obj_trans)
        obj_pose[0] = 0
        obj_pose[2] = 1
        # print "obj pose: ", obj_pose

        # import ipdb; ipdb.set_trace()
        val = ee_pose - obj_pose
        # val = val[4:7]

        # adding constraint on rotation along a specific axis
        # val[0] = ee_pose[0] - 1
        # val[1] = ee_pose[1]
        # val[2] = ee_pose[2]
        val[3] = 0
        jac[3,:] = 0
        val[4:] = 0
        val = val.reshape((len(val), 1))
        # import ipdb; ipdb.set_trace()
        print 'val: ', val
        # return (val, -1*jac)

        return (val, jac)

    def face_up(self, traj):
        t = 0
        xt = traj[self.K*t:self.K*(t+1)]
        self.robot.set_pose(self.env, xt)
        self.hl_action.plot()

        robot = self.robot.get_env_body(self.env)

        manip = robot.GetManipulator("rightarm")
        arm_inds = manip.GetArmIndices()
        arm_joints = [robot.GetJointFromDOFIndex(ind) for ind in arm_inds]

        tool_link = robot.GetLink("r_gripper_tool_frame")
        local_dir = np.array([0.,0.,1.])

        val = tool_link.GetTransform()[:2,:3].dot(local_dir)

        world_dir = tool_link.GetTransform()[:3,:3].dot(local_dir)
        jac = np.array([np.cross(joint.GetAxis(), world_dir)[:2] for joint in arm_joints]).T.copy()
        # import ipdb; ipdb.set_trace()

        return (val, jac)



    def plot_finger_pad(self, robot_body):
        rarm = robot_body.GetManipulator('rightarm')
        # ee_tool = rarm.GetLocalToolDirection()
        ee_normal = np.array([0, -1, 0])
        ee_trans = rarm.GetEndEffectorTransform()
        # vector = np.dot(ee_trans[0:3,0:3], ee_tool)
        vector = np.dot(ee_trans[0:3,0:3], ee_normal)
        # vector = ee_tool

        trans = robot_body.GetLink("r_gripper_r_finger_tip_link").GetTransform()
        # vector = axisAngleFromRotationMatrix(trans[0:3,0:3])
        pt1 = trans[0:3, 3] + np.array([ 0.0125, -0.016 , 0.])
        pt2 = pt1 + vector/100
        handles = []
        handles.append(self.plotting_env.drawarrow(p1=pt1, p2=pt2, linewidth=.001,color=(1,0,0)))
        self.hl_action.add_plot_handles(handles)

class PR2Pick(HLAction):

    def __init__(self, lineno, hl_plan, env, robot, pos, obj, loc=None, gp=None):
        super(PR2Pick, self).__init__(lineno, hl_plan, env, robot)

        self.hl_plan = hl_plan
        self.pos = pos
        self.obj = obj

        # TODO: set this using optimization domain
        self.T = 1
        # self.K = 8
        self.K = 7
        T = self.T
        K = self.K
        KT = self.K*self.T

        self.name = "pick" + str(lineno)
        self.traj = Traj(self, self.name + "_traj", self.K, self.T, is_var=True)

        self.params = [pos, self.traj]
        self.preconditions = [RobotAt(self, 0, pos, self.traj)]
        self.preconditions += [PR2IsGP(self.env, self, robot, 0, obj, self.traj)]
        self.preconditions += [NotObstructsPR2(self.env, self, robot, 1, self.traj, self.obj)]

        # self.preconditions += [NotObstructsPR2(env, self, robot, 1, self.traj, obj)]

        self.postconditions = []
        # self.postconditions += [RobotAt(self, 0, self.end, self.traj)]

        self.cost = 0
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

            # transparency = 0.9
            transparency = 0
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
        # time.sleep(20)
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

def test_pick_pr2_gradient():
    env = cans_world_env()
    obj17 = env.GetKinBody('object17')
    tran = obj17.GetTransform()
    tran[0,3] = .49268
    tran[1,3] = -.51415
    obj17.SetTransform(tran)

    hl_plan = TestDomain(env)
    pick_env = env.CloneSelf(1) # clones objects in the environment
    pick_pr2 = pick_env.GetKinBody('pr2')
    pick_pr2.SetActiveDOFs(pick_pr2.GetManipulator('rightarm').GetArmIndices())
    pick_pr2.SetDOFValues([0.3],
                    [pick_pr2.GetJoint("torso_lift_joint").GetDOFIndex()])

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
    pose = np.array([[-1.57265580e+00], [5.27875957e-01], [-1.40000000e+00], \
        [-1.58244363e+00], [-1.64143184e+00], [-9.76938766e-02], [-5.78403045e-01]])
    pos = HLParam("pos", K, 1, value=pose)
    # end = HLParam("end", K, 1, is_var=True, value=end_pose)
    # obj_pos = HLParam("obj_pos", 3, 1, is_var=False, value=obj.get_pose(env))

    pick = PR2Pick(0, hl_plan, pick_env, robot, pos, obj)
    pick.preconditions[-1].error(pose)
    import ipdb; ipdb.set_trace()
    hlas = [pick]
    ll_prob = LLProb(hlas)
    ll_prob.solve()
    import ipdb; ipdb.set_trace()


def test_pick():
    env = cans_world_env()
    obj17 = env.GetKinBody('object17')
    tran = obj17.GetTransform()
    tran[0,3] = .49268
    tran[1,3] = -.51415
    obj17.SetTransform(tran)

    hl_plan = TestDomain(env)
    pick_env = env.CloneSelf(1) # clones objects in the environment
    pick_pr2 = pick_env.GetKinBody('pr2')
    # active_dof_indices = np.r_[pick_pr2.GetManipulator('rightarm').GetArmIndices(),
    #                         pick_pr2.GetManipulator('rightarm').GetGripperIndices()]
    active_dof_indices = pick_pr2.GetManipulator('rightarm').GetArmIndices()
    pick_pr2.SetActiveDOFs(active_dof_indices)
    pick_pr2.SetDOFValues([0.3],
                    [pick_pr2.GetJoint("torso_lift_joint").GetDOFIndex()])

    robot = PR2('pr2')
    robot.tuck_arms(env)
    robot._set_active_dofs(env, ['rightarm'])
    # order matters for poses and for computing jacobian
    # robot._set_active_dofs(env, ['rightarm', 'rgripper'])
    pose = robot.get_pose(env)

    hl_plan = HLPlan(env)
    obj = Obj('object17')
    K = 7
    # obj_pose = np.array([[0.],[0.],[0.]])
    # obj.set_pose(env, obj_pose)

    # start = HLParam("pos", self.K, 1, is_var=False, value=robot.get_pose(env))
    # pose = np.array([[-1.57265580e+00], [5.27875957e-01], [-1.40000000e+00], \
    #     [-1.58244363e+00], [-1.64143184e+00], [-9.76938766e-02], [-5.78403045e-01], [0.0]])
    pose = np.array([[-1.57265580e+00], [5.27875957e-01], [-1.40000000e+00], \
        [-1.58244363e+00], [-1.64143184e+00], [-9.76938766e-02], [-5.78403045e-01]])
    pos = HLParam("pos", K, 1, value=pose)
    # end = HLParam("end", K, 1, is_var=True, value=end_pose)
    # obj_pos = HLParam("obj_pos", 3, 1, is_var=False, value=obj.get_pose(env))

    pick = PR2Pick(0, hl_plan, pick_env, robot, pos, obj)
    # pick.preconditions[-1].error(pose)
    hlas = [pick]
    ll_prob = LLProb(hlas)
    ll_prob.solve()
    import ipdb; ipdb.set_trace()

if __name__ == "__main__":
    # test_pick_pr2_gradient()
    test_pick()
