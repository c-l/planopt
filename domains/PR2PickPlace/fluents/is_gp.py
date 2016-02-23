import numpy as np
from openravepy import KinBody, Environment, RaveCreateKinBody, TriMesh, \
GeometryType, poseFromMatrix, axisAngleFromRotationMatrix
from opt.function import CollisionFn
from interface.fluents.fluent import FnEQFluent
from interface.fluents.fluent import AndFluent

class PR2IsGP(AndFluent):
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

        h_loc = lambda x: self.error(x) # function inequality constraint h(x) = 0
        h_up = lambda x: self.face_up(x) # function inequality constraint h(x) = 0
        obj_loc_list = []
        self.loc_fn = CollisionFn([self.traj], h_loc)
        self.up_fn = CollisionFn([self.traj], h_up)

        loc_fluent = FnEQFluent('loc_' + self.name, self.priority, self.hl_action)
        loc_fluent.fn = self.loc_fn

        up_fluent = FnEQFluent('up_' + self.name, self.priority, self.hl_action)
        up_fluent.fn = self.up_fn
        self.fluents = [loc_fluent, up_fluent]

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
        ee_trans = rarm.GetEndEffectorTransform()
        ee_pose = poseFromMatrix(ee_trans)
        # print "ee pose: ", ee_pose
        trans_jac = rarm.CalculateJacobian()

       # jac = np.vstack((rot_jac, trans_jac))
        jac = np.hstack((trans_jac, np.zeros((3,1)))) # gripper doesn't affect value

        obj_body = self.obj.get_env_body(self.env)
        obj_trans = obj_body.GetTransform()
        obj_trans[2,3] = obj_trans[2,3] + .125
        # obj_trans[2,3] = obj_trans[2,3] + .325
        obj_pose = poseFromMatrix(obj_trans)

        val = ee_pose[4:7] - obj_pose[4:7]
        # val = val[4:7]

        val = val.reshape((len(val), 1))

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
        jac = np.hstack((jac, np.zeros((2,1)))) # gripper opening doesn't affect manipulator facing up
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
