import numpy as np
from interface.fluents.fluent import AndFluent
from interface.fluents.fluent import FnEQFluent
from opt.function import CollisionFn
from openravepy import poseFromMatrix, matrixFromPose, quatFromAxisAngle
from numdifftools import Jacobian

class PR2InManip(AndFluent):
    def __init__(self, env, hl_action, robot, priority, obj, gp, traj, obj_traj):
        self.env = env
        self.hl_action = hl_action
        self.robot = robot
        self.priority = priority
        self.obj = obj
        self.gp = gp
        self.traj = traj
        self.obj_traj = obj_traj
        self.name = "InManip"

        self.T = hl_action.T
        self.K = hl_action.K

    def pre(self):
        traj = self.traj

        h_pos = lambda x: self.pos_error(x) # function inequality constraint h(x) = 0
        # h_pos = lambda x: self.num_check(self.pos_error, x) # numerical gradient check on pos error
        h_rot = lambda x: self.rot_error(x) # function inequality constraint h(x) = 0
        # h_rot = lambda x: self.num_check(self.rot_error, x) # numerical gradient check on rot error
        self.pos_fn = CollisionFn([self.traj, self.obj_traj], h_pos)
        self.rot_fn = CollisionFn([self.traj, self.obj_traj], h_rot)

        pos_fluent = FnEQFluent('pos_' + self.name, self.priority, self.hl_action)
        pos_fluent.fn = self.pos_fn

        rot_fluent = FnEQFluent('up_' + self.name, self.priority, self.hl_action)
        rot_fluent.fn = self.rot_fn
        self.fluents = [pos_fluent, rot_fluent]
        # self.fluents = [pos_fluent]
        # self.fluents = [rot_fluent]

    def num_check(self, fn, traj):
        val, ana_jac = fn(traj)
        jac_fn = Jacobian(lambda x: fn(x)[0].flatten())
        num_jac = jac_fn(traj.flatten())
        # assert np.allclose(num_jac, ana_jac)
        if not np.allclose(num_jac, ana_jac):
            inds = np.where(np.isclose(num_jac, ana_jac) == False)
            print inds
            print "num_jac: {}".format(num_jac[inds])
            print "ana_jac: {}".format(ana_jac[inds])
            fn(traj, debug=True)
            # import ipdb; ipdb.set_trace()
            print "ratio: {}".format(np.divide(num_jac[inds],ana_jac[inds]))

        return (val, num_jac)


    def pos_error(self,traj,debug=False, analytical=False):
        gp = np.array([0,0,.125])
        obj_K = 6
        obj_offset = self.T*self.K

        dim = 3
        val = np.zeros((self.T*dim, 1))
        jac = np.zeros((val.size, traj.size))

        for t in range(self.T):
            xt = traj[self.K*t:self.K*(t+1)]
            self.robot.set_pose(self.env, xt)

            ot = traj[obj_K*t + obj_offset:obj_K*(t+1) + obj_offset]
            self.obj.set_pose(self.env, ot)
            W_T_O = self.obj.get_transform(self.env)
            obj_pos = np.dot(W_T_O, np.r_[gp, 1])[:3]
            # obj_pos = ot[3:6].flatten() + gp
            # target[2] += 0.125
            self.hl_action.plot()


            robot_body = self.robot.get_env_body(self.env)
            # inds = robot_body.GetActiveDOFIndices()
            rarm_inds = robot_body.GetManipulator('rightarm').GetArmIndices()
            rarm_joints = [robot_body.GetJointFromDOFIndex(ind) for ind in rarm_inds]


            tool_link = robot_body.GetLink("r_gripper_tool_frame")
            link_ind = tool_link.GetIndex()

            robot_pos = tool_link.GetTransform()
            robot_pos = robot_pos[:3, 3]
            val_t = robot_pos.flatten() - obj_pos.flatten()

            rarm_jac = np.array([np.cross(joint.GetAxis(), robot_pos.flatten() - joint.GetAnchor()) for joint in rarm_joints]).T.copy()
            base_jac = np.eye(3)
            base_jac[2,2] = 0
            bodypart_jac = {"rightarm": rarm_jac, "base": base_jac}
            robot_jac_t = self.robot.jac_from_bodypart_jacs(bodypart_jac, 3)

            # if np.isclose(r_wrist_flex_joint.GetValue(0), 0.) and analytical:
            #     import ipdb; ipdb.set_trace()
            if debug:
                handles = []
                r_wrist_flex_joint = robot_body.GetJoint('r_wrist_flex_joint')
                point = r_wrist_flex_joint.GetAnchor()
                print np.array([np.cross(joint.GetAxis(), robot_pos.flatten() - joint.GetAnchor()) for joint in [r_wrist_flex_joint]]).T.copy()
                handles.append(self.hl_action.hl_plan.env.plot3(points=point,pointsize=25.0,colors=(0,0,1,0.2)))
                handles.append(self.hl_action.hl_plan.env.plot3(points=robot_pos,pointsize=25.0,colors=(0,1,1,0.2)))
                handles.append(self.hl_action.hl_plan.env.plot3(points=point + 0.1*r_wrist_flex_joint.GetAxis(),pointsize=25.0,colors=(0,1,1,0.2)))
                print "joint value:", r_wrist_flex_joint.GetValue(0)
                self.env.UpdatePublishedBodies()
                import ipdb; ipdb.set_trace()

            # import ipdb; ipdb.set_trace()

            axises = self.obj.get_axises(ot)
            obj_jac_t = -1*np.array([np.cross(axis, obj_pos - W_T_O[:3,3].flatten()) for axis in axises]).T
            obj_jac_t = np.c_[obj_jac_t, -np.eye(3)]
            # import ipdb; ipdb.set_trace()
            jac[dim*t:dim*(t+1), self.K*t:self.K*(t+1)] = robot_jac_t
            jac[dim*t:dim*(t+1), obj_K*t+obj_offset:obj_K*(t+1)+obj_offset] = obj_jac_t
            # import ipdb; ipdb.set_trace()
            val[dim*t:dim*(t+1)] = val_t.reshape((dim,1))
            # print "val: ", val

        return (10*val, 10*jac)

    def rot_error(self,traj,debug=False):
        dim = 1
        obj_K = 6
        obj_offset = self.T*self.K

        val = np.zeros((self.T*dim, 1))
        jac = np.zeros((val.size, traj.size))

        for t in range(self.T):
            xt = traj[self.K*t:self.K*(t+1)]
            self.robot.set_pose(self.env, xt)

            ot = traj[obj_K*t + obj_offset:obj_K*(t+1) + obj_offset]
            self.obj.set_pose(self.env, ot)
            W_T_O = self.obj.get_transform(self.env)
            self.hl_action.plot()

            robot_body = self.robot.get_env_body(self.env)

            tool_link = robot_body.GetLink("r_gripper_tool_frame")
            link_ind = tool_link.GetIndex()

            local_dir = np.array([0.,0.,1.])
            obj_dir = np.dot(W_T_O[:3,:3], local_dir)
            world_dir = tool_link.GetTransform()[:3,:3].dot(local_dir)

            # computing robot's jacobian
            rarm_inds = robot_body.GetManipulator('rightarm').GetArmIndices()
            rarm_joints = [robot_body.GetJointFromDOFIndex(ind) for ind in rarm_inds]
            rarm_jac = np.array([np.dot(obj_dir, np.cross(joint.GetAxis(), world_dir)) for joint in rarm_joints]).T.copy()
            rarm_jac = rarm_jac.reshape((1, len(rarm_joints)))
            base_jac = np.array(np.dot(obj_dir, np.cross([0,0,1], world_dir)))
            base_jac = np.array([[0, 0, base_jac]])
            bodypart_jac = {"rightarm": rarm_jac, "base": base_jac}
            robot_jac_t = self.robot.jac_from_bodypart_jacs(bodypart_jac, 1)

            r_wrist_flex_joint = robot_body.GetJoint('r_wrist_flex_joint')
            if debug:
                print "joint value:", r_wrist_flex_joint.GetValue(0)
                import ipdb; ipdb.set_trace()
            # computing object's jacobian
            axises = self.obj.get_axises(ot)
            obj_jac_t = np.array([np.dot(world_dir, np.cross(axis, obj_dir)) for axis in axises])
            obj_jac_t = np.r_[obj_jac_t, [0,0,0]]

            jac[dim*t:dim*(t+1), self.K*t:self.K*(t+1)] = robot_jac_t
            jac[dim*t:dim*(t+1), obj_K*t+obj_offset:obj_K*(t+1)+obj_offset] = obj_jac_t
            val[dim*t:dim*(t+1)] = np.dot(obj_dir, world_dir) + 1

        return (val, jac)
