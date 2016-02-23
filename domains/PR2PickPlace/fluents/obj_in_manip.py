import numpy as np
from interface.fluents.fluent import AndFluent
from interface.fluents.fluent import FnEQFluent
from opt.function import CollisionFn
from openravepy import poseFromMatrix, matrixFromPose
from numpy.linalg import norm
import ipdb

class ObjInManip(AndFluent):
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
        self.K = hl_action.obj_K

    def pre(self):
        # h = lambda x: self.error(x) # function inequality constraint h(x) = 0
        # self.fn = CollisionFn([self.obj_traj], h)

        h_pos = lambda x: self.pos_error(x) # function inequality constraint h(x) = 0
        h_rot = lambda x: self.rot_error(x) # function inequality constraint h(x) = 0
        # h_rot = lambda x: self.rot_error_num(x) # function inequality constraint h(x) = 0
        self.pos_fn = CollisionFn([self.obj_traj], h_pos)
        self.rot_fn = CollisionFn([self.obj_traj], h_rot)

        pos_fluent = FnEQFluent('pos_' + self.name, self.priority, self.hl_action)
        pos_fluent.fn = self.pos_fn

        rot_fluent = FnEQFluent('up_' + self.name, self.priority, self.hl_action)
        rot_fluent.fn = self.rot_fn
        self.fluents = [pos_fluent, rot_fluent]
        # self.fluents = [pos_fluent]
        # self.fluents = [rot_fluent]

    def pos_error(self, obj_traj):
        q = self.gp.value

        dim = 3
        val = np.zeros((self.T*dim, 1))
        jac = np.zeros((val.size, obj_traj.size))

        for t in range(self.T):
            xt = self.traj.value[:,t].flatten()
            self.robot.set_pose(self.env, xt)

            ot = obj_traj[self.K*t:self.K*(t+1)]
            # import ipdb; ipdb.set_trace()
            self.obj.set_pose(self.env, ot)
            W_T_O = self.obj.get_transform(self.env)
            cur = W_T_O[0:3, 3]
            self.hl_action.plot()


            robot_body = self.robot.get_env_body(self.env)
            tool_link = robot_body.GetLink("r_gripper_tool_frame")
            link_ind = tool_link.GetIndex()

            target = tool_link.GetTransform().dot(np.r_[q, [[1]]])
            target = target[:3]
            # import ipdb; ipdb.set_trace()
            # cur = cur[0:3,3]
            val_t = cur.flatten() - target.flatten()

            jac[dim*t:dim*(t+1), self.K*t:self.K*(t+1)] = np.c_[np.zeros((3,3)), np.eye(3)]
            # import ipdb; ipdb.set_trace()
            val[dim*t:dim*(t+1)] = val_t.reshape((dim,1))

        return (val, jac)

    def rot_error_val(self, obj_traj):
        q = self.gp.value

        dim = 1
        val = np.zeros((self.T*dim, 1))

        for t in range(self.T):
            xt = self.traj.value[:,t].flatten()
            self.robot.set_pose(self.env, xt)
            robot_body = self.robot.get_env_body(self.env)

            ot = obj_traj[self.K*t:self.K*(t+1)].flatten()
            self.obj.set_pose(self.env, ot)
            ot = self.obj.get_pose(self.env).flatten()
            obj_body = self.obj.get_env_body(self.env)
            obj_trans = obj_body.GetTransform()

            # obj_quat = ot[0:4]
            # obj_dir = obj_quat[1:4].copy()
            # if np.all(obj_dir == 0):
            #     obj_dir = np.array([1.,0.,0.])

            local_dir = np.array([0.,0.,1.])
            obj_dir = obj_trans[:3, :3].dot(local_dir)
            # obj_dir = obj_dir/np.linalg.norm(obj_dir)
            # tool_link = robot_body.GetLink("r_gripper_tool_frame")
            tool_link = robot_body.GetLink("r_gripper_palm_link")
            link_ind = tool_link.GetIndex()
            world_dir = tool_link.GetTransform()[:3,:3].dot(local_dir)
            # world_dir = world_dir/np.linalg.norm(world_dir)

            rot_axis = np.cross(obj_dir, world_dir)
            # rot_axis = rot_axis/np.linalg.norm(rot_axis)

            cos_theta = np.dot(obj_dir, world_dir)
            assert cos_theta <= 1.0 and cos_theta >= -1.0
            # print "obj dir: {}".format(obj_dir)
            # print "world_dir: {}".format(world_dir)
            # print "rot axis: {}".format(rot_axis)
            # print "cos theta: {}".format(cos_theta)
            sin_theta = np.sqrt(1 - cos_theta**2)
            target_quat = np.array(np.r_[cos_theta, sin_theta*rot_axis])
            # ipdb.set_trace()

            val_t = obj_dir.dot(world_dir) + 1
            # val_t = obj_quat - target_quat

            # import ipdb; ipdb.set_trace()
            # jac_t = np.zeros((4,7))
            # jac_t [1,1] = 1.0
            # jac_t [2,2] = 1.0
            # jac_t [3,3] = 1.0
            # jac[dim*t:dim*(t+1), self.K*t:self.K*(t+1)] = np.c_[np.eye(4), np.zeros((4,3))]
            # ipdb.set_trace()
            # jac[dim*t:dim*(t+1), self.K*t:self.K*(t+1)] = np.r_[norm(rot_axis), rot_axis, np.zeros(3)]
            # import ipdb; ipdb.set_trace()
            val[dim*t:dim*(t+1)] = val_t.reshape((dim,1))

        return val

    def rot_error_num(self, obj_traj):
        import numdifftools
        # import ipdb; ipdb.set_trace()
        jac_fn = numdifftools.Jacobian(lambda x: self.rot_error(x)[0].flatten())
        num_jac = jac_fn(obj_traj.flatten())
        val = self.rot_error(obj_traj)[0]
        ana_jac = self.rot_error(obj_traj)[1]
        print "num_jac: {}".format(num_jac)
        print "ana_jac: {}".format(ana_jac)
        assert np.allclose(num_jac, ana_jac)
        return val, num_jac

    def rot_error(self, obj_traj):
        q = self.gp.value

        dim = 1
        val = np.zeros((self.T*dim, 1))
        jac = np.zeros((val.size, obj_traj.size))

        for t in range(self.T):
            xt = self.traj.value[:,t].flatten()
            self.robot.set_pose(self.env, xt)
            robot_body = self.robot.get_env_body(self.env)

            ot = obj_traj[self.K*t:self.K*(t+1)].flatten()
            self.obj.set_pose(self.env, ot)
            # assert np.allclose(ot, self.obj.get_pose(self.env))
            # ipdb.set_trace()
            ot = self.obj.get_pose(self.env).flatten()
            obj_body = self.obj.get_env_body(self.env)
            obj_trans = obj_body.GetTransform()

            local_dir = np.array([0.,0.,1.])
            obj_dir = obj_trans[:3, :3].dot(local_dir)
            # obj_dir = obj_dir/np.linalg.norm(obj_dir)
            # tool_link = robot_body.GetLink("r_gripper_tool_frame")
            tool_link = robot_body.GetLink("r_gripper_palm_link")
            link_ind = tool_link.GetIndex()
            world_dir = tool_link.GetTransform()[:3,:3].dot(local_dir)

            axises = self.obj.get_axises(ot)
            rot_jac = np.array([np.dot(world_dir, np.cross(axis, obj_dir)) for axis in axises])
            jac[dim*t:dim*(t+1), self.K*t:self.K*(t+1)] = np.r_[rot_jac, [0,0,0]]
            val[dim*t:dim*(t+1)] = np.dot(obj_dir, world_dir) + 1

        return (val, jac)

    def rot_error2(self, obj_traj):
        q = self.gp.value

        dim = 1
        val = np.zeros((self.T*dim, 1))
        jac = np.zeros((val.size, obj_traj.size))

        for t in range(self.T):
            xt = self.traj.value[:,t].flatten()
            self.robot.set_pose(self.env, xt)
            robot_body = self.robot.get_env_body(self.env)

            ot = obj_traj[self.K*t:self.K*(t+1)].flatten()
            self.obj.set_pose(self.env, ot)
            ot = self.obj.get_pose(self.env).flatten()
            obj_body = self.obj.get_env_body(self.env)
            obj_trans = obj_body.GetTransform()

            # obj_quat = ot[0:4]
            # obj_dir = obj_quat[1:4].copy()
            # if np.all(obj_dir == 0):
            #     obj_dir = np.array([1.,0.,0.])

            local_dir = np.array([0.,0.,1.])
            obj_dir = obj_trans[:3, :3].dot(local_dir)
            # obj_dir = obj_dir/np.linalg.norm(obj_dir)
            # tool_link = robot_body.GetLink("r_gripper_tool_frame")
            tool_link = robot_body.GetLink("r_gripper_palm_link")
            link_ind = tool_link.GetIndex()
            world_dir = tool_link.GetTransform()[:3,:3].dot(local_dir)
            # world_dir = world_dir/np.linalg.norm(world_dir)

            # rot_axis = np.cross(obj_dir, world_dir)
            rot_axis = np.cross(world_dir, obj_dir)
            # rot_axis = rot_axis/np.linalg.norm(rot_axis)

            cos_theta = np.dot(obj_dir, world_dir)
            assert cos_theta <= 1.0 and cos_theta >= -1.0
            print "obj dir: {}".format(obj_dir)
            print "world_dir: {}".format(world_dir)
            print "rot axis: {}".format(rot_axis)

            qw = ot[0]
            qx = ot[1]
            qy = ot[2]
            qz = ot[3]
            if np.isclose(norm(qw), 1):
                import ipdb; ipdb.set_trace()
            # d = (1-qw**2)**(1.5)
            # d2 = np.sqrt(1-qw**2)
            # drdp = np.array([[-qw*(qy**2+qz**2)/qx, (qy**2+qz**2), -qx*qy, -qx*qz],
            #             [-qw*(qx**2+qz**2)/qy, -qy*qx, (qx**2+qz**2), -qy*qz],
            #             [-qw*(qx**2+qy**2)/qz, -qz*qx, -qz*qy, (qy**2+qz**2)],
            #             ])
            # drdq = np.array([[qw*qx/d, 1., 0., 0.], [qw*qy/d, 0., 1., 0.], [qw*qz/d, 0., 0., 1.]])/d2
            drdq = drdp/d
            # import ipdb; ipdb.set_trace()
            # print "cos theta: {}".format(cos_theta)
            sin_theta = np.sqrt(1 - cos_theta**2)
            target_quat = np.array(np.r_[cos_theta, sin_theta*rot_axis])
            # ipdb.set_trace()

            val_t = obj_dir.dot(world_dir) + 1
            np.dot(p, )
            # val_t = obj_quat - target_quat

            # import ipdb; ipdb.set_trace()
            # jac_t = np.zeros((4,7))
            # jac_t [1,1] = 1.0
            # jac_t [2,2] = 1.0
            # jac_t [3,3] = 1.0
            # jac[dim*t:dim*(t+1), self.K*t:self.K*(t+1)] = np.c_[np.eye(4), np.zeros((4,3))]
            # ipdb.set_trace()
            # jac[dim*t:dim*(t+1), self.K*t:self.K*(t+1)] = np.r_[norm(rot_axis), rot_axis, np.zeros(3)]
            # jac[dim*t:dim*(t+1), self.K*t:self.K*(t+1)] = np.r_[0., rot_axis, np.zeros(3)]
            jac[dim*t:dim*(t+1), self.K*t:self.K*(t+1)] = np.r_[rot_axis.dot(drdq), np.zeros(3)]
            # j1 = -qw*(1-qw**2)**(-1.5)
            # jac[dim*t:dim*(t+1), self.K*t:self.K*(t+1)] = np.r_[j1, rot_axis, np.zeros(3)]

            # import ipdb; ipdb.set_trace()
            val[dim*t:dim*(t+1)] = val_t.reshape((dim,1))

        return (val, jac)

    def error(self, obj_traj):
        EE_T_O = matrixFromPose(self.gp.value)
        O_T_EE = np.linalg.inv(EE_T_O)

        K = self.K
        val = np.zeros((self.T*K, 1))
        jac = np.zeros((val.size, obj_traj.size))
        for t in range(self.T):
            xt = self.traj.value[:,t].flatten()
            self.robot.set_pose(self.env, xt)

            # ot = matrixFromPose(self.obj_traj.value[:, t])
            ot = obj_traj[self.K*t:self.K*(t+1)]
            self.obj.set_pose(self.env, ot.flatten())
            W_T_O_cur = self.obj.get_transform(self.env)
            W_T_O_cur = poseFromMatrix(W_T_O_cur)

            self.hl_action.plot()

            robot_body = self.robot.get_env_body(self.env)
            rarm = robot_body.GetManipulator('rightarm')
            W_T_EE = rarm.GetEndEffectorTransform()

            W_T_O_target = np.dot(W_T_EE, EE_T_O)
            W_T_O_target = poseFromMatrix(W_T_O_target)

            obj_body = self.obj.get_env_body(self.env)
            robot_body = self.robot.get_env_body(self.env)

            # val = np.array((3,1))
            val[K*t:K*(t+1),0] = W_T_O_cur - W_T_O_target
            jac[K*t:K*(t+1), K*t:K*(t+1)] = np.eye(K)

        return (val, jac)
