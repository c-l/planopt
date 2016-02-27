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
        self.K = 6

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
        gp = np.array([0,0,.125])

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
            cur = W_T_O[0:3, 3] + gp
            self.hl_action.plot()
            # import ipdb; ipdb.set_trace()


            robot_body = self.robot.get_env_body(self.env)
            tool_link = robot_body.GetLink("r_gripper_tool_frame")
            link_ind = tool_link.GetIndex()

            # target = tool_link.GetTransform().dot(np.r_[q, [[1]]])
            # target = target[:3]
            target = tool_link.GetTransform()
            target = target[:3,3]
            # import ipdb; ipdb.set_trace()
            # cur = cur[0:3,3]
            val_t = cur.flatten() - target.flatten()

            jac[dim*t:dim*(t+1), self.K*t:self.K*(t+1)] = np.c_[np.zeros((3,3)), np.eye(3)]
            # import ipdb; ipdb.set_trace()
            val[dim*t:dim*(t+1)] = val_t.reshape((dim,1))

        return (val, jac)

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
        # q = self.gp.value
        q = np.array([0,0,.125]).reshape((3,1))

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
