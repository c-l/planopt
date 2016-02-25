import numpy as np
from interface.fluents.fluent import AndFluent
from interface.fluents.fluent import FnEQFluent
from opt.function import CollisionFn
from openravepy import poseFromMatrix, matrixFromPose, quatFromAxisAngle

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
        h_rot = lambda x: self.rot_error(x) # function inequality constraint h(x) = 0
        self.pos_fn = CollisionFn([self.traj], h_pos)
        self.rot_fn = CollisionFn([self.traj], h_rot)

        pos_fluent = FnEQFluent('pos_' + self.name, self.priority, self.hl_action)
        pos_fluent.fn = self.pos_fn

        rot_fluent = FnEQFluent('up_' + self.name, self.priority, self.hl_action)
        rot_fluent.fn = self.rot_fn
        self.fluents = [pos_fluent, rot_fluent]
        # self.fluents = [pos_fluent]
        # self.fluents = [rot_fluent]

    def pos_error(self,traj):
        gp = np.array([0,0,.125])

        dim = 3
        val = np.zeros((self.T*dim, 1))
        jac = np.zeros((val.size, traj.size))

        for t in range(self.T):
            xt = traj[self.K*t:self.K*(t+1)]
            self.robot.set_pose(self.env, xt)

            ot = self.obj_traj.value[:,t].flatten()
            self.obj.set_pose(self.env, ot)
            W_T_O = self.obj.get_transform(self.env)
            target = W_T_O[0:3, 3] + gp
            # target[2] += 0.125
            self.hl_action.plot()


            robot_body = self.robot.get_env_body(self.env)
            inds = robot_body.GetActiveDOFIndices()
            joints = [robot_body.GetJointFromDOFIndex(ind) for ind in inds]

            tool_link = robot_body.GetLink("r_gripper_tool_frame")
            link_ind = tool_link.GetIndex()

            # cur = tool_link.GetTransform().dot(np.r_[q, [[1]]])
            # cur = cur[:3]
            # grasp is fixed
            cur = tool_link.GetTransform()
            cur = cur[:3, 3]
            val_t = cur.flatten() - target

            jac_t = np.array([np.cross(joint.GetAxis(), cur.flatten() - joint.GetAnchor()) for joint in joints]).T.copy()
            jac[dim*t:dim*(t+1), self.K*t:self.K*(t+1)] = jac_t
            # import ipdb; ipdb.set_trace()
            val[dim*t:dim*(t+1)] = val_t.reshape((dim,1))

        return (val, jac)

    def rot_error(self,traj):
        dim = 1
        val = np.zeros((self.T*dim, 1))
        jac = np.zeros((val.size, traj.size))

        for t in range(self.T):
            xt = traj[self.K*t:self.K*(t+1)]
            self.robot.set_pose(self.env, xt)

            ot = self.obj_traj.value[:,t].flatten()
            self.obj.set_pose(self.env, ot)
            W_T_O = self.obj.get_transform(self.env)
            self.hl_action.plot()

            robot_body = self.robot.get_env_body(self.env)
            inds = robot_body.GetActiveDOFIndices()
            joints = [robot_body.GetJointFromDOFIndex(ind) for ind in inds]

            tool_link = robot_body.GetLink("r_gripper_tool_frame")
            link_ind = tool_link.GetIndex()

            local_dir = np.array([0.,0.,1.])
            obj_dir = np.dot(W_T_O[:3,:3], local_dir)
            # import ipdb; ipdb.set_trace()
            world_dir = tool_link.GetTransform()[:3,:3].dot(obj_dir)
            rot_jac = np.array([np.dot(obj_dir, np.cross(joint.GetAxis(), world_dir)) for joint in joints]).T.copy()

            jac[dim*t:dim*(t+1), self.K*t:self.K*(t+1)] = rot_jac
            # val[dim*t:dim*(t+1)] = np.dot(local_dir, world_dir) + 1
            val[dim*t:dim*(t+1)] = np.dot(obj_dir, world_dir) + 1

        return (val, jac)
