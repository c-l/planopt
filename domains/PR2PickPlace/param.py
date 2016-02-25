import numpy as np
import openravepy
from utils import obj_pose_from_transform, transform_from_obj_pose, _axis_rot_matrices
from interface.hl_param import HLParam
import ipdb


class ObjLoc(HLParam):
    def __init__(self, name, rows=3, cols=1, is_var=True,
                 value=None, is_resampled=False, region=None):
        super(ObjLoc, self).__init__(name, rows, cols, is_var, value, is_resampled)
        if self.is_resampled:
            assert region is not None
        if region is not None:
            assert self.is_var and self.is_resampled
            self.min_x, self.max_x, self.min_y, self.max_y = region

    def generator(self):
        while True:
            x = settings.RANDOM_STATE.rand() * (self.max_x - self.min_x) + self.min_x
            y = settings.RANDOM_STATE.rand() * (self.max_y - self.min_y) + self.min_y
            yield np.array([[x], [y], [0]])

class Obj(HLParam):
    def __init__(self, name):
        self.name = name
        self.is_var = False
        self.is_resampled = False

    def get_pose(self, env):
        obj = self.get_env_body(env)
        obj_transform = obj.GetTransform()
        pose = obj_pose_from_transform(obj_transform)
        return pose

    def set_pose(self, env, pose):
        transform = transform_from_obj_pose(pose)
        # ipdb.set_trace()
        obj = self.get_env_body(env)
        obj.SetTransform(transform)

    def get_axises(self, pose):
        Rz, Ry, Rx = _axis_rot_matrices(pose)
        axis_x = np.dot(Rz, np.dot(Ry, [1,0,0]))
        # axis_y = np.dot(Rx, [0,1,0])
        axis_y = np.dot(Rz, [0,1,0])
        # axis_z = np.dot(np.dot(Ry, Rx), [0,0,1])
        axis_z = [0,0,1]
        # ipdb.set_trace()
        # return (axis_z, axis_y, [1,0,0])
        return (axis_z, axis_y, axis_x)
        # return ([0,0,1], [0,1,0], [1,0,0])


    def get_transform(self, env):
        return self.get_env_body(env).GetTransform()

    def get_env_body(self, env):
        return env.GetKinBody(self.name)

class Robot(Obj):
    pass

class PR2(Robot):
    bodypart_to_dofs = {"rightarm": 7, "leftarm":7, "rgripper": 1, "lgripper": 1,
        "torso": 1, "base": 3}

    def __init__(self, name, active_bodyparts=None):
        super(PR2, self).__init__(name)
        self.bodyparts_to_inds = {}
        self.active_bodyparts = active_bodyparts
        self.dofs = self._get_dofs()

    def init_for_env(self, env):
        self._set_active_dofs(env, self.active_bodyparts)

    def _get_dofs(self):
        total_dofs = 0
        for active_bodypart in self.active_bodyparts:
            if active_bodypart in self.bodypart_to_dofs:
                dofs = self.bodypart_to_dofs[active_bodypart]
                self.bodyparts_to_inds[active_bodypart] = np.array(range(dofs)) + total_dofs
                total_dofs += dofs
            else:
                raw_input('not a valid active bodypart!')
        return total_dofs

    def get_pose(self, env):
        robot = self.get_env_body(env)
        pose = robot.GetActiveDOFValues()
        pose = np.reshape(pose, (len(pose), 1))
        return pose
        # robot = self.get_env_body(env)
        # pose = robot.GetDOFValues()
        # return pose

    def set_pose(self, env, dof_vals):
        robot = self.get_env_body(env)
        pose = robot.SetActiveDOFValues(dof_vals.flatten())
        # self.get_env_body(env).SetDOFValues(dof_vals)

    def tuck_arms(self, env):
        pr2_l_arm_tucked = [0.06024, 1.248526, 1.789070, -1.683386, -1.7343417, -0.0962141, -0.0864407]

        pr2_r_arm_tucked = [-0.023593, 1.1072800, -1.5566882, -2.124408, -1.4175, -1.8417, 0.21436]
        pr2_torso = [0.3]
        robot = self.get_env_body(env)
        # robot.SetDOFValues(pr2_r_arm_tucked,
        #                robot.GetManipulator("rightarm").GetArmIndices())
        robot.SetDOFValues(pr2_l_arm_tucked,
                       robot.GetManipulator("leftarm").GetArmIndices())
        robot.SetDOFValues(pr2_torso,
                        [robot.GetJoint("torso_lift_joint").GetDOFIndex()])

    def _set_active_dofs(self, env, active_bodyparts):
        robot = self.get_env_body(env)
        active_dofs_inds = np.ndarray(0)
        # for active_bodypart in sorted(active_bodyparts):
        # we assume that base is the last DOF throughout this code
        if "base" in active_bodyparts:
            assert active_bodyparts[-1] == "base"

        for active_bodypart in active_bodyparts:
            if active_bodypart in {'rightarm', 'leftarm'}:
                active_dofs_inds = np.r_[active_dofs_inds, robot.GetManipulator(active_bodypart).GetArmIndices()]
            elif active_bodypart in {'rgripper'}:
                active_dofs_inds = np.r_[active_dofs_inds, robot.GetManipulator("rightarm").GetGripperIndices()]
            elif active_bodypart in {'lgripper'}:
                active_dofs_inds = np.r_[active_dofs_inds, robot.GetManipulator("leftarm").GetGripperIndices()]
            elif active_bodypart == 'torso':
                active_dofs_inds = np.r_[active_dofs_inds, robot.GetJoint("torso_lift_joint").GetDOFIndex()]

        if 'base' in active_bodyparts:
            robot.SetActiveDOFs(
                active_dofs_inds,
                openravepy.DOFAffine.X + openravepy.DOFAffine.Y + openravepy.DOFAffine.RotationAxis,
                [0, 0, 1])
        else:
            # active_dofs = sorted(active_dofs)
            robot.SetActiveDOFs(active_dofs_inds)

    def jac_from_bodypart_jacs(self, bodypart_jac, dim):
        # jac = np.ndarray(0)
        jac = None
        for bodypart in self.active_bodyparts:
            if bodypart in bodypart_jac:
                if jac is None:
                    jac = bodypart_jac[bodypart].copy()
                else:
                    jac = np.hstack((jac, bodypart_jac[bodypart]))
            else:
                assert bodypart in self.bodypart_to_dofs
                dofs = self.bodypart_to_dofs[bodypart]
                jac = np.hstack((jac, np.zeros((dim, dofs))))
        return jac
