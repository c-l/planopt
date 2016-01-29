import numpy as np
from utils import *
from numpy import random
import openravepy


class HLParam(object):

    def __init__(self, name, rows, cols, is_var=True, value=None, index=None, is_resampled=False):
        self.name = name
        self.rows = rows
        self.cols = cols

        self.is_var = is_var
        if value is None:
            self.value = np.zeros((rows, cols))
        else:
            self.value = value
        self.index = index
        self.is_resampled = is_resampled
        self.gen = None

    def init(self):
        pass

    def get_value(self):
        return self.value

    def set(self, value):
        self.value = value

    def resample(self):
        if self.gen is None:
            self.gen = self.generator()
        self.value = next(self.gen)

    def generator(self):
        raise NotImplementedError

    def reset_gen(self):
        self.gen = self.generator()


class Grasp(HLParam):

    def generator(self):
        yield np.array([[0], [0.6], [0]], dtype=np.float)
        yield np.array([[0], [-0.6], [0]], dtype=np.float)
        # yield np.array([[0.6], [0], [0]], dtype=np.float)
        # yield np.array([[-0.6], [0], [0]], dtype=np.float)


class RP(HLParam):
    pass


class ObjLoc(HLParam):
    # object location
    # random.seed([1])
    # random.seed([2]) # difficult one
    random.seed([3])
    # random.seed([4])
    # random.seed([5])
    # random.seed([6])
    # random.seed([7])

    def __init__(self, name, rows, cols, is_var=True,
                 value=None, ro=None, index=None, region=None):
        super(ObjLoc, self).__init__(
            name, rows, cols, is_var, value, ro, index)
        self.region = region
        if region is not None:
            self.in_region = True
        else:
            self.in_region = False
        if self.in_region:
            assert is_var is True

            self.min_x = region[0, 0]
            self.max_x = region[0, 1]
            self.min_y = region[1, 0]
            self.max_y = region[1, 1]
            self.min_z = region[2, 0]
            self.max_z = region[2, 1]

    def generator(self):
        while True:
            x = random.random() * (self.max_x - self.min_x) + self.min_x
            y = random.random() * (self.max_y - self.min_y) + self.min_y
            yield np.array([[x], [y], [0]])
        # yield np.array([[3.5],[4.3],[0]])


class Obj(HLParam):

    def __init__(self, name):
        self.name = name
        self.is_var = False
        self.is_resampled = False

    def get_pose(self, env):
        transform = self.get_env_body(env).GetTransform()
        return mat_to_base_pose(transform)

    def set_pose(self, env, t):
        trans = base_pose_to_mat(t)
        self.get_env_body(env).SetTransform(trans)

    def get_env_body(self, env):
        return env.GetKinBody(self.name)

class Robot(Obj):
    pass

class PR2(Robot):
    def __init__(self, name):
        super(PR2, self).__init__(name)

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
        active_dofs = np.ndarray(0)
        for active_bodypart in sorted(active_bodyparts):
            if active_bodypart in {'rightarm', 'leftarm'}:
                active_dofs = np.r_[active_dofs, robot.GetManipulator(active_bodypart).GetArmIndices()]
            elif active_bodypart == 'torso':
                active_dofs = np.r_[active_dofs, robot.GetJoint("torso_lift_joint").GetDOFIndex()]
            elif '~door:' in active_bodypart:
                active_dofs = np.r_[active_dofs, robot.GetJoint('hinge').GetDOFIndex()]

        if 'base' in active_bodyparts:
            robot.SetActiveDOFs(
                active_dofs,
                openravepy.DOFAffine.X + openravepy.DOFAffine.Y + openravepy.DOFAffine.RotationAxis,
                [0, 0, 1])
        else:
            robot.SetActiveDOFs(active_dofs)

class Traj(HLParam):
    def __init__(self, hl_action, name, rows, cols, is_var=True, value=None, index=None):
        super(Traj, self).__init__(name, rows, cols, is_var, value, index)
        self.hl_action = hl_action

    # # TODO: make this less hacky
    # def resample(self):
    #     self.value = straight_line()
