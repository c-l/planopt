
from openravepy import KinBody, Environment, RaveCreateKinBody, TriMesh, \
GeometryType
import numpy as np
from test_utils import *
from interface.hl_actions.hl_action import HLAction
from interface.hl_plan import HLPlan
from interface.hl_param import HLParam, Traj, Robot, Obj
from interface.fluents.not_obstructs import NotObstructs

class TestAction(HLAction):
    def __init__(self, hl_plan, env, robot, pos, obj):
        super(TestAction, self).__init__(0, hl_plan, env, robot)

        self.name = "Test" + str(0)
        self.pos = pos

        self.K = 3
        self.T = 1

        self.precondition = NotObstructs(env, self, robot, 1, self.pos, obj)


def not_obstructs_test_env():
    env = Environment()  # create openrave environment
    env.SetViewer('qtcoin')  # attach viewer (optional)
    add_cyl_robot(env)
    add_object(env)

    return env

def test_not_obstructs_btn_two_cylinders():
    env = not_obstructs_test_env()

    hl_plan = HLPlan(env)
    robot = Robot(env.GetRobots()[0].GetName())
    robot_pose = np.array([[0.1],[0.],[0.]])
    robot.set_pose(env, robot_pose)

    # obj has a radius of .35
    obj = Obj('obj')
    pose = np.array([[0.],[0.],[0.]])
    obj.set_pose(env, pose)

    pos = HLParam("pos", (3, 1), is_var=False, value=robot.get_pose(env))
    obj_pos = HLParam("obj_pos", (3, 1), is_var=False, value=obj.get_pose(env))
    action = TestAction(hl_plan, env, robot, pos, obj)
    fluent = action.precondition
    fluent.pre()
    raw_input('')
    poses = [np.array([[0.7],[0.],[0.]]), \
            np.array([[0.65],[0.],[0.]]), \
            np.array([[0.55],[0.],[0.]]), \
            np.array([[0.45],[0.],[0.]])]
    values = [0., -0.05, 0.05, 0.15]
    grads = [np.array([[0., 0., 0.]]),
            np.array([[-1.0 ,0., 0.]]), \
            np.array([[-1.0 ,0., 0.]]), \
            np.array([[-1.0 ,0., 0.]])]
    for i in range(len(poses)):
        val, grad = fluent.collisions(poses[i])
        print 'val: ', val
        print 'grad: ', grad
        assert np.allclose(values[i], val, atol=2e-2)
        assert np.allclose(grads[i], grad, atol=2e-2)

test_not_obstructs_btn_two_cylinders()
