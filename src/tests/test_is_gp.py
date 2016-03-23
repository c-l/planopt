
from openravepy import KinBody, Environment, RaveCreateKinBody, TriMesh, \
GeometryType
import numpy as np
from test_utils import *
from interface.hl_actions.hl_action import HLAction
from interface.hl_plan import HLPlan
from interface.hl_param import HLParam, Traj, Robot, Obj
from interface.fluents.is_gp import IsGP

class TestIsGPAction(HLAction):
    def __init__(self, hl_plan, env, robot, obj, gp, pos, obj_pos):
        super(TestIsGPAction, self).__init__(0, hl_plan, env, robot)

        self.name = "Test" + str(0)
        self.obj = obj
        self.gp = gp
        self.pos = pos
        self.obj_pos = obj_pos

        self.K = 3
        self.T = 1

        self.precondition = IsGP(env, self, robot, 1, self.obj, self.gp, self.pos, self.obj_pos)


def is_gp_test_env():
    env = Environment()  # create openrave environment
    env.SetViewer('qtcoin')  # attach viewer (optional)
    add_cyl_robot(env)
    add_object(env)

    return env

def test_is_gp():
    env = is_gp_test_env()

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
    gp = HLParam("gp", (3, 1), is_var=False, value=pose)
    action = TestIsGPAction(hl_plan, env, robot, obj, gp, pos, obj_pos)
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
        val, grad = fluent.distance_from_obj(poses[i], 0.05, (3,1))
        print 'val: ', val
        print 'grad: ', grad
        import ipdb; ipdb.set_trace()

        # assert np.allclose(values[i], val, atol=2e-2)
        # assert np.allclose(grads[i], grad, atol=2e-2)

test_is_gp()
