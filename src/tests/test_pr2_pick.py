from openravepy import KinBody, Environment, RaveCreateKinBody, TriMesh, \
GeometryType, poseFromMatrix, axisAngleFromRotationMatrix
import numpy as np
from test_utils import *
from interface.ll_prob import LLProb
from interface.hl_actions.hl_action import HLAction
from interface.hl_plan import HLPlan
from interface.hl_param import HLParam, Traj
import time

import sys
DOMAIN_PATH = "../domains/PR2PickPlace/"
sys.path.insert(0, DOMAIN_PATH)
from fluents.not_obstructs import NotObstructsPR2
from fluents.is_gp import PR2IsGP
from actions.pick import PR2Pick
from param import Obj, PR2

class TestDomain(object):
    def __init__(self, env):
        self.env = env

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
    # tran[1,3] = -.51415
    tran[1,3] = -.34
    obj17.SetTransform(tran)

    # robot = PR2('pr2')
    # active_body_parts = ['rightarm', 'rgripper', 'torso', 'base']

    active_bodyparts = ['rightarm', 'rgripper']
    robot = PR2('pr2', active_bodyparts)
    robot.tuck_arms(env)

    hl_plan = TestDomain(env)
    pick_env = env.CloneSelf(1) # clones objects in the environment
    robot.init_for_env(env)
    robot.init_for_env(pick_env)

    pose = robot.get_pose(env)
    robot_body = robot.get_env_body(env)

    hl_plan = HLPlan(env)
    obj = Obj('object17')
    K = robot.dofs
    # obj_pose = np.array([[0.],[0.],[0.]])
    # obj.set_pose(env, obj_pose)

    # start = HLParam("pos", self.K, 1, is_var=False, value=robot.get_pose(env))
    pose = np.array([[-1.57265580e+00], [5.27875957e-01], [-1.40000000e+00], \
        [-1.58244363e+00], [-1.64143184e+00], [-9.76938766e-02], [-5.78403045e-01], [0.0]])
    # pose = np.array([[-1.57265580e+00], [5.27875957e-01], [-1.40000000e+00], \
    #     [-1.58244363e+00], [-1.64143184e+00], [-9.76938766e-02], [-5.78403045e-01]])
    pos = HLParam("pos", K, 1, value=pose)
    # end = HLParam("end", K, 1, is_var=True, value=end_pose)
    # obj_pos = HLParam("obj_pos", 3, 1, is_var=False, value=obj.get_pose(env))

    pick = PR2Pick(0, hl_plan, pick_env, robot, pos, obj)
    # pick.preconditions[-1].error(pose)
    hlas = [pick]
    ll_prob = LLProb(hlas)
    ll_prob.solve()

def test_pick_full_DOF():
    env = cans_world_env()
    obj17 = env.GetKinBody('object17')
    tran = obj17.GetTransform()
    tran[0,3] = .49268
    # tran[1,3] = -.51415
    tran[1,3] = -.34
    obj17.SetTransform(tran)

    # robot = PR2('pr2')
    # active_body_parts = ['rightarm', 'rgripper', 'torso', 'base']

    # active_bodyparts = ['rightarm', 'rgripper', 'base']
    active_bodyparts = ['rightarm', 'rgripper']
    robot = PR2('pr2', active_bodyparts)
    robot.tuck_arms(env)

    hl_plan = TestDomain(env)
    pick_env = env.CloneSelf(1) # clones objects in the environment
    robot.init_for_env(env)
    robot.init_for_env(pick_env)

    pose = robot.get_pose(env)
    robot_body = robot.get_env_body(env)

    hl_plan = HLPlan(env)
    obj = Obj('object17')
    K = robot.dofs
    # obj_pose = np.array([[0.],[0.],[0.]])
    # obj.set_pose(env, obj_pose)

    # start = HLParam("pos", self.K, 1, is_var=False, value=robot.get_pose(env))
    # pose = np.array([[-1.57265580e+00], [5.27875957e-01], [-1.40000000e+00], \
    #     [-1.58244363e+00], [-1.64143184e+00], [-9.76938766e-02], [-5.78403045e-01], [0.0],[0.],[0.],[0.]])
    # pose = np.array([[-1.57265580e+00], [5.27875957e-01], [-1.40000000e+00], \
    #     [-1.58244363e+00], [-1.64143184e+00], [-9.76938766e-02], [-5.78403045e-01]])
    pose = np.array([[-1.57265580e+00], [5.27875957e-01], [-1.40000000e+00], \
        [-1.58244363e+00], [-1.64143184e+00], [-9.76938766e-02], [-5.78403045e-01], [0.]])
    start = HLParam("start", K, 1, value=pose, is_var=False)
    # start = HLParam("start", K, 1, value=pose, is_var=True)
    end = HLParam("end", K, 1, value=pose)
    # end = HLParam("end", K, 1, is_var=True, value=end_pose)
    # obj_pos = HLParam("obj_pos", 3, 1, is_var=False, value=obj.get_pose(env))

    pick = PR2Pick(0, hl_plan, pick_env, robot, start, end, obj)
    # pick.preconditions[-1].error(pose)
    hlas = [pick]
    ll_prob = LLProb(hlas)
    ll_prob.solve()

if __name__ == "__main__":
    # test_pick_pr2_gradient()
    # test_pick()
    test_pick_full_DOF()
