from openravepy import KinBody, Environment, RaveCreateKinBody, TriMesh, \
GeometryType, poseFromMatrix, matrixFromPose
import numpy as np
from test_utils import *
from interface.ll_prob import LLProb
from interface.hl_actions.hl_action import HLAction
from interface.hl_plan import HLPlan
from interface.hl_param import HLParam, Traj, Robot


import sys
DOMAIN_PATH = "../domains/PR2PickPlace/"
sys.path.insert(0, DOMAIN_PATH)

from fluents.in_manip import PR2InManip
from fluents.obj_in_manip import ObjInManip
from actions.action import PR2HLAction
from actions.move import PR2Move
from actions.move_with_obj import PR2MoveWithObj
from actions.obj_move import PR2ObjMove
from param import Obj, PR2
import ipdb

from utils import obj_pose_from_transform

class TestDomain(object):
    def __init__(self, env):
        self.env = env

class TestAction(HLAction):
    def __init__(self, hl_plan, env, robot, pos, obj):
        super(TestAction, self).__init__(0, hl_plan, env, robot)

        self.name = "Test" + str(0)
        self.pos = pos

        self.K = 11
        self.T = 1

        self.precondition = NotObstructsPR2(env, self, robot, 1, self.pos, obj)

class DummyInManipAction():
    T = 1
    K = 7
    obj_K = 6

    def plot(self):
        pass

def test_not_obstructs_pr2():
    env = cans_world_env()
    robot = env.GetRobots()[0]
    import ctrajoptpy
    cc = ctrajoptpy.GetCollisionChecker(env)
    collisions = cc.BodyVsAll(robot)
    import ipdb; ipdb.set_trace()
    handles = []
    for c in collisions:
        linkAParent = c.GetLinkAParentName()
        linkBParent = c.GetLinkBParentName()
        print linkAParent
        print linkBParent
        linkA = c.GetLinkAName()
        linkB = c.GetLinkBName()
        print linkA
        print linkB

        if linkAParent == robot.GetName():
            ptRobot = c.GetPtA()
            ptObj = c.GetPtB()
        elif linkBParent == robot.GetName():
            ptRobot = c.GetPtB()
            ptObj = c.GetPtA()
        else:
            continue

        distance = c.GetDistance()
        print distance
        handles += plot_collision(env, ptRobot, ptObj, distance)
    import ipdb; ipdb.set_trace()

def test_pr2():
    env = cans_world_env()
    robot = PR2('pr2')
    robot.tuck_arms(env)
    robot._set_active_dofs(env, ['rightarm', 'torso', 'base'])
    robot.set_pose(env, np.array([-0.023593, 1.10728, -1.5566882, -2.124408, -1.4175, \
        -1.8417,  0.21436,  0.,  0.2,  0., -0.]))
    import ipdb; ipdb.set_trace()
    print "end of test_pr2"

def test_not_obstructs_pr2_table():
    env = cans_world_env()
    robot = PR2('pr2')
    robot.tuck_arms(env)
    robot._set_active_dofs(env, ['rightarm', 'torso', 'base'])
    # pose = np.array([-0.023593, 1.10728, -1.5566882, -2.124408, -1.4175, \
    #     -1.8417,  0.21436,  0.,  0.2,  0., -0.])
    # robot.set_pose(env, pose)
    pose = robot.get_pose(env)

    hl_plan = HLPlan(env)
    obj = Obj('object9')
    # obj_pose = np.array([[0.],[0.],[0.]])
    # obj.set_pose(env, obj_pose)

    pos = HLParam("pos", 11, 1, is_var=False, value=robot.get_pose(env))
    obj_pos = HLParam("obj_pos", 3, 1, is_var=False, value=obj.get_pose(env))
    action = TestAction(hl_plan, env, robot, pos, obj)
    fluent = action.precondition
    fluent.pre()

    val, grad = fluent.collisions(pose)

def test_move_pr2_gradient():
    env = cans_world_env()
    obj17 = env.GetKinBody('object17')
    tran = obj17.GetTransform()
    tran[0,3] = .49268
    tran[1,3] = -.51415
    obj17.SetTransform(tran)

    hl_plan = TestDomain(env)
    move_env = env.CloneSelf(1) # clones objects in the environment
    move_pr2 = move_env.GetKinBody('pr2')
    move_pr2.SetActiveDOFs(move_pr2.GetManipulator('rightarm').GetArmIndices())
    move_pr2.SetDOFValues([0.3],
                    [move_pr2.GetJoint("torso_lift_joint").GetDOFIndex()])

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
    start_pose = np.array([[-1.57265580e+00], [5.27875957e-01], [-1.40000000e+00], \
        [-1.58244363e+00], [-1.64143184e+00], [-9.76938766e-02], [-5.78403045e-01]])
    end_pose = np.array([[-1.31066711e+00], [1.08996543e+00], [-1.40000000e+00],\
        [-2.10243169e+00], [2.93131497e+00], [-6.18669215e-01], [5.20253411e-01]])
    start = HLParam("start", K, 1, is_var=False, value=start_pose)
    end = HLParam("end", K, 1, is_var=False, value=end_pose)
    obj_pos = HLParam("obj_pos", 3, 1, is_var=False, value=obj.get_pose(env))

    move = PR2Move(0, hl_plan, move_env, robot, start, end, obj)
    hlas = [move]
    ll_prob = LLProb(hlas)
    ll_prob.solve()
    import ipdb; ipdb.set_trace()

def test_move_pr2_with_obj_wrt_robot():
    env = cans_world_env()
    obj17 = env.GetKinBody('object17')
    tran = obj17.GetTransform()
    tran[0,3] = .49268
    tran[1,3] = -.51415
    obj17.SetTransform(tran)

    hl_plan = TestDomain(env)
    move_env = env.CloneSelf(1) # clones objects in the environment

    active_bodyparts = ['rightarm']
    robot = PR2('pr2', active_bodyparts)
    robot.tuck_arms(env)
    robot.init_for_env(env)
    robot.init_for_env(move_env)
    pose = robot.get_pose(env)

    hl_plan = HLPlan(env)
    obj = Obj('object17')
    K = 7
    # obj_pose = np.array([[0.],[0.],[0.]])
    # obj.set_pose(env, obj_pose)

    # start = HLParam("pos", self.K, 1, is_var=False, value=robot.get_pose(env))
    start_pose = np.array([[-1.4352011 ],\
       [ 0.63522797],\
       [-1.19804084],\
       [-1.55807855],\
       [-0.9962505 ],\
       [-0.75279673],\
       [-1.13988334]])
    end_pose = np.array([[-1.28868338],\
       [ 0.53921863],\
       [-1.41293145],\
       [-1.67185359],\
       [-0.74117023],\
       [-0.73603889],\
       [-1.29004773]])
    robot.set_pose(env, start_pose)

    # getting grasp pose
    robot_body = robot.get_env_body(env)
    rarm = robot_body.GetManipulator('rightarm')
    # import ipdb; ipdb.set_trace()
    # W_T_EE = rarm.GetEndEffectorTransform()
    W_T_EE = robot_body.GetLink("r_gripper_tool_frame").GetTransform()
    EE_T_W = np.linalg.inv(W_T_EE)
    W_T_O = obj17.GetTransform()
    EE_T_O = np.dot(EE_T_W, W_T_O)
    EE_T_O_pose = poseFromMatrix(EE_T_O)
    import ipdb; ipdb.set_trace()

    gp_val = np.array([0,0,.125]).reshape((3,1))
    gp = HLParam("gp", 3, 1, is_var=False, value=gp_val)

    # start_pose = np.array([[-1.07265580e+00], [5.17875957e-01], [-1.30000000e+00], \
    #     [-1.08244363e+00], [-1.04143184e+00], [-9.06938766e-02], [-5.38403045e-01]])
    # obj trajectory to follow
    obj_start = poseFromMatrix(tran).reshape((7,1))
    end_tran = tran.copy()
    end_tran[1, 3] = -0.34
    obj_end = poseFromMatrix(end_tran).reshape((7,1))
    # print "obj_start:", obj_start
    # print "obj_end:", obj_end
    obj_traj = lininterp(obj_start, obj_end, 10)

    start = HLParam("start", K, 1, is_var=False, value=start_pose)
    end = HLParam("end", K, 1, is_var=True, value=end_pose)
    # obj_pos = HLParam("obj_pos", 3, 1, is_var=False, value=obj.get_pose(env))

    move = PR2MoveWithObj(0, hl_plan, move_env, robot, start, end, obj, obj_traj, gp)
    hlas = [move]
    ll_prob = LLProb(hlas)
    ll_prob.solve()
    import ipdb; ipdb.set_trace()

def test_obj_pose():
    from transformations import rotation_matrix, concatenate_matrices, euler_from_matrix
    from utils import _axis_rot_matrices, _ypr_from_rot_matrix
    alpha, beta, gamma = 0.123, -1.234, 2.345
    origin, xaxis, yaxis, zaxis = (0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)
    # I = identity_matrix()
    Rx_tf = rotation_matrix(gamma, xaxis)
    Ry_tf = rotation_matrix(beta, yaxis)
    Rz_tf = rotation_matrix(alpha, zaxis)

    obj = Obj("obj")
    Rz, Ry, Rx = _axis_rot_matrices([alpha, beta, gamma, 0., 0., 0.])
    assert np.allclose(Rx_tf[:3,:3], Rx)
    assert np.allclose(Ry_tf[:3,:3], Ry)
    assert np.allclose(Rz_tf[:3,:3], Rz)
    R = concatenate_matrices(Rz_tf, Ry_tf, Rx_tf)
    rot_mat = np.dot(Rz, np.dot(Ry, Rx))
    euler = euler_from_matrix(R, 'sxyz')
    assert np.allclose(R[:3,:3], rot_mat)
    assert np.allclose([gamma, beta, alpha], euler)
    assert np.allclose([alpha, beta, gamma], _ypr_from_rot_matrix(R[:3,:3]))

def test_err_obj_in_manip():
    import numdifftools

    env = cans_world_env()
    obj17 = env.GetKinBody('object17')
    tran = obj17.GetTransform()
    tran[0,3] = .49268
    tran[1,3] = -.51415
    obj17.SetTransform(tran)
    obj_K = 6
    obj_traj_val = obj_pose_from_transform(tran).reshape((obj_K,1))

    robot = PR2('pr2')
    robot.tuck_arms(env)
    robot._set_active_dofs(env, ['rightarm'])
    pose = robot.get_pose(env)

    obj = Obj('object17')

    gp_val = np.array([0,0,.125]).reshape((3,1))
    gp = HLParam("gp", 3, 1, is_var=False, value=gp_val)

    # traj_val = np.array([[-1.4352011 ],\
    #    [ 0.63522797],\
    #    [-1.19804084],\
    #    [-1.55807855],\
    #    [-0.9962505 ],\
    #    [-0.75279673],\
    #    [-1.13988334]])

    traj_val = np.array([[-1.07265580e+00], [5.17875957e-01], [-1.30000000e+00], \
        [-1.08244363e+00], [-1.04143184e+00], [-9.06938766e-02], [-5.38403045e-01]])

    dummy_action = DummyInManipAction()
    traj = Traj(dummy_action, "traj", 7, 1, is_var=True, value=traj_val)
    obj_traj = Traj(dummy_action, "obj_traj", obj_K, 1, is_var=False, value=obj_traj_val)

    fluent = ObjInManip(env, dummy_action, robot, 0, obj, gp, traj, obj_traj)

    # val, jac = fluent.pos_error(obj_traj_val.flatten())
    # jac_fn = numdifftools.Jacobian(lambda x: fluent.pos_error(x)[0])
    # num_jac = jac_fn(obj_traj_val.flatten())[0]
    # print "analytic pos jac:\t{}\tnumerical pos jac:\t{}\tthe same?:\t{}".format(jac, num_jac, np.allclose(jac, num_jac))

    val, jac = fluent.rot_error(obj_traj_val.flatten())

    jac_fn = numdifftools.Jacobian(lambda x: fluent.rot_error(x)[0])
    num_jac = jac_fn(obj_traj_val.flatten())[0]
    print "analytic rot jac:\t{}\tnumerical rot jac:\t{}\tthe same?:\t{}".format(jac, num_jac, np.allclose(jac, num_jac))

def test_err_in_manip():
    import numdifftools

    env = cans_world_env()
    obj17 = env.GetKinBody('object17')
    tran = obj17.GetTransform()
    tran[0,3] = .49268
    tran[1,3] = -.51415
    obj17.SetTransform(tran)
    obj_K = 6
    obj_traj = pose_from_transform(tran).reshape((obj_K,1))

    robot = PR2('pr2')
    robot.tuck_arms(env)
    robot._set_active_dofs(env, ['rightarm'])
    pose = robot.get_pose(env)

    obj = Obj('object17')

    gp_val = np.array([0,0,.125]).reshape((3,1))
    gp = HLParam("gp", 3, 1, is_var=False, value=gp_val)

    # traj_val = np.array([[-1.4352011 ],\
    #    [ 0.63522797],\
    #    [-1.19804084],\
    #    [-1.55807855],\
    #    [-0.9962505 ],\
    #    [-0.75279673],\
    #    [-1.13988334]])

    traj_val = np.array([[-1.07265580e+00], [5.17875957e-01], [-1.30000000e+00], \
        [-1.08244363e+00], [-1.04143184e+00], [-9.06938766e-02], [-5.38403045e-01]])

    dummy_action = DummyInManipAction()
    traj = Traj(dummy_action, "traj", 7, 1, is_var=True, value=traj_val)
    obj_traj = Traj(dummy_action, "obj_traj", obj_K, 1, is_var=False, value=obj_traj)

    fluent = PR2InManip(env, dummy_action, robot, 0, obj, gp, traj, obj_traj)
    val, jac = fluent.pos_error(traj_val)
    pos_error_fn = lambda x: fluent.pos_error(x)[0]
    jac_fn = numdifftools.Jacobian(pos_error_fn)
    num_jac = jac_fn(traj_val.flatten())[0]
    print "analytic pos jac:\t{}\tnumerical pos jac:\t{}\tthe same?:\t{}".format(jac, num_jac, np.allclose(jac, num_jac))

    val, jac = fluent.rot_error(traj_val)
    rot_error_fn = lambda x: fluent.rot_error(x)[0]
    jac_fn = numdifftools.Jacobian(rot_error_fn)
    num_jac = jac_fn(traj_val.flatten())[0]
    print "analytic rot jac:\t{}\tnumerical rot jac:\t{}\tthe same?:\t{}".format(jac, num_jac, np.allclose(jac, num_jac))


def test_move_pr2_with_obj_wrt_obj():
    env = cans_world_env()
    obj17 = env.GetKinBody('object17')
    tran = obj17.GetTransform()
    tran[0,3] = .49268
    tran[1,3] = -.51415
    obj17.SetTransform(tran)

    hl_plan = TestDomain(env)
    move_env = env.CloneSelf(1) # clones objects in the environment
    move_pr2 = move_env.GetKinBody('pr2')
    move_pr2.SetActiveDOFs(move_pr2.GetManipulator('rightarm').GetArmIndices())
    move_pr2.SetDOFValues([0.3],
                    [move_pr2.GetJoint("torso_lift_joint").GetDOFIndex()])

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
    start_pose = np.array([[-1.4352011 ],\
       [ 0.63522797],\
       [-1.19804084],\
       [-1.55807855],\
       [-0.9962505 ],\
       [-0.75279673],\
       [-1.13988334]])
    end_pose = np.array([[-1.28868338],\
       [ 0.53921863],\
       [-1.41293145],\
       [-1.67185359],\
       [-0.74117023],\
       [-0.73603889],\
       [-1.29004773]])
    robot.set_pose(env, start_pose)
    # traj_val = lininterp(start_pose, end_pose, 10)
    traj_val = lininterp(start_pose, end_pose, 2)

    # getting grasp pose
    robot_body = robot.get_env_body(env)
    rarm = robot_body.GetManipulator('rightarm')
    # import ipdb; ipdb.set_trace()
    W_T_EE = rarm.GetEndEffectorTransform()
    EE_T_W = np.linalg.inv(W_T_EE)
    W_T_O = obj17.GetTransform()
    EE_T_O = np.dot(EE_T_W, W_T_O)
    EE_T_O_pose = poseFromMatrix(EE_T_O)
    # gp = HLParam("gp", 7, 1, is_var=False, value=EE_T_O_pose)
    gp_val = np.array([0,0,.125]).reshape((3,1))
    gp = HLParam("gp", 3, 1, is_var=False, value=gp_val)

    # start_pose = np.array([[-1.07265580e+00], [5.17875957e-01], [-1.30000000e+00], \
    #     [-1.08244363e+00], [-1.04143184e+00], [-9.06938766e-02], [-5.38403045e-01]])
    # obj trajectory to follow
    # obj_start = poseFromMatrix(tran).reshape((7,1))
    obj_start = obj_pose_from_transform(tran).reshape((6,1))
    end_tran = tran.copy()
    end_tran[1, 3] = -0.34
    obj_end = obj_pose_from_transform(end_tran).reshape((6,1))
    # print "obj_start:", obj_start
    # print "obj_end:", obj_end
    # obj_traj = lininterp(obj_start, obj_end, 10)
    obj_traj = lininterp(obj_start, obj_end, 2)

    start = HLParam("start", K, 1, is_var=False, value=start_pose)
    end = HLParam("end", K, 1, is_var=False, value=end_pose)
    # obj_pos = HLParam("obj_pos", 3, 1, is_var=False, value=obj.get_pose(env))

    move = PR2ObjMove(0, hl_plan, move_env, robot, start, end, traj_val, obj, gp)
    hlas = [move]
    ll_prob = LLProb(hlas)
    ll_prob.solve()
    import ipdb; ipdb.set_trace()

def lininterp(start, end, timesteps):
    return np.array([np.linspace(s,e,timesteps) for s,e in zip(start,end)])

def test_robot():
    env = pick_test_env()
    import ipdb; ipdb.set_trace()
    print ""

def plot_collision(env, ptA, ptB, distance):
    handles = []
    if not np.allclose(ptA, ptB, atol=1e-3):
        if distance < 0:
            handles.append(env.drawarrow(p1=ptA, p2=ptB, linewidth=.002,color=(1,0,0)))
        else:
            handles.append(env.drawarrow(p1=ptA, p2=ptB, linewidth=.002,color=(0,0,0)))
    return handles

if __name__ == "__main__":
    # test_not_obstructs_pr2()
    # test_pr2()
    # test_not_obstructs_pr2_table()
    # test_move_pr2_gradient()
    test_move_pr2_with_obj_wrt_robot()
    # test_err_in_manip()
    # test_err_obj_in_manip()
    # test_obj_pose()
    # test_move_pr2_with_obj_wrt_obj()
    # test_robot()
