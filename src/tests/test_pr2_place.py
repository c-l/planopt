
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
from actions.place import PR2Place
from param import Obj, PR2, ObjLoc

def test_place():
    env = pr2_small_cans_test_env()

    # robot = PR2('pr2')
    # active_body_parts = ['rightarm', 'rgripper', 'torso', 'base']

    # active_bodyparts = ['rightarm', 'rgripper', 'base']
    active_bodyparts = ['rightarm', 'rgripper']
    robot = PR2('pr2', active_bodyparts)
    robot.tuck_arms(env)

    hl_plan = TestDomain(env)
    place_env = env.CloneSelf(1) # clones objects in the environment
    robot.init_for_env(env)
    robot.init_for_env(place_env)

    pose = robot.get_pose(env)
    robot_body = robot.get_env_body(env)

    hl_plan = HLPlan(env)
    obj = Obj('object7')
    K = robot.dofs
    # obj_pose = np.array([[0.],[0.],[0.]])
    # obj.set_pose(env, obj_pose)

    # start = HLParam("pos", self.K, 1, is_var=False, value=robot.get_pose(env))
    # pose = np.array([[-1.57265580e+00], [5.27875957e-01], [-1.40000000e+00], \
    #     [-1.58244363e+00], [-1.64143184e+00], [-9.76938766e-02], [-5.78403045e-01], [0.0],[0.],[0.],[0.]])
    # pose = np.array([[-1.57265580e+00], [5.27875957e-01], [-1.40000000e+00], \
    #     [-1.58244363e+00], [-1.64143184e+00], [-9.76938766e-02], [-5.78403045e-01]])
    pose = np.array([[-1.57265580e+00], [5.27875957e-01], [-1.40000000e+00], \
        [-1.58244363e+00], [-1.64143184e+00], [-9.76938766e-02], [-5.78403045e-01], [0.4]])
    start = HLParam("start", K, 1, value=pose, is_var=False)

    end = HLParam("end", K, 1, value=pose)
    loc = HLParam("loc", 3, 1, is_var=False, value=obj.get_pose(env)[3:6].reshape((3,1)))
    # import ipdb; ipdb.set_trace()

    place = PR2Place(0, hl_plan, place_env, robot, start, end, obj, loc)
    hlas = [place]
    ll_prob = LLProb(hlas)
    # ll_prob.solve()
    import ipdb; ipdb.set_trace()
    ll_prob.solve_at_priority(-1)
    ll_prob.solve_at_priority(1)

if __name__ == "__main__":
    test_place()
