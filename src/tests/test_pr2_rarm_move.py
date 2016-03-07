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

def lininterp(start, end, timesteps):
    return np.array([np.linspace(s,e,timesteps) for s,e in zip(start,end)])

def test_pr2_rarm_move():
    env = pr2_small_cans_test_env()
    # active_bodyparts = ['rightarm', 'rgripper']
    active_bodyparts = ['rightarm']
    robot = PR2('pr2', active_bodyparts)
    robot.tuck_arms(env)

    hl_plan = TestDomain(env)
    move_env = env.CloneSelf(1) # clones objects in the environment
    robot.init_for_env(env)
    robot.init_for_env(move_env)

    obj = Obj('object7')

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

    K = 7
    start = HLParam("start", K, 1, is_var=False, value=start_pose)
    end = HLParam("end", K, 1, is_var=False, value=end_pose)

    move = PR2Move(0, hl_plan, move_env, robot, start, end, obj)
    hlas = [move]
    ll_prob = LLProb(hlas)
    # ll_prob.solve()
    # import ipdb; ipdb.set_trace()
    ll_prob.solve_at_priority(-1)
    ll_prob.solve_at_priority(1)

if __name__ == "__main__":
    test_pr2_rarm_move()
