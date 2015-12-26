import numpy as np
from interface.hl_actions.move import Move
from interface.hl_actions.pick import Pick
from interface.hl_actions.place import Place
from interface.ll_prob import LLProb
from interface.hl_param import HLParam, Traj
from interface.hl_plan import HLPlan
from openravepy import *
from utils import mat_to_base_pose
from test_utils import *
import ipdb


class GP(HLParam):

    def generator(self):
        yield np.array([[0], [0.55], [0]], dtype=np.float)
        yield np.array([[0], [-0.55], [0]], dtype=np.float)
        yield np.array([[0.55], [0], [0]], dtype=np.float)
        yield np.array([[-0.55], [0], [0]], dtype=np.float)

def test_move():
    env = move_test_env()
    robot = env.GetRobots()[0]

    hl_plan = HLPlan(env, robot)
    move_env = env.CloneSelf(1) # clones objects in the environment
    move_robot = move_env.GetRobots()[0]

    start = HLParam("start", 3, 1, is_var=False, value=np.array([[-2], [0], [0]]))
    end = HLParam("end", 3, 1, is_var=False, value=np.array([[2], [0], [0]]))

    move = Move(0, hl_plan, move_env, move_robot, start, end)

    hlas = [move]
    ll_prob = LLProb(hlas)
    ll_prob.solve()


def test_pick():
    env = pick_test_env()
    robot = env.GetRobots()[0]

    # start = HLParam("start", 3, 1, is_var=False, value=np.array([[-2], [0], [0]]))
    # end = HLParam("end", 3, 1, is_var=False, value=np.array([[2], [0], [0]]))

    hl_plan = HLPlan(env, robot)
    pick_env = env.CloneSelf(1) # clones objects in the environment
    pick_robot = pick_env.GetRobots()[0]

    rp = HLParam("rp", 3, 1)
    gp = GP("gp", 3, 1, is_resampled=True)
    pick_obj = pick_env.GetKinBody('obj')
    obj_loc = HLParam("obj_loc", 3, 1, is_var=False, value=mat_to_base_pose(pick_obj.GetTransform()))

    gp.resample()

    import ipdb; ipdb.set_trace()
    pick = Pick(0, hl_plan, pick_env, pick_robot, rp, pick_obj, obj_loc, gp)
    hlas = [pick]

    ll_prob = LLProb(hlas)
    ll_prob.solve()
    assert np.allclose(pick.pos.value, np.array([[-1.41840404],[-0.18333333],[ 0.        ]]))


def test_pick_and_move():
    env = pick_test_env()
    robot = env.GetRobots()[0]

    # start = HLParam("start", 3, 1, is_var=False, value=np.array([[-2], [0], [0]]))
    # end = HLParam("end", 3, 1, is_var=False, value=np.array([[2], [0], [0]]))

    hl_plan = HLPlan(env, robot)
    pick_env = env.CloneSelf(1) # clones objects in the environment
    pick_robot = pick_env.GetRobots()[0]
    move_env = env.CloneSelf(1) # clones objects in the environment
    move_robot = move_env.GetRobots()[0]

    rp = HLParam("rp", 3, 1)
    end = HLParam("end", 3, 1, is_var=False, value=np.array([[2],[0],[0]]))
    gp = GP("gp", 3, 1, is_resampled=True)
    pick_obj = pick_env.GetKinBody('obj')
    move_obj = move_env.GetKinBody('obj')
    obj_loc = HLParam("obj_loc", 3, 1, is_var=False, value=mat_to_base_pose(pick_obj.GetTransform()))

    gp.resample()

    pick = Pick(0, hl_plan, pick_env, pick_robot, rp, pick_obj, obj_loc, gp)
    move = Move(0, hl_plan, move_env, move_robot, rp, end, move_obj, gp)
    hlas = [pick, move]

    ll_prob = LLProb(hlas)
    ll_prob.solve()

    ipdb.set_trace()


def test_pick_move_and_place():
    env = pick_test_env()
    robot = env.GetRobots()[0]

    hl_plan = HLPlan(env, robot)
    place_env = env.CloneSelf(1) # clones objects in the environment
    place_robot = place_env.GetRobots()[0]
    pick_env = env.CloneSelf(1) # clones objects in the environment
    pick_robot = pick_env.GetRobots()[0]
    move_env = env.CloneSelf(1) # clones objects in the environment
    move_robot = move_env.GetRobots()[0]

    rp1 = HLParam("rp1", 3, 1)
    rp2 = HLParam("rp2", 3, 1)
    gp = GP("gp", 3, 1, is_resampled=True)
    place_obj = place_env.GetKinBody('obj')
    pick_obj = pick_env.GetKinBody('obj')
    move_obj = move_env.GetKinBody('obj')
    obj_loc = HLParam("obj_loc", 3, 1, is_var=False, value=mat_to_base_pose(pick_obj.GetTransform()))
    target_loc = HLParam("target_loc", 3, 1, is_var=False, value=np.array([[2],[0.5],[0]]))

    gp.resample()

    pick = Pick(0, hl_plan, pick_env, pick_robot, rp1, pick_obj, obj_loc, gp)
    move = Move(0, hl_plan, move_env, move_robot, rp1, rp2, move_obj, gp)
    import ipdb; ipdb.set_trace()
    place = Place(0, hl_plan, place_env, place_robot, rp2, place_obj, target_loc, gp)
    hlas = [pick, move, place]

    ll_prob = LLProb(hlas)
    ll_prob.solve()

# test_move()
# test_pick()
# test_pick_and_move()
test_pick_move_and_place()
