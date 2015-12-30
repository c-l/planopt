import numpy as np
from interface.hl_actions.move import Move
from interface.hl_actions.pick import Pick
from interface.hl_actions.place import Place
from interface.ll_prob import LLProb
from interface.hl_param import HLParam, Traj, Obj, Robot
from utils import mat_to_base_pose
from test_utils import *
import ipdb


class GP(HLParam):

    def generator(self):
        yield np.array([[0], [0.55], [0]], dtype=np.float)
        yield np.array([[0], [-0.55], [0]], dtype=np.float)
        yield np.array([[0.55], [0], [0]], dtype=np.float)
        yield np.array([[-0.55], [0], [0]], dtype=np.float)

class HLPlan(object):
    def __init__(self, env):
        self.env = env

def test_move():
    env = move_test_env()

    hl_plan = HLPlan(env)
    move_env = env.CloneSelf(1) # clones objects in the environment

    robot = Robot(env.GetRobots()[0].GetName())
    start = HLParam("start", 3, 1, is_var=False, value=np.array([[-2], [0], [0]]))
    end = HLParam("end", 3, 1, is_var=False, value=np.array([[2], [0], [0]]))

    move = Move(0, hl_plan, move_env, robot, start, end)

    hlas = [move]
    ll_prob = LLProb(hlas)
    ll_prob.solve()


def test_pick():
    env = pick_test_env()

    hl_plan = HLPlan(env)
    pick_env = env.CloneSelf(1) # clones objects in the environment

    robot = Robot(env.GetRobots()[0].GetName())
    rp = HLParam("rp", 3, 1)

    # collision checking returns -1 * normal when objects are touching
    gp_val =  np.array([[0], [0.56], [0]], dtype=np.float)
    # gp_val =  np.array([[0], [0.55], [0]], dtype=np.float)
    # gp_val =  np.array([[0], [0.54], [0]], dtype=np.float)
    # gp_val =  np.array([[0], [0.552], [0]], dtype=np.float)
    gp = GP("gp", 3, 1, value=gp_val, is_resampled=True)
    obj = Obj("obj")
    obj_loc = HLParam("obj_loc", 3, 1, is_var=False, value=obj.get_pose(env))

    # gp.resample()

    pick = Pick(0, hl_plan, pick_env, robot, rp, obj, obj_loc, gp)
    hlas = [pick]

    ll_prob = LLProb(hlas)
    ll_prob.solve()
    assert np.allclose(pick.pos.value, np.array([[-2.0],[-0.61],[ 0.]]))


def test_pick_and_move():
    env = pick_test_env()

    # start = HLParam("start", 3, 1, is_var=False, value=np.array([[-2], [0], [0]]))
    # end = HLParam("end", 3, 1, is_var=False, value=np.array([[2], [0], [0]]))

    hl_plan = HLPlan(env)
    pick_env = env.CloneSelf(1) # clones objects in the environment
    move_env = env.CloneSelf(1) # clones objects in the environment

    robot = Robot(env.GetRobots()[0].GetName())
    rp = HLParam("rp", 3, 1)
    end = HLParam("end", 3, 1, is_var=False, value=np.array([[2],[0],[0]]))
    gp = GP("gp", 3, 1, is_resampled=True)
    obj = Obj("obj")
    obj_loc = HLParam("obj_loc", 3, 1, is_var=False, value=obj.get_pose(env))

    gp.resample()

    pick = Pick(0, hl_plan, pick_env, robot, rp, obj, obj_loc, gp)
    move = Move(0, hl_plan, move_env, robot, rp, end, obj, gp)
    hlas = [pick, move]

    ll_prob = LLProb(hlas)
    ll_prob.solve()

    ipdb.set_trace()


def test_pick_move_and_place():
    env = pick_test_env()
    robot = Robot(env.GetRobots()[0].GetName())

    hl_plan = HLPlan(env)
    place_env = env.CloneSelf(1) # clones objects in the environment
    pick_env = env.CloneSelf(1) # clones objects in the environment
    move_env = env.CloneSelf(1) # clones objects in the environment

    rp1 = HLParam("rp1", 3, 1)
    rp2 = HLParam("rp2", 3, 1)
    gp = GP("gp", 3, 1, is_resampled=True)
    obj = Obj("obj")
    obj_loc = HLParam("obj_loc", 3, 1, is_var=False, value=obj.get_pose(env))
    target_loc = HLParam("target_loc", 3, 1, is_var=False, value=np.array([[2],[0.5],[0]]))

    gp.resample()

    pick = Pick(0, hl_plan, pick_env, robot, rp1, obj, obj_loc, gp)
    move = Move(0, hl_plan, move_env, robot, rp1, rp2, obj, gp)
    import ipdb; ipdb.set_trace()
    place = Place(0, hl_plan, place_env, robot, rp2, obj, target_loc, gp)
    hlas = [pick, move, place]

    ll_prob = LLProb(hlas)
    ll_prob.solve()

# test_move()
# test_pick()
# test_pick_and_move()
test_pick_move_and_place()
