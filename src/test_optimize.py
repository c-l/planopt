from interface.pr_creator import PRCreator
from interface.ll_prob import LLProb
import StringIO
from IPython import embed as shell
import openravepy
import settings
import sys
import time
import numpy as np
sys.path.append("../domains")
from twocan_opt import TwoCanOpt
settings.pddlToOpt = TwoCanOpt
settings.RANDOM_STATE = np.random.RandomState(12345)
settings.DO_SQP = False
settings.BACKTRACKING_REFINEMENT = False
settings.DO_EARLY_CONVERGE = True
settings.DO_ADMM = False
settings.DOMAIN = settings.MULTICAN_DOMAIN

def get_pr(plan, env_file):
    f = open("temp", "w")
    f.write(plan)
    f.close()
    env = openravepy.Environment()
    env.SetViewer("qtcoin")
    time.sleep(1)
    env.Load(env_file)
    pr_creator = PRCreator(env)
    pr = pr_creator.create_pr("temp", None, 0)
    return pr

def optimize(pr, recently_sampled, all_sampled_params, first_prio):
    llprob = LLProb(pr.action_list)
    llprob.solve_at_priority_regular(priority=first_prio, recently_sampled=recently_sampled)
    # llprob.solve_at_priority(priority=-1, recently_sampled=recently_sampled)
    # llprob.solve_at_priority(priority=0, recently_sampled=recently_sampled)
    llprob.solve_at_priority(priority=2)
    if llprob.recently_converged_vio_fluent is not None:
        # early converged based on a violated fluent
        violated_fluents = [llprob.recently_converged_vio_fluent]
    else:
        fluents = [f for a in pr.action_list for f in a.preconditions + a.postconditions]
        violated_fluents = pr.find_violated_fluents(fluents, 2)
    if not violated_fluents:
        print "Success"
        print llprob.traj_cost
        pr.execute(pause=False)
        while True:
            pr.execute(pause=False)
        return None
    else:
        print "Failed"
        print llprob.traj_cost
        recently_sampled = pr.randomized_resample(all_sampled_params, violated_fluents, mode="fluent", count=0)
        return recently_sampled

def simple_plan():
    # plan = "\n    0: MOVE ROBOTINITLOC GP_CAN2\n"
    plan = "\n    0: MOVE ROBOTINITLOC GP_CAN2\n    1: PICK CAN2 CAN2INITLOC GP_CAN2 GRASP_CAN2\n"
    pr = get_pr(plan)
    sampled_params = pr.world.get_sampled_params()
    sampled_params.sort(key=lambda p: p.sample_priority)
    recently_sampled = sampled_params
    for param in sampled_params:
        if param.__class__.__name__ == "RP":
            param.value = param.obj_loc.value + np.array([[0], [-0.6], [0]], dtype=np.float)
        if param.__class__.__name__ == "Grasp":
            param.value = np.array([[0], [0.6], [0]], dtype=np.float)
        if param.__class__.__name__ == "ObjLoc":
            # (0.5, 6.5, -1.5, 2.5)
            if "can1" in param.name:
                param.value = np.array([[3], [0], [0]], dtype=np.float)
            if "can2" in param.name:
                param.value = np.array([[3.1], [0], [0]], dtype=np.float)
    optimize(pr, recently_sampled)


def swap_good_init():
    plan = "0: MOVE ROBOTINITLOC GP_CAN1\n	1: PICK CAN1 CAN1INITLOC GP_CAN1 GRASP_CAN1\n	2: MOVE_W_OBJ GP_CAN1 PDP_CAN1_CAN1TEMPLOC CAN1 GRASP_CAN1\n	3: PLACE CAN1 CAN1TEMPLOC PDP_CAN1_CAN1TEMPLOC GRASP_CAN1\n	4: MOVE PDP_CAN1_CAN1TEMPLOC GP_CAN2\n	5: PICK CAN2 CAN2INITLOC GP_CAN2 GRASP_CAN2\n	6: MOVE_W_OBJ GP_CAN2 PDP_CAN2_GOAL2 CAN2 GRASP_CAN2\n	7: PLACE CAN2 GOAL2 PDP_CAN2_GOAL2 GRASP_CAN2\n	8: MOVE PDP_CAN2_GOAL2 GP_CAN1\n	9: PICK CAN1 CAN1TEMPLOC GP_CAN1 GRASP_CAN1\n	10: MOVE_W_OBJ GP_CAN1 PDP_CAN1_CAN1TEMPLOC CAN1 GRASP_CAN1\n	11: PLACE CAN1 CAN1TEMPLOC PDP_CAN1_CAN1TEMPLOC GRASP_CAN1\n	12: MOVE PDP_CAN1_CAN1TEMPLOC GP_CAN2\n	13: PICK CAN2 GOAL2 GP_CAN2 GRASP_CAN2\n	14: MOVE_W_OBJ GP_CAN2 PDP_CAN2_CAN2TEMPLOC CAN2 GRASP_CAN2\n	15: PLACE CAN2 CAN2TEMPLOC PDP_CAN2_CAN2TEMPLOC GRASP_CAN2\n	16: MOVE PDP_CAN2_CAN2TEMPLOC GP_CAN1\n	17: PICK CAN1 CAN1TEMPLOC GP_CAN1 GRASP_CAN1\n	18: MOVE_W_OBJ GP_CAN1 PDP_CAN1_GOAL1 CAN1 GRASP_CAN1\n	19: PLACE CAN1 GOAL1 PDP_CAN1_GOAL1 GRASP_CAN1\n	20: MOVE PDP_CAN1_GOAL1 GP_CAN2\n	21: PICK CAN2 CAN2TEMPLOC GP_CAN2 GRASP_CAN2\n	22: MOVE_W_OBJ GP_CAN2 PDP_CAN2_GOAL2 CAN2 GRASP_CAN2\n	23: PLACE CAN2 GOAL2 PDP_CAN2_GOAL2 GRASP_CAN2"
    pr = get_pr(plan)
    sampled_params = pr.world.get_sampled_params()
    sampled_params.sort(key=lambda p: p.sample_priority)
    recently_sampled = sampled_params
    for param in sampled_params:
        if param.__class__.__name__ == "RP":
            param.value = param.obj_loc.value + np.array([[0], [-0.6], [0]], dtype=np.float)
        if param.__class__.__name__ == "Grasp":
            param.value = np.array([[0], [0.6], [0]], dtype=np.float)
        if param.__class__.__name__ == "ObjLoc":
            # (0.5, 6.5, -1.5, 2.5)
            if "can1" in param.name:
                param.value = np.array([[3], [0], [0]], dtype=np.float)
            if "can2" in param.name:
                param.value = np.array([[3.1], [0], [0]], dtype=np.float)
    optimize(pr, recently_sampled)

def swap_good_init_but_locs_stuck():
    plan = "0: MOVE ROBOTINITLOC GP_CAN1\n	1: PICK CAN1 CAN1INITLOC GP_CAN1 GRASP_CAN1\n	2: MOVE_W_OBJ GP_CAN1 PDP_CAN1_CAN1TEMPLOC CAN1 GRASP_CAN1\n	3: PLACE CAN1 CAN1TEMPLOC PDP_CAN1_CAN1TEMPLOC GRASP_CAN1\n	4: MOVE PDP_CAN1_CAN1TEMPLOC GP_CAN2\n	5: PICK CAN2 CAN2INITLOC GP_CAN2 GRASP_CAN2\n	6: MOVE_W_OBJ GP_CAN2 PDP_CAN2_GOAL2 CAN2 GRASP_CAN2\n	7: PLACE CAN2 GOAL2 PDP_CAN2_GOAL2 GRASP_CAN2\n	8: MOVE PDP_CAN2_GOAL2 GP_CAN1\n	9: PICK CAN1 CAN1TEMPLOC GP_CAN1 GRASP_CAN1\n	10: MOVE_W_OBJ GP_CAN1 PDP_CAN1_CAN1TEMPLOC CAN1 GRASP_CAN1\n	11: PLACE CAN1 CAN1TEMPLOC PDP_CAN1_CAN1TEMPLOC GRASP_CAN1\n	12: MOVE PDP_CAN1_CAN1TEMPLOC GP_CAN2\n	13: PICK CAN2 GOAL2 GP_CAN2 GRASP_CAN2\n	14: MOVE_W_OBJ GP_CAN2 PDP_CAN2_CAN2TEMPLOC CAN2 GRASP_CAN2\n	15: PLACE CAN2 CAN2TEMPLOC PDP_CAN2_CAN2TEMPLOC GRASP_CAN2\n	16: MOVE PDP_CAN2_CAN2TEMPLOC GP_CAN1\n	17: PICK CAN1 CAN1TEMPLOC GP_CAN1 GRASP_CAN1\n	18: MOVE_W_OBJ GP_CAN1 PDP_CAN1_GOAL1 CAN1 GRASP_CAN1\n	19: PLACE CAN1 GOAL1 PDP_CAN1_GOAL1 GRASP_CAN1\n	20: MOVE PDP_CAN1_GOAL1 GP_CAN2\n	21: PICK CAN2 CAN2TEMPLOC GP_CAN2 GRASP_CAN2\n	22: MOVE_W_OBJ GP_CAN2 PDP_CAN2_GOAL2 CAN2 GRASP_CAN2\n	23: PLACE CAN2 GOAL2 PDP_CAN2_GOAL2 GRASP_CAN2"
    pr = get_pr(plan)
    sampled_params = pr.world.get_sampled_params()
    sampled_params.sort(key=lambda p: p.sample_priority)
    recently_sampled = sampled_params
    for param in sampled_params:
        if param.__class__.__name__ == "RP":
            param.value = param.obj_loc.value + np.array([[0], [-0.6], [0]], dtype=np.float)
        if param.__class__.__name__ == "Grasp":
            param.value = np.array([[0], [0.6], [0]], dtype=np.float)
        if param.__class__.__name__ == "ObjLoc":
            # (0.5, 6.5, -1.5, 2.5)
            if "can1" in param.name:
                param.value = np.array([[6], [0], [0]], dtype=np.float)
            if "can2" in param.name:
                param.value = np.array([[6.1], [0], [0]], dtype=np.float)
    optimize(pr, recently_sampled)

def swap_bad_init_left_center():
    plan = "0: MOVE ROBOTINITLOC GP_CAN1\n	1: PICK CAN1 CAN1INITLOC GP_CAN1 GRASP_CAN1\n	2: MOVE_W_OBJ GP_CAN1 PDP_CAN1_CAN1TEMPLOC CAN1 GRASP_CAN1\n	3: PLACE CAN1 CAN1TEMPLOC PDP_CAN1_CAN1TEMPLOC GRASP_CAN1\n	4: MOVE PDP_CAN1_CAN1TEMPLOC GP_CAN2\n	5: PICK CAN2 CAN2INITLOC GP_CAN2 GRASP_CAN2\n	6: MOVE_W_OBJ GP_CAN2 PDP_CAN2_GOAL2 CAN2 GRASP_CAN2\n	7: PLACE CAN2 GOAL2 PDP_CAN2_GOAL2 GRASP_CAN2\n	8: MOVE PDP_CAN2_GOAL2 GP_CAN1\n	9: PICK CAN1 CAN1TEMPLOC GP_CAN1 GRASP_CAN1\n	10: MOVE_W_OBJ GP_CAN1 PDP_CAN1_CAN1TEMPLOC CAN1 GRASP_CAN1\n	11: PLACE CAN1 CAN1TEMPLOC PDP_CAN1_CAN1TEMPLOC GRASP_CAN1\n	12: MOVE PDP_CAN1_CAN1TEMPLOC GP_CAN2\n	13: PICK CAN2 GOAL2 GP_CAN2 GRASP_CAN2\n	14: MOVE_W_OBJ GP_CAN2 PDP_CAN2_CAN2TEMPLOC CAN2 GRASP_CAN2\n	15: PLACE CAN2 CAN2TEMPLOC PDP_CAN2_CAN2TEMPLOC GRASP_CAN2\n	16: MOVE PDP_CAN2_CAN2TEMPLOC GP_CAN1\n	17: PICK CAN1 CAN1TEMPLOC GP_CAN1 GRASP_CAN1\n	18: MOVE_W_OBJ GP_CAN1 PDP_CAN1_GOAL1 CAN1 GRASP_CAN1\n	19: PLACE CAN1 GOAL1 PDP_CAN1_GOAL1 GRASP_CAN1\n	20: MOVE PDP_CAN1_GOAL1 GP_CAN2\n	21: PICK CAN2 CAN2TEMPLOC GP_CAN2 GRASP_CAN2\n	22: MOVE_W_OBJ GP_CAN2 PDP_CAN2_GOAL2 CAN2 GRASP_CAN2\n	23: PLACE CAN2 GOAL2 PDP_CAN2_GOAL2 GRASP_CAN2"
    pr = get_pr(plan)
    sampled_params = pr.world.get_sampled_params()
    sampled_params.sort(key=lambda p: p.sample_priority)
    recently_sampled = sampled_params
    for param in sampled_params:
        if param.__class__.__name__ == "RP":
            param.value = param.obj_loc.value + np.array([[0], [0.6], [0]], dtype=np.float)
        if param.__class__.__name__ == "Grasp":
            param.value = np.array([[0], [-0.6], [0]], dtype=np.float)
        if param.__class__.__name__ == "ObjLoc":
            # (0.5, 6.5, -1.5, 2.5)
            if "can1" in param.name:
                param.value = np.array([[3], [0], [0]], dtype=np.float)
            if "can2" in param.name:
                param.value = np.array([[3.1], [0], [0]], dtype=np.float)
    optimize(pr, recently_sampled)

def swap_bad_init_left_and_right():
    plan = "0: MOVE ROBOTINITLOC GP_CAN1\n	1: PICK CAN1 CAN1INITLOC GP_CAN1 GRASP_CAN1\n	2: MOVE_W_OBJ GP_CAN1 PDP_CAN1_CAN1TEMPLOC CAN1 GRASP_CAN1\n	3: PLACE CAN1 CAN1TEMPLOC PDP_CAN1_CAN1TEMPLOC GRASP_CAN1\n	4: MOVE PDP_CAN1_CAN1TEMPLOC GP_CAN2\n	5: PICK CAN2 CAN2INITLOC GP_CAN2 GRASP_CAN2\n	6: MOVE_W_OBJ GP_CAN2 PDP_CAN2_GOAL2 CAN2 GRASP_CAN2\n	7: PLACE CAN2 GOAL2 PDP_CAN2_GOAL2 GRASP_CAN2\n	8: MOVE PDP_CAN2_GOAL2 GP_CAN1\n	9: PICK CAN1 CAN1TEMPLOC GP_CAN1 GRASP_CAN1\n	10: MOVE_W_OBJ GP_CAN1 PDP_CAN1_CAN1TEMPLOC CAN1 GRASP_CAN1\n	11: PLACE CAN1 CAN1TEMPLOC PDP_CAN1_CAN1TEMPLOC GRASP_CAN1\n	12: MOVE PDP_CAN1_CAN1TEMPLOC GP_CAN2\n	13: PICK CAN2 GOAL2 GP_CAN2 GRASP_CAN2\n	14: MOVE_W_OBJ GP_CAN2 PDP_CAN2_CAN2TEMPLOC CAN2 GRASP_CAN2\n	15: PLACE CAN2 CAN2TEMPLOC PDP_CAN2_CAN2TEMPLOC GRASP_CAN2\n	16: MOVE PDP_CAN2_CAN2TEMPLOC GP_CAN1\n	17: PICK CAN1 CAN1TEMPLOC GP_CAN1 GRASP_CAN1\n	18: MOVE_W_OBJ GP_CAN1 PDP_CAN1_GOAL1 CAN1 GRASP_CAN1\n	19: PLACE CAN1 GOAL1 PDP_CAN1_GOAL1 GRASP_CAN1\n	20: MOVE PDP_CAN1_GOAL1 GP_CAN2\n	21: PICK CAN2 CAN2TEMPLOC GP_CAN2 GRASP_CAN2\n	22: MOVE_W_OBJ GP_CAN2 PDP_CAN2_GOAL2 CAN2 GRASP_CAN2\n	23: PLACE CAN2 GOAL2 PDP_CAN2_GOAL2 GRASP_CAN2"
    pr = get_pr(plan)
    sampled_params = pr.world.get_sampled_params()
    sampled_params.sort(key=lambda p: p.sample_priority)
    recently_sampled = sampled_params
    for param in sampled_params:
        if param.__class__.__name__ == "RP":
            param.value = param.obj_loc.value + np.array([[0], [0.6], [0]], dtype=np.float)
        if param.__class__.__name__ == "Grasp":
            param.value = np.array([[0], [-0.6], [0]], dtype=np.float)
        if param.__class__.__name__ == "ObjLoc":
            # (0.5, 6.5, -1.5, 2.5)
            if "can1" in param.name:
                param.value = np.array([[6], [0], [0]], dtype=np.float)
            if "can2" in param.name:
                param.value = np.array([[1], [0], [0]], dtype=np.float)
    optimize(pr, recently_sampled)

def swap_good_init_left_and_right():
    plan = "0: MOVE ROBOTINITLOC GP_CAN1\n	1: PICK CAN1 CAN1INITLOC GP_CAN1 GRASP_CAN1\n	2: MOVE_W_OBJ GP_CAN1 PDP_CAN1_CAN1TEMPLOC CAN1 GRASP_CAN1\n	3: PLACE CAN1 CAN1TEMPLOC PDP_CAN1_CAN1TEMPLOC GRASP_CAN1\n	4: MOVE PDP_CAN1_CAN1TEMPLOC GP_CAN2\n	5: PICK CAN2 CAN2INITLOC GP_CAN2 GRASP_CAN2\n	6: MOVE_W_OBJ GP_CAN2 PDP_CAN2_GOAL2 CAN2 GRASP_CAN2\n	7: PLACE CAN2 GOAL2 PDP_CAN2_GOAL2 GRASP_CAN2\n	8: MOVE PDP_CAN2_GOAL2 GP_CAN1\n	9: PICK CAN1 CAN1TEMPLOC GP_CAN1 GRASP_CAN1\n	10: MOVE_W_OBJ GP_CAN1 PDP_CAN1_CAN1TEMPLOC CAN1 GRASP_CAN1\n	11: PLACE CAN1 CAN1TEMPLOC PDP_CAN1_CAN1TEMPLOC GRASP_CAN1\n	12: MOVE PDP_CAN1_CAN1TEMPLOC GP_CAN2\n	13: PICK CAN2 GOAL2 GP_CAN2 GRASP_CAN2\n	14: MOVE_W_OBJ GP_CAN2 PDP_CAN2_CAN2TEMPLOC CAN2 GRASP_CAN2\n	15: PLACE CAN2 CAN2TEMPLOC PDP_CAN2_CAN2TEMPLOC GRASP_CAN2\n	16: MOVE PDP_CAN2_CAN2TEMPLOC GP_CAN1\n	17: PICK CAN1 CAN1TEMPLOC GP_CAN1 GRASP_CAN1\n	18: MOVE_W_OBJ GP_CAN1 PDP_CAN1_GOAL1 CAN1 GRASP_CAN1\n	19: PLACE CAN1 GOAL1 PDP_CAN1_GOAL1 GRASP_CAN1\n	20: MOVE PDP_CAN1_GOAL1 GP_CAN2\n	21: PICK CAN2 CAN2TEMPLOC GP_CAN2 GRASP_CAN2\n	22: MOVE_W_OBJ GP_CAN2 PDP_CAN2_GOAL2 CAN2 GRASP_CAN2\n	23: PLACE CAN2 GOAL2 PDP_CAN2_GOAL2 GRASP_CAN2"
    pr = get_pr(plan)
    sampled_params = pr.world.get_sampled_params()
    sampled_params.sort(key=lambda p: p.sample_priority)
    recently_sampled = sampled_params
    for param in sampled_params:
        if param.__class__.__name__ == "RP":
            param.value = param.obj_loc.value + np.array([[0], [-0.6], [0]], dtype=np.float)
        if param.__class__.__name__ == "Grasp":
            param.value = np.array([[0], [0.6], [0]], dtype=np.float)
        if param.__class__.__name__ == "ObjLoc":
            # (0.5, 6.5, -1.5, 2.5)
            if "can1" in param.name:
                param.value = np.array([[6], [0], [0]], dtype=np.float)
            if "can2" in param.name:
                param.value = np.array([[1], [0], [0]], dtype=np.float)
    optimize(pr, recently_sampled)

def swap_start_in_object_bottom():
    plan = "0: MOVE ROBOTINITLOC GP_CAN1\n	1: PICK CAN1 CAN1INITLOC GP_CAN1 GRASP_CAN1\n	2: MOVE_W_OBJ GP_CAN1 PDP_CAN1_CAN1TEMPLOC CAN1 GRASP_CAN1\n	3: PLACE CAN1 CAN1TEMPLOC PDP_CAN1_CAN1TEMPLOC GRASP_CAN1\n	4: MOVE PDP_CAN1_CAN1TEMPLOC GP_CAN2\n	5: PICK CAN2 CAN2INITLOC GP_CAN2 GRASP_CAN2\n	6: MOVE_W_OBJ GP_CAN2 PDP_CAN2_GOAL2 CAN2 GRASP_CAN2\n	7: PLACE CAN2 GOAL2 PDP_CAN2_GOAL2 GRASP_CAN2\n	8: MOVE PDP_CAN2_GOAL2 GP_CAN1\n	9: PICK CAN1 CAN1TEMPLOC GP_CAN1 GRASP_CAN1\n	10: MOVE_W_OBJ GP_CAN1 PDP_CAN1_CAN1TEMPLOC CAN1 GRASP_CAN1\n	11: PLACE CAN1 CAN1TEMPLOC PDP_CAN1_CAN1TEMPLOC GRASP_CAN1\n	12: MOVE PDP_CAN1_CAN1TEMPLOC GP_CAN2\n	13: PICK CAN2 GOAL2 GP_CAN2 GRASP_CAN2\n	14: MOVE_W_OBJ GP_CAN2 PDP_CAN2_CAN2TEMPLOC CAN2 GRASP_CAN2\n	15: PLACE CAN2 CAN2TEMPLOC PDP_CAN2_CAN2TEMPLOC GRASP_CAN2\n	16: MOVE PDP_CAN2_CAN2TEMPLOC GP_CAN1\n	17: PICK CAN1 CAN1TEMPLOC GP_CAN1 GRASP_CAN1\n	18: MOVE_W_OBJ GP_CAN1 PDP_CAN1_GOAL1 CAN1 GRASP_CAN1\n	19: PLACE CAN1 GOAL1 PDP_CAN1_GOAL1 GRASP_CAN1\n	20: MOVE PDP_CAN1_GOAL1 GP_CAN2\n	21: PICK CAN2 CAN2TEMPLOC GP_CAN2 GRASP_CAN2\n	22: MOVE_W_OBJ GP_CAN2 PDP_CAN2_GOAL2 CAN2 GRASP_CAN2\n	23: PLACE CAN2 GOAL2 PDP_CAN2_GOAL2 GRASP_CAN2"
    pr = get_pr(plan)
    sampled_params = pr.world.get_sampled_params()
    sampled_params.sort(key=lambda p: p.sample_priority)
    recently_sampled = sampled_params
    for param in sampled_params:
        if param.__class__.__name__ == "RP":
            param.value = param.obj_loc.value + np.array([[0], [-0.6], [0]], dtype=np.float)
        if param.__class__.__name__ == "Grasp":
            param.value = np.array([[0], [-0.3], [0]], dtype=np.float)
        if param.__class__.__name__ == "ObjLoc":
            # (0.5, 6.5, -1.5, 2.5)
            if "can1" in param.name:
                param.value = np.array([[6], [0], [0]], dtype=np.float)
            if "can2" in param.name:
                param.value = np.array([[1], [0], [0]], dtype=np.float)
    optimize(pr, recently_sampled)

def swap_start_in_object_top():
    plan = "0: MOVE ROBOTINITLOC GP_CAN1\n	1: PICK CAN1 CAN1INITLOC GP_CAN1 GRASP_CAN1\n	2: MOVE_W_OBJ GP_CAN1 PDP_CAN1_CAN1TEMPLOC CAN1 GRASP_CAN1\n	3: PLACE CAN1 CAN1TEMPLOC PDP_CAN1_CAN1TEMPLOC GRASP_CAN1\n	4: MOVE PDP_CAN1_CAN1TEMPLOC GP_CAN2\n	5: PICK CAN2 CAN2INITLOC GP_CAN2 GRASP_CAN2\n	6: MOVE_W_OBJ GP_CAN2 PDP_CAN2_GOAL2 CAN2 GRASP_CAN2\n	7: PLACE CAN2 GOAL2 PDP_CAN2_GOAL2 GRASP_CAN2\n	8: MOVE PDP_CAN2_GOAL2 GP_CAN1\n	9: PICK CAN1 CAN1TEMPLOC GP_CAN1 GRASP_CAN1\n	10: MOVE_W_OBJ GP_CAN1 PDP_CAN1_CAN1TEMPLOC CAN1 GRASP_CAN1\n	11: PLACE CAN1 CAN1TEMPLOC PDP_CAN1_CAN1TEMPLOC GRASP_CAN1\n	12: MOVE PDP_CAN1_CAN1TEMPLOC GP_CAN2\n	13: PICK CAN2 GOAL2 GP_CAN2 GRASP_CAN2\n	14: MOVE_W_OBJ GP_CAN2 PDP_CAN2_CAN2TEMPLOC CAN2 GRASP_CAN2\n	15: PLACE CAN2 CAN2TEMPLOC PDP_CAN2_CAN2TEMPLOC GRASP_CAN2\n	16: MOVE PDP_CAN2_CAN2TEMPLOC GP_CAN1\n	17: PICK CAN1 CAN1TEMPLOC GP_CAN1 GRASP_CAN1\n	18: MOVE_W_OBJ GP_CAN1 PDP_CAN1_GOAL1 CAN1 GRASP_CAN1\n	19: PLACE CAN1 GOAL1 PDP_CAN1_GOAL1 GRASP_CAN1\n	20: MOVE PDP_CAN1_GOAL1 GP_CAN2\n	21: PICK CAN2 CAN2TEMPLOC GP_CAN2 GRASP_CAN2\n	22: MOVE_W_OBJ GP_CAN2 PDP_CAN2_GOAL2 CAN2 GRASP_CAN2\n	23: PLACE CAN2 GOAL2 PDP_CAN2_GOAL2 GRASP_CAN2"
    pr = get_pr(plan)
    sampled_params = pr.world.get_sampled_params()
    sampled_params.sort(key=lambda p: p.sample_priority)
    recently_sampled = sampled_params
    for param in sampled_params:
        if param.__class__.__name__ == "RP":
            param.value = param.obj_loc.value + np.array([[0], [0.6], [0]], dtype=np.float)
        if param.__class__.__name__ == "Grasp":
            param.value = np.array([[0], [0.3], [0]], dtype=np.float)
        if param.__class__.__name__ == "ObjLoc":
            # (0.5, 6.5, -1.5, 2.5)
            if "can1" in param.name:
                param.value = np.array([[6], [0], [0]], dtype=np.float)
            if "can2" in param.name:
                param.value = np.array([[1], [0], [0]], dtype=np.float)
    optimize(pr, recently_sampled)

def swap_bad_init_far_right():
    plan = "0: MOVE ROBOTINITLOC GP_CAN1\n	1: PICK CAN1 CAN1INITLOC GP_CAN1 GRASP_CAN1\n	2: MOVE_W_OBJ GP_CAN1 PDP_CAN1_CAN1TEMPLOC CAN1 GRASP_CAN1\n	3: PLACE CAN1 CAN1TEMPLOC PDP_CAN1_CAN1TEMPLOC GRASP_CAN1\n	4: MOVE PDP_CAN1_CAN1TEMPLOC GP_CAN2\n	5: PICK CAN2 CAN2INITLOC GP_CAN2 GRASP_CAN2\n	6: MOVE_W_OBJ GP_CAN2 PDP_CAN2_GOAL2 CAN2 GRASP_CAN2\n	7: PLACE CAN2 GOAL2 PDP_CAN2_GOAL2 GRASP_CAN2\n	8: MOVE PDP_CAN2_GOAL2 GP_CAN1\n	9: PICK CAN1 CAN1TEMPLOC GP_CAN1 GRASP_CAN1\n	10: MOVE_W_OBJ GP_CAN1 PDP_CAN1_CAN1TEMPLOC CAN1 GRASP_CAN1\n	11: PLACE CAN1 CAN1TEMPLOC PDP_CAN1_CAN1TEMPLOC GRASP_CAN1\n	12: MOVE PDP_CAN1_CAN1TEMPLOC GP_CAN2\n	13: PICK CAN2 GOAL2 GP_CAN2 GRASP_CAN2\n	14: MOVE_W_OBJ GP_CAN2 PDP_CAN2_CAN2TEMPLOC CAN2 GRASP_CAN2\n	15: PLACE CAN2 CAN2TEMPLOC PDP_CAN2_CAN2TEMPLOC GRASP_CAN2\n	16: MOVE PDP_CAN2_CAN2TEMPLOC GP_CAN1\n	17: PICK CAN1 CAN1TEMPLOC GP_CAN1 GRASP_CAN1\n	18: MOVE_W_OBJ GP_CAN1 PDP_CAN1_GOAL1 CAN1 GRASP_CAN1\n	19: PLACE CAN1 GOAL1 PDP_CAN1_GOAL1 GRASP_CAN1\n	20: MOVE PDP_CAN1_GOAL1 GP_CAN2\n	21: PICK CAN2 CAN2TEMPLOC GP_CAN2 GRASP_CAN2\n	22: MOVE_W_OBJ GP_CAN2 PDP_CAN2_GOAL2 CAN2 GRASP_CAN2\n	23: PLACE CAN2 GOAL2 PDP_CAN2_GOAL2 GRASP_CAN2"
    pr = get_pr(plan)
    sampled_params = pr.world.get_sampled_params()
    sampled_params.sort(key=lambda p: p.sample_priority)
    recently_sampled = sampled_params
    for param in sampled_params:
        if param.__class__.__name__ == "RP":
            param.value = param.obj_loc.value + np.array([[0], [0.6], [0]], dtype=np.float)
        if param.__class__.__name__ == "Grasp":
            param.value = np.array([[0], [-0.6], [0]], dtype=np.float)
        if param.__class__.__name__ == "ObjLoc":
            # (0.5, 6.5, -1.5, 2.5)
            if "can1" in param.name:
                param.value = np.array([[6], [0], [0]], dtype=np.float)
            if "can2" in param.name:
                param.value = np.array([[6.1], [0], [0]], dtype=np.float)
    optimize(pr, recently_sampled)

def swap_bad_init_right_center():
    plan = "0: MOVE ROBOTINITLOC GP_CAN1\n	1: PICK CAN1 CAN1INITLOC GP_CAN1 GRASP_CAN1\n	2: MOVE_W_OBJ GP_CAN1 PDP_CAN1_CAN1TEMPLOC CAN1 GRASP_CAN1\n	3: PLACE CAN1 CAN1TEMPLOC PDP_CAN1_CAN1TEMPLOC GRASP_CAN1\n	4: MOVE PDP_CAN1_CAN1TEMPLOC GP_CAN2\n	5: PICK CAN2 CAN2INITLOC GP_CAN2 GRASP_CAN2\n	6: MOVE_W_OBJ GP_CAN2 PDP_CAN2_GOAL2 CAN2 GRASP_CAN2\n	7: PLACE CAN2 GOAL2 PDP_CAN2_GOAL2 GRASP_CAN2\n	8: MOVE PDP_CAN2_GOAL2 GP_CAN1\n	9: PICK CAN1 CAN1TEMPLOC GP_CAN1 GRASP_CAN1\n	10: MOVE_W_OBJ GP_CAN1 PDP_CAN1_CAN1TEMPLOC CAN1 GRASP_CAN1\n	11: PLACE CAN1 CAN1TEMPLOC PDP_CAN1_CAN1TEMPLOC GRASP_CAN1\n	12: MOVE PDP_CAN1_CAN1TEMPLOC GP_CAN2\n	13: PICK CAN2 GOAL2 GP_CAN2 GRASP_CAN2\n	14: MOVE_W_OBJ GP_CAN2 PDP_CAN2_CAN2TEMPLOC CAN2 GRASP_CAN2\n	15: PLACE CAN2 CAN2TEMPLOC PDP_CAN2_CAN2TEMPLOC GRASP_CAN2\n	16: MOVE PDP_CAN2_CAN2TEMPLOC GP_CAN1\n	17: PICK CAN1 CAN1TEMPLOC GP_CAN1 GRASP_CAN1\n	18: MOVE_W_OBJ GP_CAN1 PDP_CAN1_GOAL1 CAN1 GRASP_CAN1\n	19: PLACE CAN1 GOAL1 PDP_CAN1_GOAL1 GRASP_CAN1\n	20: MOVE PDP_CAN1_GOAL1 GP_CAN2\n	21: PICK CAN2 CAN2TEMPLOC GP_CAN2 GRASP_CAN2\n	22: MOVE_W_OBJ GP_CAN2 PDP_CAN2_GOAL2 CAN2 GRASP_CAN2\n	23: PLACE CAN2 GOAL2 PDP_CAN2_GOAL2 GRASP_CAN2"
    pr = get_pr(plan)
    sampled_params = pr.world.get_sampled_params()
    sampled_params.sort(key=lambda p: p.sample_priority)
    recently_sampled = sampled_params
    for param in sampled_params:
        if param.__class__.__name__ == "RP":
            param.value = param.obj_loc.value + np.array([[0], [0.6], [0]], dtype=np.float)
        if param.__class__.__name__ == "Grasp":
            param.value = np.array([[0], [-0.6], [0]], dtype=np.float)
        if param.__class__.__name__ == "ObjLoc":
            # (0.5, 6.5, -1.5, 2.5)
            if "can1" in param.name:
                param.value = np.array([[4], [0], [0]], dtype=np.float)
            if "can2" in param.name:
                param.value = np.array([[5], [0], [0]], dtype=np.float)
    optimize(pr, recently_sampled)

def swap_bad_grasp_far_right():
    # plan = "0: MOVE ROBOTINITLOC GP_CAN1\n	1: PICK CAN1 CAN1INITLOC GP_CAN1 GRASP_CAN1\n	2: MOVE_W_OBJ GP_CAN1 PDP_CAN1_CAN1TEMPLOC CAN1 GRASP_CAN1\n	3: PLACE CAN1 CAN1TEMPLOC PDP_CAN1_CAN1TEMPLOC GRASP_CAN1\n	4: MOVE PDP_CAN1_CAN1TEMPLOC GP_CAN2\n	5: PICK CAN2 CAN2INITLOC GP_CAN2 GRASP_CAN2\n	6: MOVE_W_OBJ GP_CAN2 PDP_CAN2_GOAL2 CAN2 GRASP_CAN2\n	7: PLACE CAN2 GOAL2 PDP_CAN2_GOAL2 GRASP_CAN2\n	8: MOVE PDP_CAN2_GOAL2 GP_CAN1\n	9: PICK CAN1 CAN1TEMPLOC GP_CAN1 GRASP_CAN1\n	10: MOVE_W_OBJ GP_CAN1 PDP_CAN1_CAN1TEMPLOC CAN1 GRASP_CAN1\n	11: PLACE CAN1 CAN1TEMPLOC PDP_CAN1_CAN1TEMPLOC GRASP_CAN1\n	12: MOVE PDP_CAN1_CAN1TEMPLOC GP_CAN2\n	13: PICK CAN2 GOAL2 GP_CAN2 GRASP_CAN2\n	14: MOVE_W_OBJ GP_CAN2 PDP_CAN2_CAN2TEMPLOC CAN2 GRASP_CAN2\n	15: PLACE CAN2 CAN2TEMPLOC PDP_CAN2_CAN2TEMPLOC GRASP_CAN2\n	16: MOVE PDP_CAN2_CAN2TEMPLOC GP_CAN1\n	17: PICK CAN1 CAN1TEMPLOC GP_CAN1 GRASP_CAN1\n	18: MOVE_W_OBJ GP_CAN1 PDP_CAN1_GOAL1 CAN1 GRASP_CAN1\n	19: PLACE CAN1 GOAL1 PDP_CAN1_GOAL1 GRASP_CAN1\n	20: MOVE PDP_CAN1_GOAL1 GP_CAN2\n	21: PICK CAN2 CAN2TEMPLOC GP_CAN2 GRASP_CAN2\n	22: MOVE_W_OBJ GP_CAN2 PDP_CAN2_GOAL2 CAN2 GRASP_CAN2\n	23: PLACE CAN2 GOAL2 PDP_CAN2_GOAL2 GRASP_CAN2"
    plan = "0: MOVE ROBOTINITLOC GP_CAN2\n	1: PICK CAN2 CAN2INITLOC GP_CAN2 GRASP_CAN2\n"
    pr = get_pr(plan, env_file="../envs/swap_world.dae")
    sampled_params = pr.world.get_sampled_params()
    sampled_params.sort(key=lambda p: p.sample_priority)
    recently_sampled = sampled_params
    for param in pr.action_list[0].params:
        if param.name == "robotinitloc":
            param.value = np.array([[1], [2], [0]], dtype=np.float)        
    for param in sampled_params:
        if param.__class__.__name__ == "RP":
            param.value = param.obj_loc.value + np.array([[0], [0.6], [0]], dtype=np.float)
        if param.__class__.__name__ == "Grasp":
            param.value = np.array([[0], [-0.6], [0]], dtype=np.float)
        if param.__class__.__name__ == "ObjLoc":
            # (0.5, 6.5, -1.5, 2.5)
            if "can1" in param.name:
                param.value = np.array([[3], [0], [0]], dtype=np.float)
            if "can2" in param.name:
                param.value = np.array([[3.1], [0], [0]], dtype=np.float)
    recently_sampled = optimize(pr, recently_sampled, sampled_params, first_prio=-1)
    while True:
        for param in recently_sampled:
            if param.__class__.__name__ == "RP":
                param.value = param.obj_loc.value + np.array([[0], [-0.6], [0]], dtype=np.float)
        recently_sampled = optimize(pr, recently_sampled, sampled_params, first_prio=0)

def putaway_easy():
    plan = "0: MOVE ROBOTINITLOC GP_CAN1\n	1: PICK CAN1 CAN1INITLOC GP_CAN1 GRASP_CAN1\n	2: MOVE_W_OBJ GP_CAN1 PDP_CAN1_GOAL1 CAN1 GRASP_CAN1\n	3: PLACE CAN1 GOAL1 PDP_CAN1_GOAL1 GRASP_CAN1\n	4: MOVE PDP_CAN1_GOAL1 GP_CAN2\n	5: PICK CAN2 CAN2INITLOC GP_CAN2 GRASP_CAN2\n	6: MOVE_W_OBJ GP_CAN2 PDP_CAN2_GOAL2 CAN2 GRASP_CAN2\n	7: PLACE CAN2 GOAL2 PDP_CAN2_GOAL2 GRASP_CAN2"
    pr = get_pr(plan, env_file="../envs/putaway_world_easy.dae")
    sampled_params = pr.world.get_sampled_params()
    sampled_params.sort(key=lambda p: p.sample_priority)
    for param in sampled_params:
        param.resample()
    recently_sampled = sampled_params
    recently_sampled = optimize(pr, recently_sampled, sampled_params, first_prio=-1)
    while True:
        recently_sampled = optimize(pr, recently_sampled, sampled_params, first_prio=0)

def main():
    # swap_good_init_but_locs_stuck()
    # swap_bad_init()
    swap_bad_grasp_far_right()
    # swap_good_init_left_and_right()
    # swap_start_in_object_top()
    # simple_plan()
    # putaway_easy()

if __name__ == "__main__":
    main()
