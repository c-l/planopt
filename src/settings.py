import time
DOMAIN_PATH = "../domains/"
errFileName = "robotics_autogen_err1.txt"
ENVPATH="../environments/"

RECORD = False 
# RECORD = True 

REPLAN_FOR_GRASP = False

def init_settings():
    import numpy as np 
    global seed
    seed = int(time.time())
    seed = 123456
    print "SEED: {}".format(seed)

    global RANDOM_STATE
    RANDOM_STATE = np.random.RandomState(seed)

FF = "ff"
MP = "mp"
FD = "fd"
FDOPTIMALMODE = False
FFEXEC = "../planners/FF-v2.3/ff"
FDEXEC = "../planners/FD/src/plan-ipc seq-sat-fd-autotune-1 "
FDOPTEXEC = "../planners/FD/src/plan-ipc seq-opt-lmcut "
# FDOPTEXEC = "../planners/FD/src/plan-ipc seq-opt-fdss-2 " 
MPEXEC = "../planners/M/Mp"

LOG_DOMAIN = 0
TWO_DOMAIN = 1

def set_domain(dom):
    global DOMAIN, pddlDomainFile, pddlDomainFileNoGeomEff, initialProblemFile, pddlToOpt, PLANNER_TO_USE 
    DOMAIN = dom
    if dom == LOG_DOMAIN:
        pddlDomainFile = DOMAIN_PATH + "log_dom.pddl"
        pddlDomainFileNoGeomEff = pddlDomainFile
        initialProblemFile = DOMAIN_PATH + "log_prob.pddl"

        import sys
        sys.path.insert(0, DOMAIN_PATH)
        from log_opt import LogOpt
        pddlToOpt = LogOpt
        PLANNER_TO_USE = FD

    elif dom == TWO_DOMAIN:
        pddlDomainFile = DOMAIN_PATH + "twobox_dom.pddl"
        pddlDomainFileNoGeomEff = pddlDomainFile
        initialProblemFile = DOMAIN_PATH + "twobox_prob.pddl"

        import sys
        sys.path.insert(0, DOMAIN_PATH)
        from twobox_opt import TwoBoxOpt
        pddlToOpt = TwoBoxOpt
        PLANNER_TO_USE = FD

envFile = ENVPATH+"created_info.dae"

# DISABLE_BASE = True
DISABLE_BASE = False
DO_BACKTRACKING = True
REPLAN = False
REDETECT = False
USE_SOFT_LIMITS = False
PRINT_GEN_COUNT = False
USE_MAX_ITERATIONS = False
APPLY_DSH_PATCH = False
INCREASE_MARGINS = False

# runs hybridPlanner automatically, no raw_inputs, use seed that generation script used, and only motion plan, no actual execution
# second arg is seconds before planning times out
# run_test_mode = (False, 65000)
run_test_mode = (True, 6000)

if run_test_mode[0]:
    try:
        with open("../tests/seed.txt", 'r') as f:
            seed = int(f.read())
    except (IOError, ValueError):
        print("Could not read seed that generation script used!")

