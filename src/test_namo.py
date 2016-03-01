import subprocess
import signal
from IPython import embed as shell
import numpy as np

NUM_TEST = 50
TIMEOUT = 1200
VIEWER = False
SEEDS = [11395659, 43736804, 26283602, 11583896, 25950117, 35437313,
         19194086, 42831540,  5082254, 24053257, 45035977, 27577047,
         27596765,  7483192, 32065540, 35186516, 43123152, 18424368,
         1177673, 36262570, 10538906, 20669186, 31757514, 28883354,
         34016297, 41862321, 33104806, 33421315, 21355764, 48675298,
         1254095, 40012734, 11630486,     3350, 40252104, 16159427,
         11841531,  6196559, 38427582, 24590870, 15301911, 16729724,
         37284131, 15641914, 41540643, 41826604,  8318855, 21752869,
         23125969,   608123]

class Alarm(Exception):
  pass

def alarm_handler(signum, frame):
  raise Alarm

def swap_test():
    backtrack_info = []
    sqp_info = []
    earlyconvergesqp_info = []

    raw_input("Deleting old results, press enter to continue")
    open("results_swap.txt", "w").close()
    for i in range(NUM_TEST):
        print "\n\n\nTesting %d of %d"%(i, NUM_TEST)
        seed = SEEDS[i]
        viewer_arg = ["-v"] if VIEWER else []

        for mode, info in [("backtrack", backtrack_info), ("sqp", sqp_info), ("earlyconvergesqp", earlyconvergesqp_info)]:
            hp_instance = subprocess.Popen(["python", "../src/hybridPlanner.py", "-d", "tc", "--%s"%mode,
                                            "-e", "../envs/swap_world.dae", "-s", str(SEEDS[i])] + viewer_arg)
            signal.signal(signal.SIGALRM, alarm_handler)
            signal.alarm(TIMEOUT)
            try:
                hp_instance.communicate()
                signal.alarm(0)
                if hp_instance.returncode == 0:
                    with open("hp_output.txt", "r") as f:
                        traj_cost, total_time, replan_count = f.readlines()
                    info.append((i, traj_cost, total_time, replan_count))
                else:
                    info.append("fail")
            except Alarm:
                print "Timed out!"
                hp_instance.kill()
                info.append("timeout")

        with open("results_swap.txt", "a") as f:
            f.write("Backtracking\n")
            f.write("%s\n"%backtrack_info)
            f.write("SQP\n")
            f.write("%s\n"%sqp_info)
            f.write("Early converge SQP\n")
            f.write("%s\n"%earlyconvergesqp_info)
    print "Testing complete."

def swap_other_initmodes_test():
    sqp_straight_info = []
    sqp_l2_info = []
    earlyconvergesqp_straight_info = []
    earlyconvergesqp_l2_info = []

    raw_input("Deleting old results, press enter to continue")
    open("results_swap_other_initmodes.txt", "w").close()
    for i in range(NUM_TEST):
        print "\n\n\nTesting %d of %d"%(i, NUM_TEST)
        seed = SEEDS[i]
        viewer_arg = ["-v"] if VIEWER else []

        for mode, init_mode, info in [("sqp", "straight", sqp_straight_info),
                                      ("sqp", "l2", sqp_l2_info),
                                      ("earlyconvergesqp", "straight", earlyconvergesqp_straight_info),
                                      ("earlyconvergesqp", "l2", earlyconvergesqp_l2_info)]:
            hp_instance = subprocess.Popen(["python", "../src/hybridPlanner.py", "-d", "tc", "--%s"%mode, "--%s"%init_mode,
                                            "-e", "../envs/swap_world.dae", "-s", str(SEEDS[i])] + viewer_arg)
            signal.signal(signal.SIGALRM, alarm_handler)
            signal.alarm(TIMEOUT)
            try:
                hp_instance.communicate()
                signal.alarm(0)
                if hp_instance.returncode == 0:
                    with open("hp_output.txt", "r") as f:
                        traj_cost, total_time, replan_count = f.readlines()
                    info.append((i, traj_cost, total_time, replan_count))
                else:
                    info.append("fail")
            except Alarm:
                print "Timed out!"
                hp_instance.kill()
                info.append("timeout")

        with open("results_swap_other_initmodes.txt", "a") as f:
            f.write("SQP straight line\n")
            f.write("%s\n"%sqp_straight_info)
            f.write("SQP l2\n")
            f.write("%s\n"%sqp_l2_info)
            f.write("Early converge SQP straight line\n")
            f.write("%s\n"%earlyconvergesqp_straight_info)
            f.write("Early converge SQP l2\n")
            f.write("%s\n"%earlyconvergesqp_l2_info)
    print "Testing complete."

def _is_failure(x):
    return x in ("fail", "timeout")

def _failure_in_lst(x):
    return "fail" in x or "timeout" in x

def swap_parse(f_name):
    with open(f_name, "r") as f:
        _, bt, _, sqp, _, early = f.readlines()[-6:]
    bt_info = {}
    sqp_info = {}
    early_info = {}
    for mode, d in [(bt, bt_info), (sqp, sqp_info), (early, early_info)]:
        for i, x in enumerate(eval(mode)):
            seed = SEEDS[i]
            if _is_failure(x):
                d[seed] = (x, x, x)
                continue
            tc, t, rc = x
            d[seed] = (float(tc.strip()), float(t.strip()), int(rc.strip()))
        succ = len(filter(lambda x: not _failure_in_lst(x), d.values())) * 1.0 / len(d)
        fail = len(filter(lambda x: "fail" in x, d.values())) * 1.0 / len(d)
        timeout = len(filter(lambda x: "timeout" in x, d.values())) * 1.0 / len(d)
        traj_cost = np.average([v[0] for v in d.values() if not _failure_in_lst(v)])
        total_time = np.average([v[1] for v in d.values() if not _failure_in_lst(v)])
        replan_count = np.average([v[2] for v in d.values() if not _failure_in_lst(v)])
        l = len(d)
        d["succ"] = succ
        d["fail"] = fail
        d["timeout"] = timeout
        d["traj_cost"] = traj_cost
        d["total_time"] = total_time
        d["replan_count"] = replan_count
        d["len"] = l
    for name, d in [("Backtrack", bt_info), ("SQP", sqp_info), ("Early Converge", early_info)]:
        print "%s num experiments: %d"%(name, d["len"])
        print "%s success rate: %f"%(name, d["succ"])
        print "%s failure rate: %f"%(name, d["fail"])
        print "%s timeout rate: %f"%(name, d["timeout"])
        print "%s traj cost: %f"%(name, d["traj_cost"])
        print "%s total time: %f"%(name, d["total_time"])
        print "%s replan count: %f"%(name, d["replan_count"])

def putaway_test(num_obstr):
    backtrack_info = []
    sqp_info = []
    earlyconvergesqp_info = []

    raw_input("Deleting old results, press enter to continue")
    open("results_putaway_%dobstr.txt"%num_obstr, "w").close()
    for i in range(NUM_TEST):
        print "\n\n\nTesting %d of %d"%(i, NUM_TEST)
        seed = SEEDS[i]
        viewer_arg = ["-v"] if VIEWER else []
        world = subprocess.Popen(["python", "../envs/world.py", "mc%d"%(num_obstr+2), str(SEEDS[i])])
        world.communicate()

        for mode, info in [("backtrack", backtrack_info), ("sqp", sqp_info), ("earlyconvergesqp", earlyconvergesqp_info)]:
            hp_instance = subprocess.Popen(["python", "../src/hybridPlanner.py", "-d", "mc", "--%s"%mode,
                                            "-e", "../envs/putaway_world.dae", "-s", str(SEEDS[i])] + viewer_arg)
            signal.signal(signal.SIGALRM, alarm_handler)
            signal.alarm(TIMEOUT)
            try:
                hp_instance.communicate()
                signal.alarm(0)
                if hp_instance.returncode == 0:
                    with open("hp_output.txt", "r") as f:
                        traj_cost, total_time, replan_count = f.readlines()
                    info.append((i, num_obstr, traj_cost, total_time, replan_count))
                else:
                    info.append("fail")
            except Alarm:
                print "Timed out!"
                hp_instance.kill()
                info.append("timeout")

        with open("results_putaway_%dobstr.txt"%num_obstr, "a") as f:
            f.write("Backtracking\n")
            f.write("%s\n"%backtrack_info)
            f.write("SQP\n")
            f.write("%s\n"%sqp_info)
            f.write("Early converge SQP\n")
            f.write("%s\n"%earlyconvergesqp_info)
    print "Testing complete."

if __name__ == "__main__":
    # swap_test()
    # swap_other_initmodes_test()
    # swap_parse("iros_16_results/results_putaway_namo.txt")
    putaway_test(num_obstr=0)
