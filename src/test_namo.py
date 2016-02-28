import subprocess
import signal

NUM_TEST = 50
TIMEOUT = 600
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

def main():
    backtrack_info = []
    sqp_info = []
    earlyconvergesqp_info = []

    raw_input("Deleting old results.txt, press enter to continue")
    open("results.txt", "w").close()
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
                    info.append((traj_cost, total_time, replan_count))
                else:
                    info.append("fail")
            except Alarm:
                print "Timed out!"
                hp_instance.kill()
                info.append("timeout")

        with open("results.txt", "a") as f:
            f.write("Backtracking\n")
            f.write("%s\n"%backtrack_info)
            f.write("SQP\n")
            f.write("%s\n"%sqp_info)
            f.write("Early converge SQP\n")
            f.write("%s\n"%earlyconvergesqp_info)
    print "Testing complete."

if __name__ == "__main__":
    main()
