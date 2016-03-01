import utils
import settings
import time
from interface.hl_actions import *
from interface.hl_actions.move import Move
from interface.hl_actions.pick import Pick
from interface.hl_actions.place import Place
from IPython import embed as shell
# from hl_actions import hl_action # not sure why this is needed, but otherwise hl_action.ActionError is not found
# from rapprentice.PR2 import PR2, Arm, mirror_arm_joints
from hl_param import *
from utils import *
# from ll_plan import LLPlan
from ll_prob import LLProb
from fluents.fluent import AndFluent
# TODO: not sure if this dependency should be here
from fluents.not_obstructs import NotObstructs

try:
    import openrave_input
except ImportError:
    print "Warning: ROS imports failed. Okay if not using ROS."

JOINT_REF_PRIORITIES = [2]

class PlanRefinement(object):
    def __init__(self, env, world):
        self.env = env
        self.original_env = self.env.CloneSelf(1) # clones objects in the environment
        self.world = world
        self.hl_params = world.param_map.values()
        self.robot = self.env.GetRobots()[0]
        self.unmovable_objects = {self.env.GetKinBody('table'),
                                  self.env.GetKinBody('table6'),
                                  self.env.GetKinBody('walls'),
                                  self.env.GetKinBody('drawer_outer'),
                                  self.env.GetKinBody('drawer')}

        self.action_list = []
        self.saved_env_states = []  # the stored state at index n is the state of the environment before action n
        self.instantiation_generator = None
        self.resume_from_lineno = 0
        self.gp_counters = {}
        self.grasp_counters = {}
        self.pdp_counters = {}

    def reset_all(self):
        self.action_list = []
        self.saved_env_states = []
        self.instantiation_generator = None

    def set_resume_from(self, resume_from_lineno):
        print "Setting resume from: {}".format(resume_from_lineno)
        self.resume_from_lineno = resume_from_lineno
        self.saved_env_states = resume_from_lineno * [None]

    def prep_partial(self, resume_from=0):
        self.resume_from_lineno = resume_from
        self.action_list = self.action_list[:self.resume_from_lineno]
        self.instantiation_generator = None

    def reset_all_actions(self):
        self.reset_actions(self.action_list)

    def reset_actions(self, actions):
        for action in actions:
            action.reset()

    def remove_plots(self):
        for action in self.action_list:
            action.remove_plots()

    def get_all_but_params(self, params_to_delete):
        return self.world.get_all_but_params(params_to_delete)

    def get_next_instantiation(self):
        if self.instantiation_generator is None:
            if settings.BACKTRACKING_REFINEMENT:
                self.instantiation_generator = self._try_backtracking_refine()
            else:
                self.instantiation_generator = self._try_joint_refine()

        error = self.instantiation_generator.next()
        if error is not None:
            raise error

    # def backtracking_resample(self, sampled_params):
    #     for param in sampled_params:
    #         try:
    #             param.resample()
    #             break
    #         except StopIteration:
    #             param.reset_gen()
    #             param.resample()
    #             continue
    #     else:
    #         print "Need to raise an error"
    #         raise StopIteration

    def randomized_resample(self, sampled_params, violated_fluents, mode, count):
        if count >= 3:
            raise StopIteration
        lst = []
        for f in violated_fluents:
            for p in f.hl_action.params:
                if p in sampled_params and p not in lst:
                    lst.append(p)
        if not lst:
            return
        param_to_index = {}
        for p in lst:
            for i, a in enumerate(self.action_list):
                if p in a.params:
                    param_to_index[p] = i
                    break
        lst.sort(key=param_to_index.get)
        if mode == "first":
            try:
                lst[0].resample()
            except StopIteration:
                lst[0].reset_gen()
                lst[0].resample()
            return [lst[0]]
        elif mode == "random":
            settings.RANDOM_STATE.shuffle(lst)
            try:
                lst[0].resample()
            except StopIteration:
                lst[0].reset_gen()
                lst[0].resample()
            return [lst[0]]
        elif mode == "all":
            for p in lst:
                try:
                    p.resample()
                except StopIteration:
                    p.reset_gen()
                    p.resample()
            return lst
        elif mode == "fluent":
            lst = []
            for f in violated_fluents:
                for p in f.params_to_resample():
                    lst.append(p)
                    try:
                        p.resample()
                    except StopIteration:
                        p.reset_gen()
                        p.resample()
            return lst
        else:
            raise Exception("Invalid mode!")

    def _try_joint_refine(self):
        sampled_params = self.world.get_sampled_params()
        sampled_params.sort(key=lambda p: p.sample_priority)
        recently_sampled = sampled_params
        for param in sampled_params:
            param.resample()

        llprob = LLProb(self.action_list)
        llprob.solve_at_priority(-1, recently_sampled=recently_sampled)
        all_useful_fluents = set()
        count = 0
        while True:
            count += 1
            llprob.solve_at_priority(0, recently_sampled=recently_sampled)
            for priority in JOINT_REF_PRIORITIES:
                llprob.solve_at_priority(priority)
                if llprob.recently_converged_vio_fluent is not None:
                    # early converged based on a violated fluent
                    violated_fluents = [llprob.recently_converged_vio_fluent]
                else:
                    fluents = [f for a in self.action_list for f in a.preconditions + a.postconditions]
                    violated_fluents = self.find_violated_fluents(fluents, priority)
                if len(violated_fluents) == 0:
                    if priority == JOINT_REF_PRIORITIES[-1]:
                        self.total_cost = llprob.traj_cost
                        yield None
                    continue
                else:
                    try:
                        self.save_useful_fluents(violated_fluents, all_useful_fluents)
                        recently_sampled = self.randomized_resample(sampled_params, violated_fluents, mode="fluent", count=count)
                    except StopIteration:
                        for f in all_useful_fluents:
                            self.remove_plots()
                            yield f
                        if len(all_useful_fluents) == 0:
                            print "Resampling all params, no violated fluents to raise..."
                            for p in sampled_params:
                                try:
                                    p.resample()
                                except StopIteration:
                                    p.reset_gen()
                                    p.resample()
                        # reset set of useful fluents, since we yielded them all
                        all_useful_fluents = set()
                        count = 0
                    break # out of for loop

    def _try_backtracking_refine(self):
        # add the preconditions of the next action to the postconditions, for each action
        for i in range(len(self.action_list) - 1):
            a, a_next = self.action_list[i:i+2]
            for precon in a_next.preconditions:
                if precon.do_extension:
                    a.postconditions.append(precon)
                    for p in precon.extension_params:
                        if p not in a.params:
                            a.params.append(p)

        # find first action where every sampled param occurs
        sampled_params = self.world.get_sampled_params()
        sampled_params.sort(key=lambda p: p.sample_priority)
        actions_params = []
        last_ind_to_actions = {}
        seen = set()
        llprobs = {}
        for a in self.action_list:
            for p in sampled_params:
                if p in a.params and p not in seen:
                    actions_params.append((a, p))
                    seen.add(p)
            llprobs[a] = LLProb([a])
            last_ind_to_actions.setdefault(len(actions_params) - 1, []).append(a)

        i = 0
        all_useful_fluents = set()
        while i < len(actions_params):
            a, p = actions_params[i]
            # because we set this to False a few lines down
            if a.is_move():
                a.end.is_var = True
            try:
                p.resample()
            except StopIteration:
                if i == 0:
                    # backtracking exhausted
                    for f in all_useful_fluents:
                        self.remove_plots()
                        yield f
                    p.reset_gen()
                    # reset set of useful fluents, since we yielded them all
                    all_useful_fluents = set()
                    continue
                p.reset_gen()
                a.remove_plots()
                i -= 1
                continue
            if i not in last_ind_to_actions:
                i += 1
                continue
            violated_fluents = []
            successes = []
            for a in last_ind_to_actions[i]:
                # straight-line initialization
                straight_line_init_succ = llprobs[a].solve_at_priority(-1, fix_sampled_params=True)
                successes.append(straight_line_init_succ)
                vio_prio = -1
                # optimize -- fix sampled params, so they are not optimized over
                if straight_line_init_succ:
                    llprobs[a].solve_at_priority(2, fix_sampled_params=True)
                    vio_prio = 2
                # after optimizing a move, fix the end robot pose
                if a.is_move():
                    a.end.is_var = False
                # get violated fluents for this action
                violated_fluents.extend(self.find_violated_fluents([f for f in a.preconditions + a.postconditions], priority=vio_prio))
            if all(successes) and len(violated_fluents) == 0:
                i += 1
            else:
                # i stays the same
                self.save_useful_fluents(violated_fluents, all_useful_fluents)

        self.total_cost = sum(l.traj_cost for l in llprobs.values())
        yield None

    # TODO: whether a fluent is useful should be determined by domain file?
    def save_useful_fluents(self, fluents, all_useful_fluents):
        for fluent in fluents:
            # only movable object errors are useful
            if isinstance(fluent, NotObstructs) and fluent.obj.name in self.world.movable_objects:
                # don't raise obstruction for pick/place resulting from obj itself
                if fluent.obj.name not in fluent.hl_action.end.name:
                    all_useful_fluents.add(fluent)

    def find_violated_fluents(self, fluents, priority):
        # REQUIREMENT: this function assumes fluents are ordered by action, so
        # that propagate_useful_fluents will do the right thing
        violated_fluents = []
        for fluent in fluents:
            if fluent.priority > priority:
                continue
            if isinstance(fluent, AndFluent):
                violated_fluents.extend(self.find_violated_fluents(fluent.fluents, priority))

            if not np.all(fluent.satisfied()):
                violated_fluents.append(fluent)
        return violated_fluents

    def execute(self, speedup=1, pause=True):
        self.remove_plots()
        # make all objects fully opaque
        utils.set_transparency(self.robot, 0)
        tfs = {}
        tfs[self.robot.GetName()] = self.robot.GetTransform()
        for obj_name in self.world.movable_objects:
            utils.set_transparency(self.env.GetKinBody(obj_name), 0)
            tfs[obj_name] = self.env.GetKinBody(obj_name).GetTransform()

        if pause:
            raw_input("Press enter to run in simulation!")
        for action in self.action_list:
            if action.name.startswith("move"):
                T = self.robot.GetTransform()
                if action.obj:
                    obj = self.env.GetKinBody(action.obj.name)
                    obj_T = obj.GetTransform()
                for ts in range(action.traj.cols):
                    T[:3, 3] = action.traj.value[:, ts]
                    self.robot.SetTransform(T)
                    if action.obj:
                        assert action.traj.cols == action.obj_traj.cols
                        obj_T[:3, 3] = action.obj_traj.value[:, ts]
                        obj.SetTransform(obj_T)
                    time.sleep(0.02 / speedup)
            if action.name.startswith("pick"):
                utils.set_color(action.obj.get_env_body(self.env), [0, 1, 0])
            if action.name.startswith("place"):
                utils.set_color(action.obj.get_env_body(self.env), [0, 1, 1])

        # restore transparency
        utils.set_transparency(self.robot, 0.7)
        for obj_name in self.world.movable_objects:
            utils.set_transparency(self.env.GetKinBody(obj_name), 0.7)

        # restore transforms
        for n, t in tfs.items():
            self.env.GetKinBody(n).SetTransform(t)

    def setActionListNames(self, hlplan):
        self.action_list_names = hlplan.actionList
        self.action_effects_dict = hlplan.effectDict
        self.action_precond_dict = hlplan.preconditionDict
        self.state_list = hlplan.stateList

    def add_parent_action(self, lineno, action):
        self._add_action(lineno, action)

    def add_action(self, lineno, action_fn, *args):
        env = self.original_env.CloneSelf(1) # clones objects in the environment
        robot = env.GetRobots()[0].GetName() # TODO: fix this hack, make it part of pddl
        action = action_fn(*(lineno, self, env, robot) + args)
        self._add_action(lineno, action)

    def _add_action(self, lineno, action):
        if action.lineno == len(self.action_list):
            self.action_list.append(action)
        elif action.lineno < len(self.action_list):
            self.action_list[action.lineno] = action
        else:
            raise Exception("Bad action lineno: {}".format(action.lineno))
