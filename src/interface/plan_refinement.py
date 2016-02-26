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

    def backtracking_resample(self, sampled_params):
        for param in sampled_params:
            try:
                param.resample()
                break
            except StopIteration:
                param.reset_gen()
                param.resample()
                continue
        else:
            print "Need to raise an error"
            raise StopIteration

    def _try_joint_refine(self):
        index = 0

        sampled_params = self.world.get_sampled_params()
        for param in sampled_params:
            param.index = index
            param.resample()
            index += 1
        sampled_params.sort(key=lambda f: -f.index)

        llprob = LLProb(self.action_list)
        llprob.solve_at_priority(-1, fix_sampled_params=True)
        while True:
            llprob.solve()

            fluents = [f for a in self.action_list for f in a.preconditions + a.postconditions]
            violated_fluents = self.find_violated_fluents(fluents)
            if len(violated_fluents) == 0:
                yield None
            else:
                try:
                    self.backtracking_resample(sampled_params)
                except StopIteration:
                    self.propagate_useful_fluent(violated_fluents)

    def _try_backtracking_refine(self):
        # add the preconditions of the next action to the postconditions, for each action
        for i in range(len(self.action_list) - 1):
            a, a_next = self.action_list[i:i+2]
            if "move" in a.name:
                for v in a.world_state.values():
                    if v not in a.params:
                        a.params.append(v)
            for precon in a_next.preconditions:
                if hasattr(precon, "do_extension") and precon.do_extension:
                    a.postconditions.append(precon)
                    if precon.extension_param not in a.params:
                        a.params.append(precon.extension_param)
                    if hasattr(precon, "traj") and precon.traj not in a.params:
                        a.params.append(precon.traj)
                    if hasattr(precon, "obj_traj") and precon.obj_traj not in a.params:
                        a.params.append(precon.obj_traj)

        # find first action where every sampled param occurs
        sampled_params = self.world.get_sampled_params()
        action_to_param = dict([a.name, []] for a in self.action_list)
        for p in sampled_params:
            for a in self.action_list:
                if p in a.params:
                    action_to_param[a.name].append(p)
                    break

        llprobs = [LLProb([a]) for a in self.action_list]

        i = 0
        violated_fluents = None
        while i < len(self.action_list):
            a = self.action_list[i]
            if "move" in a.name:
                a.end.is_var = True
            for p in action_to_param[a.name]:
                try:
                    p.resample()
                except StopIteration:
                    if i == 0:
                        # backtracking exhausted
                        self.propagate_useful_fluent(violated_fluents)
                        raise Exception("Should not get here because error propagation should occur!")
                    p.reset_gen()
                    p.resample()
                    # move index back to STRICTLY previous action which has a resampled param
                    self.action_list[i].remove_plots()
                    i -= 1
                    while not action_to_param[self.action_list[i].name]:
                        self.action_list[i].remove_plots()
                        i -= 1
                    break
            else:
                # straight-line initialization
                llprobs[i].solve_at_priority(-1, fix_sampled_params=True)
                # optimize -- fix sampled params, so they are not optimized over
                llprobs[i].solve_at_priority(1, fix_sampled_params=True)
                # after optimizing a move, fix the end robot pose
                if "move" in a.name:
                    a.end.is_var = False

                fluents = [f for f in a.preconditions + a.postconditions]
                violated_fluents = self.find_violated_fluents(fluents)
                if len(violated_fluents) == 0:
                    i += 1
                else:
                    # move index back to previous OR CURRENT action which has a resampled param
                    while not action_to_param[self.action_list[i].name]:
                        self.action_list[i].remove_plots()
                        i -= 1

        yield None

    # TODO: whether a fluent is useful should be determined by domain file?
    def propagate_useful_fluent(self, fluents):
        for fluent in fluents:
            if isinstance(fluent, NotObstructs):
                # TODO: put remove_plots in a better location
                self.remove_plots()
                raise fluent

    def find_violated_fluents(self, fluents):
        # REQUIREMENT: this function assumes fluents are ordered by action, so
        # that propagate_useful_fluents will do the right thing
        violated_fluents = []
        for fluent in fluents:
            if isinstance(fluent, AndFluent):
                violated_fluents.extend(self.find_violated_fluents(fluent.fluents))

            if not np.all(fluent.satisfied()):
                violated_fluents.append(fluent)
        return violated_fluents

    def execute(self, speedup=1):
        self.remove_plots()
        # make all objects fully opaque
        utils.set_transparency(self.robot, 0)
        for obj_name in self.world.movable_objects:
            utils.set_transparency(self.env.GetKinBody(obj_name), 0)

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
