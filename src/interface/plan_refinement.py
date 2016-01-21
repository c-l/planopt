import utils
import settings
import time
from interface.hl_actions import *
from interface.hl_actions.move import Move
from interface.hl_actions.pick import Pick
from interface.hl_actions.place import Place
# from hl_actions import hl_action # not sure why this is needed, but otherwise hl_action.ActionError is not found
# from rapprentice.PR2 import PR2, Arm, mirror_arm_joints
from hl_param import *
from utils import *
# from ll_plan import LLPlan
from ll_prob import LLProb

# TODO: not sure if this dependency should be here
from interface.fluents.fluent import AndFluent

import random

try:
    import openrave_input
except:
    print "Warning: ROS imports failed. Okay if not using ROS."


class PlanRefinement(object):
    def __init__(self, env, world):
        self.env = env
        self.original_env = self.env.CloneSelf(1) # clones objects in the environment
        self.world = world
        # self.hl_params = {}
        self.hl_params = world.param_map.values()
        # self.sampled_params = world.params_to_sample
        self.robot = self.env.GetRobots()[0]



        # self.use_ros = settings.use_ros

        # self.timings = {
        #   'mp': 0,
        #   'backtrack': 0,
        # }

        # if self.use_ros:
        #     self.pr2 = PR2(self.robot())
        # else:
        #     self.pr2 = None

        self.unmovable_objects = {self.env.GetKinBody('table'),
                                  self.env.GetKinBody('table6'),
                                  self.env.GetKinBody('walls'),
                                  self.env.GetKinBody('drawer_outer'),
                                  self.env.GetKinBody('drawer')}

        self.action_list = []
        self.saved_env_states = []  # the stored state at index n is the state of the environment before action n
        ##TODO[SS]: clean up: where is instantiation_generator set
        self.instantiation_generator = None
        self.resume_from_lineno = 0

        self.pick_counters = {}
        self.place_counters = {}
        # self.mp_completed_index = 0
        # self.last_target_cache = {}

    # def robot(self):
    #     return self.env.GetRobot('pr2')

    # def robot(self):
    #     return self.env.GetRobots()[0]

    def reset_all(self):
        self.action_list = []
        self.saved_env_states = []
        self.instantiation_generator = None
        # self.resume_from_lineno = 0
        # self.mp_completed_index = 0

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

    def clean_actions(self):
        for action in self.action_list:
            action.clean()

    def get_all_but_params(self, params_to_delete):
        return self.world.get_all_but_params(params_to_delete)

    def get_next_instantiation(self):
        if self.instantiation_generator is None:
            self.instantiation_generator = self._try_refine()

        import ipdb; ipdb.set_trace()
        error = self.instantiation_generator.next()
        if error is not None:
            raise error

    def backtracking_resample(self, sampled_params):
        hl_params = sampled_params
        hl_params.sort(key=lambda f: -f.index)

        for param in hl_params:
            try:
                param.resample()
                break
            except StopIteration:
                param.reset_gen()
                param.resample()
                continue
        else:
            print 'Need to raise an error'
            raise StopIteration

    def _try_refine(self):
        # TODO: rewrite
        initializations = 20
        index = 0

        sampled_params = self.world.get_sampled_params()
        for param in sampled_params:
            param.index = index
            param.resample()
            index += 1

        for _ in range(initializations):
            llprob = LLProb(self.action_list)
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
                    import ipdb; ipdb.set_trace()

        import ipdb; ipdb.set_trace() # BREAKPOINT
        yield None

    # TODO: whether a fluent is useful should be determined by domain file?
    def propagate_useful_fluent(self, fluents):
        for fluent in fluents:
            from fluents.not_obstructs import NotObstructs
            if isinstance(fluent, NotObstructs):
                # TODO: put clean actions in a better location
                import ipdb; ipdb.set_trace()
                self.clean_actions()
                raise fluent

    def find_violated_fluents(self, fluents):
        violated_fluents = []
        for fluent in fluents:
            if isinstance(fluent, AndFluent):
                violated_fluents.extend(self.find_violated_fluents(fluent.fluents))

            if not np.all(fluent.satisfied()):
                # violated_action = fluent.hl_action
                violated_fluents.append(fluent)
                print fluent, " is violated"
        return violated_fluents

    # def find_violated_fluents(self):
    #     violated_fluents = []
    #
    #     for action in self.action_list:
    #         for fluent in action.preconditions + action.postconditions:
    #             # if not fluent.satisfied():
    #             if not np.all(fluent.satisfied()):
    #                 violated_action = action.name
    #                 violated_fluents.append(fluent)
    #                 print violated_action, "'s fluent:", fluent, "violates constraints"
    #
    #     if len(violated_fluents) == 0:
    #         return None
    #     else:
    #         return violated_fluents

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
        # self.add_action(action)
        self._add_action(lineno, action)

    def _add_action(self, lineno, action):
        if action.lineno == len(self.action_list):
            self.action_list.append(action)
        elif action.lineno < len(self.action_list):
            self.action_list[action.lineno] = action
        else:
            raise Exception("Bad action lineno: {}".format(action.lineno))
