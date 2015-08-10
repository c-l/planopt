import utils
import settings
import time
from interface.hl_actions import *
from interface.hl_actions.move import Move
from interface.hl_actions.pick import Pick
from interface.hl_actions.place import Place
# from hl_actions import hl_action # not sure why this is needed, but otherwise hl_action.ActionError is not found
# from rapprentice.PR2 import PR2, Arm, mirror_arm_joints
from interface.env_manager import EnvManager
from hl_param import *

from utils import *

import sys
sys.path.insert(0,"../")

from envs.world import World
from ll_plan import LLPlan

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
        self.hl_params = world.hl_params
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

    def save_openrave_state(self, n):
        if n < len(self.saved_env_states):
            self.saved_env_states[n] = EnvManager.save_openrave_state(self.env)
        elif n == len(self.saved_env_states):
            self.saved_env_states.append(EnvManager.save_openrave_state(self.env))
        else:
            raise Exception("Trying to save state with index {}, but \
              saved_env_states is only {} long!".format(n, len(self.saved_env_states)))

    def restore_openrave_state(self, n):
        # print 'RESETTING ENV TO STATE: {}'.format(n)
        with self.env:
            EnvManager.restore_openrave_state(self.env, self.saved_env_states[n])

    def reset_all_actions(self):
        self.reset_actions(self.action_list)

    def reset_actions(self, actions):
        for action in actions:
            action.reset()

    def replan_with_margins(self, starting_state):
        # import trajoptpy
        # self.viewer = trajoptpy.GetViewer(self.env)
        # trajoptpy.SetInteractive(True)
        EnvManager.restore_openrave_state(self.env, starting_state)
        for action in self.action_list:
            action.replan_with_margins(settings.INCREASE_MARGINS)
            action.execute_trajevents()

    def execute_all(self, starting_state):
        if settings.INCREASE_MARGINS:
            for action in self.action_list:
                action.replan_with_margins(True)
        while True:
            EnvManager.restore_openrave_state(self.env, starting_state)
            raw_input("Run in sim!")
            for action in self.action_list:
                action.execute_trajevents(sim_only=False)
            again = raw_input("Again? ([y]/n)")
            if again == 'n':
                break

    def get_next_instantiation(self):
        if self.instantiation_generator is None:
            self.instantiation_generator = self._try_refine()

        error = self.instantiation_generator.next()
        if error is not None:
            raise error

    def _try_refine(self):
        # TODO: rewrite
        num_boxes=1

        init_later_actions = []
        for action in self.action_list:
            if "pick" in action.name:
                action.init_opt()
            elif "place" in action.name:
                action.init_opt()
            else:
                init_later_actions.append(action)
        params = self.hl_params.values()

        for param in params:
            param.dual_update()
            param.initialize_to_consensus()

        for hl_action in self.action_list:
            hl_action.plot()

        for action in init_later_actions:
            action.init_opt()

        for hl_action in self.action_list:
            hl_action.plot()

        llplan = LLPlan(params, self.action_list)
        llplan.solve()
        
        yield None
        # print "\r\nTrying to find error-free instantiation..."
        # self.reset_actions(self.action_list[self.resume_from_lineno:])
        # step_for_ik = self.resume_from_lineno

        # if len(self.saved_env_states) == self.resume_from_lineno:
        #     self.save_openrave_state(step_for_ik)

    def clone_envs(self, num_actions):
        local_envs = []

        for i in range(num_actions):
            env = self.env.CloneSelf(1) # clones objects in the environment
            local_envs.append(env)
        return local_envs

    def pick_place_boxes(self):
        # self.env = World().generate_room_env()
        num_boxes = 1
        fake_env, target_loc_values = World().generate_boxes_env(num=num_boxes)
        target_locs = []
        i=0
        for loc in target_loc_values:
            target_locs.append(ObjLoc("target"+"_loc"+str(i), 3, 1, is_var=False, value=mat_to_base_pose(loc)))
            i += 1

        self.robot = self.env.GetRobots()[0]
        self.num_actions = 4*len(target_locs)
        # self.ro = 0.02
        # self.ro = 0.2
        self.ro = 2
        # self.ro = 10
        # self.ro = 20
        # self.ro = 300

        hla_envs = self.clone_envs(self.num_actions)

        rps = []
        gps = []
        objs = []
        obj_locs =[]
        for i in range(num_boxes):
            gps.append(GP("gp"+str(i), 3, 1, ro=self.ro))
            obj = self.env.GetKinBody('obj'+str(i))
            objs.append(obj)
            obj_locs.append(ObjLoc("obj"+str(i)+"_loc", 3, 1, is_var=False, value=mat_to_base_pose(obj.GetTransform())))

        robot_positions = 1 + 2*num_boxes
        for i in range(robot_positions):
            if i == 0:
                rps.append(RP("rp_init", 3, 1, is_var=False, value=mat_to_base_pose(self.robot.GetTransform())))
            else:
                rps.append(RP("rp"+str(i), 3, 1, ro=self.ro))


        rp_index=1
        for i in range(num_boxes):
            lineno = 4*i
            env = hla_envs[lineno]
            robot = env.GetRobots()[0]
            # obj = env.GetKinBody(objs[i].GetName())
            obj = Movable(objs[i].GetName())
            pick = self.add_action(Pick(lineno, self, env, robot, rps[rp_index], obj, obj_locs[i], gps[i], name="pick"+str(i)))
            lineno = 1+4*i
            env = hla_envs[lineno]
            robot = env.GetRobots()[0]
            # obj = env.GetKinBody(objs[i].GetName())
            obj = Movable(objs[i].GetName())
            place = self.add_action(Place(lineno, self, env, robot, rps[rp_index+1], obj, target_locs[i], gps[i], name="place"+str(i)))
            rp_index += 2

        for rp in rps:
            rp.dual_update()
            rp.initialize_to_consensus()

        for gp in gps:
            gp.dual_update()
            gp.initialize_to_consensus()

        for hl_action in self.action_list:
            hl_action.plot()

        # import ipdb; ipdb.set_trace() # BREAKPOINT
        for i in range(num_boxes):
            lineno = 2+4*i
            env = hla_envs[lineno]
            robot = env.GetRobots()[0]
            move1 = self.add_action(Move(lineno, self, env, robot, rps[2*i], rps[2*i+1], name="move"+str(2*i+1)))

            lineno = 4*i+3
            env = hla_envs[lineno]
            robot = env.GetRobots()[0]
            # obj = env.GetKinBody(objs[i].GetName())
            obj = Movable(objs[i].GetName())
            move2 = self.add_action(Move(lineno, self, env, robot, rps[2*i+1], rps[2*i+2], obj, gps[i], "move"+str(2*i+2)))

        params = rps + gps
        llplan = LLPlan(params, self.action_list)
        llplan.solve()
    
    def setActionListNames(self, hlplan):
        self.action_list_names = hlplan.actionList
        self.action_effects_dict = hlplan.effectDict
        self.action_precond_dict = hlplan.preconditionDict
        self.state_list = hlplan.stateList

    def add_action(self, lineno, action_fn, *args):
        env = self.original_env.CloneSelf(1) # clones objects in the environment
        robot = env.GetRobots()[0]
        action = action_fn(*(lineno, self, env, robot) + args)
        # self.add_action(action)
        if action.lineno == len(self.action_list):
            self.action_list.append(action)
        elif action.lineno < len(self.action_list):
            self.action_list[action.lineno] = action
        else:
            raise Exception("Bad action lineno: {}".format(action.lineno))

