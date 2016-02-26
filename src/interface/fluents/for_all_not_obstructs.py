from fluent import AndFluent
from not_obstructs import NotObstructs
import ctrajoptpy

class ForAllNotObstructs(AndFluent):
    def __init__(self, env, world_state, hl_action, robot, priority, traj, objs, dsafe=0.05):
        self.name = "ForAllNotObstructs"
        self.env = env
        self.world_state = world_state
        self.hl_action = hl_action
        self.robot = robot
        self.priority = priority

        self.K = self.hl_action.K
        self.T = self.hl_action.T
        self.cc = ctrajoptpy.GetCollisionChecker(env)
        self.dsafe = dsafe

        self.fluents = []
        for obj in objs:
            obj_loc = None
            if obj in world_state:
                obj_loc = world_state[obj]
            fluent = NotObstructs(env, hl_action, robot, priority, traj, obj, obj_loc)
            self.fluents.append(fluent)

    def pre(self):
        for fluent in self.fluents:
            fluent.pre()
