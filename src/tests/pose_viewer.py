#Allows you to view poses

import sys
sys.path.append('../src')
sys.path.append('../src/interface')
sys.path.append("../tests")
from settings import *
import settings

import openravepy
import interface
import utils
import rapprentice.kinematics_utils as ku
from rapprentice.PR2 import Arm, mirror_arm_joints
from interface.pose_generators import env_posegens

from interface.env_manager import EnvManager

#from openravepy import IkParameterizationType, IkParameterization, raveLogInfo, IkFilterOptions
import numpy as np
import pickle

import getopt
import time
import ipdb

class PoseViewer:
    def __init__(self, envFile):
        #If you want to run this file, ignore this function and go modify _setup_pose_gen, _view_poses_helper, and _setup_environment
        if type(envFile) == str:
            self.env = openravepy.Environment()
            self.env.StopSimulation()
            self.env.SetViewer('qtcoin')
            self.env.Load(envFile)
        else:
            self.env = envFile

        self.robot = self.env.GetRobots()[0]
        EnvManager.init_openrave_state(self.env)

        # loading the IK models
        self.robot.SetActiveManipulator('leftarm')
        ikmodel = openravepy.databases.inversekinematics.InverseKinematicsModel(
            self.robot, iktype=openravepy.IkParameterization.Type.Transform6D)
        if not ikmodel.load():
            ikmodel.autogenerate()
        self.robot.SetActiveManipulator('rightarm')
        ikmodel = openravepy.databases.inversekinematics.InverseKinematicsModel(
            self.robot, iktype=openravepy.IkParameterization.Type.Transform6D)
        if not ikmodel.load():
            ikmodel.autogenerate()

        utils.set_rave_limits_to_soft_joint_limits(self.robot)
        # start arms on the side
        self._reset_arms()
        # start torso up
        self._reset_torso()
        self.initial_robot_t = np.identity(4)

        self._setup_environment()
        #self._setup_pose_gen()
        self._setup_action()
        raw_input("Press Enter to start!")


    def _reset_arms(self):
        #Arms on the side
        left_joints = Arm.L_POSTURES['side2']
        right_joints = mirror_arm_joints(left_joints).tolist()
        self.robot.SetDOFValues(
            right_joints + left_joints,
            self.robot.GetManipulator("rightarm").GetArmIndices().tolist() +
            self.robot.GetManipulator("leftarm").GetArmIndices().tolist())

    def _reset_torso(self):
        # start torso up
        v = self.robot.GetActiveDOFValues()
        v[self.robot.GetJoint('torso_lift_joint').GetDOFIndex()] = 0.3
        self.robot.SetDOFValues(v)

    def _reset_base(self):
        self.robot.SetTransform(self.initial_robot_t)

    def _setup_environment(self):
        """Environment variables that are NOT specific to any pose generator go here"""
        #Add object that are unmovable to this set
        #This should match the one in 'PlanRefinement.__init__()' in 'plan_refinement.py'
        self.unmovable_objects = {self.env.GetKinBody('table'),
                                  self.env.GetKinBody('table6'),
                                  self.env.GetKinBody('walls'),
                                  self.env.GetKinBody('drawer_outer'),
                                  self.env.GetKinBody('drawer')
                                 }

    def _setup_action(self):
        """
        If you want to view poses from a specific action, you must set up that up here.

        1) Openup 'plan_refinement.py' and see how your action is imported
            EX: from hl_actions.putdown_action import PutdownAction
        2) Scroll down to see how to initialize your action class
            EX: PutdownAction(self, self.pr2, self.unmovable_objects, lineno, obj, location_str, manip, pose_name))
        2) Wi
            EX: self.putdown_pose_generator = env_posegens.get_putdown_pose_generator(self.env, self.unmovable_objects, manip=self.manip, obj=self.obj, surface=self.table)
        3) Figure out what arguments the function takes and initialize those arguments here
        4) Get the pose generator
        """
        #Leave these alone:
        self.pr2 = None
        lineno = 0
        pose_name = None



        ##### Import your action here #####
        settings.DOMAIN = settings.KITCHEN_DOMAIN
        from hl_actions.putdown_action import PutdownAction
        from hl_actions.pickfromwasher_action import PickFromWasherAction
        from hl_actions.pickup_action import PickupAction
        from hl_actions.door_action import DoorAction
        from hl_actions.movetogp_action import MovetoGPAction

        ##### DEFINE VARIABLES FOR YOUR ACTION HERE #####
        #Define the object you are manipulating
        self.objName = 'object1'
        #self.objName = 'cloth1'
        
        #For pickup/putdown actions, define the surface here
        #self.location_str = 'microwave'
        self.location_str = 'fridgewide'
        #self.location_str = 'fridgewideshelf1'
        #self.location_str = 'washer'
        #self.location_str = 'dishwasher'
        #hingeID = 1
        hingeID = None
        
        #Set the active manipulator here
        self.manip_name = 'rightarm'

        if not self.env.GetKinBody(self.objName):
            print self.objName +" not in current env!"
            exit()

        if not self.env.GetKinBody(self.location_str):
            print self.location_str +" not in current env!"
            exit()

        self.obj = self.env.GetKinBody(self.objName)
        obj = self.obj

        self.location = self.env.GetKinBody(self.location_str)
        location = self.location

        #x_offset = -0.5 #Washer
        #y_offset = 0.25 #Washer
        #x_offset = 0.7 #Microwave
        #y_offset = 0.4 #Microwave
        #rot_amount = -np.pi #Microwave
        #x_offset = -0.4 #Fridge
        #y_offset = -0.7 #Fridge
        #rot_amount = np.pi/2 #Fridge
        x_offset = 2
        y_offset = 2
        rot_amount = 1
        rotation = openravepy.matrixFromAxisAngle([0,0,rot_amount])
        loc_t = self.location.GetTransform()
        self.initial_robot_t = np.eye(4)
        self.initial_robot_t[0,3] = loc_t[0,3]+x_offset
        self.initial_robot_t[1,3] = loc_t[1,3]+y_offset
        self.initial_robot_t = self.initial_robot_t.dot(rotation)
        self.robot.SetTransform(self.initial_robot_t)

        print "Adjust the initial robot position!"
        import IPython; IPython.embed()

        location_str = self.location_str
        #If placing into an appliance don't forget to open it up to see what's inside
        #self.env.GetKinBody(self.location_str).SetDOFValues([2.7]) #Limits are defined in the xml file, could be negative, could be positive
        #self.env.GetKinBody(self.location_str).SetDOFValues([-2.7])
        openDoor = False

        self.manip = self.robot.GetManipulator(self.manip_name)
        manip_name = self.manip_name


        ##### Initialize your action here #####
        #Instead of manip, pass in the manip name!!
        #self.action = PutdownAction(self, self.pr2, self.unmovable_objects, lineno, obj, location_str, manip_name, pose_name)
        #self.action = PickFromWasherAction(self, self.pr2, self.unmovable_objects, lineno, obj, location, manip_name, pose_name)
        #self.action = PickupAction(self, self.pr2, self.unmovable_objects, lineno, obj, manip_name, pose_name)
        #self.action = DoorAction(self, self.pr2, self.unmovable_objects, lineno, location, manip_name, pose_name, openDoor, hingeID=hingeID)
        self.action = MovetoGPAction(self, self.pr2, self.unmovable_objects, lineno, location, manip_name, None, pose_name)

        #Leave these alone:
        self.action._init_data_generator()

        self.action._construct_action = lambda x, y: []

        self.target_poses_list = []
        try:
            while True:
                self.target_poses_list.append(self.action.target_generator.next())
        except StopIteration:
            pass

        self._count_successful_ik(self.action,self.target_poses_list)

    def _count_successful_ik(self,action,poses_list):
        totalBasePoses = len(poses_list)
        IkPosesPerSet = 0.0
        currSet = poses_list[0]
        action.get_trajevents(currSet,None)
        if action.action_refinements:
            for currPose in action.action_refinements:
                if type(currPose) is interface.action_refinement.TrajActionRefinementFromPose:
                    IkPosesPerSet += 1
        
        numBaseSucc = 0.0
        numIkSucc = 0.0

        for currSet in poses_list:
            action.get_trajevents(currSet,None)
            self._reset_arms()
            self._reset_torso()
            self._reset_base()
            if action.action_refinements:
                for currPose in action.action_refinements:
                    #baseCollision is a hack to break out of two for loops
                    baseCollision = False
                    if type(currPose) is interface.action_refinement.TrajActionRefinementFromPose:
                        for bodypart, pose in sorted(currPose._get_world_frame_pose_by_bodypart().items(), key=lambda x: x[0] != "base"):
                            dofs = self._find_ik_helper(action, bodypart, pose)
                            if bodypart == 'base':
                                if dofs is None:
                                    baseCollision = True
                                    break
                                else:
                                    numBaseSucc += 1
                                    self.set_robot_dofs(bodypart,dofs)
                            else:
                                if dofs is not None:
                                    numIkSucc += 1
                    if type(currPose) is interface.action_refinement.TrajActionRefinementFromDofs:
                        if 'torso' in currPose.dofs_by_bodypart:
                            self.set_robot_dofs('torso',currPose.dofs_by_bodypart['torso'])
                            collisions = action.collision_checker.get_collisions()
                            if collisions:
                                break
                    if baseCollision:
                        break
        if numBaseSucc != 0.0:
            print "Base poses with no collisions: %.2f%%" %(numBaseSucc/totalBasePoses*100)
        else:
            print "Ether base poses are NOT part of the action, or they all failed."
        if IkPosesPerSet != 0.0:
            #The % of successful IK's is an OVER estimation!
            print "Successful IK's: %.2f%%" %(numIkSucc/IkPosesPerSet/totalBasePoses*100)

    def view_poses(self):
        import ipdb
        setNum = 0
        currSet = self.target_poses_list[setNum]
        self.action.get_trajevents(currSet,None)

        poseNum = 0
        poseNum = self.find_next_pose_index(self.action,poseNum)
        currPose = self.action.action_refinements[poseNum]

        points = self.view_pose(self.action,currPose)

        s = 'a'
        while s != 'q':
            time.sleep(0.1)
            with self.env:
                openravepy.raveLogInfo('setNum = ' + str(setNum) +', pose = ' + str(currPose.debug_str))
                s = raw_input('s for next set of poses, n for next pose in set, q for quit\n    ')
                if s == 's':
                    self._reset_arms()
                    self._reset_torso()
                    self._reset_base()

                    setNum = (setNum+1) % len(self.target_poses_list)
                    currSet = self.target_poses_list[setNum]
                    self.action.get_trajevents(currSet,None)

                    self._find_and_set_torso_height(currSet)

                    #Find 1'st valid pose
                    poseNum = 0
                    poseNum = self.find_next_pose_index(self.action,poseNum)
                    currPose = self.action.action_refinements[poseNum]
                elif s == 'n':
                    poseNum = (poseNum+1) % len(self.action.action_refinements)
                    nextPoseNum = self.find_next_pose_index(self.action,poseNum)
                    if nextPoseNum < poseNum:
                        self._reset_arms()
                        self._reset_torso()
                        self._reset_base()
                    poseNum = nextPoseNum
                    currPose = self.action.action_refinements[poseNum]

                points = self.view_pose(self.action,currPose)

    def find_next_pose_index(self,action,poseNum):
        while type(action.action_refinements[poseNum]) is not interface.action_refinement.TrajActionRefinementFromPose:
            poseNum = (poseNum+1) % len(action.action_refinements)
        if 'Dofs' in str(type(action.action_refinements[poseNum])) and 'torso' not in action.action_refinements[poseNum].dofs_by_bodypart:
            poseNum = (poseNum+1) % len(action.action_refinements)
            poseNum = self.find_next_pose_index(action,poseNum)
        return poseNum

    def view_pose(self,action,currPose):
        points = []
        for bodypart, pose in sorted(currPose._get_world_frame_pose_by_bodypart().items(), key=lambda x: x[0] != "base"):
            dofs = self._find_ik_helper(action, bodypart, pose)
            if dofs is not None:
                self.set_robot_dofs(bodypart,dofs)
            else:
                points.append(utils.plot_transform(self.env,pose))
        return points

    def _find_ik_helper(self, hl_action, bodypart, pose):
        if bodypart == 'base':
            starting_t = hl_action.robot().GetTransform()
            hl_action.robot().SetTransform(pose)
            collisions = hl_action.collision_checker.get_collisions()
            success = not collisions
            hl_action.robot().SetTransform(starting_t)
            if success:
                return list(utils.mat_to_base_pose(pose))
        else:
            manip_to_use = hl_action.robot().GetManipulator(bodypart)
            ik = manip_to_use.FindIKSolution(
                pose, openravepy.IkFilterOptions.CheckEnvCollisions)
            success = ik is not None
            if success:
                armIndices = manip_to_use.GetArmIndices()
                currJoints = hl_action.robot().GetDOFValues(armIndices)
                return ku.closer_joint_angles(ik, currJoints)

    def _find_and_set_torso_height(self,currSet):
        for currPose in currSet:
            if type(currPose) is interface.action_refinement.TrajActionRefinementFromDofs:
                if 'torso' in currPose.dofs_by_bodypart:
                    self.set_robot_dofs('torso',currPose.dofs_by_bodypart['torso'])

    def set_robot_dofs(self,bodypart,dofs):
        if bodypart == 'base':
            self.env.GetRobot('pr2').SetTransform(utils.base_pose_to_mat(dofs))
        elif bodypart == 'torso':
            self.env.GetRobot('pr2').SetDOFValues(dofs, [self.env.GetRobot('pr2').GetJoint('torso_lift_joint').GetDOFIndex()])
        else:
            self.env.GetRobot('pr2').SetDOFValues(
                dofs, self.env.GetRobot('pr2').GetManipulator(bodypart).GetArmIndices())

if __name__ == "__main__":
    usage_str = "Usage: python %s -e [env_filename]" %sys.argv[0]
    envFile = settings.ENV_PATH+settings.DEFAULT_ENV_FILE 

    try:
        opts, args = getopt.getopt(sys.argv[1:],"e:p:")
    except getopt.GetoptError:
        print(usage_str)
        sys.exit(2)

    init_settings()
    settings.REPORT_PUTDOWN_OBSTRUCTIONS = False

    for opt, arg in opts:
        if opt == "-e":
            print("Using env file: %s" %arg)
            envFile = arg

    viewer = PoseViewer(envFile)
    viewer.view_poses()
