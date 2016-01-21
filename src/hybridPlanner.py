import StringIO
from task_planner.PDDLPatcher import *
import sys, subprocess, time, os
# from interface import planning_primitives
from interface import pr_graph, graph_traversal
from task_planner.HLPlan import *
from settings import *
import settings
import getopt
import utils
import re
import copy
from task_planner import PlannerOps
import shutil
import IPython
import openravepy
import numpy as np
# from rapprentice.PR2 import PR2, Arm, mirror_arm_joints
# from interface.plan_refinement import RolloutError

totalExecTime = 0
#myPatcher = None
import ipdb


class HybridPlanner:
    def __init__(self, pddlDomainFile, pddlDomainFileNoGeomEff,
                 initialProblemFile, viewer, envFile, planner=FF, pw_file=False):
        self.pid = repr(os.getpid())
        self.initialProblemFile = initialProblemFile
        self.editedProblemFile = DOMAIN_PATH+"temp/incremental_temp_" + \
            initialProblemFile.replace(DOMAIN_PATH, "").replace(".pddl",  \
                                                                    "_" + self.pid + ".pddl")
        dir = os.path.dirname(self.editedProblemFile)
        if not os.path.exists(dir):
            os.makedirs(dir)
        shutil.copyfile(self.initialProblemFile, self.editedProblemFile)
        tempFileList = [fname for fname in os.listdir(DOMAIN_PATH) if "temp/incremental_temp_" in fname]
        print "Number of temp files: " + repr(len(tempFileList))
        if len(tempFileList) > 25:
            print "\n\n"
            print "Consider cleaning up using: \n rm " + DOMAIN_PATH+"temp/incremental_temp_*"
            print "\n\n"
            time.sleep(2)

        self.pddlDomainFile = pddlDomainFile
        self.pddlDomainFileNoGeomEff = pddlDomainFileNoGeomEff
        self.iteration = 0
        self.replanCount = 0
        self._init_openrave(viewer, envFile, pw_file)

        # if run_test_mode[0]:
        #     pass
        # else:
        #     if 'robotics' in self.pddlDomainFile:
        #         goalObject = raw_input("Enter object to pick up, or press return for default:")
        #         if len(goalObject) > 0:
        #             self.editedProblemFile = utils.setGoalObject(goalObject, self.editedProblemFile)

        self.problemFileList = [self.editedProblemFile]
        self.patcher = PDDLPatcher(self.editedProblemFile)
        self.plannerName = planner
        self.planList = []
        self.currentPlanIndex = -1
        self.cacheClearCount = 0
        self.pr_graph = pr_graph.PRGraph(self.env)

    def _init_openrave(self, viewer=False, envFile=None, pw_file=False):
        if type(envFile) == str:
            self.env = openravepy.Environment()
            self.env.StopSimulation()
            if viewer:
                self.env.SetViewer('qtcoin')
            self.env.Load(envFile)
        else:
            self.env = envFile

        fname = "pw_file.txt"
        if pw_file:
          if os.path.isfile(fname):
            with open(fname, "r") as f:
              transforms = eval(f.read())
            for b in self.env.GetBodies():
              if b.GetName() in transforms.keys():
                b.SetTransform(openravepy.matrixFromPose(eval(transforms[b.GetName()])))
          else:
            print("Could not find pw_file.txt, keeping transforms as is")

        robot = self.env.GetRobots()[0]
        # EnvManager.init_openrave_state(self.env)

        # PR2 robot intialization
        # # loading the IK models
        # robot.SetActiveManipulator('leftarm')
        # ikmodel = openravepy.databases.inversekinematics.InverseKinematicsModel(
        #     robot, iktype=openravepy.IkParameterization.Type.Transform6D)
        # if not ikmodel.load():
        #     ikmodel.autogenerate()
        # robot.SetActiveManipulator('rightarm')
        # ikmodel = openravepy.databases.inversekinematics.InverseKinematicsModel(
        #     robot, iktype=openravepy.IkParameterization.Type.Transform6D)
        # if not ikmodel.load():
        #     ikmodel.autogenerate()

        # # utils.set_rave_limits_to_soft_joint_limits(robot)
        # # start arms on the side
        # left_joints = Arm.L_POSTURES['side2']
        # right_joints = mirror_arm_joints(left_joints).tolist()
        # # robot.SetDOFValues(
        #     # right_joints + left_joints,
        #     # robot.GetManipulator("rightarm").GetArmIndices().tolist() +
        #     # robot.GetManipulator("leftarm").GetArmIndices().tolist())
        # # start torso up
        # v = robot.GetActiveDOFValues()
        # v[robot.GetJoint('torso_lift_joint').GetDOFIndex()] = 0.3
        # # robot.SetDOFValues(v)

        if not run_test_mode[0]:
          raw_input("Press Enter to start!")

    def updatePlan(self, generatedPlan):
        self.planList.append(generatedPlan)
        self.currentPlanIndex += 1


    def getCurrentPlan(self):
        return self.planList[self.currentPlanIndex]



    def getNewPlan(self, pddlProblemFile, oldPlan, origState, failureStep, replanMode, \
                   resumeFrom, plannerOutFname):
        strPlanFileH, plannerOutStr, planCount = self.runPlanner(pddlProblemFile, \
                                                                 plannerOutFname, replanMode)
        if strPlanFileH == -1:
            reinterpreted = terminateOrReinterpret()
            generatedPlan = self.getCurrentPlan()
            print "Retrying Plan (from {0}):".format(resumeFrom)
            generatedPlan.printPlan()
        else:
            saved_info = copy.copy(strPlanFileH), plannerOutStr, planCount, self.iteration, plannerOutFname
            stateList = []
            if self.plannerName == MP or self.plannerName == FD:
                stateList = self.planner.getStateList()
            generatedPlan = HLPlan(self.pddlDomainFile, self.pddlDomainFileNoGeomEff,
                pddlProblemFile, strPlanFileH, plannerOutStr,
                self.plannerName, planCount, stateList,
                origState)
            # print "Generated plan: "
            # generatedPlan.printPlan()
            # print "Trimmed plan:"

            # generatedPlan.trimActions()

            # generatedPlan.printPlan()
            if oldPlan != None:
                resumeFrom = generatedPlan.incorporate(oldPlan, failureStep)
                print "Need to resume refinement from step " + repr(resumeFrom) + " in:"
                generatedPlan.printPlan()
        return generatedPlan, resumeFrom, strPlanFileH


    def addToPRGraph(self, resumeFrom, resume_count, old_key, generatedPlan, strPlanFileH, \
                     errorStr):
        if old_key not in resume_count:
            resume_count[old_key] = 0
        if self.iteration > 1:
            new_key = '{}{}'.format(old_key, resume_count[old_key])
            resume_count[old_key] += 1
            self.pr_graph.addEdge(old_key, new_key, generatedPlan, resumeFrom, errorStr)
        else:
            print "Adding root node: *"
            self.pr_graph.addEdge('_', '*', generatedPlan, resumeFrom, "")
        print "\nWill try to pick objects in order: " + repr(getObjSeqInPlan(strPlanFileH, 'object')) + "\n"
        raw_input('!')


    def tryRefiningPRNode(self, startTime, prevPDDLFile, pddlProblemFile, oldPlan, failureStep, \
                          resumeFrom, errorStr, strPlanFileH, resume_key):
        #This function is where ActionError's get parsed and new predicates (such as 'obstructs') are added to the state.
        success = False
        usefulErrors = False
        childCount = 0
        while not usefulErrors:
            try:
                self.tryMotionPlanning(resume_key, resumeFrom, startTime)
                sys.exit(0)
            except pr_graph.Fluent as f:
                errorStr = f.pddl_error_info
                failureStep = self.getFailedActionNumberAndProps(errorStr)[0]
                #strPlanFileH.seek(0)
                usefulErrors = True
                cur_plan = f.cur_plan
                self.iteration += 1

        if errorStr == "":
            print "Lower level failed without error message."
            sys.exit()
        print "Got facts: \n" + errorStr + "\n"
        prevPDDLFile = pddlProblemFile
        pddlProblemFile = self.editedProblemFile.replace(".pddl", "_" + repr(self.iteration) + ".pddl")
        oldPlan = copy.deepcopy(cur_plan) # copy.deepcopy(generatedPlan)
        failureStep, propList = self.getFailedActionNumberAndProps(errorStr)
        # failureStep = 0
        origState = copy.deepcopy(oldPlan.origStateList[failureStep])
        oldPlan.patchState(failureStep, propList)
        oldPlan.writeInitFile(failureStep, prevPDDLFile, pddlProblemFile)
        oldPlan.truncateFrom(failureStep)
        return pddlProblemFile, oldPlan, origState, failureStep, errorStr


    def iterativePlan(self):
        startTime = time.time()
        prevPDDLFile = None
        pddlProblemFile = self.editedProblemFile
        oldPlan = None
        origState = None
        failureStep = None
        replanMode = False
        resumeFrom = 0
        resumeCount = {}  # counts how many times resume() has been called on a given node. Used for auto node key generation
        oldKey = '*'
        errorStr = ""
        searchAlgo = None
        while True:
            # try:
            self.iteration += 1
            plannerOutFname = pddlProblemFile + ".out"
            if self.iteration > 1:
                replanMode = True

            if self.pr_graph.error_explored(oldKey, errorStr):
                print "Already tried this!!"
            else:
                generatedPlan, resumeFrom, strPlanFileH = self.getNewPlan(pddlProblemFile, oldPlan, \
                                                                                origState, failureStep, \
                                                                                replanMode, resumeFrom, \
                                                                                plannerOutFname)
                if strPlanFileH != -1:
                    self.updatePlan(generatedPlan)
                    # add new node
                    self.addToPRGraph(resumeFrom, resumeCount, oldKey, generatedPlan, strPlanFileH, errorStr)
            # prompt for where to resume
            print "Available Plans:"
            for key in self.pr_graph.graph.nodes():
                print "Plan key: {}".format(key)
                if self.pr_graph.saved_plans.has_key(key):
                    self.pr_graph.saved_plans[key].printPlan()
            self.pr_graph.show_graph()
            #resumeKey = raw_input("resumeKey: ")
            if searchAlgo == None:
                # can use "daisyChainBFS" or "default" or "IDIBDFS"
                searchAlgo = graph_traversal.GraphTraversal(self.pr_graph, self.pr_graph.graph_source).getSearchRoutine("default")
            resumeKey = searchAlgo.next()
            oldKey = resumeKey

            print "\n\n resuming with node {} \n\n".format(resumeKey)

            pddlProblemFile, oldPlan, origState, failureStep, errorStr = self.tryRefiningPRNode(startTime, prevPDDLFile,
                                                                                                pddlProblemFile, oldPlan,
                                                                                                failureStep, resumeFrom,
                                                                                                errorStr,
                                                                                                strPlanFileH, resumeKey)
            # except RolloutError as e:
            #     print e.curr_state
            #     print EnvManager.belief_state_discrete
            #     print EnvManager.belief_state_continuous
            #     raw_input('Rollout failed, replanning!')
            #     self.pr_graph = pr_graph.PRGraph(self.env)
            #     EnvManager.move_objs_ml_loc(self.env)
            #     self.replanCount += 1
            #     self.iteration = 0

            #     prevPDDLFile = pddlProblemFile
            #     pddlProblemFile = self.editedProblemFile.replace(".pddl", "_" + repr(self.iteration) + "-" + repr(self.replanCount) + ".pddl")
            #     fileMgr = InitFileMgr(prevPDDLFile)
            #     fileMgr.replaceInitState(e.curr_state)
            #     fileMgr.writeCurrentPDDL(pddlProblemFile)

            #     #Copied from above
            #     oldPlan = None
            #     origState = None
            #     failureStep = None
            #     replanMode = False
            #     resumeFrom = 0
            #     resumeCount = {}  # counts how many times resume() has been called on a given node. Used for auto node key generation
            #     oldKey = '*'
            #     errorStr = ""
            #     searchAlgo = None


    def tryMotionPlanning(self, resume_key, resumeFrom, startTime):
        # planning_primitives.test(strPlanFileH, self.ORSetup, objList=[],
        #                          reinterpreted=reinterpreted,
        #                          resumeFrom=resumeFrom, startTime=startTime)
        self.pr_graph.resume(resume_key)

        print "Success. Quitting."
        print "Replan count: "+ repr(self.iteration)
        print "Cache clearing count: "+ repr(self.cacheClearCount)
        endTime = time.time()
        print "Execution took " + repr(endTime-startTime) + " seconds"
        if run_test_mode[0]:
            sys.exit(0)
        else:
            raw_input("Hit 'Enter' to close.")
            sys.exit(0)


    def getFailedActionNumberAndProps(self, errorStr):
        errorLines = errorStr.lower().split("\n")
        failedActionNumber = int(errorLines[0].strip("linenumber: "))
        propList = filter(lambda x:len(x) > 0, errorLines[1:])
        return failedActionNumber, propList


    def runPlanner(self, pFile, oFile, replanMode):
        if self.plannerName == FF:
            self.planner = PlannerOps.FF(pddlDomainFile = self.pddlDomainFile,
                                    pddlProblemFile = pFile,
                                    outputFile = oFile)
        if self.plannerName == FD:
            self.planner = PlannerOps.FD(replanMode = replanMode, pddlDomainFile = self.pddlDomainFile,
                                    pddlProblemFile = pFile,
                                    outputFile = oFile)
        if self.plannerName == MP:
            self.planner = PlannerOps.MP(pddlDomainFile = self.pddlDomainFile,
                                    pddlProblemFile = pFile,
                                    outputFile = oFile)
        return self.planner.getResult()


def terminateOrReinterpret():
    return True
    if run_test_mode[0]:
        reinterpret = 'y'
    else:
        reinterpret = raw_input("Planner failed. Reinterpret (y/n)?")
    if reinterpret == 'n':
        print "Quitting"
        sys.exit(-1)
    return True


def getObjSeqInPlan(file_object_or_name, objectNamePrefix = 'object'):
        objSeq = []
        if type(file_object_or_name) is str:
            file_obj = open(file_object_or_name)
        else:
            file_obj = file_object_or_name

        planStr = file_obj.read().lower()
        planLines = planStr.strip().split("\n")
        for line in planLines:
            if 'grasp' in line:
                fixedWLine = re.sub(r' +', ' ', line)
                objMatch = re.search('grasp\w* \w+', fixedWLine)
                obj = objMatch.group().split()[1]
                objSeq.append(obj)

        file_obj.seek(0)

        return objSeq

def usage_str():
    return "Use -v for viewer, -m to use MP, -e for envfile name, -d [l|t] for domain"

if __name__ == "__main__":
    try:
        opts, args = getopt.getopt(sys.argv[1:],"vpme:rd:")
    except getopt.GetoptError:
        print(usage_str())
        sys.exit(-1)

    init_settings()

    viewer = False
    pw_file = False
    for opt, arg in opts:
        if opt == "-v":
            print "Setting viewer mode to True"
            viewer = True
        elif opt == "-p":
            print "Setting trajopt debug to True"
            settings.TRAJOPT_DEBUG = True
        elif opt == "-m":
            print("Using MP instead of FF")
            planner = MP
        elif opt == "-e":
            print("Using env file: %s" %arg)
            envFile = arg
        elif opt == "-d":
            if arg == "l":
                set_domain(LOG_DOMAIN)
                print("\n\nRUNNING LOG DOMAIN\n\n")
            elif arg == "t":
                set_domain(TWO_DOMAIN)
                print("\n\nRUNNING TWO BOX DOMAIN\n\n")
            elif arg == "tc":
                set_domain(TWOCAN_DOMAIN)
                print("\n\nRUNNING TWO CAN DOMAIN\n\n")
            else:
                print("Bad domain input: need l or t for logistics or two box domain respectively")
                sys.exit(-1)
        elif opt == "-r":
            print("Transforming objects to match result of previous plannerWrapper run")
            pw_file = True
    try:
      planner = settings.PLANNER_TO_USE
    except AttributeError as e:
      print("No domain provided.")
      print(usage_str())
      sys.exit(-1)

    if run_test_mode[0]:
        # settings.toggle_use_ros()
        hp = HybridPlanner(settings.pddlDomainFile, settings.pddlDomainFileNoGeomEff,
                           settings.initialProblemFile, viewer, envFile, planner=planner, pw_file=pw_file)
        hp.iterativePlan = utils.timeout(seconds=run_test_mode[1])(hp.iterativePlan)
        try:
            hp.iterativePlan()
        except utils.TimeoutError as e:
            print("Timed out: %s" %e)
            sys.exit(1)
    else:
        hp = HybridPlanner(settings.pddlDomainFile, settings.pddlDomainFileNoGeomEff,
                           settings.initialProblemFile, viewer, envFile, planner=planner, pw_file=pw_file)

        hp.iterativePlan()
