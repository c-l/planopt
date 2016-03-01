from PDDLPatcher import *
import PDDLActionParser
import utils
import settings
import pdb

class HLPlan:
    def __init__(self, pddlDomainFile, pddlDomainFileNoGeomEff, pddlProblemFile, strPlanFH,
                 plannerOutput, plannerName, planCount = 0, stateList = None,
                 origState = None):
        self.planStrF = strPlanFH
        self.rawOutput = plannerOutput
        self.plannerName = plannerName
        self.planCount = planCount
        self.actionList = []
        self.stateList = stateList
        self.pddlDomainFile = pddlDomainFile
        self.pddlDomainFileNoGeomEff = pddlDomainFileNoGeomEff
        self.pddlProblemFile = pddlProblemFile
        self.origStateList = copy.deepcopy(stateList)

        self.populateActionList()
        self.populateStateList(origState)

        self.readActionPredicates()


    def equals(self, plan2):
        if (self.actionList != plan2.actionList):
            return False
        stateList1 = self.stateList
        stateList2 = plan2.stateList
        # state lists must have same number of elements because
        # action lists match
        for i in range(len(stateList1)):
            if stateList1[i].rawStringForm() != stateList2[i].rawStringForm():
                return False
        return True


    def revertAllStatesUntilEx(self, limit):
        for i in range(limit):
            self.revertState(i)


    def revertState(self, index):
        self.stateList[index] = copy.deepcopy(self.origStateList[index])

    def resetStates(self):
        for i in range(len(self.stateList)):
            self.revertState(i)

    def populateActionList(self):
        self.planStrF.seek(0)
        for line in self.planStrF.read().split("\n"):
            if line.strip() == "":
                continue
            self.actionList.append(line.strip().split(":")[1])


    def populateStateList(self, origState):
        if self.plannerName == settings.MP or self.plannerName == settings.FD:
            return
        Patcher = PDDLPatcher(self.pddlProblemFile, self.rawOutput,
            self.plannerName, self.planCount)

        if len(Patcher.stateList) == 0:
            Patcher.populateStateList(self.rawOutput, self.plannerName, self.planCount)

        self.stateList = Patcher.stateList
        self.populateOrigStateList(origState)


    def populateOrigStateList(self, origState):
        # use parser to get predicted state list from orig state
        if origState is None:
            self.origStateList = copy.deepcopy(self.stateList)
            return
        origStatePDDLFname = settings.DOMAIN_PATH+"incremental_temp_1.pddl"
        fileMgr = InitFileMgr(self.pddlProblemFile)
        fileMgr.replaceInitState(origState)
        fileMgr.writeCurrentPDDL(origStatePDDLFname)
        parser = PDDLActionParser.PDDLActionParser(self.pddlDomainFileNoGeomEff, \
                                                       origStatePDDLFname, self.planStrF)
        print "Using parser to get original state list"
        self.origStateList = parser.execute()


    def incorporate(self, plan2, failStep):
        '''prepend plan2[0:failStep] to this plan '''
        self.actionList = plan2.actionList[0:failStep] + self.actionList
        self.stateList = plan2.stateList + self.stateList
        self.origStateList = plan2.origStateList + self.origStateList

        # resumeFrom = failStep
        resumeFrom = 0 # setting resumeFrom to failStep assumes that the previous step's IKs won't affect the failure of the next actions which is not true
        deletedActions = []
        axns = self.trimActions()
        deletedActions.extend(axns)
        oldPlanRemovals = filter(lambda x: x<failStep, deletedActions)
        if len(oldPlanRemovals) > 1:
            print "Deleted more than one action from old plan"
            pdb.set_trace()

        if len(oldPlanRemovals) > 0:
            resumeFrom = oldPlanRemovals[0]
        return resumeFrom


    def trimActions(self, moveAction = "move", handoffAction = "handoff"):
        '''get rid of repetitive move and handoff actions -- irrelevant when motion planner
        used for movement'''
        n = len(self.actionList)
        if n < 2:
            return []

        removed = []
        oldIndex = 0
        i = 0
        while i < len(self.actionList)-1:
            if moveAction in self.getActionName(i).lower() and moveAction in self.getActionName(i+1).lower():
                _, start, end = self.actionList[i].split()
                del self.actionList[i]
                self.actionList[i] = self.actionList[i].replace(end, start)
                del self.stateList[i+1]
                del self.origStateList[i+1]
                removed.append(oldIndex)
            elif handoffAction in self.getActionName(i).lower() and handoffAction in self.getActionName(i+1).lower():
                del self.actionList[i]
                del self.actionList[i]
                del self.stateList[i+1]
                del self.stateList[i+1]
                del self.origStateList[i+1]
                del self.origStateList[i+1]
                removed.append(oldIndex)
                removed.append(oldIndex+1)
            else:
                i += 1
            oldIndex += 1
        return removed

    def trimStateActions(self):
      num_states = len(self.stateList)
      i = 0
      count = 0

      while i < num_states - 1: # excludes final state
        for j in range(i+1, num_states):
          if self.stateList[i].rawStringForm() == self.stateList[j].rawStringForm():
            for _ in range(i+1, j+1):
              count += 1
              del self.stateList[i+1]
              del self.actionList[i]
            break
        i += 1
        num_states = len(self.stateList)
      print("Removed %d actions" %count)


    def getActionName(self, i):
        return self.actionList[i].split()[0]


    def getStrIOPlan(self):
        i=0
        str = ""
        for aline in self.actionList:
            str += "\t" + repr(i) + ":" + aline + "\n"
            i+=1
        return utils.getStringFile(str)


    def printPlan(self):
        print self.getStrIOPlan().read()


    def writeInitFile(self, stateNum, prevPDDLFName, newPDDLFName):
        fileMgr = InitFileMgr(prevPDDLFName)
        fileMgr.replaceInitState(self.stateList[stateNum])
        fileMgr.writeCurrentPDDL(newPDDLFName)


    def patchState(self, stateNum, propList):
        deltaState = State()
        deltaState.addProps(propList)
        self.stateList[stateNum].patch(deltaState)


    def truncateFrom(self, zeroIndexedNum):
        '''truncate state and acction lists starting from element at zeroIndexedNum'''
        self.stateList = self.stateList[:zeroIndexedNum]
        self.origStateList = self.origStateList[:zeroIndexedNum]
        self.actionList = self.actionList[:zeroIndexedNum]

    def readActionPredicates(self):
        """reads action effects/preconditions and stores them as a dictionary"""
        parser = PDDLActionParser.PDDLActionParser(self.pddlDomainFile, self.pddlProblemFile, self.planStrF)
        parser.execute()
        self.preconditionDict = parser.preconditionDict
        self.effectDict = parser.effectDict
