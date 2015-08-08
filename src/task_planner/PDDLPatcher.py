from InitFileMgr import *
import pdb
import copy
import settings

class PDDLPatcher:
    def __init__(self, pddlFile, plannerOutput = "", plannerName = "", planCount = 0):
        self.initFileMgr = InitFileMgr(pddlFile)
        self.stateList = []
        if len(plannerOutput) > 0:
            self.populateStateList(plannerOutput, plannerName, planCount)
        
        
    def getCurrentInitState(self):
        return copy.copy(self.initFileMgr.getCurrentState())
        
        
    def populateStateList(self, plannerOutput, planner, planCount=0): 
        numStates = len(OutputParser(plannerOutput, planner, planCount).getPartialStatesList())
        for stateNum in range(numStates):
            self.patch(plannerOutput, stateNum, planner, planCount)
            self.stateList.append(copy.deepcopy(self.getCurrentInitState()))

        
    def patch(self, plannerOutput, stateNum,  planner, planCount=0):
        if planner == settings.FF:
            self.patchWithFFOutput(plannerOutput, stateNum)
        if planner == settings.FD:
            self.patchWithFDOutput(plannerOutput, stateNum, planCount)
        

    def patchWithFFOutput(self, plannerOutput, stateNum):
        op = OutputParser(plannerOutput, settings.FF)
        propSet = op.getPropSet()

        deltaState = op.getStateByIndex(stateNum)

        #deltaState.printState()
        ##Compile away FF's CWA: figure out the set of props
        ## it has CWA with, include those not in true set as false
        ## ff only considers cwa wrt the set of props in propset
        deltaState.makeCWAExplicit(propSet)
        self.initFileMgr.patchInitState(deltaState)

 
    def patchWithFDOutput(self, fdOutStr, stateNum, planCount):
        ''' no need to do any patching. return fd state
            because we add the pruned atoms back while parsing'''
        print "returning state "+repr(stateNum)    
        op = OutputParser(fdOutStr, settings.FD, planCount)
        #op.parseFDOutput(fdOutStr, planCount)

        self.initFileMgr.replaceInitState(op.getStateByIndex(stateNum))


    def patchWithProps(self, inputFromContinuous):
        ## Suppose inputFromContinuous is a set of 
        ## propositions to be added to the state
        
        ## Should work with prop-literals of the form
        ## (not-p x) as well
        print "Patching with props: "
        print inputFromContinuous
        print
        deltaState = State()
        z=map(lambda x: deltaState.addProposition(x), inputFromContinuous)
        self.initFileMgr.patchInitState(deltaState)


    def patchStateNumWithProps(self, stateNum, ffOutputFile,\
                                   factsFromContinuous):
        ## first get initFileMgr to the right state num
        self.initFileMgr.rollbackTo(0)
        self.patchWithFFOutput(ffOutputFile, stateNum)
        self.patchWithProps(factsFromContinuous)
    
    
    def forgetLearnedFactsAbout(self, symbol):
        '''symbol can be a predicate like obstructs '''
        self.initFileMgr.purgeFactsWithSymbol(symbol)


    def patchWithNewInterpretation(self, symbol, factsFromContinuous):
        '''symbol can be a predicate like obstructs '''
        self.initFileMgr.purgeFactsWithSymbol(symbol)
        self.patchWithProps(factsFromContinuous)


    def printInitState(self):
        self.initFileMgr.printInitState()

    def writeCurrentInitState(self, ofname):
        self.initFileMgr.writeCurrentPDDL(ofname)


if __name__ == "__main__":
    # mypatcher = PDDLPatcher("/tmp/test2.out")
    # myPatcher.printInitState()
    # myPatcher.patchWithFFOutput("/tmp/test.out", 4)
    # myPatcher.printInitState()
    # myPatcher.writeCurrentInitState("/tmp/test3.out")
    # myPatcher.patchWithProps(["(a b c)"])
    # myPatcher.printInitState()
    
    # myPatcher.patchStateNumWithProps(0, "/tmp/test.out", ["(start obj1)"])
    # myPatcher.printInitState()
    
    # myPatcher.patchWithNewInterpretation("p1", ["(x p1 p2)"])
    # myPatcher.printInitState()
    path = "../domains/"

    myPatcher = PDDLPatcher(path+"robotics_autogen_prob.pddl")
    myPatcher.printInitState()
    
    errorStr = open(path+"robotics_autogen_err1.txt").read()
    
    errorLines = errorStr.lower().split("\n")

    failedActionNumber = int(errorLines[0].strip("linenumber: "))
    propList = errorLines[1:]
    
    myPatcher.patchWithFFOutput(path+"robotics_autogen_plan1.pddl", \
                                    failedActionNumber-1)
    myPatcher.printInitState()
    myPatcher.patchWithProps(propList)
    myPatcher.printInitState()
    myPatcher.writeCurrentInitState(path + "robotics_autogen_prob2.pddl")

    errorStr = open(path+"robotics_autogen_err2.txt").read()
    errorLines =  errorStr.lower().split("\n")
    failedActionNumber = int(errorLines[0].strip("linenumber: "))
    propList = filter(lambda x: len(x)>0, errorLines[1:])
    
    myPatcher.patchWithFFOutput(path+"robotics_autogen_plan2.pddl", \
                                    failedActionNumber-1)
    myPatcher.printInitState()
    myPatcher.patchWithProps(propList)
    myPatcher.printInitState()
    myPatcher.writeCurrentInitState(path + "robotics_autogen_prob3.pddl")
