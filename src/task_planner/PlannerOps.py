

import utils 
from OutputParser import *
from settings import *
import glob
import sys, subprocess, time, os
import StringIO
import PDDLActionParser
import IPython
def execCmd(cmd,  successStr, outputFname, pollTime = 2):
    ''' A generic utility for executing a command. 
    outputFname stores stdout and stderr'''
    initTime = time.time()
    print "Executing %s...   " %(cmd), 
    sys.stdout.flush()
    ## Using subprocess.call so that all the messages from the planner
    ## can be captured and parsed for the string "Solution found!"

    dumpF = open(outputFname,  "w")
    p = subprocess.Popen([cmd], shell = True, stdout=dumpF, stderr=dumpF)
    startTime = time.time()
    #p = subprocess.Popen([cmd], shell = True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    
    while p.poll() == None:
        # time.sleep(pollTime)
        if time.time() - startTime >350:
            #ans = raw_input("Still running. Continue (c), Restart (r), Quit(q)? ")
            ans = "q"
	    print "Killing process" 
	    startTime = time.time()
            if ans.strip() == "q":
                os.kill(p.pid+1,  9)
                sys.exit(-1)
            elif ans.strip() == "c":
                continue
            elif ans.strip() == "r":
                os.kill(p.pid+1,  9)
                return execCmd(cmd, successStr)
            else:
                print "Unrecognized response. Continuing."

    dumpF.close()
    # time.sleep(5)
    endTime = time.time()

    dumpF = open(outputFname, "r")
    msg = dumpF.read()
    #out, err = p.communicate()
    #msg = out+err
    
    print "\nPlanning time: {0}".format(endTime-initTime)

    if successStr in msg:
        print "Success!"
        print
        return msg
    else:
        print "Failure... Planner message:"
        print msg
        return -1


class Planner(object):
    def __init__(self, pddlDomainFile, pddlProblemFile, outputFile):
        self.pddlProblemFile = pddlProblemFile
        self.pddlDomainFile = pddlDomainFile
        self.outputFile = outputFile
        for fileName in glob.glob(self.outputFile):
            os.remove(fileName)
        self.successStr = ""

    def runPlanner(self):
        return "No planner defined"

    def getStateList(self):
        parser = PDDLActionParser.PDDLActionParser(self.pddlDomainFile, self.pddlProblemFile, self.planStrF)
        return parser.execute()
    

class MP(Planner):
    def __init__(self, **args):
        super(MP, self).__init__(**args)
        self.successStr = "PLAN FOUND"
        
        
    def getCmdLine(self, options = "", minHorizon = 3):
        return MPEXEC + " -A 2 -S 1 -F "+ repr(minHorizon) + " "+ self.pddlDomainFile\
            + " " + self.pddlProblemFile


    def runPlanner(self):
        mCmdLine = self.getCmdLine()
        retVal = execCmd(mCmdLine, self.successStr, self.outputFile)
        if retVal == -1:
            return -1, -1, -1

        rawOutput = tryIO(self.outputFile, "read")
        planStr = "\n".join([line.split(":")[1].strip() for line in rawOutput.split("\n") if "STEP" in line])

        return planStr, rawOutput, 1


    def getResult(self):
        planStr, rawOut, planCount = self.runPlanner()
        if planStr == -1:
            return -1, -1, -1

        linenum = 0
        outStr = ""
        for line in planStr.split("\n"):
            if line.strip() != "":
                outStr += "\t" + repr(linenum) + ": " + line.replace("(", " ").replace(")", " ").replace(",", " ").strip().upper()\
                    + "\n"
                linenum += 1 
        self.planStrF = utils.getStringFile(outStr)
        return self.planStrF, rawOut, planCount

    
class FF(Planner):
    def __init__(self, **args):
        super(FF, self).__init__(**args)
        self.successStr = "found legal plan"


    def getCmdLine(self, options = ""):
        return FFEXEC + " -o " + self.pddlDomainFile +" -f " + self.pddlProblemFile \
            + " " + options  
    
    def runPlanner(self):
        ffCmdLine = self.getCmdLine()
        retVal = execCmd(ffCmdLine, self.successStr, self.outputFile)
        if retVal ==-1:
            return -1,-1,-1

        rawOutput = tryIO(self.outputFile, "read")
        ffOutStr = OutputParser(rawOutput).getFFPlan()
        ffOutStr = '\n\n    0: MOVE ROBOTINITLOC GP_CAN2\n        1: PICK CAN2 CAN2INITLOC GP_CAN2 GRASP_CAN2\n        2: MOVE_W_OBJ GP_CAN2 PDP_CAN2_CAN2TEMPLOC CAN2 GRASP_CAN2\n        3: PLACE CAN2 CAN2TEMPLOC PDP_CAN2_CAN2TEMPLOC GRASP_CAN2\n        4: MOVE PDP_CAN2_CAN2TEMPLOC GP_CAN2\n        5: PICK CAN2 CAN2TEMPLOC GP_CAN2 GRASP_CAN2\n        6: MOVE_W_OBJ GP_CAN2 PDP_CAN2_GOAL2 CAN2 GRASP_CAN2\n        7: PLACE CAN2 GOAL2 PDP_CAN2_GOAL2 GRASP_CAN2\n     \n\n'
        return ffOutStr, rawOutput, 1

    def getResult(self):
        planStr, rawOut, planCount = self.runPlanner()
        if planStr == -1:
            return -1,-1,-1
        
        self.planStrF = utils.getStringFile(planStr)
        return utils.getStringFile(planStr), rawOut, planCount


class FD(Planner):
    def __init__(self, pollTime = 5, planQuality = 1, timeLimit = 60000, replanMode = False, **args):
        self.pollTime = pollTime
        self.planQuality = planQuality
        self.timeLimit = timeLimit
        super(FD, self).__init__(**args)
        self.execString = FDEXEC
        if FDOPTIMALMODE:
            print "\n\nRunning FD in optimal mode \n\n"
            self.execString = FDOPTEXEC

    def getCmdLine(self, options = ""):
        return self.execString + " " + self.pddlDomainFile +"  " + self.pddlProblemFile \
                             + " "+ self.outputFile + " " + options
                             
    def runPlanner(self):
        '''fdOutputFName.X only gets the output plan. messages go to fdOutputFName '''
        fdCmdLine = self.getCmdLine()
        ext = "."+repr(self.planQuality)
        for fileName in glob.glob(self.outputFile+".*"):
            os.remove(fileName)

        p = subprocess.Popen([fdCmdLine], shell = True, \
                                 stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        print "Running " + fdCmdLine
        startTime = time.time()

        while p.poll() == None:
            print "Available plans: {0}, runtime: {1}".format(repr(glob.glob(self.outputFile+".*")), repr(time.time()-startTime))
            time.sleep(self.pollTime)
            if (os.path.exists(self.outputFile+ext)  or \
                    ((time.time() - startTime) > self.timeLimit)):
                killproc = subprocess.Popen(["pkill -KILL downward"], shell = True)
                print "killing planning process"
                print "Available plans: "+ repr(glob.glob(self.outputFile+".*"))
                break
        endTime = time.time()

        out, err = p.communicate()
        msg = out+err

        print "\nPlanning time: {0}".format(endTime-startTime)

        if "Solution found" in msg:
            print "Success!"
        else:
            print "Failure... Planner message:"
            print msg
            return -1,-1,-1

        tryIO(self.outputFile+"_raw", "write", msg)
        
        bestExt = ""
        planCount = 1
        if not FDOPTIMALMODE:
            planCount = len(glob.glob(self.outputFile+".*"))
            bestExt = "."+repr(planCount)
        print "\n\n\n"
        print "Using plan from "+ self.outputFile+bestExt
        fdPlanStr = tryIO(self.outputFile+bestExt, "read")

        return fdPlanStr, msg, planCount


    def getResult(self):
        fdPlanStr, plannerOutput, planCount = self.runPlanner()
        if fdPlanStr == -1:
            return -1, -1, -1
        # modify planStr to add line numbers etc.
        planLines = fdPlanStr.replace("(", "").replace(")","").split("\n")
        planStr = ""
        for lineNum in range(0,len(planLines)):
            if planLines[lineNum].strip()!="":
                planStr += "\t"+repr(lineNum)+": "+  planLines[lineNum].upper() + "\n"

        self.planStrF = utils.getStringFile(planStr)
        return utils.getStringFile(planStr), plannerOutput, planCount 

