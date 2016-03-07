#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright (C) 2012 Rosen Diankov <rosen.diankov@gmail.com>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

#Based off of OpenRave example code
#translation keys are wasdqe and rotation keys are ijkluo
#see pickfromside_pose_generator.py in src/interface/pose_generators folder to see an example of how to use it.

import threading
import time
import openravepy
from openravepy import databases
from openravepy import IkParameterizationType, IkParameterization, RaveCreateKinBody, raveLogInfo, raveLogWarn, IkFilterOptions
from numpy import array, zeros
import numpy as np

import ipdb
import sys
sys.path.append('../')
import utils
from rapprentice.PR2 import Arm, mirror_arm_joints

import settings
import getopt
import pickle

import termios, fcntl, sys, os

def execute(envFile=None,saveFile='pose.p', num_grasps=8):
    objectName = 'object1'

    iktype = IkParameterizationType.Transform6D

    env = openravepy.Environment()
    env.Load(envFile)
    env.SetViewer('qtcoin')
    with env:
        # load the Transform6D IK models
        ikmodels = []
        # robot = env.GetRobots()[0]
        robot = env.GetKinBody('pr2')
        torso = robot.GetJoint('torso_lift_joint')
        manips = robot.GetManipulators()
        #ipdb.set_trace()
        #manip = robot.GetManipulators()[1]
        manip = robot.GetManipulator('rightarm')
        #robot.SetActiveManipulator(manip)
        robot.SetActiveManipulator('rightarm')
        ikmodel = databases.inversekinematics.InverseKinematicsModel(robot, iktype=iktype)
        if not ikmodel.load():
            ikmodel.autogenerate()
            if not ikmodel.has():
                raw_input('')
        ikmodels.append(ikmodel)

        # create the picker
        pickers = []
        raveLogInfo('creating picker for %s'%str(ikmodel.manip))
        picker = RaveCreateKinBody(env,'')
        picker.InitFromBoxes(array([ [0.05,0,0,0.05,0.01,0.01], [0,0.05,0,0.01,0.05,0.01], [0,0,0.05,0.01,0.01,0.05] ]),True)
        for igeom,geom in enumerate(picker.GetLinks()[0].GetGeometries()):
            color = zeros(3)
            color[igeom] = 1
            geom.SetDiffuseColor(color)
        picker.SetName(ikmodel.manip.GetName())
        env.Add(picker,True)
        # have to disable since not part of collision
        picker.Enable(False)

        #print env.GetBodies()
        obj = env.GetKinBody(objectName)
        if obj is None:
            print "%s is not in this environment"%(objectName)
            exit()
        picker.SetTransform(obj.GetTransform())
        pickers.append([picker,ikmodel.manip])
        #pickers.append(robot)

    # utils.set_rave_limits_to_soft_joint_limits(robot)
    # # start arms on the side
    # left_joints = Arm.L_POSTURES['side2']
    # right_joints = mirror_arm_joints(left_joints).tolist()
    # robot.SetDOFValues(
    #     right_joints + left_joints,
    #     robot.GetManipulator("rightarm").GetArmIndices().tolist() +
    #     robot.GetManipulator("leftarm").GetArmIndices().tolist())
    # start torso up
    v = robot.GetActiveDOFValues()
    v[torso.GetDOFIndex()] = 0.3
    robot.SetDOFValues(v)

    #import IPython; IPython.embed()

    raveLogInfo('picker loaded, try moving them')
    poseList = obtainPose(env,robot,torso,pickers,iktype)
    posesList = []

    # object in which we are generating our poses from
    obj = env.GetKinBody(objectName)
    objTrans = obj.GetTransform()

    #creates an annulus of poses around object
    for i in range(num_grasps):
        rot_ang = i * (2 * np.pi) / num_grasps
        rot = openravepy.matrixFromAxisAngle((0, 0, rot_ang))

        #t = objTrans.dot(rot)

        if poseList:
            newPoseList = []
            for pose in poseList:
                newPoseList.append(rot.dot(pose))
            posesList.append(newPoseList)

    if posesList:
        # saves posesList into saveFile
        pickle.dump(posesList, open(saveFile, "wb"))
        for pose in posesList:
            print pose
    else:
        print "No poses were generated!"

def obtainPose(env, robot, torso, pickers, iktype):
    #Terminal initialization code for input based on https://docs.python.org/2/faq/library.html#how-do-i-get-a-single-keypress-at-a-time
    fd = sys.stdin.fileno()

    oldterm = termios.tcgetattr(fd)
    newattr = termios.tcgetattr(fd)
    newattr[3] = newattr[3] & ~termios.ICANON & ~termios.ECHO
    termios.tcsetattr(fd, termios.TCSANOW, newattr)

    oldflags = fcntl.fcntl(fd, fcntl.F_GETFL)
    fcntl.fcntl(fd, fcntl.F_SETFL, oldflags | os.O_NONBLOCK)

    #Constants
    d = 0.02
    angle = np.pi/16
    transSpeeds = [d/2,d,d*2]
    angleSpeeds = [angle/2,angle,angle*2]
    speed = 1

    #Initial transform values
    currBaseTrans = robot.GetTransform()
    currTorsoOffset = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0.3],[0,0,0,1]])
    currManipTrans = np.array([[1,0,0,0.7],[0,1,0,-0.3],[0,0,1,0.7],[0,0,0,1]]).dot(openravepy.matrixFromAxisAngle((np.pi,0,0)))
    target = currBaseTrans.dot(currTorsoOffset).dot(currManipTrans)
    for ipicker,(picker,manip) in enumerate(pickers):
        sol = manip.FindIKSolution(IkParameterization(target,iktype),IkFilterOptions.IgnoreEndEffectorCollisions)
        robot.SetDOFValues(sol,manip.GetArmIndices())
        picker.SetTransform(target)
    baseTransChange = None
    torsoOffsetChange = None
    manipTransChange = None

    #Initial Settings
    collisionFree = True
    exit = False

    poseList = []

    try:
        while not exit:
            if env.GetViewer() is None:
                break
            with env:
                try:
                    # keyboard input being mapped to matrixes to change translation and rotation of end-effector
                    char = sys.stdin.read(3)
                    if char == 't':
                        manipTransChange = openravepy.matrixFromPose((1,0,0,0,0,0, d))
                    elif char == 'g':
                        manipTransChange = openravepy.matrixFromPose((1,0,0,0,0,0, -d))
                    elif char == 'f':
                        manipTransChange = openravepy.matrixFromPose((1,0,0, 0, 0,-d, 0))
                    elif char == 'h':
                        manipTransChange = openravepy.matrixFromPose((1,0,0,0,0,d, 0))
                    elif char == 'r':
                        manipTransChange = openravepy.matrixFromPose((1,0,0, 0, -d, 0, 0))
                    elif char == 'y':
                        manipTransChange = openravepy.matrixFromPose((1,0,0,0,d,0, 0))
                    elif char == 'j':
                        manipTransChange = openravepy.matrixFromAxisAngle((0, 0, -angle))
                    elif char == 'l':
                        manipTransChange = openravepy.matrixFromAxisAngle((0, 0, angle))
                    elif char == 'i':
                        manipTransChange = openravepy.matrixFromAxisAngle((0, angle, 0))
                    elif char == 'k':
                        manipTransChange = openravepy.matrixFromAxisAngle((0, -angle, 0))
                    elif char == 'u':
                        manipTransChange = openravepy.matrixFromAxisAngle((-angle, 0, 0))
                    elif char == 'o':
                        manipTransChange = openravepy.matrixFromAxisAngle((angle, 0, 0))
                    elif char == 'a':
                        baseTransChange = np.array([[1,0,0,0],[0,1,0,d],[0,0,1,0],[0,0,0,1]])
                    elif char == 'd':
                        baseTransChange = np.array([[1,0,0,0],[0,1,0,-d],[0,0,1,0],[0,0,0,1]])
                    elif char == 'w':
                        baseTransChange = np.array([[1,0,0,d],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
                    elif char == 's':
                        baseTransChange = np.array([[1,0,0,-d],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
                    elif char == 'q':
                        baseTransChange = openravepy.matrixFromAxisAngle((0,0,angle))
                    elif char == 'e':
                        baseTransChange = openravepy.matrixFromAxisAngle((0,0,-angle))
                    elif char == '\x1b[A': #Up Arrow
                        torsoOffsetChange = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,d],[0,0,0,1]])
                    elif char == '\x1b[B': #Down Arrow
                        torsoOffsetChange = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,-d],[0,0,0,1]])
                    elif char == ' ':
                        speed = (speed+1)%3
                        d = transSpeeds[speed]
                        angle = angleSpeeds[speed]
                        #print "Changed speed d=%f, angle=%f" %(d,angle)
                    elif char == 'c':
                        collisionFree = not collisionFree
                    elif char == 'x':
                        exit = True #Exit and save
                    elif char == 'n':
                        import ipdb; ipdb.set_trace()
                        raveLogInfo('saving pose')
                        poseList.append(currBaseTrans.dot(currManipTrans))
                    raveLogInfo('input is ' + char)

                except IOError: pass

                if baseTransChange is not None:
                    nextBaseTrans = currBaseTrans.dot(baseTransChange)
                    if env.CheckCollision(robot):
                        #If it's already in collision, allow it to move
                        robot.SetTransform(nextBaseTrans)
                        currBaseTrans = nextBaseTrans
                    else:
                        #If it's not in collision, don't allow it to move into collision
                        robot.SetTransform(nextBaseTrans)
                        if env.CheckCollision(robot) and collisionFree:
                            robot.SetTransform(currBaseTrans)
                        else:
                            currBaseTrans = nextBaseTrans
                    #Adjust the gripper transform
                    target = currBaseTrans.dot(currTorsoOffset).dot(currManipTrans)
                    for ipicker,(picker,manip) in enumerate(pickers):
                        picker.SetTransform(target)
                    baseTransChange = None

                if torsoOffsetChange is not None:
                    nextTorsoOffset = currTorsoOffset.dot(torsoOffsetChange)
                    if nextTorsoOffset[2,3] > np.max(torso.GetLimits()):
                        nextTorsoOffset[2,3] = np.max(torso.GetLimits())
                    elif nextTorsoOffset[2,3] < np.min(torso.GetLimits()):
                        nextTorsoOffset[2,3] = np.min(torso.GetLimits())
                    robot_dofs = robot.GetActiveDOFValues()
                    torso_dof_index = torso.GetDOFIndex()
                    robot_dofs[torso_dof_index] = nextTorsoOffset[2,3]
                    if env.CheckCollision(robot):
                        #If it's already in collision, allow it to move
                        robot.SetDOFValues(robot_dofs)
                        currTorsoOffset = nextTorsoOffset
                    else:
                        #If it's not in collision, don't allow it to move into collision
                        robot.SetDOFValues(robot_dofs)
                        if env.CheckCollision(robot) and collisionFree:
                            robot_dofs[torso_dof_index] = currTorsoOffset[2,3]
                            robot.SetDOFValues(robot_dofs)
                        else:
                            currTorsoOffset = nextTorsoOffset
                    #Adjust the gripper transform
                    target = currBaseTrans.dot(currTorsoOffset).dot(currManipTrans)
                    for ipicker,(picker,manip) in enumerate(pickers):
                        picker.SetTransform(target)
                    torsoOffsetChange = None

                if manipTransChange is not None:
                    for ipicker,(picker,manip) in enumerate(pickers):
                        nextManipTrans = currManipTrans.dot(manipTransChange)
                        target = currBaseTrans.dot(currTorsoOffset).dot(nextManipTrans)
                        sol = None
                        if not collisionFree:
                            sol = manip.FindIKSolution(IkParameterization(target,iktype),IkFilterOptions.IgnoreEndEffectorCollisions)
                            if sol == None:
                                raveLogInfo('Out of reach!')
                        else:
                            sol = manip.FindIKSolution(IkParameterization(target,iktype),IkFilterOptions.CheckEnvCollisions)
                            if sol == None:
                                raveLogInfo('Unable to find collision free IK')
                        if sol is not None:
                            robot.SetDOFValues(sol,manip.GetArmIndices())
                            picker.SetTransform(target) #RGB Coordinate Axis
                            currManipTrans = nextManipTrans
                    manipTransChange = None

    finally:
        termios.tcsetattr(fd, termios.TCSAFLUSH, oldterm)
        fcntl.fcntl(fd, fcntl.F_SETFL, oldflags)

    return poseList


if __name__ == "__main__":
    usage_str = "Usage: python %s -n [num_grasps] -e [env_filename] -s [save_filename]" %sys.argv[0]
    # envFile = settings.ENV_PATH+settings.DEFAULT_ENV_FILE

    saveFile = 'z.p'
    num_grasps = 8

    try:
        opts, args = getopt.getopt(sys.argv[1:],"vn:s:e:t:")
    except getopt.GetoptError:
        print(usage_str)
        sys.exit(2)

    for opt, arg in opts:
        if opt == "-n":
            num_grasps = int(arg)
        if opt == "-e":
            envFile = arg
        elif opt == "-s":
            saveFile = arg

    try:
        execute(envFile=envFile, saveFile=saveFile, num_grasps=num_grasps)
    except ValueError:
        print(usage_str)
