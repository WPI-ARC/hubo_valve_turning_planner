#!/usr/bin/env python
# Ben Suay, RAIL
# Jim Mainprice, ARC
# July 2013, October 2013
# Worcester Polytechnic Institute

# openrave-robot.py /your/path/to/your.robot.xml --info=joints
# On that page you can find more examples on how to use openrave-robot.py.
# http://openrave.org/docs/latest_stable/command_line_tools/

from openravepy import *

import sys
if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *
    import numpy
import time
from wpi_planning_utilities.rodrigues import *
from wpi_planning_utilities.TransformMatrix import *
from wpi_planning_utilities.str2num import *
from wpi_planning_utilities.TSR import *
from math import *
from copy import *
import os # for file operations
from base_wheel_turning import *
import rave2realhubo

class ConstrainedPath():

    def __init__(self, myName, robot ):

        self.name = myName
        self.elements = []
        self.valveType = None
        self.path = None
        self.robot = robot
        
    def PlayInOpenRAVE(self):
        for e in self.elements:
            [success, why] = e.PlayInOpenRAVE()
            if(not success):
                return why

        return [True, ""]

    def GetOpenRAVETrajectory(self, filepath):

        # Only reload from file when needed
        if self.path is not None :
            print "no reload in GetOpenRAVETrajectory"
            return self.path

        myPath = [] # a list of configurations for the whole path
        for e in self.elements:
            # populate the path with the list of configurations
            # of each path element.
            listOfQs = e.GetOpenRAVETrajectory(self.robot, filepath)
            myPath.extend(listOfQs)

        self.path = myPath
        return self.path

    # Returns if true if the robot current confuration
    # is set at the initial configuration of the trajectory
    def IsRobotAtInitConfig(self,jointNames):

        if self.path is None :
            return False

        q_cur = self.robot.GetDOFValues()
        q_init = self.path[0]

        q_diff =  array(q_cur) - array(q_init)
        max_error_val = q_diff.max()
        max_error_id = q_diff.argmax()

        print "MAX ERROR value between current conf and init is : " + str(max_error_val) + " , at joint : " + str(jointNames[max_error_id])

        # TODO This threshold is set arbitrarily
        if( max_error_val >= 0.03 ):
            return False

        return True

    # Returns true if is two handed trajectory
    def IsTwoHandedTurning(self):

        # Fix or self.valveType != "W"
        if self.name == "GetReady" or self.name == "EndTask" \
            or self.name == "Grasp" or self.name == "UnGrasp" :
            return False
        else:
            return True

class ConstrainedPathElement():

    def __init__(self, myName):
        self.name = myName
        self.startik = None
        self.goalik = None
        self.TSR = None
        self.smoothing = None
        self.psample = None
        self.mimicdof = None
        self.filename = None
        self.errorCode = None
        # The following attributes are 
        # lists. And they will be used
        # when playing trajectories with
        # cbirrt 'traj' command
        self.cbirrtProblems = None
        self.cbirrtRobots = None
        self.cbirrtTrajectories = None
        
        # The following attributes
        # will auto-generate hand trajectories 
        self.openHandsBefore = False
        self.openHandsAfter = False
        self.closeHandsBefore = False
        self.closeHandsAfter = False
        self.hands = None
        self.padValve = False

        # Active Dofs for the path element
        self.activedofs = None

    def GetOpenRAVETrajectory(self, robot, filepath):

        # To be safe set all dofs activedofs
        # robot.SetActiveDOFs( self.alldofs )

        myPathElementQs = []
        traj = RaveCreateTrajectory(robot.GetEnv(),'').deserialize(open(filepath+self.filename+'.txt','r').read())
        cs = traj.GetConfigurationSpecification()
        robotJointValsGroup = cs.GetGroupFromName("joint_values "+robot.GetName())
        trajLength = traj.GetNumWaypoints()

        wpFirst = traj.GetWaypoint(0)
        qFirst = wpFirst[robotJointValsGroup.offset:(robotJointValsGroup.offset+robotJointValsGroup.dof)]

        wpLast = traj.GetWaypoint(trajLength-1)
        qLast = wpLast[robotJointValsGroup.offset:(robotJointValsGroup.offset+robotJointValsGroup.dof)]

        leftHandFinger1Idx = -1.0
        leftHandFinger2Idx = -1.0
        leftHandFinger3Idx = -1.0

        rightHandFinger1Idx = -1.0
        rightHandFinger2Idx = -1.0
        rightHandFinger3Idx = -1.0
            
        rightHandFinger4Idx = -1.0
        rightHandFinger5Idx = -1.0
        rightHandFinger6Idx = -1.0

        for jIdx, joint in enumerate(robot.GetJoints()):
            if ( joint.GetName() == 'LF11' ):
                leftHandFinger1Idx = jIdx
            if ( joint.GetName() == 'LF12' ):
                leftHandFinger2Idx = jIdx
            if ( joint.GetName() == 'LF13' ):
                leftHandFinger3Idx = jIdx

            if ( joint.GetName() == 'RF11' ):
                rightHandFinger1Idx = jIdx
            if ( joint.GetName() == 'RF12' ):
                rightHandFinger2Idx = jIdx
            if ( joint.GetName() == 'RF13' ):
                rightHandFinger3Idx = jIdx
            if ( joint.GetName() == 'RF21' ):
                rightHandFinger4Idx = jIdx
            if ( joint.GetName() == 'RF22' ):
                rightHandFinger5Idx = jIdx
            if ( joint.GetName() == 'RF23' ):
                rightHandFinger6Idx = jIdx
                

        freq = 25 # Play speed in Hz.
        howManySeconds = 6 # Play time in sec.
        howManyTimes = howManySeconds*freq
                
        # This is where we open/close the hands before the trajectory
        if( self.openHandsBefore ):
            print "Trying to make a trajectory that opens the hands (before)"
            qOpenHandsBefore = deepcopy(qFirst)
            qOpenHandsBefore[leftHandFinger1Idx] = -1.0
            qOpenHandsBefore[leftHandFinger2Idx] = -1.0
            qOpenHandsBefore[leftHandFinger3Idx] = -1.0

            qOpenHandsBefore[rightHandFinger1Idx] = -1.0
            qOpenHandsBefore[rightHandFinger2Idx] = -1.0
            qOpenHandsBefore[rightHandFinger3Idx] = -1.0
            
            qOpenHandsBefore[rightHandFinger4Idx] = -1.0
            qOpenHandsBefore[rightHandFinger5Idx] = -1.0
            qOpenHandsBefore[rightHandFinger6Idx] = -1.0
            
            for i in range(howManyTimes):
                myPathElementQs.append(qOpenHandsBefore)

        if( self.closeHandsBefore ):
            print "Trying to make a trajectory that closes the hands (before)"
            qCloseHandsBefore = deepcopy(qFirst)
            qCloseHandsBefore[leftHandFinger1Idx] = 1.0
            qCloseHandsBefore[leftHandFinger2Idx] = 1.0
            qCloseHandsBefore[leftHandFinger3Idx] = 1.0
            
            qCloseHandsBefore[rightHandFinger1Idx] = 1.0
            qCloseHandsBefore[rightHandFinger2Idx] = 1.0
            qCloseHandsBefore[rightHandFinger3Idx] = 1.0
            qCloseHandsBefore[rightHandFinger4Idx] = 1.0
            qCloseHandsBefore[rightHandFinger5Idx] = 1.0
            qCloseHandsBefore[rightHandFinger6Idx] = 1.0
            
            for i in range(howManyTimes):
                myPathElementQs.append(qCloseHandsBefore)

        # This is where we get the trajectory
        for i in range(trajLength):
             wp = traj.GetWaypoint(i)
             q = wp[robotJointValsGroup.offset:(robotJointValsGroup.offset+robotJointValsGroup.dof)]
             q[leftHandFinger1Idx] = 0.0
             q[leftHandFinger2Idx] = 0.0
             q[leftHandFinger3Idx] = 0.0
            
             q[rightHandFinger1Idx] = 0.0
             q[rightHandFinger2Idx] = 0.0
             q[rightHandFinger3Idx] = 0.0
             q[rightHandFinger4Idx] = 0.0
             q[rightHandFinger5Idx] = 0.0
             q[rightHandFinger6Idx] = 0.0
             myPathElementQs.append(q)

             
        # This is where we open/close the hands after the trajectory
        if( self.openHandsAfter ):
            print "Trying to make a trajectory that opens the hands (after)"
            qOpenHandsAfter = deepcopy(qLast)
            print "leftHandFinger1Idx: " + str(leftHandFinger1Idx)
            print "leftHandFinger2Idx: " + str(leftHandFinger2Idx)
            print "leftHandFinger3Idx: " + str(leftHandFinger3Idx)
            qOpenHandsAfter[leftHandFinger1Idx] = -1.0
            qOpenHandsAfter[leftHandFinger2Idx] = -1.0
            qOpenHandsAfter[leftHandFinger3Idx] = -1.0
            print "rightHandFinger1Idx: " + str(rightHandFinger1Idx)
            print "rightHandFinger2Idx: " + str(rightHandFinger2Idx)
            print "rightHandFinger3Idx: " + str(rightHandFinger3Idx)
            qOpenHandsAfter[rightHandFinger1Idx] = -1.0
            qOpenHandsAfter[rightHandFinger2Idx] = -1.0
            qOpenHandsAfter[rightHandFinger3Idx] = -1.0
            print "rightHandFinger4Idx: " + str(rightHandFinger4Idx)
            print "rightHandFinger5Idx: " + str(rightHandFinger5Idx)
            print "rightHandFinger6Idx: " + str(rightHandFinger6Idx)
            qOpenHandsAfter[rightHandFinger4Idx] = -1.0
            qOpenHandsAfter[rightHandFinger5Idx] = -1.0
            qOpenHandsAfter[rightHandFinger6Idx] = -1.0
            
            for i in range(howManyTimes):
                myPathElementQs.append(qOpenHandsAfter)

        if( self.closeHandsAfter ):
            print "Trying to make a trajectory that closes the hands (after)"
            qCloseHandsAfter = deepcopy(qLast)
            qCloseHandsAfter[leftHandFinger1Idx] = 1.0
            qCloseHandsAfter[leftHandFinger2Idx] = 1.0
            qCloseHandsAfter[leftHandFinger3Idx] = 1.0

            qCloseHandsAfter[rightHandFinger1Idx] = 1.0
            qCloseHandsAfter[rightHandFinger2Idx] = 1.0
            qCloseHandsAfter[rightHandFinger3Idx] = 1.0
            qCloseHandsAfter[rightHandFinger4Idx] = 1.0
            qCloseHandsAfter[rightHandFinger5Idx] = 1.0
            qCloseHandsAfter[rightHandFinger6Idx] = 1.0

            for i in range(howManyTimes):
                myPathElementQs.append(qCloseHandsAfter)

        return myPathElementQs

    def PlayInOpenRAVE(self):
        # play this path element in openrave for
        # confirmation / error check
        answers = []

        if(len(self.cbirrtProblems) != len(self.cbirrtTrajectories)):
            return [False, "Error: size of problem - trajectory mismatch."]
        else:
            try:
                for i in range(len(self.cbirrtTrajectories)):
                    answers.append(self.cbirrtProblems[i].SendCommand('traj '+self.cbirrtTrajectories[i]+'.txt'))
                    print "traj call answer: ",str(answers[i])

                for r in self.cbirrtRobots:
                    r.WaitForController(0)

            except openrave_exception, e:
                print e
                return [False, "OS exception in PlayTrajectory."]

            for r in self.cbirrtRobots:
                r.GetController().Reset(0)

        time.sleep(2)
        return [True, ""]
