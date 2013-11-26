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

# ------------------------------------------------------------------------------
# ------------------------------------------------------------------------------
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
# ------------------------------------------------------------------------------
# ------------------------------------------------------------------------------
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
        self.padWaist = False

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

# ------------------------------------------------------------------------------
# ------------------------------------------------------------------------------
class DrcHuboValveTurningTSRs():

    def __init__(self,robotManips):

        # Robot manipulators
        self.robotManips = robotManips

        # Plan in Box
        # Use a manipbox for free space motions (only moves inside a box)
        self.use_manipbox = True
        self.draw_manip_box = False
        self.manipbox_draw = None
        self.manipbox_dim = None

        # Get ready TSRs
        self.TSRChainStringFeetandHead_current2init = ""
        self.TSRChainStringFeetandHead_init2start_bh = ""
        self.TSRChainStringFeetandHead_init2start_lh = ""
        self.TSRChainStringFeetandHead_init2start_rh = ""

        # Two hands TSRs
        self.TSRChainString_start2goal = ""
        self.TSRChainStringFeetandHead_goal2start = ""
        self.TSRChainMimicDOF=None

        # One handed
        self.TSRChainString = ""

        # Finish
        self.TSRChainStringFeetandHead_init2home = ""

# ------------------------------------------------------------------------------
    def SetInit2Start(self,T0_TSY):

        T_rh = None
        T_lf = None

        # Defines a box in which to have the end-effector manipulate
        if self.use_manipbox :
            hand_box = matrix( self.manipbox_dim + [-1000,1000,-1000,1000,-1000,1000] )
            T_rh = T0_TSY
            T_lh = T0_TSY
        else :
            hand_box = matrix([-1000,1000,-1000,1000,-1000,1000,-1000,1000,-1000,1000,-1000,1000])
            T_rh = self.robotManips[1].GetEndEffectorTransform()
            T_lh = self.robotManips[0].GetEndEffectorTransform()

        # Define Task Space Region strings
        # Left Hand
        TSRStringLH1 = SerializeTSR(0,'NULL',T_lh,eye(4),hand_box)
        TSRStringLH0 = SerializeTSR(0,'NULL',self.robotManips[0].GetEndEffectorTransform(),eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))
        # Right Hand
        TSRStringRH1 = SerializeTSR(1,'NULL',T_rh,eye(4),hand_box)
        TSRStringRH0 = SerializeTSR(1,'NULL',self.robotManips[1].GetEndEffectorTransform(),eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))

        # Define TSR for this path
        # Left Foot
        TSRStringLF1 = SerializeTSR(2,'NULL', self.robotManips[2].GetEndEffectorTransform(), eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))
        # Right Foot
        TSRStringRF1 = SerializeTSR(3,'NULL', self.robotManips[3].GetEndEffectorTransform(), eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))
        # Head
        TSRStringH = SerializeTSR(4,'NULL', self.robotManips[4].GetEndEffectorTransform(),eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))

        # We defined Task Space Regions. Now let's concatenate them.
        self.TSRChainStringFeetandHead_init2start_bh = SerializeTSRChain(0,0,1,1,TSRStringLH1,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRStringRH1,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRStringLF1,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRStringRF1,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRStringH,'NULL',[])

        self.TSRChainStringFeetandHead_init2start_lh = SerializeTSRChain(0,0,1,1,TSRStringLH1,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRStringRH0,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRStringLF1,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRStringRF1,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRStringH,'NULL',[])

        self.TSRChainStringFeetandHead_init2start_rh = SerializeTSRChain(0,0,1,1,TSRStringLH0,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRStringRH1,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRStringLF1,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRStringRF1,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRStringH,'NULL',[])

# ------------------------------------------------------------------------------
    def SetCurrent2Init(self):

        # Define TSR for this path
        TSRStringLF0 = SerializeTSR(2,'NULL',self.robotManips[2].GetEndEffectorTransform(),eye(4),matrix([0,0,0,0,-100,100,0,0,0,0,0,0]))
        TSRStringRF0 = SerializeTSR(3,'NULL',self.robotManips[3].GetEndEffectorTransform(),eye(4),matrix([0,0,0,0,-100,100,0,0,0,0,0,0]))
        TSRStringH = SerializeTSR(4,'NULL',self.robotManips[4].GetEndEffectorTransform(),eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))
        self.TSRChainStringFeetandHead_current2init = SerializeTSRChain(0,0,1,1,TSRStringLF0,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRStringRF0,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRStringH,'NULL',[])

# ------------------------------------------------------------------------------
    def SetTwoHandsTurn(self,valveJointInd,T0_w0L,Tw0_eL,Bw0L,\
                                           T0_w0R,Tw0_eR,Bw0R,\
                                           T0_w0H,Tw0_eH,Bw0H):
        # Define Task Space Regions
        # Left Hand
        TSRStringLH2 = SerializeTSR(0,'NULL',T0_w0L,Tw0_eL,Bw0L)
        # Right Hand
        TSRStringRH2 = SerializeTSR(1,'crank crank',T0_w0R,Tw0_eR,Bw0R)
        # Left Foot
        TSRStringLF = SerializeTSR(2,'NULL',self.robotManips[2].GetEndEffectorTransform(),eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))
        # Right Foot
        TSRStringRF = SerializeTSR(3,'NULL',self.robotManips[3].GetEndEffectorTransform(),eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))
        # Head
        TSRStringH = SerializeTSR(4,'NULL',T0_w0H,Tw0_eH,Bw0H)

        self.TSRChainMimicDOF = 1

        self.TSRChainStringFeetandHead_goal2start = SerializeTSRChain(0,0,1,1,TSRStringLF,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRStringRF,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRStringH,'NULL',[])

        self.TSRChainString_start2goal = SerializeTSRChain(0,0,1,1,TSRStringLH2,'crank',matrix([valveJointInd]))+' '+SerializeTSRChain(0,0,1,1,TSRStringRH2,'NULL',matrix([]))+' '+self.TSRChainStringFeetandHead_goal2start

# ------------------------------------------------------------------------------
    def SetLeftHandTurn(self,valveJointInd,T0_w0L,Tw0_eL,Bw0L,T0_w0H,Tw0_eH,Bw0H,T0_RH):

        # Define Task Space Regions
        # Left Hand
        TSRString1 = SerializeTSR(0,'NULL',T0_w0L,Tw0_eL,Bw0L)
        # Right Hand
        TSRString2 = SerializeTSR(1,'NULL',T0_RH,eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))
        # Left Foot
        TSRString3 = SerializeTSR(2,'NULL',self.robotManips[2].GetEndEffectorTransform(),eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))
        # Right Foot

        TSRString4 = SerializeTSR(3,'NULL',self.robotManips[3].GetEndEffectorTransform(),eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))
        # Head
        TSRString5 = SerializeTSR(4,'NULL',T0_w0H,Tw0_eH,Bw0H)

        self.TSRChainStringFeetHeadandRightHand_start2init = SerializeTSRChain(0,0,1,1,TSRString2,'NULL',[])+SerializeTSRChain(0,0,1,1,TSRString3,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRString4,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRString5,'NULL',[])

        self.TSRChainStringFeetRightHandandHead_goal2start = SerializeTSRChain(0,0,1,1,TSRString2,'NULL',[])+SerializeTSRChain(0,0,1,1,TSRString3,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRString4,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRString5,'NULL',[])

        self.TSRChainStringFeetandHead_goal2start = SerializeTSRChain(0,0,1,1,TSRString3,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRString4,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRString5,'NULL',[])

        self.TSRChainString = SerializeTSRChain(0,0,1,1,TSRString1,'crank',matrix([valveJointInd]))+' '+SerializeTSRChain(0,0,1,1,TSRString2,'NULL',matrix([]))+' '+self.TSRChainStringFeetandHead_goal2start

        # Which joint do we want the CBiRRT to mimic the TSR for?
        self.TSRChainMimicDOF = 1

# ------------------------------------------------------------------------------
    def SetRightHandTurn(self,valveJointInd,T0_w0R,Tw0_eR,Bw0R,T0_w0H,Tw0_eH,Bw0H,T0_LH):

        # Define Task Space Regions
        # Left Hand
        TSRString1 = SerializeTSR(0,'NULL',T0_LH,eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))
        # Right Hand
        TSRString2 = SerializeTSR(1,'NULL',T0_w0R,Tw0_eR,Bw0R)
        # Left Foot
        TSRString3 = SerializeTSR(2,'NULL',self.robotManips[2].GetEndEffectorTransform(),eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))
        # Right Foot
        TSRString4 = SerializeTSR(3,'NULL',self.robotManips[3].GetEndEffectorTransform(),eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))
        # Head
        TSRString5 = SerializeTSR(4,'NULL',T0_w0H,Tw0_eH,Bw0H)

#        if( hands == "BH" ):
#            self.TSRs.TSRChainStringFeetHeadandLeftHand_start2init = SerializeTSRChain(0,0,1,1,TSRString1,'NULL',[])+SerializeTSRChain(0,0,1,1,TSRString3,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRString4,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRString5,'NULL',[])

        self.TSRChainStringFeetandHead_goal2start = SerializeTSRChain(0,0,1,1,TSRString3,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRString4,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRString5,'NULL',[])
        
        self.TSRChainStringFeetLeftHandandHead_goal2start = SerializeTSRChain(0,0,1,1,TSRString1,'NULL',[])+SerializeTSRChain(0,0,1,1,TSRString3,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRString4,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRString5,'NULL',[])

        self.TSRChainString = SerializeTSRChain(0,0,1,1,TSRString1,'NULL',matrix([]))+' '+SerializeTSRChain(0,0,1,1,TSRString2,'crank',matrix([valveJointInd]))+' '+self.TSRChainStringFeetandHead_goal2start

        # Which joint do we want the CBiRRT to mimic the TSR for?
        self.TSRChainMimicDOF = 1

# ------------------------------------------------------------------------------
    def SetEnd(self, robotid, currentik, initik, T0_TSY):
        
        robotid.SetActiveDOFValues(currentik)
        
        T_rh = None
        T_lf = None

        # Defines a box in which to have the end-effector manipulate
        if self.use_manipbox :
            hand_box = matrix( self.manipbox_dim + [-1000,1000,-1000,1000,-1000,1000] )
            T_rh = T0_TSY
            T_lh = T0_TSY
        else :
            hand_box = matrix([-1000,1000,-1000,1000,-1000,1000,-1000,1000,-1000,1000,-1000,1000])
            T_rh = self.robotManips[1].GetEndEffectorTransform()
            T_lh = self.robotManips[0].GetEndEffectorTransform()

        # Define Task Space Region strings
        # Left Hand
        TSRStringLH1 = SerializeTSR(0,'NULL',T_lh,eye(4),hand_box)
        TSRStringLH0 = SerializeTSR(0,'NULL',self.robotManips[0].GetEndEffectorTransform(),eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))
        # Right Hand
        TSRStringRH1 = SerializeTSR(1,'NULL',T_rh,eye(4),hand_box)
        TSRStringRH0 = SerializeTSR(1,'NULL',self.robotManips[1].GetEndEffectorTransform(),eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))
        # Left Foot
        TSRStringLF1 = SerializeTSR(2,'NULL',self.robotManips[2].GetEndEffectorTransform(),eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))
        # Right Foot
        TSRStringRF1 = SerializeTSR(3,'NULL',self.robotManips[3].GetEndEffectorTransform(),eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))
        # Head
        TSRStringH = SerializeTSR(4,'NULL',self.robotManips[4].GetEndEffectorTransform(),eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))

        if(type(initik) == type("")):
            robotid.SetActiveDOFValues(str2num(initik))
        else:
            robotid.SetActiveDOFValues(initik)

        robotid.GetController().Reset(0)
        
        # Left Foot
        TSRStringLF2 = SerializeTSR(2,'NULL',self.robotManips[2].GetEndEffectorTransform(),eye(4),matrix([0,0,0,0,-100,100,0,0,0,0,0,0]))
        # Right Foot
        TSRStringRF2 = SerializeTSR(3,'NULL',self.robotManips[3].GetEndEffectorTransform(),eye(4),matrix([0,0,0,0,-100,100,0,0,0,0,0,0]))

        # We have the strings. Let's chain them together.
        
        self.TSRChainStringFeetandHead_current2init_bh = SerializeTSRChain(0,0,1,1,TSRStringLH1,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRStringRH1,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRStringLF1,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRStringRF1,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRStringH,'NULL',[])
        self.TSRChainStringFeetandHead_current2init_lh = SerializeTSRChain(0,0,1,1,TSRStringRH0,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRStringLF1,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRStringRF1,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRStringH,'NULL',[])
        self.TSRChainStringFeetandHead_current2init_rh = SerializeTSRChain(0,0,1,1,TSRStringLH0,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRStringLF1,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRStringRF1,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRStringH,'NULL',[])
        self.TSRChainStringFeetandHead_init2home = SerializeTSRChain(0,0,1,1,TSRStringLF2,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRStringRF2,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRStringH,'NULL',[])

