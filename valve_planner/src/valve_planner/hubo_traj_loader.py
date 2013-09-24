#!/usr/bin/env python
# Ben Suay, RAIL
# May 2013
# Worcester Polytechnic Institute
#

# http://openrave.org/docs/latest_stable/command_line_tools/
# openrave-robot.py /your/path/to/your.robot.xml --info=joints
# On that page you can find more examples on how to use openrave-robot.py.

from openravepy import *
import sys
if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *
    import numpy
import time
from rodrigues import *
from TransformMatrix import *
from str2num import *
from TSR import *
from math import *
from copy import *
import os # for file operations
from RaveCBiRRT import *

def trans_to_str(T):
    myStr = ""
    for c in range(0,3):
        for r in range(0,3):
            myStr += str(T[r,c])+" "
    
    for r in range(0,3):
        myStr += str(T[r,3])+" "

    #print "Tee string : " 
    #print myStr
    return myStr

class HuboPlusWheelTurning:

    def __init__(self,
                 HuboModelPath = '../../openHubo/huboplus/rlhuboplus.robot.xml',
                 WheelModelPath = '../../../drc_common/models/driving_wheel.robot.xml' ):
        
        # Set those variables to show or hide the interface
        # Do it using the member functions
        self.StopAtKeyStrokes = False
        self.ShowUserInterface = False
        self.ViewerStarted = False

        self.T_Wheel = None
        self.HuboModelPath = HuboModelPath
        self.WheelModelPath = WheelModelPath

        # Start Environment
        self.env = Environment()
        RaveSetDebugLevel(DebugLevel.Info) # set output level to debug
        self.robotid = self.env.ReadRobotURI(self.HuboModelPath)
        self.crankid = self.env.ReadRobotURI(self.WheelModelPath)
        self.env.Add(self.robotid)
        self.env.Add(self.crankid)

        print "init HuboPlusWheelTurning"

    def KillOpenrave(self):
        self.env.Destroy()
        RaveDestroy()

    def SetViewer(self,arg=True):
        print "SetViewer"
        self.ShowUserInterface = arg

    def SetStopKeyStrokes(self,arg=True):
        print "SetStopKeyStrokes"
        self.StopAtKeyStrokes = arg

    def SetWheelPosition(self,trans,rot):
        print "SetWheelPosition"
        self.T_Wheel = MakeTransform(rotationMatrixFromQuat(rot),matrix(trans))
        self.crankid.SetTransform(array(self.T_Wheel))

    def SetRobotConfiguration(self,jointValues):
        print "SetRobotConfiguration"
        values = []
        values.append( jointValues['HPY'] ) # 0
        values.append( jointValues['RHY'] ) # 1
        values.append( jointValues['LHY'] ) # 2
        values.append( jointValues['RHR'] ) # 3
        values.append( jointValues['HPY'] ) # 4
        values.append( jointValues['LHR'] ) # 5
        values.append( jointValues['LHP'] ) # 6
        values.append( jointValues['RKP'] ) # 7
        values.append( jointValues['LKP'] ) # 8
        values.append( jointValues['RAP'] ) # 9
        values.append( jointValues['LAP'] ) # 10
        values.append( jointValues['RAR'] ) # 11
        values.append( jointValues['LAR'] ) # 12
        values.append( jointValues['RSP'] ) # 13 
        values.append( jointValues['LSP'] ) # 14 
        values.append( jointValues['RSR'] ) # 15
        values.append( jointValues['LSR'] ) # 16
        values.append( jointValues['RSY'] ) # 17 
        values.append( jointValues['LSY'] ) # 18
        values.append( jointValues['REP'] ) # 19
        values.append( jointValues['LEP'] ) # 20
        values.append( jointValues['RWY'] ) # 21
        values.append( jointValues['LWY'] ) # 22
        values.append( jointValues['RWP'] ) # 23
        values.append( jointValues['LWP'] ) # 24
        values.append( jointValues['HNR'] ) # 25
        values.append( jointValues['HNP'] ) # 26

        for i in range(27,57):
            values.append(0)

#        values.append( jointValues['rightIndexKnuckle2'] ) # 27
#        values.append( jointValues['rightIndexKnuckle3'] ) # 28
#        values.append( jointValues['rightIndexKnuckle1'] ) # 29
#        values.append( jointValues['rightMiddleKnuckle2'] ) # 30
#        values.append( jointValues['rightMiddleKnuckle3'] ) # 31
#        values.append( jointValues['rightMiddleKnuckle1'] ) # 32
#        values.append( jointValues['rightRingKnuckle2'] ) # 33
#        values.append( jointValues['rightRingKnuckle3'] ) # 34
#        values.append( jointValues['rightRingKnuckle1'] ) # 35
#        values.append( jointValues['rightPinkyKnuckle2'] ) # 36
#        values.append( jointValues['rightPinkyKnuckle3'] ) # 37
#        values.append( jointValues['rightPinkyKnuckle1'] ) # 38
#        values.append( jointValues['rightThumbKnuckle2'] ) # 39
#        values.append( jointValues['rightThumbKnuckle3'] ) # 40
#        values.append( jointValues['rightThumbKnuckle1'] ) # 41
#        values.append( jointValues['leftIndexKnuckle2'] ) # 42
#        values.append( jointValues['leftIndexKnuckle3'] ) # 43
#        values.append( jointValues['leftIndexKnuckle1'] ) # 44
#        values.append( jointValues['leftMiddleKnuckle2'] ) # 45
#        values.append( jointValues['leftMiddleKnuckle3'] ) # 46
#        values.append( jointValues['leftMiddleKnuckle1'] ) # 47
#        values.append( jointValues['leftRingKnuckle2'] ) # 48
#        values.append( jointValues['leftRingKnuckle3'] ) # 49
#        values.append( jointValues['leftRingKnuckle1'] ) # 50
#        values.append( jointValues['leftPinkyKnuckle2'] ) # 51
#        values.append( jointValues['leftPinkyKnuckle3'] ) # 52
#        values.append( jointValues['leftPinkyKnuckle1'] ) # 53
#        values.append( jointValues['leftThumbKnuckle2'] ) # 54
#        values.append( jointValues['leftThumbKnuckle3'] ) # 55
#        values.append( jointValues['leftThumbKnuckle1'] ) # 56
        self.robotid.SetDOFValues( values )
              
    def Run(self):  


        # This is a list of handles of the objects that are
        # drawn on the screen in OpenRAVE Qt-Viewer.
        # Keep appending to the end, and pop() if you want to delete.
        handles = [] 

        normalsmoothingitrs = 150;
        fastsmoothingitrs = 20;

                # Right Hand Joints 
        # Open - Closed Values
        rhanddofs = range(27,42)
        rhandclosevals = [0.439, 0.683, 0.497, 0.439, 0.683, 0.497, 0.439, 0.683, 0.497, 0.439, 0.683, 0.497, 0, 0, 1.2]
        rhandopenvals = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.08]

        # Left Hand Joints
        lhanddofs = range(42,57)
        lhandclosevals = [0.439, 0.683, 0.497, 0.439, 0.683, 0.497, 0.439, 0.683, 0.497, 0.439, 0.683, 0.497, 0, 0, 1.2]
        lhandopenvals =  [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.08]

        # Start the Viewer and draws the world frame
        if self.ShowUserInterface and not self.ViewerStarted :
            cam_rot = dot(xyz_rotation([3*pi/2,0,0]),xyz_rotation([0,-pi/2,0]))
            cam_rot = dot(cam_rot,xyz_rotation([-pi/10,0,0])) # inclination of the camera
            T_cam = MakeTransform(cam_rot,transpose(matrix([2.0, 0.00, 01.4])))
            self.env.SetViewer('qtcoin')
            self.env.GetViewer().SetCamera(array(T_cam))
            self.env.GetViewer().EnvironmentSync()
            self.ViewerStarted = True
            #handles.append( misc.DrawAxes(self.env,T_cam,1) )
            handles.append( misc.DrawAxes(self.env,MakeTransform(rodrigues([0,0,0]),transpose(matrix([0,0,0]))),1) )
        if( self.StopAtKeyStrokes ):
            print "Press Enter to exit..."
            sys.stdin.readline()
            
        # Wheel Joint Index  
        crankjointind = 0
        # Set the wheel joints back to 0 for replanning
        self.crankid.SetDOFValues([0],[crankjointind])
        self.crankid.GetController().Reset(0)

        # Move the wheel infront of the robot
        if self.T_Wheel is None:
            self.T_Wheel = MakeTransform(dot(rodrigues([0,0,pi/2]),rodrigues([pi/2,0,0])),transpose(matrix([0.18, 0.0851953, 0.85])))
            self.crankid.SetTransform(array(self.T_Wheel))
  
        # Draw wheel position
        if self.ShowUserInterface :
            print self.robotid.GetJoints()[11].GetAnchor()
            T_RightFoot = self.robotid.GetLinks()[0].GetTransform()
            T_Torso = self.robotid.GetLinks()[8].GetTransform()
            handles.append( misc.DrawAxes(self.env,self.T_Wheel,0.5) )  
            handles.append( misc.DrawAxes(self.env,T_Torso,0.5) ) 
            handles.append( misc.DrawAxes(self.env,T_RightFoot,0.5) ) 
            print T_Torso
            print T_RightFoot

        manips = self.robotid.GetManipulators()
        crankmanip = self.crankid.GetManipulators()
        
        try:
            cbirrtHubo = RaveCBiRRT(self.env,'rlhuboplus')
            cbirrtWheel = RaveCBiRRT(self.env,'crank')
        except openrave_exception, e:
            print e
            return []

        try:
            answer= cbirrtHubo.solve('traj movetraj0.txt');
            self.robotid.WaitForController(0)
            # debug
            print "traj call answer: ",str(answer)
        except openrave_exception, e:
            print e
            return []

        self.robotid.GetController().Reset(0)
        time.sleep(1)

        self.robotid.SetDOFValues(rhandclosevals,rhanddofs)
        self.robotid.SetDOFValues(lhandclosevals,lhanddofs)
        time.sleep(1)

        try:
            answer= cbirrtHubo.solve('traj movetraj1.txt');
            answer= cbirrtWheel.solve('traj movetraj1.txt');
            self.robotid.WaitForController(0)
            # debug
            print "traj call answer: ",str(answer)
        except openrave_exception, e:
            print e
            return []

        self.robotid.GetController().Reset(0)
        time.sleep(1)

        self.robotid.SetDOFValues(rhandopenvals,rhanddofs)
        self.robotid.SetDOFValues(lhandopenvals,lhanddofs)
        time.sleep(1)

        try:
            answer= cbirrtHubo.solve('traj movetraj2.txt');
            self.robotid.WaitForController(0)
            # debug
            print "traj call answer: ",str(answer)
        except openrave_exception, e:
            print e
            return []
        
        self.robotid.GetController().Reset(0)
        time.sleep(1)

        try:
            answer= cbirrtHubo.solve('traj movetraj3.txt');
            self.robotid.WaitForController(0)
            # debug
            print "traj call answer: ",str(answer)
        except openrave_exception, e:
            print e
            return []
        
        self.robotid.GetController().Reset(0)    
        
        if( self.StopAtKeyStrokes ):
            print "Press Enter to exit..."
            sys.stdin.readline()

        file_names = [ 'movetraj0.txt','movetraj1.txt','movetraj2.txt','movetraj1.txt','movetraj3.txt']
        return file_names


if __name__ == "__main__":
    planner = HuboPlusWheelTurning()
    planner.SetViewer(True)
    planner.SetStopKeyStrokes(True)
    planner.Run()
    planner.KillOpenrave()

    
