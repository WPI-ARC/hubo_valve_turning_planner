#!/usr/bin/env python
# Jim Mainprice of ARC, and,
# Ben Suay of RAIL
# May 2013
# Worcester Polytechnic Institute
#

# http://openrave.org/docs/latest_stable/command_line_tools/
# openrave-robot.py /your/path/to/your.robot.xml --info=joints
# On that page you can find more examples on how to use openrave-robot.py.

from openravepy import *
import rospy
import roslib

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
from wpi_planning_utilities.RaveCBiRRT import *

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

# Robot agnostic class for valve turning
# Contains some ROS functions
class BaseWheelTurning:

    def __init__(self, HuboModelPath, WheelModelPath ):
        
		# Height for crouching
        self.crouch = 0.05

        self.default_trajectory_dir = roslib.packages.get_pkg_dir('valve_planner')+"/trajectories/"
        
        # Set those variables to show or hide the interface
        # Do it using the member functions
        self.StopAtKeyStrokes = False
        self.ShowUserInterface = False
        self.ViewerStarted = False

        self.T_Wheel = None # Wheel transform in world coords.
        self.r_Wheel = None # Default Wheel radius
        self.HuboModelPath = HuboModelPath
        self.WheelModelPath = WheelModelPath

        # User specified poses
        self.useUserPoses = False
        self.T0_LH_USER = None
        self.T0_RH_USER = None
        self.LH_USER_offset = None
        self.RH_USER_offset = None

        # Start Environment
        self.env = Environment()
        #self.env.SetDebugLevel(DebugLevel.Verbose)
        self.env.SetDebugLevel(DebugLevel.Info) # set output level to debug
        self.robotid = self.env.ReadRobotURI(self.HuboModelPath)
        self.crankid = self.env.ReadRobotURI(self.WheelModelPath)
        self.env.Add(self.robotid)
        self.env.Add(self.crankid)

        # Hands values
        self.rhandopenvals = None
        self.rhandclosevals = None
        self.rhanddofs = None
        self.lhandopenvals = None
        self.lhandclosevals = None
        self.lhanddofs = None

        # Waist Padding
        self.myWaist1 = None
        self.myWaist2 = None
        self.myWaist3 = None

        # Valve
        self.myValveHandle = None
        self.infocylinder = None
        self.wallPadding = None

        self.probs_cbirrt = None
        self.probs_crankmover = None

        self.T0_tsy_home = self.robotid.GetLinks()[12].GetTransform()
        self.T0_lar_home = self.robotid.GetManipulators()[2].GetEndEffectorTransform()
        self.T0_rar_home = self.robotid.GetManipulators()[3].GetEndEffectorTransform()
        self.Ttsy_lar_home = array(dot(linalg.inv(self.T0_tsy_home),self.T0_lar_home))
        self.Ttsy_rar_home = array(dot(linalg.inv(self.T0_tsy_home),self.T0_rar_home))

        self.wheelDistFromTSY = 0.4

        self.drawingHandles = []

        # CBiRRT variables
        # polyscale: changes the scale of the support polygon
        # polytrans: shifts the support polygon around
        self.footlinknames = ' Body_RAR Body_LAR polyscale 0.5 0.5 0 ' #polytrans -0.015 0 0.0 '

        # Center of Gravity Target
        self.cogtarg = [0, 0, 0]
        self.cogTargStr = str(self.cogtarg).strip("[]").replace(', ',' ')

    # Create joint dictionary for fast access
    def GenerateJointDict(self):
        self.jointDict = {}
        self.jointNames = {}
        for jIdx, j in enumerate(self.robotid.GetJoints()):
            self.jointDict[j.GetName()] = jIdx
            self.jointNames[jIdx] = j.GetName()

    def PrintJointLimits(self):
        for jIdx, j in enumerate(self.robotid.GetJoints()):
            print "%s, \t%.3f, \t%.3f" % ( j.GetName() , j.GetLimits()[0] , j.GetLimits()[1] )

    # Returns the transform of a particular link
    def GetT0_RefLink(self, frame):
        T0_RefLink = None
        for l in self.robotid.GetLinks():
            if(l.GetName() == frame):
                T0_RefLink = l.GetTransform()
        return T0_RefLink

    def AreConfigEqual(self, q1, q2, tol):
        if len(q1) != len(q2):
            return False
        for a, b in zip(q1, q2):
            if not abs(a-b) < tol:
                return False
        return True

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

    def RemoveObstFromEnv(self):
        # Remove the valve (cylinder or box)
        if(self.env.GetKinBody("valve") is not None):
            self.env.RemoveKinBody(self.myValveHandle)

        if(self.env.GetKinBody("rotatedLever") is not None):
            self.env.RemoveKinBody(self.myRotatedLeverHandle)

        if(self.env.GetKinBody("4by4") is not None):
            self.env.RemoveKinBody(self.my4by4)
            
        if(self.env.GetKinBody("1by1") is not None):
            self.env.RemoveKinBody(self.my1by1)

        if(self.env.GetKinBody("supportbeams") is not None):
            self.env.RemoveKinBody(self.mysupport)

    def ResetEnv(self):

        self.RemoveObstFromEnv()
        
        if(self.infocylinder != None):
            self.infocylinder = None

        # Remove the CBiRRT problems
        if(self.probs_cbirrt != None):
            self.env.Remove(self.probs_cbirrt)
            self.probs_cbirrt = None

        if(self.probs_crankmover != None):
            self.env.Remove(self.probs_crankmover)
            self.probs_crankmover = None

        # Reset the valve's joint for replanning
        self.crankid.SetDOFValues([0],[0])
        self.crankid.GetController().Reset(0)

    def SetRightHandPoseFromQuaterninonInFrame(self,frame,trans,rot):
        print "Set Right Hand Pose From Quaterninon"
        T0_RefLink = self.GetT0_RefLink(frame)
        if(T0_RefLink == None):
            rospy.logerr("In base_wheel_turning, SetRightHandPoseFromQuaterninonInFrame: Couldn't find the reference link name.")
        else:
            print "rotation matrix from quat - using openrave function"
            T_RH_RefLink = MakeTransform(matrixFromQuat([rot[3],rot[0],rot[1],rot[2]])[0:3,0:3],matrix(trans))
            T0_RH_RViz = T0_RefLink * T_RH_RefLink
            T_RH_RViz_RH_Rave = MakeTransform( dot(xyz_rotation([pi/2,0,0]),xyz_rotation([0,0,pi/2])),transpose(matrix([0,0,0])))
            self.T0_RH_USER = T0_RH_RViz * T_RH_RViz_RH_Rave
        print "T0_LH_USER : "
        print self.T0_RH_USER
        return

    def SetLeftHandPoseFromQuaterninonInFrame(self,frame,trans,rot):
        print "Set Left Hand Pose From Quaterninon"
        T0_RefLink = self.GetT0_RefLink(frame)
        if(T0_RefLink == None):
            rospy.logerr("In base_wheel_turning, SetLeftHandPoseFromQuaterninonInFrame: Couldn't find the reference link name.")
        else:
            print "rotation matrix from quat - using openrave function"
            T_LH_RefLink = MakeTransform(matrixFromQuat([rot[3],rot[0],rot[1],rot[2]])[0:3,0:3],matrix(trans))
            T0_LH_RViz = T0_RefLink * T_LH_RefLink
            T_LH_RViz_LH_Rave = MakeTransform( dot(xyz_rotation([pi/2,0,0]),xyz_rotation([0,0,pi/2])),transpose(matrix([0,0,0])))
            self.T0_LH_USER = T0_LH_RViz * T_LH_RViz_LH_Rave
        print "T0_RH_USER : "
        print self.T0_LH_USER
        return

    def SetUserPoses( self, UserPoses ):

        # TODO handle mixed user set and auto
        useLeft  = UserPoses[0]
        useRight = UserPoses[1]

        self.T0_LH_USER = None
        self.T0_RH_USER = None
        self.LH_USER_offset = None
        self.RH_USER_offset = None

        if not useLeft and not useRight :
            self.useUserPoses = False
            return

        if useLeft :
            self.SetLeftHandPoseFromQuaterninonInFrame( UserPoses[2], UserPoses[3], UserPoses[4] )
            self.LH_USER_offset = UserPoses[5]
            self.drawingHandles.append( misc.DrawAxes(self.env,self.T0_LH_USER,1) )
        if useRight :
            self.SetRightHandPoseFromQuaterninonInFrame( UserPoses[6], UserPoses[7], UserPoses[8] )
            self.RH_USER_offset = UserPoses[9]
            self.drawingHandles.append( misc.DrawAxes(self.env,self.T0_RH_USER,1) )

        return

    def SetValvePoseFromQuaternionInFrame(self,frame,trans,rot):
        print "SetWheelPoseFromQuaternion"

        #self.tiltDiff = acos(dot(linalg.inv(self.crankid.GetManipulators()[0].GetEndEffectorTransform()),self.crankid.GetLinks()[0].GetTransform())[1,1])
        
        self.tiltDiff = 0

        self.refLinkName = frame # For future reference
        self.T0_RefLink = self.GetT0_RefLink(self.refLinkName)
        
        if(self.T0_RefLink == None):
            rospy.logerr("In base_wheel_turning, SetWheelPoseFromQuaternion: Couldn't find the reference link name.")
        else:
            print "rotation matrix from quat - using openrave function"
            self.TRefLink_Wheel = MakeTransform(matrixFromQuat([rot[3],rot[0],rot[1],rot[2]])[0:3,0:3],matrix(trans))
            self.T0_WheelRViz = dot(self.T0_RefLink,self.TRefLink_Wheel)   
            self.TWheelRViz_WheelRave = MakeTransform(dot(rodrigues([0,pi/2-self.tiltDiff,0]),rodrigues([0,0,pi/2])),transpose(matrix([0,0,0])))
            self.T0_WheelRave = dot(self.T0_WheelRViz,self.TWheelRViz_WheelRave)

            # Set wheel location
            #
            # deprecated
            # # TODO We shall plan after crouchin. But until then just think that the valve 
            # # is self.crouch higher. This should be the height of the real wheel after the 
            # # robot crouches.
            # self.T0_WheelRave[2,3] += self.crouch

            self.crankid.SetTransform(array(self.T0_WheelRave))

        self.wheelDistFromTSY = self.T0_WheelRave[0,3]
        print "wheelDistFromTSY : "  + str(self.wheelDistFromTSY)

    def PadValve(self,valveType):

        T_valve = self.crankid.GetManipulators()[0].GetTransform()

        self.wallPadding = self.AddWall('wall_padding',0.03)
        self.wallPadding.GetLinks()[0].GetGeometries()[0].SetDiffuseColor(array((0,0,1)))
        self.wallPadding.GetLinks()[0].GetGeometries()[0].SetTransparency(0.3)
        T_wall = deepcopy(T_valve)
        T_wall[2,3] += 0.3
        self.wallPadding.SetTransform(T_wall)

        if(self.env.GetKinBody("valve") is not None):
            self.env.RemoveKinBody(self.myValveHandle)
        else:
            return

        self.myValveHandle = RaveCreateKinBody(self.env,'')

        myPaddingValue = 0.03
        if(valveType == "RL"): #TODO # valve type: lever with right end at the origin of rotation
            
            self.myValveHandle.InitFromBoxes(numpy.array([[self.r_Wheel*0.5,0,0,self.r_Wheel*0.5+myPaddingValue,0.01+myPaddingValue,0.005+myPaddingValue]]),True)
            
        if(valveType == "LL"): #TODO # valve type: lever with left end at the origin of rotation
            
            self.myValveHandle.InitFromBoxes(numpy.array([[-self.r_Wheel*0.5,0,0,self.r_Wheel*0.5+myPaddingValue,0.01+myPaddingValue,0.005+myPaddingValue]]),True)

        if(valveType == "W"): # valve type: wheel
            # Create a cylinder
            self.infocylinder._vGeomData = [self.r_Wheel+0.07,0.05] # radius and height/thickness        
            self.myValveHandle.InitFromGeometries([self.infocylinder]) # we could add more cylinders in the list

        self.myValveHandle.SetName('valve')
        self.myValveHandle.SetTransform(T_valve)
        self.myValveHandle.GetLinks()[0].GetGeometries()[0].SetTransparency(0.3)
        self.env.Add(self.myValveHandle,True)

        
        if(self.env.GetKinBody("rotatedLever") is not None):
            self.env.RemoveKinBody(self.myRotatedLeverHandle)

            # Do the following only if the rotatedLever body is in the environment.
            self.myRotatedLeverHandle = RaveCreateKinBody(self.env,'')
            myPaddingValue = 0.01
            
            self.myRotatedLeverHandle.InitFromBoxes(numpy.array([[-self.r_Wheel*0.5,0,0,self.r_Wheel*0.5+myPaddingValue,0.01+myPaddingValue,0.005+myPaddingValue]]),True)
            self.myRotatedLeverHandle.SetName('rotatedLever')
            self.env.Add(self.myRotatedLeverHandle,True)
            self.myRotatedLeverHandle.SetTransform(array(dot(self.crankid.GetManipulators()[0].GetTransform(),MakeTransform(rodrigues([0,0,pi/2]),transpose(matrix([0,0,0]))))))
            self.myRotatedLeverHandle.GetLinks()[0].GetGeometries()[0].SetTransparency(0.3)            
        
    def UnpadValve(self,valveType):

        if(self.env.GetKinBody("valve") is not None):
            self.env.RemoveKinBody(self.myValveHandle)
        else :
            return

        self.myValveHandle = RaveCreateKinBody(self.env,'')

        if(self.wallPadding is not None):
            self.env.RemoveKinBody(self.wallPadding)

        if(valveType == "RL"): 
            self.myValveHandle.InitFromBoxes(numpy.array([[self.r_Wheel*0.5,0,0,self.r_Wheel*0.5,0.01,0.005]]),True)
            
        if(valveType == "LL"): 
            self.myValveHandle.InitFromBoxes(numpy.array([[-self.r_Wheel*0.5,0,0,self.r_Wheel*0.5,0.01,0.005]]),True)
 
        if(valveType == "W"): # valve type: wheel
            # Create a cylinder
            self.infocylinder._vGeomData = [self.r_Wheel,0.01] # radius and height/thickness        
            self.myValveHandle.InitFromGeometries([self.infocylinder]) # we could add more cylinders in the list

        self.myValveHandle.SetName('valve')
        self.myValveHandle.SetTransform(self.crankid.GetManipulators()[0].GetTransform())
        self.myValveHandle.GetLinks()[0].GetGeometries()[0].SetTransparency(0.0)
        self.env.Add(self.myValveHandle,True)

        if(self.env.GetKinBody("rotatedLever") is not None):
            self.env.RemoveKinBody(self.myRotatedLeverHandle)

            # Do the following only if the rotatedLever body is in the environment.
            self.myRotatedLeverHandle = RaveCreateKinBody(self.env,'')
            
            self.myRotatedLeverHandle.InitFromBoxes(numpy.array([[-self.r_Wheel*0.5,0,0,self.r_Wheel*0.5,0.01,0.005]]),True)
            self.myRotatedLeverHandle.SetName('rotatedLever')
            self.env.Add(self.myRotatedLeverHandle,True)
            self.myRotatedLeverHandle.SetTransform(array(dot(self.crankid.GetManipulators()[0].GetTransform(),MakeTransform(rodrigues([0,0,pi/2]),transpose(matrix([0,0,0]))))))
            self.myRotatedLeverHandle.GetLinks()[0].GetGeometries()[0].SetTransparency(0.0)            

        

    def PadWaist(self,T):
        print "adding a wall"
        self.myWaist1 = RaveCreateKinBody(self.env,'')
        self.myWaist1.SetName('waist_pad1')
        self.myWaist1.InitFromBoxes(numpy.array([[0.11,0,-0.05,0.005,0.18,.10]]),True) # False for not visible
        self.myWaist1.GetLinks()[0].GetGeometries()[0].SetDiffuseColor(array((0.3,0.3,0.3)))
        #self.myWaist.GetLinks()[0].GetGeometries()[0].SetTransparency(0.5)
        self.myWaist1.SetTransform(T)

        self.myWaist2 = RaveCreateKinBody(self.env,'')
        self.myWaist2.SetName('waist_pad2')
        self.myWaist2.InitFromBoxes(numpy.array([[0,0.17,-0.05,0.11,0.005,.10]]),True) # False for not visible
        self.myWaist2.GetLinks()[0].GetGeometries()[0].SetDiffuseColor(array((0.3,0.3,0.3)))
        #self.myWaist.GetLinks()[0].GetGeometries()[0].SetTransparency(0.5)
        self.myWaist2.SetTransform(T)

        self.myWaist3 = RaveCreateKinBody(self.env,'')
        self.myWaist3.SetName('waist_pad3')
        self.myWaist3.InitFromBoxes(numpy.array([[0,-0.17,-0.05,0.11,0.005,.10]]),True) # False for not visible
        self.myWaist3.GetLinks()[0].GetGeometries()[0].SetDiffuseColor(array((0.3,0.3,0.3)))
        #self.myWaist.GetLinks()[0].GetGeometries()[0].SetTransparency(0.5)
        self.myWaist3.SetTransform(T)

        self.env.Add(self.myWaist1,True)
        self.env.Add(self.myWaist2,True)
        self.env.Add(self.myWaist3,True)

    def UnPadWaist(self):

        if( self.myWaist1 is not None):
            self.env.Remove(self.myWaist1)
        if( self.myWaist2 is not None):
            self.env.Remove(self.myWaist2)
        if( self.myWaist3 is not None):
            self.env.Remove(self.myWaist3)

    def CreateValve(self,valveRadius,valveType):

        self.r_Wheel = valveRadius

        self.myValveHandle = RaveCreateKinBody(self.env,'')

        if(valveType == "RL"): # valve type: lever with right end at the origin of rotation
            self.myValveHandle.InitFromBoxes(numpy.array([[self.r_Wheel*0.5,0,0,self.r_Wheel*0.5,0.01,0.005]]),True)
        if(valveType == "LL"): # valve type: lever with left end at the origin of rotation
            self.myValveHandle.InitFromBoxes(numpy.array([[-self.r_Wheel*0.5,0,0,self.r_Wheel*0.5,0.01,0.005]]),True)
        elif(valveType == "W"): # valve type: wheel
            # Create a cylinder
            self.infocylinder = KinBody.Link.GeometryInfo()
            self.infocylinder._type = KinBody.Link.GeomType.Cylinder
            self.infocylinder._vGeomData = [self.r_Wheel,0.01] # radius and height/thickness
            self.infocylinder._bVisible = True
            self.infocylinder._fTransparency = 0.7
            self.infocylinder._vDiffuseColor = [0,1,1]  
            self.myValveHandle.InitFromGeometries([self.infocylinder]) # we could add more cylinders in the list

        self.myValveHandle.SetName('valve')
        self.env.Add(self.myValveHandle,True)
        
        self.myValveHandle.SetTransform(self.crankid.GetManipulators()[0].GetTransform())
        # self.Add4by4()
        # self.Add1by1()
        self.AddWPIWheelSupport()

    def AddRotatedLever(self,valveRadius,valveType):

        self.myRotatedLeverHandle = RaveCreateKinBody(self.env,'')

        self.myRotatedLeverHandle.InitFromBoxes(numpy.array([[-valveRadius*0.5,0,0,valveRadius*0.5,0.01,0.005]]),True)
        self.myRotatedLeverHandle.SetName('rotatedLever')
        self.env.Add(self.myRotatedLeverHandle,True)
        
        
        self.myRotatedLeverHandle.SetTransform(array(dot(self.crankid.GetManipulators()[0].GetTransform(),MakeTransform(rodrigues([0,0,pi/2]),transpose(matrix([0,0,0]))))))
    
    def AddWPIWheelSupport(self):
        self.Add1by1()
        self.AddSupportBeams()

    def AddSupportBeams(self):
        print "adding support beams"
        self.mysupport = RaveCreateKinBody(self.env,'')
        self.mysupport.SetName('supportbeams')
        behindValveClearance = 0.15

        # Tall narrow support beams
        # self.mysupport.InitFromBoxes(numpy.array([[behindValveClearance,0,0,0.001,0.1525,1.0]]),True) # False for not visible
        
        # Tall wide wall
        #self.mysupport.InitFromBoxes(numpy.array([[behindValveClearance,0,0,0.001,1.0,1.0]]),True) # False for not visible
        #self.mysupport.GetLinks()[0].GetGeometries()[0].SetDiffuseColor(array((0,0,1)))
        #self.mysupport.GetLinks()[0].GetGeometries()[0].SetTransparency(0.5)

        T_valve = self.crankid.GetManipulators()[0].GetEndEffectorTransform()
        x = T_valve[0,3]
        y = T_valve[1,3]
        z = T_valve[2,3]
        self.mysupport.SetTransform(array(MakeTransform(rodrigues([0,0,0]),transpose(matrix([x,y,z])))))
        self.env.Add(self.mysupport,True)
    
    def Add1by1(self):
        print "adding a wall"
        self.my1by1 = RaveCreateKinBody(self.env,'')
        self.my1by1.SetName('1by1')
        behindValveClearance = 0.075
        self.my1by1.InitFromBoxes(numpy.array([[0,0,behindValveClearance,0.1525,0.1525,0.001]]),True) # False for not visible
        self.my1by1.GetLinks()[0].GetGeometries()[0].SetDiffuseColor(array((0,0,1)))
        self.my1by1.GetLinks()[0].GetGeometries()[0].SetTransparency(0.5)
        self.my1by1.SetTransform(self.crankid.GetManipulators()[0].GetEndEffectorTransform())
        self.env.Add(self.my1by1,True)

    def Add4by4(self):
        print "adding a wall"
        self.my4by4 = RaveCreateKinBody(self.env,'')
        self.my4by4.SetName('4by4')
        behindValveClearance = 0.1
        self.my4by4.InitFromBoxes(numpy.array([[0,0,behindValveClearance,0.61,0.61,0.001]]),True) # False for not visible
        self.my4by4.GetLinks()[0].GetGeometries()[0].SetDiffuseColor(array((0,0,1)))
        self.my4by4.GetLinks()[0].GetGeometries()[0].SetTransparency(0.5)
        self.my4by4.SetTransform(self.crankid.GetManipulators()[0].GetEndEffectorTransform())
        self.env.Add(self.my4by4,True)

    def AddWall(self,name='wall',behindValveClearance=0.00):
        print "adding a wall"
        body = RaveCreateKinBody(self.env,'')
        body.SetName(name)
        # body.InitFromBoxes(numpy.array([[0,0,behindValveClearance,0.80,0.60,0.001]]),True) # False for not visible
        body.InitFromBoxes(numpy.array([[0,0,behindValveClearance,2.0,2.0,0.001]]),True) # False for not visible
        body.GetLinks()[0].GetGeometries()[0].SetDiffuseColor(array((0.5,0.5,1,0.5)))
        self.env.Add(body,True)
        return body

    def InitFromTaskWallEnv(self):
        self.env.Load(roslib.packages.get_pkg_dir("wpi_drc_sim")+'/../models/drc_task_wall.env.xml')

    def PushObjectBack(self,obj,length):
        T = obj.GetTransform()
        T[0,3] += length
        obj.SetTransform(T)

    def PushWallsBack(self,length):
        if self.wallPadding is not None :
            self.PushObjectBack(self.wallPadding,length)
        if self.my1by1 is not None :
            self.PushObjectBack(self.my1by1,length)
        if self.mysupport is not None :
            self.PushObjectBack(self.mysupport,length)

    def MoveCurrentConfigurationOutOfCollision(self):
        # moves the walls back until the robot is not in collision
        for l in linspace(0.00, 0.40, num=50) :
            is_in_col = self.env.CheckCollision(self.robotid)
            if is_in_col is True :
                self.PushWallsBack(l)
            else:
                break
        print "moved the wall back by : " + str(l) + " m."
        return not is_in_col

    def SetWheelPoseFromTransform(self,T0_Wheel):
        print "SetWheelPoseFromTransform"
        self.T_Wheel = T0_Wheel
        self.crankid.SetTransform(array(self.T_Wheel))        

    def RemoveFiles(self):

        # Try to delete all existing trajectory files
        try:
            print "Removing qhullout.txt"
            os.remove("qhullout.txt")
        except OSError, e:
            print e

        for fname in os.listdir(self.default_trajectory_dir):
            if( fname.find("movetraj") != -1 ):
                print "Info: Removing old trajectory file: "+fname
                os.remove(self.default_trajectory_dir+fname)

    def StartViewer(self):
        # Start the Viewer and draws the world frame
        if self.ShowUserInterface and not self.ViewerStarted :

            # You can print the camera orientation using
            # print self.env.GetViewer().GetCameraTransform()
            T_cam =   ([[ 0.90542259, -0.29885114,  0.30149283, -0.13575509], \
                        [-0.42436735, -0.65569112,  0.62448499, -1.19536948], \
                        [ 0.01105812, -0.69336654, -0.72050022,  2.05750942], \
                        [ 0.,          0.,          0.,          1.,        ]])

            self.env.SetViewer('qtcoin')
            self.env.GetViewer().SetCamera(array(T_cam))
            self.env.GetViewer().EnvironmentSync()
            self.ViewerStarted = True

    def SetProblems(self):
        self.probs_cbirrt = RaveCreateModule(self.env,'CBiRRT')
        self.probs_crankmover = RaveCreateModule(self.env,'CBiRRT')

        try:
            self.env.AddModule(self.probs_cbirrt,self.robotid.GetName())
            self.env.AddModule(self.probs_crankmover,self.crankid.GetName())
        except openrave_exception, e:
            print e

        print "Getting Loaded Problems"
        self.probs = self.env.GetLoadedProblems()

    def StartViewerAndSetValvePos(self, handles, valveType="w"):

        # Start the Viewer and draws the world frame
        if self.ShowUserInterface and not self.ViewerStarted :
#            cam_rot = dot(xyz_rotation([3*pi/2,0,0]),xyz_rotation([0,-pi/2,0]))
#            cam_rot = dot(cam_rot,xyz_rotation([-pi/10,0,0])) # inclination of the camera
#            T_cam = MakeTransform(cam_rot,transpose(matrix([2.0, 0.00, 01.4])))

            T_cam = ([[-0.00259953,  0.83345133, -0.55258675,  0.86188555], \
                      [ 0.99941609, -0.01666103, -0.02983091, -0.00784874], \
                      [-0.03406928, -0.55234164, -0.83292137,  1.92326021], \
                      [ 0.,          0.,          0.,          1.        ]])

            self.env.SetViewer('qtcoin')
            self.env.GetViewer().SetCamera(array(T_cam))
            self.env.GetViewer().EnvironmentSync()
            self.ViewerStarted = True
            # you can draw the camera frame
            #handles.append( misc.DrawAxes(self.env,T_cam,1) )
            self.drawingHandles.append( misc.DrawAxes(self.env,MakeTransform(rodrigues([0,0,0]),transpose(matrix([0,0,0]))),1) )
            

        # Move the wheel infront of the robot
        # These are the default transform and radius values
        #
        # Get the in TSY
        
        self.wheelHeightFromTSY = 0.15
        self.TSYHeight = self.robotid.GetLinks()[12].GetTransform()[2,3]
        self.wheelHeight = self.TSYHeight + self.wheelHeightFromTSY - self.crouch

        wheelY = 0.0

        self.worldPitch = 0.0

        # Find the difference of angle between the wheel's end effector and the link (23 degrees)
        # We should only do this if we're using the logitech wheel.
        self.tiltDiff = acos(dot(linalg.inv(self.crankid.GetManipulators()[0].GetEndEffectorTransform()),self.crankid.GetLinks()[0].GetTransform())[1,1])
        if self.T_Wheel is None:
            # pi/2-tiltDiff+worldPitch makes sure that the wheel's pitch angle is 0 when it's straight up in the world
            # TODO: This is super hacky. Make it right. As in, define everythig in TSY frame, and then convert it to world frame.
            self.T_Wheel = MakeTransform(dot(rodrigues([0,0,pi/2]),rodrigues([pi/2-self.tiltDiff+self.worldPitch,0,0])),transpose(matrix([self.wheelDistFromTSY, wheelY, self.wheelHeight])))
            self.crankid.SetTransform(array(self.T_Wheel))

        if(self.r_Wheel == None):
            self.r_Wheel = 0.2

        self.myValveHandle = RaveCreateKinBody(self.env,'')

        if(valveType == "RL"): # valve type: right-lever
            self.myValveHandle.InitFromBoxes(numpy.array([[-self.r_Wheel*0.5,0,0,self.r_Wheel*0.5,0.01,0.005]]),True)
        if(valveType == "LL"): # valve type: left-lever
            self.myValveHandle.InitFromBoxes(numpy.array([[self.r_Wheel*0.5,0,0,self.r_Wheel*0.5,0.01,0.005]]),True)
        elif(valveType == "W"): # valve type: wheel
            # Create a cylinder
            self.infocylinder = KinBody.Link.GeometryInfo()
            self.infocylinder._type = KinBody.Link.GeomType.Cylinder
            self.infocylinder._vGeomData = [self.r_Wheel,0.01] # radius and height/thickness
            self.infocylinder._bVisible = True
            self.infocylinder._fTransparency = 0.0
            self.infocylinder._vDiffuseColor = [0,1,1]           
            self.myValveHandle.InitFromGeometries([self.infocylinder]) # we could add more cylinders in the list

        self.myValveHandle.SetName('valve')
        self.env.Add(self.myValveHandle,True)
        
        self.myValveHandle.SetTransform(self.crankid.GetManipulators()[0].GetTransform())
  
        # Draw wheel position
        if self.ShowUserInterface :
            print self.robotid.GetJoints()[11].GetAnchor()
            T_RightFoot = self.robotid.GetLinks()[0].GetTransform()
            T_Torso = self.robotid.GetLinks()[8].GetTransform()
            #self.drawingHandles.append( misc.DrawAxes(self.env,self.T_Wheel,0.5) )  
            #self.drawingHandles.append( misc.DrawAxes(self.env,T_Torso,0.5) ) 
            #self.drawingHandles.append( misc.DrawAxes(self.env,T_RightFoot,0.5) ) 
            print T_Torso
            print T_RightFoot


    # Interpolates linearly two configurations
    def Interpolate( self, a, b, u ):

        out = deepcopy(a)
        for i in range(0,len(out)):
            out[i] += u*(b[i]-a[i])
            # print u, out[i], b[i], a[i]
        return out

    def CreateBackToLimitsTrajectory(self,q_cur):

        self.SetProblems()

        q_in  = deepcopy(q_cur)
    
        # Set the in bound configuration to be 0.07 rad inside the 
        # joint limits of the model
        for jIdx, j in enumerate(self.robotid.GetJoints()):
            lower = j.GetLimits()[0]
            upper = j.GetLimits()[1]
            # TODO remove this after test in simulator!!!!
            # q_cur[jIdx] = lower+numpy.random.rand(len(lower))*(upper-lower) + lower
            print jIdx, q_cur[jIdx], lower, upper
            joint_padding = 1e-3
            if q_cur[jIdx] < ( lower + joint_padding ) :
                q_in[jIdx] = lower + 0.07 # 4 deg
            if q_cur[jIdx] > ( upper - joint_padding ) :
                q_in[jIdx] = upper - 0.07 # 4 deg   

        # If q_cur and q_in are not equal then construct a linear interpolated
        # trajectory to the new configuration
        # otherwise return None
        if not self.AreConfigEqual( q_cur, q_in, 1e-3 ) :
            nb_conf = 30
            dt = 0.001
            
            configSpec = self.robotid.GetActiveConfigurationSpecification()
            g = configSpec.GetGroupFromName('joint_values')
            g.interpolation = 'linear'
            configSpec = ConfigurationSpecification()
            configSpec.AddGroup(g)
            # Uncomment for velocities
            # configSpec.AddDerivativeGroups(1,False)
            configSpec.AddDeltaTimeGroup()

            traj = RaveCreateTrajectory(self.robotid.GetEnv(),'')
            traj.Init( configSpec )
            t = 0.0
            for i in range(0,nb_conf):
                wp = self.Interpolate( q_cur, q_in, i/float(nb_conf) )
                # Uncomment for velocities
                # wp = append( wp, wp + self.Interpolate( q_cur, q_in, (i+1)/float(nb_conf) ) )
                wp = append( wp,  t )
                traj.Insert( traj.GetNumWaypoints(), wp )
                t += dt
            # self.ShowTraj( traj )
            return traj

        return None

    # Show traj
    def ShowTraj(self,traj):

        print "Play Back Trajectory (num waypoints : %d)!!!!" % traj.GetNumWaypoints()
        execute_in_loop = False

        while True :

            for i in range(traj.GetNumWaypoints()):
                # get the waypoint values, this holds velocites, time stamps, etc
                data = traj.GetWaypoint(i)
                # extract the robot joint values only
                with self.env: # have to lock environment since accessing robot
                    q = traj.GetConfigurationSpecification().ExtractJointValues(data,self.robotid,self.robotid.GetActiveDOFIndices())
                    self.robotid.SetDOFValues(q)
                    #print q
                dt = 1/float(25)  # 25 Hz
                time.sleep( dt )
                #print "i : %d, dt : %f" % ( i, dt )

            if not execute_in_loop :
                break

        print "end playback"
        return 

    def Playback(self,retimed=False):

        if( self.StopAtKeyStrokes ):
            print "Press Enter to play the trajectories..."
            sys.stdin.readline()

        retimed_str = ''
        if( retimed ):
            retimed_str = '_retimed'

        close_hands = False
        if( close_hands ):
            # Playback 0:(home-init) -> 1:(init-start) -> 2:(start-goal) -> 3:(goal-start) -> 4:(start-init) -> 5:(init-home)
            self.robotid.SetDOFValues(self.rhandclosevals,self.rhanddofs)
            self.robotid.SetDOFValues(self.lhandclosevals,self.lhanddofs)

        try:
            print 'traj '+self.default_trajectory_dir+'movetraj0'+retimed_str+'.txt'
            answer= self.probs_cbirrt.SendCommand('traj '+self.default_trajectory_dir+'movetraj0'+retimed_str+'.txt');
            self.robotid.WaitForController(0)
            # debug
            print "traj call answer: ",str(answer)
            if(answer != '1'):
                return 40 # error code 4: playback error, 0: at 0th trajectory
        except openrave_exception, e:
            print e
            return []
        
        self.robotid.GetController().Reset(0)
        time.sleep(1)

        if( close_hands ):
            self.robotid.SetDOFValues(self.rhandopenvals,self.rhanddofs)
            self.robotid.SetDOFValues(self.lhandopenvals,self.lhanddofs)
        
        try:
            answer= self.probs_cbirrt.SendCommand('traj '+self.default_trajectory_dir+'movetraj1'+retimed_str+'.txt');
            self.robotid.WaitForController(0)
            # debug
            print "traj call answer: ",str(answer)
            if(answer != '1'):
                return 41 # error code 4: playback error, 1: at 1st trajectory
        except openrave_exception, e:
            print e
            return []
        
        self.robotid.GetController().Reset(0)
        time.sleep(1)

        if( close_hands ):
            self.robotid.SetDOFValues(self.rhandclosevals,self.rhanddofs)
            self.robotid.SetDOFValues(self.lhandclosevals,self.lhanddofs)
            time.sleep(1)

        try:
            answer= self.probs_cbirrt.SendCommand('traj '+self.default_trajectory_dir+'movetraj2'+retimed_str+'.txt');
            answer= self.probs_crankmover.SendCommand('traj '+self.default_trajectory_dir+'movetraj2'+retimed_str+'.txt');
            self.robotid.WaitForController(0)
            # debug
            print "traj call answer: ",str(answer)
            if(answer != '1'):
                return 42 # error code 4: playback error, 2: at 2nd trajectory
        except openrave_exception, e:
            print e
            return []
        
        self.robotid.GetController().Reset(0)
        time.sleep(1)

        if( close_hands ):
            self.robotid.SetDOFValues(self.rhandopenvals,self.rhanddofs)
            self.robotid.SetDOFValues(self.lhandopenvals,self.lhanddofs)
            time.sleep(1)

        try:
            answer= self.probs_cbirrt.SendCommand('traj '+self.default_trajectory_dir+'movetraj3'+retimed_str+'.txt');
            self.robotid.WaitForController(0)
            # debug
            print "traj call answer: ",str(answer)
            if(answer != '1'):
                return 43 # error code 4: playback error, 3: at 3rd trajectory
        except openrave_exception, e:
            print e
            return []
        
        self.robotid.GetController().Reset(0)
        time.sleep(1)

        try:
            answer= self.probs_cbirrt.SendCommand('traj '+self.default_trajectory_dir+'movetraj4'+retimed_str+'.txt');
            self.robotid.WaitForController(0)
            # debug
            print "traj call answer: ",str(answer)
            if(answer != '1'):
                return 44 # error code 4: playback error, 4: at 4th trajectory
        except openrave_exception, e:
            print e
            return []
        
        self.robotid.GetController().Reset(0)
        time.sleep(1)

        if( close_hands ):
            self.robotid.SetDOFValues(self.rhandclosevals,self.rhanddofs)
            self.robotid.SetDOFValues(self.lhandclosevals,self.lhanddofs)
            time.sleep(1)

        try:
            answer= self.probs_cbirrt.SendCommand('traj '+self.default_trajectory_dir+'movetraj5'+retimed_str+'.txt');
            self.robotid.WaitForController(0)
            # debug
            print "traj call answer: ",str(answer)
            if(answer != '1'):
                return 45 # error code 4: playback error, 5: at 5th trajectory
        except openrave_exception, e:
            print e
            return []
        
        self.robotid.GetController().Reset(0)

        try:
            answer= self.probs_cbirrt.SendCommand('traj '+self.default_trajectory_dir+'movetraj6'+retimed_str+'.txt');
            self.robotid.WaitForController(0)
            # debug
            print "traj call answer: ",str(answer)
            if(answer != '1'):
                return 45 # error code 4: playback error, 5: at 5th trajectory
        except openrave_exception, e:
            print e
            return []
        
        self.robotid.GetController().Reset(0)

        try:
            answer= self.probs_cbirrt.SendCommand('traj '+self.default_trajectory_dir+'movetraj7'+retimed_str+'.txt');
            self.robotid.WaitForController(0)
            # debug
            print "traj call answer: ",str(answer)
            if(answer != '1'):
                return 45 # error code 4: playback error, 5: at 5th trajectory
        except openrave_exception, e:
            print e
            return []
        
        self.robotid.GetController().Reset(0)

        try:
            answer= self.probs_cbirrt.SendCommand('traj '+self.default_trajectory_dir+'movetraj8'+retimed_str+'.txt');
            self.robotid.WaitForController(0)
            # debug
            print "traj call answer: ",str(answer)
            if(answer != '1'):
                return 45 # error code 4: playback error, 5: at 5th trajectory
        except openrave_exception, e:
            print e
            return []
        
        self.robotid.GetController().Reset(0)
        
        if( self.StopAtKeyStrokes ):
            print "Press Enter to exit..."
            sys.stdin.readline()

        # file_names = [ 'movetraj0.txt','movetraj1.txt','movetraj2.txt','movetraj3.txt','movetraj4.txt','movetraj5.txt' ]
        # return file_names
            
        return 0 # If you are here, there is no error, return 0

    def PlaybackFiles( self, files ):

        for f in files :

            try:
                answer = self.probs_cbirrt.SendCommand( 'traj ' + self.default_trajectory_dir + f );
                self.robotid.WaitForController(0)
                # debug
                print "traj call answer: " , str(answer)
                if(answer != '1'):
                    return 41 # error code 4: playback error, 1: at 1st trajectory
            except openrave_exception, e:
                print e
                return
