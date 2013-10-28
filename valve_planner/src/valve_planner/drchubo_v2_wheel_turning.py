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
from math import *
from copy import *
import os # for file operations
from base_wheel_turning import *
from drchubo_constrainted_path import *
import rave2realhubo

class DrcHuboV2WheelTurning( BaseWheelTurning ):

    def __init__(self, HuboModelPath, WheelModelPath ):

        BaseWheelTurning.__init__( self, HuboModelPath, WheelModelPath )        
        
        # 0: Not initialized
        # 1: GetReady() successful
        # 2: At init
        # 3: Ready to turn
        # 4: Started turning
        # 5: Finished turning
        # 6: Finished task
        self.state = 0
        
        self.T0_LH1 = None
        self.T0_RH1 = None
        self.initik = None
        self.homeik = None
        self.startik = None
        
        # These distances have to be in accordance with the padding
        # defined in the base class
        self.hand_offset = 0.02 # between the valve and hand when turning
        self.hand_entry_back_off = 0.11 # when entering the valve before
        self.hand_exit_back_off = 0.11 # when exiting the valve after turn

        # Grasp list
        self.use_grasplist = False

        # Manipulator names
        self.leftArm = "leftArm"
        self.rightArm = "rightArm"
        self.leftFoot = "leftFoot"
        self.rightFoot = "rightFoot"
        self.head = "Head"

        # keep manipulator objects for easy access
        self.robotManips = self.robotid.GetManipulators()
        self.valveManip = self.crankid.GetManipulators()

        # TSR storage
        self.TSRs = DrcHuboValveTurningTSRs(self.robotManips)

        # keep link objects for easy access
        self.robotLinks = self.robotid.GetLinks()
        self.valveLinks = self.crankid.GetLinks()

        # Only plans arm motion for turning the wheel
        self.onlyArms = False
        self.planAllDOFIk = True # TODO fix this
        self.alldofs = self.GetActiveDOFs()
        
        # Set those variables to show or hide the interface
        # Do it using the member functions
        self.StopAtKeyStrokes = False
        self.ShowUserInterface = False
        self.ViewerStarted = False

        # Right Hand Joints 
        # Open - Closed Values
        self.lhanddofs = [7,20,23]
        self.lhandclosevals = [-0.15, -0.15, -0.15]
        self.lhandopenvals = [-1.490, -1.490, -1.490]

        # Left Hand Joints
        self.rhanddofs = [33,42,45,48]
        self.rhandclosevals = [-0.15, -0.15, -0.15, -0.15]
        self.rhandopenvals = [-1.490, -1.490, -1.490, -1.490]

        self.bothhandscloseval = -0.15
        self.bothhandsopenval = -1.490
        
        self.GenerateJointDict()

        self.optPlay = False
        self.optTaskWall = False
        self.optWall = False
        self.optDemo = False

        self.useIKFast = True

        # Stores last trajectory
        self.trajectory = None

    def SetHandDOFs(self,hand,vals):
        if(hand == "LH"):
            self.robotid.SetDOFValues(multiply(ones(len(self.lhanddofs)),vals),self.lhanddofs)
        elif(hand == "RH"):
            self.robotid.SetDOFValues(multiply(ones(len(self.rhanddofs)),vals),self.rhanddofs)
        elif(hand == "BH"):
            self.robotid.SetDOFValues(multiply(ones(len(self.lhanddofs)),vals),self.lhanddofs)
            self.robotid.SetDOFValues(multiply(ones(len(self.rhanddofs)),vals),self.rhanddofs)

    def OpenHands(self,hand,fname="openHandsHere"):
        # Wait 4 seconds to fully open the hands when exporting for ach
        waitThisMuch = 6.0 # seconds.
        rave2realhubo.openHandsHere(self.robotid,25.0,waitThisMuch,hand,fname)
        print "waiting for "+hand+" to open..."
        # time.sleep(waitThisMuch)
        self.SetHandDOFs(hand,self.bothhandsopenval)

    def CloseHands(self,hand,fname="closeHandsHere",fake=False):
        # Wait 4 seconds to fully close the hands when exporting for ach
        waitThisMuch = 6.0 # seconds.
        rave2realhubo.closeHandsHere(self.robotid,25.0,waitThisMuch,hand,fname)
        print "waiting for "+hand+" to close..."
        # time.sleep(waitThisMuch)
        if(fake):
            self.SetHandDOFs(hand,self.bothhandsopenval)
        else:
            self.SetHandDOFs(hand,self.bothhandscloseval)

    def AvoidSingularity(self,robot):
        # This function sets the robot's joints to values
        # that are close to zero as much as possible, while
        # taking the joint limits into consideration
        for jIdx, j in enumerate(robot.GetJoints()):
            lims = j.GetLimits()
            if(lims[1] > 0.0):
                robot.SetDOFValues([0.001],[jIdx])
            else:
                robot.SetDOFValues([-0.001],[jIdx])

        robot.GetController().Reset(0)

    def IKFast(self, manipname, T, allSolutions=True):
        self.robotid.SetActiveManipulator(manipname)
        ikmodel = databases.inversekinematics.InverseKinematicsModel(self.robotid,iktype=IkParameterizationType.Transform6D)
        ikmodel.load()

        # check if ik solution(s) exist
        if(allSolutions):
            return ikmodel.manip.FindIKSolutions(array(T),IkFilterOptions.CheckEnvCollisions)
        else:
            return ikmodel.manip.FindIKSolution(array(T),IkFilterOptions.CheckEnvCollisions)

    def ExportTraj2RealHubo(self,trajfilename,activedofs):
        self.robotid.SetActiveDOFs( self.alldofs )
        traj = RaveCreateTrajectory(self.env,'').deserialize(open(trajfilename+'.txt','r').read())
        cs = traj.GetConfigurationSpecification()
        drchuboJointValsGroup = cs.GetGroupFromName("joint_values "+self.robotid.GetName())
        drchuboJointVelocitiesGroup = cs.GetGroupFromName("joint_velocities "+self.robotid.GetName())
        deltatimeGroup = cs.GetGroupFromName("deltatime")
        rave2realhubo.traj2ach(self.env,self.robotid,traj,trajfilename,drchuboJointValsGroup.offset,drchuboJointVelocitiesGroup.offset,deltatimeGroup.offset)
        self.robotid.SetActiveDOFs( activedofs )
    
    def RenameTrajectory(self,src,dst):
        try:
            os.rename(src,dst)
        except OSError, e:
            # No file cmovetraj
            print e
            return [False, "OS exception in RenameTrajectory."]

        return [True, ""]

    # -------------------------------------------------------------------------
    # -------------------------------------------------------------------------
    # -------------------------------------------------------------------------
    # Plans a trajectory using a CBiRRT problem and goaljoints (active dofs)
    def PlanTrajectory(self, q_init, q_goal, TSRChainString, smoothingitrs, error_code_str, mimicdof, psample, activedofs ):
        
        # String to number
        if(type(q_init) == type("")):
            q_init = str2num(q_init)

        if(type(q_goal) == type("")):
            q_goal = str2num(q_goal)

        if self.planAllDOFIk :
            # Gets only the active dofs of init and goal
            self.robotid.SetActiveDOFs( self.alldofs )
            self.robotid.SetActiveDOFValues( q_goal )
            self.robotid.GetController().Reset(0)
            self.robotid.SetActiveDOFs( activedofs )
            q_goal = self.robotid.GetActiveDOFValues()

            self.robotid.SetActiveDOFs( self.alldofs )
            self.robotid.SetActiveDOFValues( q_init )
            self.robotid.GetController().Reset(0)
            self.robotid.SetActiveDOFs( activedofs )
            q_init = self.robotid.GetActiveDOFValues()

        # Convert q_target to numbers
        if(type(q_goal) == type("")):
            goaljoints = deepcopy(str2num(q_goal))
        else:
            goaljoints = deepcopy(q_goal)

        self.robotid.SetActiveDOFValues(q_init)
        #self.robotid.GetController().Reset(0)
        #time.sleep(2)
        q_tmp = self.robotid.GetDOFValues()
        self.robotid.GetController().SetDesired(q_tmp)
#        print q_tmp
#        print self.q_cur
        if self.AreConfigEqual(q_tmp,self.q_cur,1e-3) :
            # Move the robot out of collision with pading 
            # when current configuration is q_init
            print "q_init == self.q_cur"
            self.MoveCurrentConfigurationOutOfCollision()

        # Change to plan with lower number of dofs (onlyArms)
        self.robotid.SetActiveDOFs( activedofs )

        # Then add extra dofs for each TSRMimicDOF
        if(mimicdof is not None):
            for i in range(mimicdof):
                goaljoints = append(goaljoints, [0], 0)        
        
        cmdStr = 'RunCBiRRT '

        try:
            if(psample != None):
                cmdStr += 'psample '+str(psample)+' '

            cmdStr += 'supportlinks 2 '+self.footlinknames+' smoothingitrs '+str(smoothingitrs)+' jointgoals '+str(len(goaljoints))+' '+Serialize1DMatrix(matrix(goaljoints))+' '+TSRChainString

            answer = self.probs_cbirrt.SendCommand(cmdStr)

            print "RunCBiRRT answer: ",str(answer)
            if(answer[0] != '1'):
                return [False, error_code_str+" - "+answer[1:]]
        except openrave_exception, e:
            print "Cannot send command RunCBiRRT: "
            print e
            return [False, "CBiRRT Plug-in Exception."]

        return [True, ""]

    # --------------------------------------------------------------------------
    def PlanPath(self, path):

        print "-- PLAN path for : " + str(path.name)

        if( self.StopAtKeyStrokes ):
            print "Press Enter to plan"
            sys.stdin.readline()

        for pe in path.elements:

            print "-- PLAN path element for : " + str(pe.name)

            if( pe.openHandsBefore ):
                # Open the hand we will use to avoid collision:
                if( pe.hands == "BH" ):
                    self.robotid.SetDOFValues( self.rhandopenvals, self.rhanddofs )
                    self.robotid.SetDOFValues( self.lhandopenvals, self.lhanddofs )
                if( pe.hands == "LH" ):
                    self.robotid.SetDOFValues( self.lhandopenvals, self.lhanddofs )
                if( pe.hands == "RH" ):
                    self.robotid.SetDOFValues( self.rhandopenvals, self.rhanddofs )

                self.OpenHands(pe.hands,self.default_trajectory_dir+"openhands_before_"+pe.filename)
            if( pe.closeHandsBefore ):
                self.CloseHands(pe.hands,self.default_trajectory_dir+"closehands_before_"+pe.filename,True)

            if pe.padValve :
                self.PadValve(path.valveType)
            if pe.padWaist :
                self.PadWaist( self.GetT0_RefLink("Body_TSY") )

            [success, why] = self.PlanTrajectory( pe.startik, pe.goalik, pe.TSR, pe.smoothing, pe.errorCode, pe.mimicdof, pe.psample, pe.activedofs )
            if(success):
                [success, why] = self.RenameTrajectory("cmovetraj.txt",self.default_trajectory_dir+pe.filename+".txt")
                if(success):
                    self.ExportTraj2RealHubo(self.default_trajectory_dir+pe.filename, pe.activedofs )
                    [success, why] = pe.PlayInOpenRAVE()

            if pe.padValve :
                self.UnpadValve(path.valveType)
            if pe.padWaist :
                self.UnPadWaist()

            if(not success):
                return [False, why]

            if( pe.openHandsAfter ):
                # Open the hand we will use to avoid collision:
                if( pe.hands == "BH" ):
                    self.robotid.SetDOFValues( self.rhandopenvals, self.rhanddofs )
                    self.robotid.SetDOFValues( self.lhandopenvals, self.lhanddofs )
                if( pe.hands == "LH" ):
                    self.robotid.SetDOFValues( self.lhandopenvals, self.lhanddofs )
                if( pe.hands == "RH" ):
                    self.robotid.SetDOFValues( self.rhandopenvals, self.rhanddofs )
            
                self.OpenHands(pe.hands,self.default_trajectory_dir+"openhands_after_"+pe.filename)

            if( pe.closeHandsAfter ):
                # super hacky way to keep the robot from being in self collision when going home
                if(pe.name == "current2init"):
                    self.CloseHands(pe.hands,self.default_trajectory_dir+"closehands_after_"+pe.filename,False)
                else:
                    self.CloseHands(pe.hands,self.default_trajectory_dir+"closehands_after_"+pe.filename,True)

        self.trajectory = path

        return [True, ""]

    # --------------------------------------------------------------------------
    # --------------------------------------------------------------------------
    # --------------------------------------------------------------------------
    def GetDefaultHandsStartPose(self, hands, valveType, adjust=False):

        # Now try to find an IK to get ready to turn the valve
        #
        # Go through different grasp indices until one of them works
        graspIndex = 0

        # Left Hand Transform in World Coordinates
        T0_LH1 = self.GetT0_LH1(hands, graspIndex, valveType, self.hand_offset )

        # Right Hand Pose in World Coordinates
        T0_RH1 = self.GetT0_RH1(hands, graspIndex, valveType, self.hand_offset )

        # Uncomment if you want to see where T0_LH1 is 
        #self.drawingHandles.append(misc.DrawAxes(self.env,matrix(T0_LH1),1))

        # Uncomment if you want to see where T0_RH1 is 
        #self.drawingHandles.append(misc.DrawAxes(self.env,matrix(T0_RH1),1))

        if( adjust ):
            if( hands == "BH" or hands == "LH" ):
                T0_LH1[2,3] += self.crouch
            if( hands == "BH" or hands == "RH" ):
                T0_RH1[2,3] += self.crouch

        return [T0_LH1, T0_RH1]

    # --------------------------------------------------------------------------
    def FindStartIK(self, hands,valveType):

        [self.T0_LH1, self.T0_RH1] = self.GetDefaultHandsStartPose( hands, valveType )

        # Open the hand we will use to avoid collision:
        if( hands == "BH" ):
            self.robotid.SetDOFValues(self.rhandopenvals,self.rhanddofs)
            self.robotid.SetDOFValues(self.lhandopenvals,self.lhanddofs)
        if( hands == "LH" ):
            self.robotid.SetDOFValues(self.lhandopenvals,self.lhanddofs)
        if( hands == "RH" ):
            self.robotid.SetDOFValues(self.rhandopenvals,self.rhanddofs)

        q_startik = self.probs_cbirrt.SendCommand('DoGeneralIK exec supportlinks 2 '+self.footlinknames+' movecog '+self.cogTargStr+' nummanips 2 maniptm 0 '+trans_to_str(self.T0_LH1)+' maniptm 1 '+trans_to_str(self.T0_RH1))

        self.robotid.SetActiveDOFValues( str2num(q_startik) )

        ik_not_found = q_startik == ''
        ik_in_collision = False

        if ik_not_found :
            print "Error : generalik did not succeeded"
        else :
            if self.robotid.CheckSelfCollision() :
                print "Error : q_startik is self colliding"
                ik_in_collision = True
            elif self.env.CheckCollision(self.robotid):
                print "Error : q_startik in collision with environment"
                ik_in_collision = True
        
        # GeneralIK does not go collision check
        if( ik_not_found or ik_in_collision ):

            if( self.useIKFast  ):
                print "Info: using IKFast."
                
                if( hands == "BH" or hands == "LH" ):
                    sol0 = self.IKFast('leftArm', array(self.T0_LH1), False)
                    if( sol0 is not None):
                        self.robotid.SetDOFValues(sol0,self.robotid.GetManipulators()[0].GetArmIndices())
                    else:
                        print "Error: IKFast Could not found q_startik."
                        return [False, 32, ""] # 3: ikfast error, 2: q_startik
       
                if( hands == "BH" or hands == "RH" ):
                    sol1 = self.IKFast('rightArm', array(self.T0_RH1), False)
                    if( sol1 is not None):
                        self.robotid.SetDOFValues(sol1,self.robotid.GetManipulators()[1].GetArmIndices())
                    else:
                        print "Error: IKFast Could not found q_startik."
                        return [False, 32, ""] # 3: ikfast error, 2: q_startik

                q_startik = self.robotid.GetActiveDOFValues()
            else:
                return [False, 22, ""] # 2: generalik error, 2: at q_startik
        else:
            print "Info: GeneralIK found a q_startik."

        return [True,0,q_startik]

    # --------------------------------------------------------------------------
    def FindStartConstraints(self, hands, valveType, compute_ik, adjust=False):

        print hands

        # set init2start TSRs
        self.TSRs.SetInit2Start(self.T0_TSY)
                 
        q_startik = ""

        # Computes a startik for the that configuration
        if( compute_ik ):
            [success, error, q_startik] = self.FindStartIK(hands,valveType)
            if( success == False):
                return [success, error]

        return [True, "", q_startik]

    # --------------------------------------------------------------------------
    def FindMaxTurnIK(self,hands,start_from_init_ik=False):

        q_cur = self.robotid.GetDOFValues()

        # TODO Made fo CW motions
        grasplist_R = self.GetGraspListRight()
        grasplist_L = self.GetGraspListLeft()
        
        if len(grasplist_L) != len(grasplist_R) :
            print "Error: grasplist size missmatch for the left and right arm"

        error_code = -1
        q_startik = None
        q_manipik = None
        i = 0
        for grasps in zip( grasplist_R, grasplist_L ):
            print "TRY: " + str(i)
            
            if start_from_init_ik :
                self.robotid.SetActiveDOFValues(str2num(self.initik))

            [error_code,q_startik] = self.FindTwoArmsIK( grasps[0], grasps[1], open_hands=True )
            if error_code == 0:
                print "FOUND startik at : " + str(i)
                self.T0_RH1 = grasps[0]
                self.T0_LH1 = grasps[1]

                if( hands == "BH" or hands == "LH" ):
                    T0_LH0 = dot(self.T0_LH1, MakeTransform(eye(3),transpose(matrix([0,self.hand_entry_back_off,0]))))
                if( hands == "BH" or hands == "RH" ):
                    T0_RH0 = dot(self.T0_RH1, MakeTransform(eye(3),transpose(matrix([0,self.hand_entry_back_off,0]))))

                if start_from_init_ik :
                    self.robotid.SetActiveDOFValues(str2num(self.initik))

                [error,q_manipik] = self.FindTwoArmsIK( T0_RH0, T0_LH0, open_hands=True)
                if error == 0:
                    print "FOUND manipik at : " + str(i)
                    return [error_code,q_startik,q_manipik]                    
            i += 1
            self.robotid.SetDOFValues(q_cur)

        return [error_code,q_startik,q_manipik]

    # --------------------------------------------------------------------------
    def FindTwoArmsIK( self, T0_RH, T0_LH, open_hands ):

        arg2 = trans_to_str(T0_LH)
        arg3 = trans_to_str(T0_RH)
        arg4 = trans_to_str(self.robotManips[2].GetEndEffectorTransform())
        arg5 = trans_to_str(self.robotManips[3].GetEndEffectorTransform())

        if open_hands :
            # Open the hand we will use to avoid collision:
            self.robotid.SetDOFValues(self.rhandopenvals,self.rhanddofs)
            self.robotid.SetDOFValues(self.lhandopenvals,self.lhanddofs)

        q_ik = self.probs_cbirrt.SendCommand('DoGeneralIK exec supportlinks 2 '+self.footlinknames+' movecog '+self.cogTargStr+' nummanips 4 maniptm 0 '+arg2+' maniptm 1 '+arg3+' maniptm 2 '+arg4+' maniptm 3 '+arg5)

        if(q_ik == '' or (self.env.CheckCollision(self.robotid) or self.robotid.CheckSelfCollision()) ):
            print "Error : GeneralIK could not find q_ik, or q_ik is in collision."

            if( self.useIKFast ):

                print "Info: using IKFast."
                sol0 = self.IKFast('leftArm', array(T0_LH), False)
                sol1 = self.IKFast('rightArm', array(T0_RH), False)
                if( (sol0 is not None) and (sol1 is not None) ):
                    self.robotid.SetDOFValues( sol0, self.robotid.GetManipulators()[0].GetArmIndices() )
                    self.robotid.SetDOFValues( sol1, self.robotid.GetManipulators()[1].GetArmIndices() )
                else:
                    print "Error : IKFast could not find goalik."
                    return [33,str2num(q_ik)] # 3: ikfast error, 3: goalik

                q_ik = self.robotid.GetActiveDOFValues()

            else:
                return [23,str2num(q_ik)] # 2: generalik error, 3: at goal ik
        else:
            print "Info : GeneralIK found an ik."
            self.robotid.SetActiveDOFValues(str2num(q_ik))
            self.robotid.GetController().Reset(0)

        return [0,str2num(q_ik)]

    # -------------------------------------------------------------------------
    # -------------------------------------------------------------------------
    # -------------------------------------------------------------------------
    def Grasp(self, hands, valveType):
        #TODO
        return

    # -------------------------------------------------------------------------
    # -------------------------------------------------------------------------
    # -------------------------------------------------------------------------
    def UnGrasp(self, hands, valveType):
        #TODO
        return

    # -------------------------------------------------------------------------
    # -------------------------------------------------------------------------
    # -------------------------------------------------------------------------
    def GetReady(self, hands, valveType):
        
        # Wherever you are, make sure you
        # 1. Go to a safe position

        # Current configuration of the robot is its initial configuration
        self.AvoidSingularity( self.robotid )
        currentik = self.robotid.GetActiveDOFValues()

        # Set TSRs for Current 2 Init
        self.TSRs.SetCurrent2Init()

        # Set a "safe pose"
        # elbows: Left Elbow Pitch: 3; Right Elbow Pitch: 29
        self.AvoidSingularity(self.robotid)
        self.robotid.SetDOFValues([-0.65,-0.65],[3,29]) 
        self.BendTheKnees()
        [T0_LFTarget, T0_RFTarget] = self.GetFeetTargetsInit()

        self.initik = self.probs_cbirrt.SendCommand('DoGeneralIK exec supportlinks 2 '+self.footlinknames+' movecog '+self.cogTargStr+' nummanips 2 maniptm 2 '+trans_to_str(T0_LFTarget)+' maniptm 3 '+trans_to_str(T0_RFTarget))

        
        if( self.initik == ''):
            print "Error: could not find initik"
            return 21 # 2: generalik error, 1: at initik

        self.robotid.SetActiveDOFValues(str2num(self.initik))

        [T0_LH,T0_RH] = self.GetHandTargetsStand()
        [error,standik] = self.FindTwoArmsIK( T0_RH, T0_LH, open_hands=True)
        if( error != 0 ):
            print "Error: could not find standik"
            return 21 # 2: generalik error, 1: at initik

        self.robotid.SetActiveDOFValues( standik )

        [success, why, q_startik] = self.FindStartConstraints( hands, valveType, False, True)
        if(not success):
            return why

        if self.use_grasplist and hands == "BH" :

            [error, startik, manipik] = self.FindMaxTurnIK( hands, self.direction )

        else :

            if self.useUserPoses :

                T0_LH0 = self.T0_LH_USER
                T0_RH0 = self.T0_RH_USER

                if( hands == "BH"):
                    self.T0_LH1 = dot(T0_LH0, MakeTransform(eye(3),transpose(matrix([0,-self.LH_USER_offset,0]))))
                    self.T0_RH1 = dot(T0_RH0, MakeTransform(eye(3),transpose(matrix([0,-self.RH_USER_offset,0]))))
                if( hands == "RH" ):
                    self.T0_RH1 = dot(T0_RH0, MakeTransform(eye(3),transpose(matrix([0,-0.05,0]))))
                if( hands == "LH" ):
                    self.T0_LH1 = dot(T0_LH0, MakeTransform(eye(3),transpose(matrix([0,-0.05,0]))))

            else :

                [self.T0_LH1,self.T0_RH1] = self.GetDefaultHandsStartPose( hands, valveType )

                T0_LH0 = deepcopy(self.T0_LH1)
                T0_RH0 = deepcopy(self.T0_RH1)

                if( hands == "BH"):
                    T0_LH0 = dot(self.T0_LH1, MakeTransform(eye(3),transpose(matrix([0,self.hand_entry_back_off,0]))))
                    T0_RH0 = dot(self.T0_RH1, MakeTransform(eye(3),transpose(matrix([0,self.hand_entry_back_off,0]))))
                if( hands == "RH" ):
                    T0_RH0 = dot(self.T0_RH1, MakeTransform(eye(3),transpose(matrix([0,0.05,0]))))
                if( hands == "LH" ):
                    T0_LH0 = dot(self.T0_LH1, MakeTransform(eye(3),transpose(matrix([0,0.05,0]))))

#            self.drawingHandles.append(misc.DrawAxes(self.env,matrix(T0_LH0),1))
#            self.drawingHandles.append(misc.DrawAxes(self.env,matrix(T0_RH0),1))

            [error,manipik] = self.FindTwoArmsIK( T0_RH0, T0_LH0, open_hands=True)

        if(error != 0):
            print "Error : Cound not find manipik!!!!"
            return ""

        # If all is good we have a currentik, an initik and a startik
        # Close both hands to avoid collision at currentik
        self.robotid.SetDOFValues( self.rhandclosevals, self.rhanddofs )
        self.robotid.SetDOFValues( self.lhandclosevals, self.lhanddofs )

        cp = ConstrainedPath( "GetReady", self.robotid )
        cp.valveType = valveType
        
        # Set the path elements
        # From current configuration to a known init configuration
        cpe0 = ConstrainedPathElement("current2init")
        cpe0.startik = currentik
        cpe0.goalik = self.initik
        cpe0.TSR = self.TSRs.TSRChainStringFeetandHead_current2init
        cpe0.smoothing = self.normalsmoothingitrs
        cpe0.errorCode = "10"
        cpe0.filename = "movetraj0"
        cpe0.hands = hands
        cpe0.cbirrtProblems = [self.probs_cbirrt]
        cpe0.cbirrtRobots = [self.robotid]
        cpe0.cbirrtTrajectories = [self.default_trajectory_dir+cpe0.filename]
        cpe0.activedofs = self.alldofs

        # 2. Open your hands after going to "ready" config.
        cpe0.openHandsAfter = True

        # Set the path elements
        # From current configuration to a known init configuration
        cpe1 = ConstrainedPathElement("init2stand")
        cpe1.startik = self.initik
        cpe1.goalik = standik

        if( hands == "BH" ):
            cpe1.TSR = self.TSRs.TSRChainStringFeetandHead_init2start_bh
        elif( hands == "LH" ):
            cpe1.TSR = self.TSRs.TSRChainStringFeetandHead_init2start_lh
        elif( hands == "RH" ):
            cpe1.TSR = self.TSRs.TSRChainStringFeetandHead_init2start_rh

        cpe1.smoothing = self.normalsmoothingitrs
        cpe1.errorCode = "10"
        cpe1.filename = "movetraj1"
        cpe1.hands = hands
        cpe1.cbirrtProblems = [self.probs_cbirrt]
        cpe1.cbirrtRobots = [self.robotid]
        cpe1.cbirrtTrajectories = [self.default_trajectory_dir+cpe1.filename]
        cpe1.padValve = True
        cpe1.padWaist = True
        cpe1.activedofs = self.GetActiveDOFs(self.onlyArms)


        # Set the path elements
        # From current configuration to a known init configuration
        cpe2 = ConstrainedPathElement("stand2manip")
        cpe2.startik = standik
        cpe2.goalik = manipik

        if( hands == "BH" ):
            cpe2.TSR = self.TSRs.TSRChainStringFeetandHead_init2start_bh
        elif( hands == "LH" ):
            cpe2.TSR = self.TSRs.TSRChainStringFeetandHead_init2start_lh
        elif( hands == "RH" ):
            cpe2.TSR = self.TSRs.TSRChainStringFeetandHead_init2start_rh

        cpe2.smoothing = self.normalsmoothingitrs
        cpe2.errorCode = "10"
        cpe2.filename = "movetraj1"
        cpe2.hands = hands
        cpe2.cbirrtProblems = [self.probs_cbirrt]
        cpe2.cbirrtRobots = [self.robotid]
        cpe2.cbirrtTrajectories = [self.default_trajectory_dir+cpe1.filename]
        cpe2.padValve = True
        cpe2.padWaist = True
        cpe2.activedofs = self.GetActiveDOFs(self.onlyArms)

        print "q_startik"
        print q_startik

        cp.elements.append(cpe0)
        cp.elements.append(cpe1)
        cp.elements.append(cpe2)
        
        [success, why] = self.PlanPath(cp)
        if(not success):
            return why
        else:
            return 0

    # --------------------------------------------------------------------------
    def FindBothHandsGoalAndExtract( self, crank_rot ):

        q_cur = self.robotid.GetDOFValues()

        # Calculate hand transforms after rotating the wheel (they will help us find the goalik):
        # How much do we want to rotate the wheel?

        # The coordinate system of the valve model we're using is not aligned with the world.
        # This means when we say "valve.SetTransform(eye(4))" XYZ axes don't match to the world's XYZ axes.
        # If they did, we could simply do "T0_w0L = valve.GetTransform()"
        # However, instead we have to fix the valve's transform to make it match world's transform when zeroed.
        # This is totally for convenience, so it's easier to think of the limits and the TSR.
        T0_w0L = dot(self.valveTroot,MakeTransform(rodrigues([0,-pi/2,0]),transpose(matrix([0,0,0]))))
        T0_w0L = dot(T0_w0L,MakeTransform(rodrigues([-pi/2,0,0]),transpose(matrix([0,0,0]))))

        # Left hand's transform in wheel's coordinates
        Tw0L_LH1 = dot(linalg.inv(T0_w0L),self.T0_LH1) # self.T0_LH1 is set in GetReady() method

        # Transform of the left hand's end effector in wheel's coords.
        # Required by CBiRRT
        Tw0_eL = Tw0L_LH1

        # Right Hand's TSR:
        # Note that the right hand is defined in the wheel coordinate frame
        T0_crankHandle = self.crankid.GetManipulators()[0].GetEndEffectorTransform()
        T0_w0R = MakeTransform(rodrigues([0,0,0]),transpose(matrix([0,0,0])))

        # End effector transform in wheel coordinates
        Tw0_eR = dot(linalg.inv(T0_crankHandle),self.T0_RH1)

        # Which joint do we want the CBiRRT to mimic the TSR for?
        TcrankHandle_crankHandleRotated = MakeTransform(rodrigues([0,0,crank_rot]),transpose(matrix([0,0,0])))

        # Where will the right hand go after turning the wheel?
        T0_crankHandleRotated = dot(T0_crankHandle,TcrankHandle_crankHandleRotated)
        TcrankHandle_RH1 = dot(linalg.inv(T0_crankHandle),self.T0_RH1)
        T0_RH2 = dot(T0_crankHandleRotated, TcrankHandle_RH1)

        # How much freedom? (note: this end effector is mimicking, everything is defined 
        # in the frame of crank)
        Bw0R = matrix([0,0,0,0,0,0,0,0,0,0,0,0])

        # Head's transforms:
        T0_w0H =  self.robotManips[4].GetEndEffectorTransform()
        Tw0_eH = eye(4)
        Bw0H = matrix([0,0,0,0,0,0,0,0,0,0,0,0])

        # Create the transform for the wheel that we would like to reach to                
        # Rotate the left hand's transform on the wheel in world transform "crank_rot" radians around it's Z-Axis
        T0_cranknew = dot(self.crankid.GetManipulators()[0].GetEndEffectorTransform(), MakeTransform(rodrigues([0,0,crank_rot]),transpose(matrix([0,0,0]))))

        self.drawingHandles.append(misc.DrawAxes(self.env,matrix(T0_cranknew),1))

        # Where will the left hand go after turning the wheel?
        T0_LH2 = dot(T0_cranknew,dot(linalg.inv(self.crankid.GetManipulators()[0].GetEndEffectorTransform()),self.T0_LH1))

        # Exit hand transforms
        T0_LH3 = dot(T0_LH2, MakeTransform(eye(3),transpose(matrix([0,self.hand_exit_back_off,0]))))
        T0_RH3 = dot(T0_RH2, MakeTransform(eye(3),transpose(matrix([0,self.hand_exit_back_off,0]))))

        #T0_LH4 = dot(self.T0_LH1, MakeTransform(eye(3),transpose(matrix([0,self.hand_exit_back_off,0]))))
        #T0_RH4 = dot(self.T0_RH1, MakeTransform(eye(3),transpose(matrix([0,self.hand_exit_back_off,0]))))

        if(self.direction == "CW"):
            Bw0L = matrix([0,0,0,0,0,0,0,crank_rot,0,0,0,0])
        elif(self.direction == "CCW"):
            Bw0L = matrix([0,0,0,0,0,0,crank_rot,0,0,0,0,0])

        # Uncomment to see T0_LH1,2,3
        self.drawingHandles.append(misc.DrawAxes(self.env,matrix(self.T0_LH1),1))    
        self.drawingHandles.append(misc.DrawAxes(self.env,matrix(T0_LH2),1))
        self.drawingHandles.append(misc.DrawAxes(self.env,matrix(T0_LH3),1))

        # Uncomment to see T0_RH1,2,3
        self.drawingHandles.append(misc.DrawAxes(self.env,matrix(self.T0_RH1),1))
        self.drawingHandles.append(misc.DrawAxes(self.env,matrix(T0_RH2),1))
        self.drawingHandles.append(misc.DrawAxes(self.env,matrix(T0_RH3),1))
        
        # Set TRSs
        self.TSRs.SetTwoHandsTurn(self.valveJointInd,T0_w0L,Tw0_eL,Bw0L,\
                                                     T0_w0R,Tw0_eR,Bw0R,\
                                                     T0_w0H,Tw0_eH,Bw0H)

        # Set Crank
        print "set crank rot : " + str(crank_rot)
        self.crankid.SetDOFValues([crank_rot],[self.valveJointInd])
        self.crankid.GetController().Reset(0)

        # GENERAL IK CALLS for goal and exit
        [error,goalik] = self.FindTwoArmsIK( T0_RH2, T0_LH2, False )
        if error != 0 :
            print "Error : Cound not find goalik!!!!"
            self.crankid.SetDOFValues( [0], [0] )
            self.crankid.GetController().Reset(0)
            self.robotid.SetDOFValues( q_cur )
            return [error,goalik,None]
        else:
            print "Info : GeneralIK found a goalik."
            self.robotid.SetActiveDOFValues( goalik )
            self.robotid.GetController().Reset(0)          

        [error,exitik1] = self.FindTwoArmsIK( T0_RH3, T0_LH3, False )       
        if error != 0 :
            print "Error : Cound not find exitik1!!!!"
            self.crankid.SetDOFValues( [0], [0] )
            self.crankid.GetController().Reset(0)
            self.robotid.SetDOFValues( q_cur )
            return [error,goalik,exitik1]
        else:
            print "Info : GeneralIK found an exitik."
            self.robotid.SetActiveDOFValues( exitik1 )
            self.robotid.GetController().Reset(0)

        self.robotid.SetDOFValues( q_cur )
        self.crankid.SetDOFValues([0],[0])
        self.crankid.GetController().Reset(0)
        return [0,goalik,exitik1]

    # -------------------------------------------------------------------------
    # -------------------------------------------------------------------------
    # -------------------------------------------------------------------------
    def BothHands(self, hands, radius, valveType ):
        
        self.robotid.SetDOFValues(self.rhandopenvals,self.rhanddofs)
        self.robotid.SetDOFValues(self.lhandopenvals,self.lhanddofs)

        # Current configuration of the robot is its initial configuration
        currentik = self.robotid.GetActiveDOFValues()

        [success, why, startik] = self.FindStartConstraints( hands, valveType, False )
        if(not success):
            return why

        # Use this flag to compute the ik solutions
        start_from_init_ik = True

        if self.use_grasplist :
            [error, startik, manipik] = self.FindMaxTurnIK( hands, start_from_init_ik )
        else:
            [error, startik] = self.FindTwoArmsIK( self.T0_RH1, self.T0_LH1, True)

        if(error != 0):
            print "Error : cound not find startik!!!!"
            return ""

        if(self.direction == "CCW"):
            multiplier = -1
        elif(self.direction == "CW"):
            multiplier = 1

        crank_rot = 0.0

        if self.use_grasplist :
            # We try different rotation angle
            # until we find a feasible plan goal and exit
            i = 0
            for crank_rot in linspace( multiplier*pi/1.5, multiplier*pi/8, num=50) :
                del self.drawingHandles[:]
                [error,goalik,exitik1] = self.FindBothHandsGoalAndExtract(crank_rot)
                # Clear drawing of frames
                if error == 0:
                    print "Info : found goalik and exitik1 with crank_rot : " + str(crank_rot)
                    break
                i+=1
        else:
            crank_rot = (multiplier)*(pi/4)
            [error,goalik,exitik1] = self.FindBothHandsGoalAndExtract(crank_rot)

        if error != 0 :
            print "Error : cound not find goal and exit iks!!!!"
            return ""

        # At this point we should have a currentik and a goalik
        cp = ConstrainedPath( "TurnValveBH", self.robotid )
        cp.valveType = valveType

        # Define current to a known start configuration
        cpe0 = ConstrainedPathElement("current2start")
        cpe0.startik = currentik
        cpe0.goalik = startik
        cpe0.TSR = self.TSRs.TSRChainStringFeetandHead_init2start_bh
        cpe0.smoothing = self.normalsmoothingitrs
        cpe0.errorCode = "12"
        cpe0.filename = "movetraj2"
        cpe0.hands = "BH"
        cpe0.cbirrtProblems = [self.probs_cbirrt]
        cpe0.cbirrtRobots = [self.robotid]
        cpe0.cbirrtTrajectories = [self.default_trajectory_dir+cpe0.filename]
        cpe0.activedofs = self.GetActiveDOFs(self.onlyArms)
        cpe0.padWaist = True

        # Define start to goal
        cpe1 = ConstrainedPathElement("start2goal")
        cpe1.startik = startik
        cpe1.goalik = goalik
        cpe1.TSR = self.TSRs.TSRChainString_start2goal
        cpe1.smoothing = self.fastsmoothingitrs
        cpe1.errorCode = "13"
        cpe1.mimicdof = self.TSRs.TSRChainMimicDOF
        cpe1.filename = "movetraj3"
        cpe1.hands = "BH"
        cpe1.cbirrtProblems = [self.probs_cbirrt, self.probs_crankmover]
        cpe1.cbirrtRobots = [self.robotid, self.crankid]
        cpe1.cbirrtTrajectories = [self.default_trajectory_dir+cpe1.filename, self.default_trajectory_dir+cpe1.filename]
        cpe1.closeHandsBefore = True
        cpe1.openHandsAfter = True
        cpe1.activedofs = self.GetActiveDOFs(self.onlyArms)
        cpe1.padWaist = True

        # Define goal to exit1
        cpe2 = ConstrainedPathElement("goal2exit1")
        cpe2.startik = goalik
        cpe2.goalik = exitik1
        cpe2.TSR = self.TSRs.TSRChainStringFeetandHead_goal2start
        cpe2.smoothing = self.normalsmoothingitrs
        cpe2.errorCode = "14"
        cpe2.filename = "movetraj4"
        cpe2.hands = "BH"
        cpe2.cbirrtProblems = [self.probs_cbirrt]
        cpe2.cbirrtRobots = [self.robotid]
        cpe2.cbirrtTrajectories = [self.default_trajectory_dir+cpe2.filename]
        cpe2.activedofs = self.GetActiveDOFs(self.onlyArms)
        cpe2.padWaist = True

        # Define exit1 to exit2
        cpe3 = ConstrainedPathElement("exit12exit2")
        cpe3.startik = exitik1
        cpe3.goalik = currentik
        cpe3.TSR = self.TSRs.TSRChainStringFeetandHead_init2start_bh
        #cpe3.TSR = self.TSRs.TSRChainStringFeetandHead_goal2start
        cpe3.smoothing = self.normalsmoothingitrs
        cpe3.errorCode = "15"
        cpe3.filename = "movetraj5"
        cpe3.hands = "BH"
        cpe3.cbirrtProblems = [self.probs_cbirrt]
        cpe3.cbirrtRobots = [self.robotid]
        cpe3.cbirrtTrajectories = [self.default_trajectory_dir+cpe3.filename]
        cpe3.padValve = True
        cpe3.padWaist = True
        cpe3.activedofs = self.GetActiveDOFs(self.onlyArms)

        # Add both elements to the path
        cp.elements.append(cpe0)
        cp.elements.append(cpe1)
        cp.elements.append(cpe2)
        cp.elements.append(cpe3)

        # Plan for start -> goal -> start
        [success, why] = self.PlanPath(cp)
        if(not success):
            return why

        print "-------------------------------------------------------"
        print " MOTION PLANNED FOR A " + str(crank_rot*180/pi) + " degrees valve rotation!!!"
        print "-------------------------------------------------------"

        return 0

    # -------------------------------------------------------------------------
    # -------------------------------------------------------------------------
    # -------------------------------------------------------------------------
    def LeftHand(self, hands, radius, valveType ):

        currentik = self.robotid.GetActiveDOFValues()

        [success, why, startik] = self.FindStartConstraints( hands, valveType, True)
        if(not success):
            return why
        
        # This is a list of handles of the objects that are
        # drawn on the screen in OpenRAVE Qt-Viewer.
        # Keep appending to the end, and pop() if you want to delete.
        # handles = []

        # Calculate hand transforms after rotating the wheel (they will help us find the goalik):
        # How much do we want to rotate the wheel?
        if(self.direction == "CCW"):
            multiplier = -1
        elif(self.direction == "CW"):
            multiplier = 1
            
        crank_rot = (multiplier)*pi/4

        T0_w0L = dot(self.valveTroot,MakeTransform(rodrigues([0,-pi/2,0]),transpose(matrix([0,0,0]))))
        T0_w0L = dot(T0_w0L,MakeTransform(rodrigues([-pi/2,0,0]),transpose(matrix([0,0,0]))))

        # Left hand's transform in wheel's coordinates
        Tw0L_LH1 = dot(linalg.inv(T0_w0L),self.T0_LH1)

        # Transform of the left hand's end effector in wheel's coords.
        # Required by CBiRRT
        Tw0_eL = Tw0L_LH1

        # How much freedom do we want to give to the left hand
        if(self.direction == "CW"):
            Bw0L = matrix([0,0,0,0,0,0,0,crank_rot,0,0,0,0])
        elif(self.direction == "CCW"):
            Bw0L = matrix([0,0,0,0,0,0,crank_rot,0,0,0,0,0])

        # Right Hand's transforms:
        T0_crankcrank = self.crankid.GetManipulators()[0].GetTransform()

        # Head's transforms:
        T0_w0H =  self.robotManips[4].GetEndEffectorTransform()
        Tw0_eH = eye(4)
        Bw0H = matrix([0,0,0,0,0,0,0,0,0,0,0,0])

        # Set TSRs
        self.TSRs.SetLeftHandTurn(self.valveJointInd,T0_w0L,Tw0_eL,Bw0L,T0_w0H,Tw0_eH,Bw0H)

        # Create the transform for the wheel that we would like to reach to
        Tcrank_rot = MakeTransform(rodrigues([crank_rot,0,0]),transpose(matrix([0,0,0])))

        # What is this?
        temp = MakeTransform(rodrigues([0,0,crank_rot]),transpose(matrix([0,0,0])))

        # Rotate the left hand's transform on the wheel in world transform "crank_rot" radians around it's Z-Axis
        T0_cranknew = dot(T0_w0L,Tcrank_rot)

        T0_cranknew = dot(self.crankid.GetManipulators()[0].GetEndEffectorTransform(), MakeTransform(rodrigues([0,0,crank_rot]),transpose(matrix([0,0,0]))))

        # Where will the left hand go after turning the wheel?
        T0_LH2 = dot(T0_cranknew,dot(linalg.inv(self.crankid.GetManipulators()[0].GetEndEffectorTransform()),self.T0_LH1))

        # Uncomment to see T0_LH2
        self.drawingHandles.append(misc.DrawAxes(self.env,matrix(T0_LH2),1))

        # check if ik solutions exist
        goalik = self.FindActiveQ([0],[array(T0_LH2)])
        if( goalik == None ):
            print "Error: No goalik found!"
            return 33 # 3: 
            
        cp = ConstrainedPath( "TurnValveLH", self.robotid )
        cp.valveType = valveType

        # Define current to a known start configuration
        cpe0 = ConstrainedPathElement("current2start")
        cpe0.startik = currentik
        cpe0.goalik = startik
        cpe0.TSR = self.TSRs.TSRChainStringFeetandHead_init2start_lh
        cpe0.smoothing = self.normalsmoothingitrs
        cpe0.errorCode = "12"
        cpe0.psample = 0
        cpe0.filename = "movetraj2"
        cpe0.hands = "LH"
        cpe0.cbirrtProblems = [self.probs_cbirrt]
        cpe0.cbirrtRobots = [self.robotid]
        cpe0.cbirrtTrajectories = [self.default_trajectory_dir+cpe0.filename]
        cpe0.activedofs = self.GetActiveDOFs(self.onlyArms)
        cpe0.padWaist = True
        
        cpe1 = ConstrainedPathElement("start2goal")
        cpe1.startik = startik
        cpe1.goalik = goalik
        cpe1.TSR = self.TSRs.TSRChainString
        cpe1.smoothing = self.fastsmoothingitrs
        cpe1.errorCode = "13"
        cpe1.mimicdof = self.TSRs.TSRChainMimicDOF
        cpe1.filename = "movetraj3"
        cpe1.hands = "LH"
        cpe1.cbirrtProblems = [self.probs_cbirrt, self.probs_crankmover]
        cpe1.cbirrtRobots = [self.robotid, self.crankid]
        cpe1.cbirrtTrajectories = [self.default_trajectory_dir+cpe1.filename, self.default_trajectory_dir+cpe1.filename]
        cpe1.activedofs = self.GetActiveDOFs(self.onlyArms)
        cpe1.closeHandsBefore = True
        cpe1.openHandsAfter = True
        cpe1.padWaist = True

        # Define goal to current
        cpe2 = ConstrainedPathElement("goal2current")
        cpe2.startik = goalik
        cpe2.goalik = currentik
        cpe2.TSR = self.TSRs.TSRChainStringFeetandHead_init2start_lh
        cpe2.smoothing = self.normalsmoothingitrs
        cpe2.errorCode = "14"
        cpe2.psample = 0
        cpe2.filename = "movetraj4"
        cpe2.hands = "LH"
        cpe2.cbirrtProblems = [self.probs_cbirrt]
        cpe2.cbirrtRobots = [self.robotid]
        cpe2.cbirrtTrajectories = [self.default_trajectory_dir+cpe2.filename]
        cpe2.activedofs = self.GetActiveDOFs(self.onlyArms)
        cpe2.padWaist = True

        cp.elements.append(cpe0)
        cp.elements.append(cpe1)

        # If the valve is round, then go back to currentik after goal
        if( valveType == "W" ):
            cp.elements.append(cpe2)
        
        [success, why] = self.PlanPath(cp)
        if(not success):
            return why
        else:
            return 0 # no error

    # -------------------------------------------------------------------------
    # -------------------------------------------------------------------------
    # -------------------------------------------------------------------------
    def RightHand(self, hands, radius, valveType):

        currentik = self.robotid.GetActiveDOFValues()

        [success, why, startik] = self.FindStartConstraints( hands, valveType, True)
        if(not success):
            return why

        # This is a list of handles of the objects that are
        # drawn on the screen in OpenRAVE Qt-Viewer.
        # Keep appending to the end, and pop() if you want to delete.
        # handles = [] 

        if(self.direction == "CCW"):
            multiplier = -1
        elif(self.direction == "CW"):
            multiplier = 1

        crank_rot = (multiplier)*pi/4

        T0_w0R = dot(self.valveTroot,MakeTransform(rodrigues([0,-pi/2,0]),transpose(matrix([0,0,0]))))
        T0_w0R = dot(T0_w0R,MakeTransform(rodrigues([-pi/2,0,0]),transpose(matrix([0,0,0]))))

        Tw0R_RH1 = dot(linalg.inv(T0_w0R),self.T0_RH1)

        Tw0_eR = Tw0R_RH1
        
        if(self.direction == "CW"):
            Bw0R = matrix([0,0,0,0,0,0,0,crank_rot,0,0,0,0])
        elif(self.direction == "CCW"):
            Bw0R = matrix([0,0,0,0,0,0,crank_rot,0,0,0,0,0])

        # Right Hand's transforms:
        T0_crankcrank = self.crankid.GetManipulators()[0].GetTransform()
            
        # Head's transforms:
        T0_w0H =  self.robotManips[4].GetEndEffectorTransform()
        Tw0_eH = eye(4);
        Bw0H = matrix([0,0,0,0,0,0,0,0,0,0,0,0])

        # Set TSRs
        self.TSRs.SetRightHandTurn(self.valveJointInd,T0_w0L,Tw0_eL,Bw0L,T0_w0H,Tw0_eH,Bw0H)

        # Create the transform for the wheel that we would like to reach to
        Tcrank_rot = MakeTransform(rodrigues([crank_rot,0,0]),transpose(matrix([0,0,0])))

        # What is this?
        temp = MakeTransform(rodrigues([0,0,crank_rot]),transpose(matrix([0,0,0])))

        # Rotate the left hand's transform on the wheel in world transform "crank_rot" radians around it's Z-Axis
        T0_cranknew = dot(T0_w0R,Tcrank_rot)


        T0_cranknew = dot(self.crankid.GetManipulators()[0].GetEndEffectorTransform(), MakeTransform(rodrigues([0,0,crank_rot]),transpose(matrix([0,0,0]))))

        T0_RH2 = dot(T0_cranknew,dot(linalg.inv(self.crankid.GetManipulators()[0].GetEndEffectorTransform()),self.T0_RH1))
        
        # Uncomment to see T0_RH2
        self.drawingHandles.append(misc.DrawAxes(self.env,matrix(T0_RH2),1))

        # check if ik solutions exist
        goalik = self.FindActiveQ([1],[array(T0_RH2)])
        if( goalik == None ):
            print "Error: No goalik found!"
            return 33

        cp = ConstrainedPath( "TurnValveRH", self.robotid )
        cp.valveType = valveType

        # Define current to a known start configuration
        cpe0 = ConstrainedPathElement("current2start")
        cpe0.startik = currentik
        cpe0.goalik = startik
        cpe0.TSR = self.TSRs.TSRChainStringFeetandHead_init2start_rh
        cpe0.smoothing = self.normalsmoothingitrs
        cpe0.errorCode = "12"
        cpe0.psample = 0
        cpe0.filename = "movetraj2"
        cpe0.hands = "RH"
        cpe0.cbirrtProblems = [self.probs_cbirrt]
        cpe0.cbirrtRobots = [self.robotid]
        cpe0.cbirrtTrajectories = [self.default_trajectory_dir+cpe0.filename]
        cpe0.activedofs = self.GetActiveDOFs(self.onlyArms)
        cpe0.padWaist = True
        
        cpe1 = ConstrainedPathElement("start2goal")
        cpe1.startik = startik
        cpe1.goalik = goalik
        cpe1.TSR = self.TSRs.TSRChainString
        cpe1.smoothing = self.fastsmoothingitrs
        cpe1.mimicdof = self.TSRs.TSRChainMimicDOF
        cpe1.filename = "movetraj3"
        cpe1.hands = "RH"
        cpe1.errorCode = "13"
        cpe1.cbirrtProblems = [self.probs_cbirrt, self.probs_crankmover]
        cpe1.cbirrtRobots = [self.robotid, self.crankid]
        cpe1.cbirrtTrajectories = [self.default_trajectory_dir+cpe1.filename, self.default_trajectory_dir+cpe1.filename]
        cpe1.closeHandsBefore = True
        cpe1.openHandsAfter = True
        cpe1.activedofs = self.GetActiveDOFs(self.onlyArms)
        cpe1.padWaist = True

        # Define goal to current
        cpe2 = ConstrainedPathElement("goal2current")
        cpe2.startik = goalik
        cpe2.goalik = currentik
        cpe2.TSR = self.TSRs.TSRChainStringFeetandHead_init2start_rh
        cpe2.smoothing = self.normalsmoothingitrs
        cpe2.errorCode = "14"
        cpe2.psample = 0
        cpe2.filename = "movetraj4"
        cpe2.hands = "RH"
        cpe2.cbirrtProblems = [self.probs_cbirrt]
        cpe2.cbirrtRobots = [self.robotid]
        cpe2.cbirrtTrajectories = [self.default_trajectory_dir+cpe2.filename]
        cpe2.activedofs = self.GetActiveDOFs(self.onlyArms)
        cpe2.padWaist = True

        cp.elements.append(cpe0)
        cp.elements.append(cpe1)
        # If the valve is round, then go back to currentik after goal
        if( valveType == "W" ):
            cp.elements.append(cpe2)

        [success, why] = self.PlanPath(cp)
        if(not success):
            return why
        else:
            return 0

    # -------------------------------------------------------------------------
    # -------------------------------------------------------------------------
    # -------------------------------------------------------------------------
    def EndTask(self, hands, valveType):
        # Wherever you are,
        currentik = self.robotid.GetActiveDOFValues()

        # Set the TSRs for initik --> home
        # To do that, we need the end effector transforms at homeIK
        self.AvoidSingularity(self.robotid)
        self.homeik = self.robotid.GetActiveDOFValues()
       
        # Set the TSRs for End motions
        self.TSRs.SetEnd(self.robotid, currentik, self.initik, self.T0_TSY )
        
        # We now have the TSRs
        # Set the robot back to currentik
        self.robotid.SetActiveDOFValues(currentik)
                
        # 1. make sure you open your hands
        # 2. Go to a safe position

        # Set the path element
        cpe0 = ConstrainedPathElement("current2init")
        cpe0.startik = currentik
        cpe0.goalik = self.initik
        if( hands == "BH" ):
            cpe0.TSR = self.TSRs.TSRChainStringFeetandHead_current2init_bh
        elif( hands == "LH" ):
            cpe0.TSR = self.TSRs.TSRChainStringFeetandHead_current2init_lh
        elif( hands == "RH" ):
            cpe0.TSR = self.TSRs.TSRChainStringFeetandHead_current2init_rh
        cpe0.smoothing = self.normalsmoothingitrs
        cpe0.errorCode = "17"
        cpe0.psample = 0
        cpe0.filename = "movetraj7"
        cpe0.hands = hands
        cpe0.cbirrtProblems = [self.probs_cbirrt]
        cpe0.cbirrtRobots = [self.robotid]
        cpe0.cbirrtTrajectories = [self.default_trajectory_dir+cpe0.filename]
        cpe0.activedofs = self.GetActiveDOFs(self.onlyArms)
        cpe0.padValve = True
        cpe0.padWaist = True

        # 3. open your hands before
        cpe0.openHandsBefore = True
        # 4. close your hands after
        cpe0.closeHandsAfter = True

        # 4. Go back home
        cpe1 = ConstrainedPathElement("init2home")
        cpe1.startik = self.initik
        cpe1.goalik = self.homeik
        cpe1.TSR = self.TSRs.TSRChainStringFeetandHead_init2home
        cpe1.smoothing = self.normalsmoothingitrs
        cpe1.errorCode = "18"
        cpe1.psample = 0
        cpe1.filename = "movetraj8"
        cpe1.hands = hands
        cpe1.cbirrtProblems = [self.probs_cbirrt]
        cpe1.cbirrtRobots = [self.robotid]
        cpe1.cbirrtTrajectories = [self.default_trajectory_dir+cpe1.filename]
        cpe1.activedofs = self.alldofs
        cpe1.padValve = True
            
        cp = ConstrainedPath( "EndTask", self.robotid )
        cp.valveType = valveType

        cp.elements.append(cpe0)
        cp.elements.append(cpe1)
        
        [success, why] = self.PlanPath(cp)
        if(not success):
            return why
        else:
            return 0

    # -------------------------------------------------------------------------
    # -------------------------------------------------------------------------
    # -------------------------------------------------------------------------

    def GetGraspListLeft(self):

        if(self.direction == "CCW"):
            multiplier = -1
        elif(self.direction == "CW"):
            multiplier = 1

        grasplist = []
        for alpha in linspace( multiplier*pi/2, 0, num=50 ) :
            grasplist.append( self.GetT0_AroundValve(self.hand_offset,alpha) )
            self.drawingHandles.append( misc.DrawAxes(self.env,grasplist[-1],1) )
        return grasplist

    def GetGraspListRight(self):

        if(self.direction == "CCW"):
            multiplier = -1
        elif(self.direction == "CW"):
            multiplier = 1        

        grasplist = []
        for alpha in linspace( multiplier*3*pi/2, multiplier*pi, num=50 ) :
            grasplist.append( self.GetT0_AroundValve(self.hand_offset,alpha) )
            self.drawingHandles.append( misc.DrawAxes(self.env,grasplist[-1],1) )
        return grasplist

    def GetT0_AroundValve( self, offset, angle ):
        temp = dot(self.valveTee, MakeTransform(rodrigues([-pi/2,0,0]),transpose(matrix([0,0,0]))))
        temp = dot(temp, MakeTransform(rodrigues([0,angle,0]),transpose(matrix([0,0,0]))))
        temp = dot(temp, MakeTransform(rodrigues([0,0,-pi/2]),transpose(matrix([0,0,0]))))
        temp = dot(temp, MakeTransform(rodrigues([0,0,pi/4]),transpose(matrix([-offset,self.r_Wheel+0.005,0]))))
        return temp

    def GetT0_LH1(self, hands, whichGrasp, valveType, offset ):

        if( hands == "BH" ):
            # Figure out where to put the left and hand on the valve
            if(valveType == "W"):
                if(whichGrasp == 0):
                    return self.GetT0_AroundValve(offset,0)
                
        if( hands == "LH" ):
            # Figure out where to put the left hand on the wheel
            temp = dot(self.valveTee, MakeTransform(rodrigues([0,0,pi/2]),transpose(matrix([0,0,0]))))
            temp = dot(temp, MakeTransform(rodrigues([-pi/2,0,0]),transpose(matrix([0,0,0]))))

            # Left Hand Pose in World Coordinates
            if(valveType == "RL"): # if lever (right end at the origin of rotation), hold it from the tip of the handle
                offset = 0.01
                return dot(temp, MakeTransform(rodrigues([0,0,0]),transpose(matrix([0,offset,-1*(self.r_Wheel-0.005)]))))
            if(valveType == "LL"): # if lever (left end at the origin of rotation), hold it from the tip of the handle
                offset = 0.01
                return dot(temp, MakeTransform(rodrigues([0,0,0]),transpose(matrix([0,offset,(self.r_Wheel-0.005)]))))

            if(valveType == "W"): # if it's a small wheel, hold it from the center but back off a little
                return dot(temp, MakeTransform(rodrigues([0,0,0]),transpose(matrix([0,offset,0]))))

        if( hands == "RH" ):
            return self.robotManips[0].GetEndEffectorTransform()

        print "Error : valve type and hand choice incompatible"
        return None


    def GetT0_RH1(self, hands, whichGrasp, valveType, offset ):
        if( hands == "BH" ):
            # Figure out where to put the right hand on the valve
            if(valveType == "W"):
                if(whichGrasp == 0):
                    return self.GetT0_AroundValve(offset,pi)

        if( hands == "LH" ):
            return self.robotManips[1].GetEndEffectorTransform()

        if( hands == "RH" ):
            temp = dot(self.valveTee, MakeTransform(rodrigues([0,0,pi/2]),transpose(matrix([0,0,0]))))
            temp = dot(temp, MakeTransform(rodrigues([-pi/2,0,0]),transpose(matrix([0,0,0]))))

            # Right Hand Pose in World Coordinates
            if(valveType == "RL"): # if lever (right end at the origin of rotation), hold it from the tip of the handle
                offset = 0.01
                return dot(temp, MakeTransform(rodrigues([0,0,0]),transpose(matrix([0,0.01,-1*(self.r_Wheel-0.005)]))))
            if(valveType == "LL"): # if lever (left end at the origin of rotation), hold it from the tip of the handle
                offset = 0.01
                return dot(temp, MakeTransform(rodrigues([0,0,0]),transpose(matrix([0,0.01,self.r_Wheel-0.005]))))
            if(valveType == "W"): # if it's a small wheel, hold it from the center but back off a little
                return dot(temp, MakeTransform(rodrigues([0,0,0]),transpose(matrix([0,offset,0]))))

        print "Error : valve type and hand choice incompatible"
        return None


    def CheckHands(self, radius, valveType):
        # Check if, because of compliance, or some other reason (active balancing / sensor error etc.)
        # the end effectors of the robot matches to {R,L}H1 for the most recent valve pose.
        # 
        # If the end effector's are not where they should be, then plan a trajectory in between
        # and go to start configuration.
        T0_CurrentLH = self.robotManips[0].GetEndEffectorTransform()
        T0_CurrentRH = self.robotManips[1].GetEndEffectorTransform()

        # set the result to false first
        goToStartIK = False

        if(not allclose(self.T0_LH1, T0_CurrentLH) ):
            print "Warning: leftArm end effector has moved."
            goToStartIK = True

        if(not allclose(self.T0_RH1, T0_CurrentRH) ):
            print "Warning: rightArm end effector has moved."
            goToStartIK = True

        return goToStartIK


    def FindActiveQ(self, manipIndices, manipTransforms):
        # Try to find a GeneralIK solution for the manipulators
        # if it fails, try finding an IKFast solution
        cmdStr = 'DoGeneralIK exec supportlinks 2'+self.footlinknames+' movecog '+self.cogTargStr+' nummanips '+str(len(manipIndices))+' maniptm'
        for i, manipIdx in enumerate(manipIndices):
            cmdStr += ' '+str(manipIdx)+' '+trans_to_str(manipTransforms[i])
        
        # command string is ready. Solve GeneralIK
        generalik = self.probs_cbirrt.SendCommand(cmdStr)

        # check if generalik succeeded without any collision
        if( generalik == '' or (self.env.CheckCollision(self.robotid) or self.robotid.CheckSelfCollision()) ):
            print "Error: generalik failed, or the solution found is in collision"

            if( self.useIKFast ):
                for i, manipIdx in enumerate(manipIndices):
                    print "Info: using IKFast."
                    sol = self.IKFast(self.robotManips[manipIdx], array(manipTransforms[i]), False)
                    if( sol is not None):
                        self.robotid.SetDOFValues(sol, self.robotManips[manipIdx].GetArmIndices())
                    else:
                        print "Error: IKFast could not find a solution."
                        return None
        else:
            return generalik

        # if we are here, it means generalik failed, or there was a collision
        # so, we tried IKFast, and it succeeded, let's return active dof configuration
        return self.robotid.GetActiveDOFValues()

    def SetRobotConfiguration(self,q):
        # q is a dictionary
        #
        # This method matches the joint indices between ROS and OpenRAVE.
        # For planning purposes, we skip head and finger
        # joints.
        for jName, jValue in q.iteritems():
            if( jName[0:2] != 'RF' and jName[0:2] != 'LF' and jName[0:2] != 'NK' ):
                rosValue = jValue
                openraveIdx  = self.jointDict[jName]
                self.robotid.SetDOFValues([rosValue],[openraveIdx])
                self.robotid.GetController().Reset(0)
            else:
                print "Info: set robot config is skipping :"+jName

    def GetFeetTargetsInit(self):
        
        T0_TSY = self.GetT0_RefLink("Body_TSY")
        if T0_TSY == None:
            print "Body_TSY does not exist"
            return None
        
        # Left Foot Target in World Coords.
        T0_LF = deepcopy(T0_TSY)
        T0_LF[0,3] = T0_TSY[0,3]+self.Ttsy_lar_home[0,3]
        T0_LF[1,3] = T0_TSY[1,3]+self.Ttsy_lar_home[1,3]
        T0_LF[2,3] = T0_TSY[2,3]+self.Ttsy_lar_home[2,3]+self.crouch

        # Right Foot Target in World Coords.
        T0_RF = deepcopy(T0_TSY)
        T0_RF[0,3] = T0_TSY[0,3]+self.Ttsy_rar_home[0,3]
        T0_RF[1,3] = T0_TSY[1,3]+self.Ttsy_rar_home[1,3]
        T0_RF[2,3] = T0_TSY[2,3]+self.Ttsy_rar_home[2,3]+self.crouch

        hlfoot = misc.DrawAxes(self.env,T0_LF,1)
        hrfoot = misc.DrawAxes(self.env,T0_RF,1)
        
        return [T0_LF, T0_RF]

    def GetCurrentHandPose(self):
        T0_LH = self.GetT0_RefLink("leftPalm")
        T0_RH = self.GetT0_RefLink("rightPalm")
        if T0_LH == None or T0_RH == None :
            print "leftPalm or rightPalm does not exist"
            return None
        return [T0_LH,T0_RH]

    def GetHandTargetsStand(self):

        [T0_LH,T0_RH] = self.GetCurrentHandPose()

        T0_LH1 = deepcopy(T0_LH)        
        T0_RH1 = deepcopy(T0_RH)  

        [tx,ty,tz] = [-0.10,0.10,0.40]
        [rx,ry,rz] = [-pi/6,0,0]
        temp = eye(4)
        temp = temp * MakeTransform(rodrigues([rx,0,0]),transpose(matrix([0,0,0])))
        temp = temp * MakeTransform(rodrigues([0,ry,0]),transpose(matrix([0,0,0])))
        temp = temp * MakeTransform(rodrigues([0,0,rz]),transpose(matrix([0,0,0])))
        T0_LH1 = T0_LH * temp
        T0_LH1 = MakeTransform( xyz_rotation([0,0,0]), transpose(matrix([tx,ty,tz]))) * T0_LH1

        [tx,ty,tz] = [-0.10,-0.10,0.40]
        [rx,ry,rz] = [pi/6,0,0]
        temp = eye(4)
        temp = temp * MakeTransform(rodrigues([rx,0,0]),transpose(matrix([0,0,0])))
        temp = temp * MakeTransform(rodrigues([0,ry,0]),transpose(matrix([0,0,0])))
        temp = temp * MakeTransform(rodrigues([0,0,rz]),transpose(matrix([0,0,0])))
        T0_RH1 = T0_RH * temp
        T0_RH1 = MakeTransform( xyz_rotation([0,0,0]), transpose(matrix([tx,ty,tz]))) * T0_RH1

        print T0_LH1
        print T0_RH1

        self.drawingHandles.append( misc.DrawAxes(self.env,matrix(T0_LH1),1) )
        self.drawingHandles.append( misc.DrawAxes(self.env,matrix(T0_RH1),1) )

        return [T0_LH1, T0_RH1]
    
    def BendTheKnees(self,howMuch=0.1):
        keepFeetParallel = False
        
        # TODO
        if(keepFeetParallel):
            pass
        #    self.robotid.SetDOFValues([],[])
        else:
            # LKP: 14, RKP: 39
            self.robotid.SetDOFValues([2*howMuch,2*howMuch],[14,39])
            # LHP: 13, RHP: 38
            self.robotid.SetDOFValues([-howMuch,-howMuch],[13,38])
            # LAP: 15, RAP: 40
            self.robotid.SetDOFValues([-howMuch,-howMuch],[15,40])

    def GetActiveDOFs(self,onlyArms=False):
        # Keep Active Joint Indices
        activedofs = []
        for m in self.robotManips:
            # only add as active the arms with the legs (no head)
            if (( not onlyArms ) and (not m.GetName() == self.head)) \
                or ( m.GetName() == self.leftArm ) \
                or ( m.GetName() == self.rightArm ):
                activedofs.extend(m.GetArmIndices())
                print "Add manipulator to plan : " + m.GetName()
                #sys.stdin.readline()

        activedofs.sort()
        return activedofs
        
    def GetManipulationBox(self):
        
        print "----------- GetManipulationBox ---------------- "

        self.T0_TSY = self.GetT0_RefLink("Body_TSY")
        if self.T0_TSY == None:
            print "Body_TSY does not exist"
            return None

        T_crank = self.crankid.GetManipulators()[0].GetTransform()
        # uncomment to draw TSY and crank
        # self.drawingHandles.append( misc.DrawAxes(self.env,matrix(eye(4)),1) )
        # self.drawingHandles.append( misc.DrawAxes(self.env,matrix(self.T0_TSY),1) )
        # self.drawingHandles.append( misc.DrawAxes(self.env,matrix(T_crank),1) )
        
        front = T_crank[0,3]
        width = 1.0 # TODO set in a better way
        height = 1.0

        self.TSRs.manipbox_dim = [0, front, -width/2, width/2, -height/2, height/2]

        if self.TSRs.draw_manip_box :
            if(self.env.GetKinBody("manipbox") is not None):
                self.env.RemoveKinBody(self.manipbox_draw)
            self.TSRs.manipbox_draw = RaveCreateKinBody(self.env,'')
            self.TSRs.manipbox_draw.SetName('manipbox')
            self.TSRs.manipbox_draw.InitFromBoxes(array([[front/2,0,0,front/2,width/2,height/2]]),True) # False for not visible
            self.TSRs.manipbox_draw.GetLinks()[0].GetGeometries()[0].SetDiffuseColor(array((0,0,1)))
            self.TSRs.manipbox_draw.GetLinks()[0].GetGeometries()[0].SetTransparency(0.7)
            self.TSRs.manipbox_draw.SetTransform(self.T0_TSY)
            self.TSRs.manipbox_draw.Enable(False)
            self.env.Add(self.manipbox_draw,True)
        return
            
    # -------------------------------------------------------------------------
    # -------------------------------------------------------------------------
    # -------------------------------------------------------------------------
    def Plan(self, handles=[], radius=None, manipulator=None, direction="CW", valveType=None, taskStage=None, UserPoses=None ):

        # Clear drawing of frames
        del self.drawingHandles[:]

        # Set all joints to plan
        self.planAllDOFIk = True
        self.robotid.SetActiveDOFs( self.alldofs )
        # Save current configuration
        self.q_cur = self.robotid.GetDOFValues()

        if(radius != None):
            self.r_Wheel = radius
        
        if(self.optWall):
            self.AddWall()

        # Set planning variables
        self.normalsmoothingitrs = 300
        self.fastsmoothingitrs = 20

        # Valve Transform End Effector in World Coordinates
        # This is the transformation matrix of the end effector 
        self.valveTee = self.valveManip[0].GetEndEffectorTransform()

        # valve has two links: 
        # 0) pole - the blue cylinder in the model, and, 
        # 1) crank - handle
        self.valveTroot = self.valveLinks[0].GetTransform()

        # keep valve's only joint index for mimicking later
        self.valveJointInd = 0

        # add cbirrt problems
        self.SetProblems()

        # reset TSRs
        self.TSRs = DrcHuboValveTurningTSRs( self.robotManips )

        # set the direction of turning
        self.direction = direction

        # Set the end-effector box
        self.GetManipulationBox()

        # Set if the pose is user defined
        if manipulator[:4] == "USER" :
            self.useUserPoses = True
            self.SetUserPoses( UserPoses )
            manipulator = manipulator[5:]
        else :
            self.useUserPoses = False

        # the error code should turn to 0 when everything is fine
        error_code = -1
            
        # Plans for one of the sequence GETREADY, TURNVALVE, END
        if( taskStage == 'GETREADY' ):
            
            if (self.state == 0):
                error_code = self.GetReady(manipulator, valveType)
                if( error_code == 0):
                    self.state = 1 # GetReady() Done.
            else:
                print "Warning: You can not plan for GetReady in this state. Please plan for Finish Task first."

        elif( taskStage == 'GRASP' ):
            print "GRASP STAGE"

        elif( taskStage == 'UNGRASP' ):
            print "UNGRASP STAGE"

        elif( taskStage == 'TURNVALVE' ):

            # In the manipulation phase the active dofs are set for all trajs
            self.planAllDOFIk = False
            self.robotid.SetActiveDOFs( self.GetActiveDOFs(self.onlyArms) )           

            if (self.state > 0):
                if( manipulator == "LH" ):
                    error_code = self.LeftHand(manipulator, radius, valveType)

                elif( manipulator == "RH" ):
                    error_code = self.RightHand(manipulator, radius, valveType)

                elif( manipulator == "BH" ):
                    error_code = self.BothHands(manipulator, radius, valveType)

                if( error_code == 0 ):
                    self.state = 2
            else:
                print "Warning: You can not plan for Turn Valve in this state. Please plan for getting ready first."

        elif( taskStage == 'END' ):

            if (self.state > 0):
                error_code = self.EndTask(manipulator, valveType)
                if( error_code == 0):
                    self.state = 0
            else:
                print "Warning: You can not plan for End Task in this state. Please plan for getting ready first."

        print "Info: planner is waiting for the next call..."

        self.robotid.SetActiveDOFs( self.alldofs )  

        if( error_code != 0 ):
            # Set the robot back current configuration
            # commented for debug
            #self.robotid.GetController().SetDesired( self.q_cur )
            #self.robotid.SetDOFValues( self.q_cur ) # Is this one necessary ?
            print "Set robot to initial configuration"

        return error_code 

def StartPlannerEnvironment(handles,play,wall,taskwall,demo,removeFiles) :

    planner = DrcHuboV2WheelTurning()
    planner.optPlay = play
    planner.optTaskWall = taskwall
    planner.optWall = wall
    planner.optDemo = demo
    planner.useIKFast = False
    if removeFiles:
        planner.RemoveFiles()
    planner.SetViewer(True)
    planner.StartViewerAndSetValvePos( handles )
    return planner

def main():
    # One can run this script from terminal passing a radius value in
    # Otherwise use the default value
    r = None
    play = False
    taskwall = False
    wall = False
    demo = False
    plan = False
    removeFiles = True

    if(len(sys.argv) >= 2):
        for index in range(1,len(sys.argv)):
            if(sys.argv[index] == "-radius" and index+1<len(sys.argv)):
                r = float(sys.argv[index+1])
            elif(sys.argv[index] == "-play"):
                play = True
            elif(sys.argv[index] == "-taskwall"):
                taskwall = True
            elif(sys.argv[index] == "-wall"):
                wall = True
            elif(sys.argv[index] == "-demo"):
                demo = True
            elif(sys.argv[index] == "-plan"):
                plan = True

    # Dependency to roslib has been removed from this file
    # the stand alone executable is supposed to be launched from the folder
    # the drchubo models are supposed to be installed in the main catkin src folder
    # HuboModelPath = roslib.packages.get_pkg_dir("drchubo_v2")+'/robots/drchubo_v2.robot.xml',
    # WheelModelPath = roslib.packages.get_pkg_dir("valve_planner")+'/models/driving_wheel_tiny.robot.xml' )

    HuboModelPath = '../../../../drchubo/robots/drchubo_v2.robot.xml'
    WheelModelPath = '../../models/driving_wheel_tiny.robot.xml'

    planner = DrcHuboV2WheelTurning(HuboModelPath,WheelModelPath)   

    planner.optPlay = play
    planner.optTaskWall = taskwall
    planner.optWall = wall
    planner.optDemo = demo

    planner.SetViewer(True)

    handles = []
    
    if play:

        removeFiles = False
        planner = StartPlannerEnvironment( handles, play, wall, taskwall, demo, removeFiles )  
        planner.SetProblems()
        planner.SetStopKeyStrokes(True)
        planner.Playback()
     
    elif plan:

        ValveSize = 0.16
        Hands = "BH"
        Direction = "CW"
        ValveType = "W"

        planner = StartPlannerEnvironment( handles, play, wall, taskwall, demo, removeFiles )  
        planner.SetStopKeyStrokes(False)

        time.sleep(1)

        print "Plan for get ready"
        if( planner.Plan( [], ValveSize, Hands, Direction, ValveType, 'GETREADY' ) == 0 ) :
            print "Plan for turnvalve ---------------------------"
            print "----------------------------------------------"
            sys.stdin.readline()
            if( planner.Plan( [], ValveSize, Hands, Direction, ValveType, 'TURNVALVE' ) == 0 ) :
                print "Plan for end ---------------------------"
                print "----------------------------------------"
                sys.stdin.readline()
                if( planner.Plan( [], ValveSize, Hands, Direction, ValveType, 'END' ) == 0 ) :
                    print "All planning succeeded -----------------------"
                    print "----------------------------------------------"
                    sys.stdin.readline()

        print planner.env.GetViewer().GetCameraTransform()

#        while True:
#            continue

#    planner.KillOpenrave()

if __name__ == "__main__":

    main()

