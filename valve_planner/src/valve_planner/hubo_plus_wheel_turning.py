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
from base_wheel_turning import *

class HuboPlusWheelTurning( BaseWheelTurning ):

    def __init__(self,
                 HuboModelPath = '../../openHubo/huboplus/rlhuboplus.robot.xml',
                 WheelModelPath = '../../../drc_common/models/driving_wheel.robot.xml' ):

        BaseWheelTurning.__init__( self, HuboModelPath, WheelModelPath )

        # Set those variables to show or hide the interface
        # Do it using the member functions
        self.StopAtKeyStrokes = False
        self.ShowUserInterface = False
        self.ViewerStarted = False

	# Right Hand Joints 
        # Open - Closed Values
        self.rhanddofs = range(27,42)
        self.rhandclosevals = [0.439, 0.683, 0.497, 0.439, 0.683, 0.497, 0.439, 0.683, 0.497, 0.439, 0.683, 0.497, 0, 0, 1.2]
        self.rhandopenvals = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.08]

        # Left Hand Joints
        self.lhanddofs = range(42,57)
        self.lhandclosevals = [0.439, 0.683, 0.497, 0.439, 0.683, 0.497, 0.439, 0.683, 0.497, 0.439, 0.683, 0.497, 0, 0, 1.2]
        self.lhandopenvals =  [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.08]

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
    
        self.RemoveFiles()

        # This is a list of handles of the objects that are
        # drawn on the screen in OpenRAVE Qt-Viewer.
        # Keep appending to the end, and pop() if you want to delete.
        handles = [] 

        normalsmoothingitrs = 150;
        fastsmoothingitrs = 20;

        self.StartViewerAndSetWheelPos( handles )

        # Wheel Joint Index  
        crankjointind = 0
        # Set the wheel joints back to 0 for replanning
        self.crankid.SetDOFValues([0],[crankjointind])
        self.crankid.GetController().Reset(0)

        manips = self.robotid.GetManipulators()
        crankmanip = self.crankid.GetManipulators()
        
        try:
            cbirrtHubo = RaveCBiRRT(self.env,'rlhuboplus')
            cbirrtWheel = RaveCBiRRT(self.env,'crank')
        except openrave_exception, e:
            print e
            return []

        # Keep Active Joint Indices
        # Note that 0 is the driving wheel
        #activedofs = [0]
        activedofs = []
        for m in manips:
            # print m.GetArmIndices()
            activedofs.extend(m.GetArmIndices())

        # Sort Active Joint Indices
        activedofs.sort()
        #print activedofs

        # Set Elbows and Thumbs Joint Values
        self.robotid.SetDOFValues([-0.95,-0.95,1,1],[19,20,41,56]) 
        self.robotid.SetActiveDOFs(activedofs)

        # Current configuration of the robot is its initial configuration
        initconfig = self.robotid.GetActiveDOFValues()

        print "robot init config : "
        print initconfig

        # List of Robot Links
        links = self.robotid.GetLinks()
        
        # List of Wheel (Crank Links)
        cranklinks = self.crankid.GetLinks()
        
        # End Effector Transforms
        Tee = []
        for i in range(len(manips)):
            # Returns End Effector Transform in World Coordinates
            Tlink = manips[i].GetEndEffectorTransform()
            Tee.append(Tlink)

        
        # Get Transformation Matrix for the Wheel
        # Note that crank's links are not rotated
        # If you want use the wheel's end effector's transformation
        # matrix (which is 23 degrees tilted) then see
        # CTee matrix below.
        #
        # crank has two links: 
        # 0) pole - the blue cylinder in the model, and, 
        # 1) crank - the driving wheel itself.
        jointtm = cranklinks[0].GetTransform()
        # handles.append(misc.DrawAxes(env,matrix(jointtm),1))
        

        # We can also get the transformation matrix
        # with the following command as a string
        jointtm_str = cbirrtHubo.solve('GetJointTransform name crank jointind '+str(crankjointind))
        # And then we can convert the string to a 1x12 array
        jointtm_str = jointtm_str.replace(" ",",")
        jointtm_num = eval('['+jointtm_str+']')

        # In this script we will use jointtm.
        # jointtm_str and jointtm_num are given as example.
        
        # Crank Transform End Effector in World Coordinates
        # This is the transformation matrix of the end effector 
        # named "dummy" in the xml file.
        # Note that dummy is tilted 23 degress around its X-Axis
        CTee = crankmanip[0].GetEndEffectorTransform()

        tilt_angle_deg = acos(dot(linalg.inv(CTee),jointtm)[1,1])*180/pi
        tilt_angle_rad = acos(dot(linalg.inv(CTee),jointtm)[1,1])        

        # Center of Gravity Target
        cogtarg = [-0.05, 0.085, 0]
        #if self.ShowUserInterface :
            #cogtm = MakeTransform(rodrigues([0,0,0]),transpose(matrix(cogtarg)))
            #handles.append(misc.DrawAxes(self.env,cogtm,1))

        # polyscale: changes the scale of the support polygon
        # polytrans: shifts the support polygon around
        footlinknames = ' Body_RAR Body_LAR polyscale 0.5 0.5 0 polytrans -0.015 0 0 '
        #footlinknames = ' Body_RAR Body_LAR polyscale 0.7 0.5 0 polytrans -0.015 0 0 '
        #footlinknames = ' Body_RAR Body_LAR polyscale 1.0 1.0 0 polytrans 0 0 0 '

        # What is this?
        handrot = rodrigues([0,-pi/2,0])
        
        # Translation Offset from the wheel center for the hands
        transoffset = [0, 0.15, 0];
        
        # Figure out where to put the left hand on the wheel
        temp = dot(CTee, MakeTransform(rodrigues([-pi/2,0,0]),transpose(matrix([0,0,0]))))
        temp = dot(temp, MakeTransform(rodrigues([0,0,-pi/2]),transpose(matrix([0,0,0]))))
        
        # Left Hand Pose in World Coordinates
        T0_LH1 = dot(temp, MakeTransform(rodrigues([0,0,0]),transpose(matrix([0,0.15,0]))))

        # Uncomment if you want to see where T0_LH1 is 
        # handles.append(misc.DrawAxes(env,matrix(T0_LH1),1))

        # Figure out where to put the right hand on the wheel
        temp = dot(CTee, MakeTransform(rodrigues([-pi/2,0,0]),transpose(matrix([0,0,0]))))
        temp = dot(temp, MakeTransform(rodrigues([0,0,-pi/2]),transpose(matrix([0,0,0]))))
        # Right Hand Pose in World Coordinates
        T0_RH1 = dot(temp, MakeTransform(rodrigues([0,0,0]),transpose(matrix([0,-0.15,0]))))
        
        # Uncomment if you want to see where T0_RH1 is 
        # handles.append(misc.DrawAxes(env,matrix(T0_RH1),1))
        
        # Define Task Space Region strings
        # Left Hand
        TSRString1 = SerializeTSR(0,'NULL',T0_LH1,eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))

        # Right Hand
        TSRString2 = SerializeTSR(1,'NULL',T0_RH1,eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))
        
        # Left Foot
        TSRString3 = SerializeTSR(2,'NULL',Tee[2],eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))

        # Head
        # Grasp transform in Head coordinates
        Tw0_eH = eye(4) 
        # How much freedom do we want to give to the Head
        # [x,x,y,y,z,z,R,R,P,P,Y,Y]
        Bw0H = matrix([0,0,-0.1,0.1,-0.1,0.01,0,0,0,0,0,0])
        TSRString4 = SerializeTSR(4,'NULL',Tee[4],Tw0_eH,Bw0H)

        # We defined Task Space Regions. Now let's concatenate them.
        TSRChainStringGrasping = SerializeTSRChain(0,1,0,1,TSRString1,'NULL',[])+' '+SerializeTSRChain(0,1,0,1,TSRString2,'NULL',[])+' '+SerializeTSRChain(0,1,1,1,TSRString3,'NULL',[])+' '+SerializeTSRChain(0,1,1,1,TSRString4,'NULL',[])
            

        if( self.StopAtKeyStrokes ):
            print "Press Enter to plan initconfig --> startik"
            sys.stdin.readline()
        
        # Get a trajectory from initial configuration to grasp configuration
        with self.robotid:
            try:
                answer = cbirrtHubo.solve('RunCBiRRT psample 0.2 supportlinks 2 '+footlinknames+' smoothingitrs '+str(normalsmoothingitrs)+' '+TSRChainStringGrasping)
                print "RunCBiRRT answer: ",str(answer)
            except openrave_exception, e:
                print "Cannot send command RunCBiRRT: "
                print e
                return []

        try:
            os.rename("cmovetraj.txt","movetraj0.txt")
        except OSError, e:
            # No file cmovetraj
            print e
            return []

        # The following is the same as commented out try-except section
        traj = RaveCreateTrajectory(self.env,'').deserialize(open('movetraj0.txt','r').read())   
        self.robotid.GetController().SetPath(traj) 
        self.robotid.WaitForController(0)
        self.robotid.GetController().Reset(0) 
        # Reset(0) releases the controller, otherwise after calling 
        # SetPath the robot controller actively holds the trajectory's final joint values
        
        # Instead of 4 lines above, we could use the following block
        # to play the trajectory
        #
        # try:
        #     answer= cbirrtHubo.solve('traj movetraj0.txt');
        #     robotid.WaitForController(0)
        #     sys.stdin.readline()
        #     # debug
        #     print "traj call answer: ",str(answer)
        # except openrave_exception, e:
        #     print e
        
        
        # Get the current configuration of the robot
        # and assign it to startik (start of the wheel
        # rotation path).
        startik = self.robotid.GetActiveDOFValues()
        
        # Left Hand's index is less than the right hand.
        # Hence it is evaluated first by the CBiRRT Module.
        # That's why We need to define the right hand's 
        # transform relative to the wheel (ask Dmitry Berenson
        # about this for more information).
        temp1 = MakeTransform(rodrigues([-pi/2,0,0]),transpose(matrix([0,0,0])))
        temp2 = MakeTransform(rodrigues([0,0,-pi/2]),transpose(matrix([0,0,0])))
        # Rotate the wheel's transform to a suitable pose
        # for the Left Hand
        # T0_w0L stands for: 
        # left hand's transform on wheel in world coordinates
        T0_w0L = dot(dot(CTee,temp1),temp2)
        # This is what's happening: 
        #
        # Tw0L_0 = linalg.inv(T0_w0L)
        # Tw0L_LH1 = Tw0L_0*T0_LH1
        #
        # Left hand's transform in wheel's coordinates
        Tw0L_LH1 = dot(linalg.inv(T0_w0L),T0_LH1)
        # Transform of the left hand's end effector in wheel's coords.
        # Required by CBiRRT
        Tw0_eL = Tw0L_LH1
        # How much freedom do we want to give to the left hand
        Bw0L = matrix([0,0,0,0,0,0,0,pi,0,0,0,0])

        # Right Hand's transforms:
        T0_crankcrank = self.crankid.GetManipulators()[0].GetTransform()
        T0_w0R = MakeTransform(rodrigues([tilt_angle_rad,0,0]),transpose(matrix([0,0,0])))
        # End effector transform in wheel coordinates
        Tw0_eR = dot(linalg.inv(T0_crankcrank),T0_RH1)

        #handles.append(misc.DrawAxes(env,matrix(Tw0_eR),1))

        # How much freedom? (note: in frame of crank)
        Bw0R = matrix([0,0,0,0,0,0,0,0,0,0,0,0])

        # Head's transforms:
        T0_w0H =  Tee[4]
        Tw0_eH = eye(4);
        Bw0H = matrix([-0.05,0.05,-0.1,0.1,-100,100,-pi,pi,-pi,pi,-pi,pi])
        
        
        # Define Task Space Regions
        # Left Hand
        TSRString1 = SerializeTSR(0,'NULL',T0_w0L,Tw0_eL,Bw0L)
        # Right Hand
        TSRString2 = SerializeTSR(1,'crank crank',T0_w0R,Tw0_eR,Bw0R)
        # Left Foot
        TSRString3 = SerializeTSR(2,'NULL',Tee[2],eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))
        # Head
        TSRString4 = SerializeTSR(4,'NULL',T0_w0H,Tw0_eH,Bw0H)
        
        TSRChainStringFootOnly = SerializeTSRChain(0,0,1,1,TSRString3,'NULL',[])

        TSRChainStringFootandHead = TSRChainStringFootOnly+' '+SerializeTSRChain(0,0,1,1,TSRString4,'NULL',[])

        TSRChainStringTurning = SerializeTSRChain(0,0,1,1,TSRString1,'crank',matrix([crankjointind]))+' '+SerializeTSRChain(0,0,1,1,TSRString2,'NULL',[])+' '+TSRChainStringFootandHead
        
        # Calculate hand transforms after rotating the wheel (they will help us find the goalik):
        # How much do we want to rotate the wheel?
        crank_rot = pi/6.5
        
        # Which joint do we want the CBiRRT to mimic the TSR for?
        TSRChainMimicDOF = 1
        
        # Create the transform for the wheel that we would like to reach to
        Tcrank_rot = MakeTransform(rodrigues([crank_rot,0,0]),transpose(matrix([0,0,0])))
        
        # What is this?
        temp = MakeTransform(rodrigues([0,0,crank_rot]),transpose(matrix([0,0,0])))
        
        # Rotate the left hand's transform on the wheel in world transform "crank_rot" radians around it's Z-Axis
        T0_cranknew = dot(T0_w0L,Tcrank_rot)
        
        # Where will the left hand go after turning the wheel?
        # This is what's happening:
        #
        # Tcranknew_LH2 = dot(Tw0L_0,T0_LH1) --> Left hand in wheel's coordinate
        # T0_LH2 = dot(T0_cranknew,Tcranknew_LH2) --> Left hand rotated around wheel's origin
        T0_LH2 = dot(T0_cranknew,dot(linalg.inv(T0_w0L),T0_LH1))

        # Uncomment to see T0_LH2
        # handles.append(misc.DrawAxes(env,matrix(T0_LH2),1))
        
        # Where will the right hand go after turning the wheel?
        T0_RH2 = dot(T0_crankcrank,dot(temp,dot(linalg.inv(T0_crankcrank),T0_RH1)))

        # Uncomment to see T0_RH2
        # handles.append(misc.DrawAxes(env,matrix(T0_RH2),1))

        arg1 = str(cogtarg).strip("[]").replace(', ',' ')
        arg2 = trans_to_str(T0_LH2)
        arg3 = trans_to_str(T0_RH2)
        arg4 = trans_to_str(Tee[2])

        # print arg1
        # print arg2
        # print arg3
        # print arg4

        if( self.StopAtKeyStrokes ):
            print "Press Enter to find a goalIK"
            sys.stdin.readline()

        self.crankid.SetDOFValues([crank_rot],[crankjointind])

        goalik = cbirrtHubo.solve('DoGeneralIK exec supportlinks 2 '+footlinknames+' movecog '+arg1+' nummanips 3 maniptm 0 '+arg2+' maniptm 1 '+arg3+' maniptm 2 '+arg4)
        
        # print "goalIK"
        # print goalik

        self.robotid.SetActiveDOFValues(str2num(goalik))
        self.crankid.SetDOFValues([crank_rot],[crankjointind])
        
        if( self.StopAtKeyStrokes ):
            print "Press Enter to go to startik"
            sys.stdin.readline()

        # Get a trajectory from goalik to grasp configuration
        goaljoints = deepcopy(goalik)
        for i in range(TSRChainMimicDOF):
            goaljoints += ' 0'

        goaljoints = str2num(goaljoints)

        self.robotid.SetActiveDOFValues(startik)
        time.sleep(0.5)
        self.robotid.SetDOFValues(self.rhandclosevals,self.rhanddofs)
        self.robotid.SetDOFValues(self.lhandclosevals,self.lhanddofs)
        # Close hands to start "turning" the wheel
        self.crankid.SetDOFValues([0],[crankjointind])
        time.sleep(0.5)
        
        if( self.StopAtKeyStrokes ):
            print "Press Enter to plan startik --> goalik (DMITRY!!!)"
            sys.stdin.readline()

        print self.robotid.GetActiveDOFValues()
        print TSRChainStringTurning

        try:
            answer = cbirrtHubo.solve('RunCBiRRT supportlinks 2 '+footlinknames+' smoothingitrs '+str(fastsmoothingitrs)+' jointgoals '+str(len(goaljoints))+' '+Serialize1DMatrix(matrix(goaljoints))+' '+TSRChainStringTurning)
            print "RunCBiRRT answer: ",str(answer)
        except openrave_exception, e:
            print "Cannot send command RunCBiRRT: "
            print e
            return []
       

        try:
            os.rename("cmovetraj.txt","movetraj1.txt")
        except OSError, e:
            # No file cmovetraj
            print e
            return []

        # The following is the same as commented out try-except section
        # traj = RaveCreateTrajectory(env,'').deserialize(open('movetraj1.txt','r').read())   
        # robotid.GetController().SetPath(traj) 
        # crankid.GetController().SetPath(traj)
        # robotid.WaitForController(0)
        # crankid.WaitForController(0)
        # robotid.GetController().Reset(0)
        # crankid.GetController().Reset(0)
            
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
        self.robotid.SetDOFValues(self.rhandopenvals,self.rhanddofs)
        self.robotid.SetDOFValues(self.lhandopenvals,self.lhanddofs)
        self.robotid.SetActiveDOFValues(str2num(goalik))

        time.sleep(2)

        if( self.StopAtKeyStrokes ):
            print "Press Enter to plan goalik --> startik "
            sys.stdin.readline()

        

        goaljoints = startik

        print self.robotid.GetActiveDOFValues()
        print TSRChainStringFootandHead
        try:
            answer = cbirrtHubo.solve('RunCBiRRT supportlinks 2 '+footlinknames+' smoothingitrs '+str(normalsmoothingitrs)+' jointgoals '+str(len(goaljoints))+' '+Serialize1DMatrix(matrix(goaljoints))+' '+TSRChainStringFootandHead)
            print "RunCBiRRT answer: ",str(answer)
        except openrave_exception, e:
            print "Cannot send command RunCBiRRT: "
            print e
            return []

        try:
            os.rename("cmovetraj.txt","movetraj2.txt")
        except OSError, e:
            # No file cmovetraj
            print e
            return []

        try:
            answer= cbirrtHubo.solve('traj movetraj2.txt');
            self.robotid.WaitForController(0)
            # debug
            print "traj call answer: ",str(answer)
        except openrave_exception, e:
            print e
            return []
        
        self.robotid.GetController().Reset(0)
        #self.robotid.SetDOFValues(rhandclosevals,rhanddofs)
        #self.robotid.SetDOFValues(lhandclosevals,lhanddofs)

        self.robotid.SetActiveDOFValues(startik)
        time.sleep(1)

        if( self.StopAtKeyStrokes ):
            print "Press Enter to plan startik --> initconfig "
            sys.stdin.readline()

        goaljoints = initconfig
        print goaljoints
        try:
            answer = cbirrtHubo.solve('RunCBiRRT supportlinks 2 '+footlinknames+' smoothingitrs '+str(normalsmoothingitrs)+' jointgoals '+str(len(goaljoints))+' '+Serialize1DMatrix(matrix(goaljoints))+' '+TSRChainStringFootandHead)
            print "RunCBiRRT answer: ",str(answer)
        except openrave_exception, e:
            print "Cannot send command RunCBiRRT: "
            print e
            return []

        try:
            os.rename("cmovetraj.txt","movetraj3.txt")
        except OSError, e:
            # No file cmovetraj
            print e
            return []

        try:
            answer= cbirrtHubo.solve('traj movetraj3.txt');
            self.robotid.WaitForController(0)
            # debug
            print "traj call answer: ",str(answer)
        except openrave_exception, e:
            print e
            return []

        self.robotid.GetController().Reset(0)
        
        return self.Playback()


if __name__ == "__main__":
    planner = HuboPlusWheelTurning()
    planner.SetViewer(True)
    planner.SetStopKeyStrokes(False)
    planner.Run()
    planner.KillOpenrave()

    
