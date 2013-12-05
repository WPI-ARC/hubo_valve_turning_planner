#!/usr/bin/env python
# Jim Mainprice, ARC
# September 2013
# Worcester Polytechnic Institute
#
# http://openrave.org/docs/latest_stable/command_line_tools/
# openrave-robot.py /your/path/to/your.robot.xml --info=joints
# On that page you can find more examples on how to use openrave-robot.py.

from openravepy import *
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
from base_wheel_turning import *
import rave2realhubo

class PlanStraightOneConfig( BaseWheelTurning ):

    def __init__(self,
                 frequency = 25,
                 joint_mapping = {},
                 HuboModelPath = '../../../../drchubo/drchubo_v2/robots/drchubo_v2.robot.xml',
                 WheelModelPath = '../../models/driving_wheel_tiny.robot.xml' ):

        BaseWheelTurning.__init__( self, HuboModelPath, WheelModelPath )

        self.robotManips = self.robotid.GetManipulators()
        self.TSRChainString = None
        self.SetTSRs()        

    def SetToHomeIk(self):
        # This function sets the robot's joints to values
        # close to zero as much as possible, while
        # taking the joint limits into consideration
        for jIdx, j in enumerate(self.robotid.GetJoints()):
            lims = j.GetLimits()
            if(lims[1] > 0.0):
                self.robotid.SetDOFValues([0.001],[jIdx])
            else:
                self.robotid.SetDOFValues([-0.001],[jIdx])

        q_tmp = self.robotid.GetDOFValues()
        self.robotid.GetController().SetDesired(q_tmp)
        self.robotid.GetController().Reset(0)

    def Export2AchTraj(self,trajfilename):

        traj = RaveCreateTrajectory(self.env,'').deserialize(open(trajfilename+'.txt','r').read())
        cs = traj.GetConfigurationSpecification()
        drchuboJointValsGroup = cs.GetGroupFromName("joint_values "+self.robotid.GetName())
        drchuboJointVelocitiesGroup = cs.GetGroupFromName("joint_velocities "+self.robotid.GetName())
        deltatimeGroup = cs.GetGroupFromName("deltatime")
        rave2realhubo.traj2ach(self.env,self.robotid,traj,trajfilename,drchuboJointValsGroup.offset,drchuboJointVelocitiesGroup.offset,deltatimeGroup.offset)

    def SetTSRs(self):

        # Define Task Space Region strings
        TSRStringH = SerializeTSR(4,'NULL',self.robotManips[4].GetEndEffectorTransform(),eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))
        self.TSRChainString = SerializeTSRChain(0,0,1,1,TSRStringH,'NULL',[])
        self.robotid.GetController().Reset(0)

    def GenerateAchTrajToConfig( self, ith , to = True):

        self.SetToHomeIk()
        self.SetTSRs()

        self.filename = ""

        if ith == 0 :

            if to :
                self.filename = self.default_trajectory_dir + "config_0" + "_to_"
            else :
                self.filename = self.default_trajectory_dir + "config_0" + "_back_"

            q = [ 0.001, 0.25, 0.001, -0.15, 0.001, 0.001, 0.001, \
              0.001, 0.001, 0.001, 0.001, 0.00100038, 0.00087495, \
              -0.31279762,  0.47264213, -0.15684743,  0.00112504, 0.001, 0.001, 0.001, \
              0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, \
              -0.25, 0.001, -0.15, 0.001, 0.001, 0.001, 0.001, \
              0.001, 0.001, 0.00099944, 0.00118748, -0.15597225,  0.47090409, \
              -0.31193323,  0.00081251, 0.001, 0.001, 0.001, 0.001, 0.001, \
              0.001, 0.001, 0.001, 0.001 ]

        elif ith == 1 :

            if to :
                self.filename = self.default_trajectory_dir + "config_1" + "_to_"
            else :
                self.filename = self.default_trajectory_dir + "config_1" + "_back_"

            q = [ 0.001, 0.25, 0.001, -0.15,  0.001, 0.001, 0.001, \
             0.001,       0.001,       0.001,       0.001,      0.00053119,  0.15602586, \
             -0.19111123,  0.38474823, -0.19060079, -0.15402593,  0.001,      0.001,       0.001, \
              0.001,       0.001,       0.001,       0.001,       0.001,       0.001,       0.001, \
             -0.25,        0.001,      -0.15,        0.001,       0.001,       0.001,       0.001, \
             0.001,       0.001,       0.00146863, -0.15401224, -0.19207804,  0.38791559, \
            -0.19280225,  0.15601234,  0.001,       0.001,       0.001,       0.001,       0.001, \
             0.001,       0.001,       0.001,       0.001     ]

        elif ith == 2:

            if to :
                self.filename = self.default_trajectory_dir + "config_2" + "_to_"
            else :
                self.filename = self.default_trajectory_dir + "config_2" + "_back_"

            q = [  1.00000000e-03,   2.50000000e-01,   1.00000000e-03,  -1.50000000e-01, \
               1.00000000e-03,   1.00000000e-03,   1.00000000e-03,   1.00000000e-03, \
               1.00000000e-03,   1.00000000e-03,   1.00000000e-03,   4.91695914e-04, \
               1.68886803e-01,  -5.10040922e-01,   8.56015301e-01,  -3.42932455e-01, \
              -1.66887045e-01,   1.00000000e-03,   1.00000000e-03,   1.00000000e-03, \
               1.00000000e-03,   1.00000000e-03,   1.00000000e-03,   1.00000000e-03, \
               1.00000000e-03,   1.00000000e-03,   1.00000000e-03,  -2.50000000e-01, \
               1.00000000e-03,  -1.50000000e-01,   1.00000000e-03,   1.00000000e-03, \
               1.00000000e-03,   1.00000000e-03,   1.00000000e-03,   1.00000000e-03, \
               1.50742512e-03,  -1.66627832e-01,  -3.42821700e-01,   8.56406691e-01, \
               -5.10543695e-01,  1.68628070e-01,   1.00000000e-03,   1.00000000e-03, \
               1.00000000e-03,   1.00000000e-03,   1.00000000e-03,   1.00000000e-03, \
               1.00000000e-03,   1.00000000e-03,   1.00000000e-03]

        elif ith == 3:
            
            if to :
                self.filename = self.default_trajectory_dir + "config_rainbow_start" + "_to_"
            else :
                self.filename = self.default_trajectory_dir + "config_rainbow_start" + "_back_"

            q = [  4.35205000e-01,   1.70997000e-01,  -1.72800000e-03,  -6.97599000e-01, \
                   1.10000000e-03,   1.88000000e-04,  -2.22044605e-16,   0.00000000e+00, \
                   0.00000000e+00,   0.00000000e+00,   5.70000000e-05,   3.10000000e-05, \
                  -1.20000000e-05,  -3.83801000e-01,   7.67872000e-01,  -3.83859000e-01, \
                   3.00000000e-05,   0.00000000e+00,   0.00000000e+00,   0.00000000e+00, \
                   0.00000000e+00,   0.00000000e+00,   0.00000000e+00,  -1.17756934e-16, \
                   0.00000000e+00,   0.00000000e+00,   4.36198000e-01,  -1.73856000e-01, \
                   1.61800000e-03,  -6.97237000e-01,  -2.21500000e-03,   6.44000000e-04, \
                   0.00000000e+00,   0.00000000e+00,   0.00000000e+00,   0.00000000e+00, \
                  -5.70000000e-05,   9.00000000e-06,  -3.83801000e-01,   7.67846000e-01, \
                  -3.83877000e-01,  -2.00000000e-05,   0.00000000e+00,   0.00000000e+00, \
                   0.00000000e+00,   0.00000000e+00,   0.00000000e+00,   0.00000000e+00, \
                   0.00000000e+00,   0.00000000e+00,   0.00000000e+00]
        else:
            return -1

        psample = 0
        smoothingitrs = 200

        if to :
            goaljoints = q
        else :
            self.SetToHomeIk()
            goaljoints = self.robotid.GetDOFValues()
            self.robotid.SetDOFValues( q )

#        self.robotid.SetDOFValues( q )
#        sys.stdin.readline()
#        self.SetToHomeIk()
#        sys.stdin.readline()

        try:
            cmdStr = 'RunCBiRRT timelimit 10 '

            if(psample != None):
                cmdStr += 'psample '+str(psample)+' '

            cmdStr += 'supportlinks 2 '+self.footlinknames+' smoothingitrs '+str(smoothingitrs)+' jointgoals '+str(len(goaljoints))+' '+Serialize1DMatrix(matrix(goaljoints))+' '+self.TSRChainString

            answer = self.probs_cbirrt.SendCommand(cmdStr)

            print "RunCBiRRT answer: ",str(answer)
            if(answer[0] != '1'):
                return -1
        except openrave_exception, e:
            print "Cannot send command RunCBiRRT: "
            print e
            return -1

        os.rename( "cmovetraj.txt", self.filename + ".txt" )

        self.Export2AchTraj( self.filename )

        return 0

    def PlayInOpenRAVE(self):
        
        try:
           answer = self.probs_cbirrt.SendCommand('traj '+self.filename+".txt")
           print "traj call answer: ", answer
           self.robotid.WaitForController(0)

        except openrave_exception, e:
            print e
            return [False, "OS exception in PlayTrajectory."]

        self.robotid.GetController().Reset(0)
        time.sleep(2)
        return [True, ""]

       
def main():

    handles = []

    robot_name = "drchubo"

    planner = PlanStraightOneConfig()
    planner.SetViewer(True)
    planner.SetStopKeyStrokes(True)
    planner.SetProblems()
    planner.StartViewerAndSetValvePos( handles )

    for to in [True, False] :
        for i in [0,1,2,3] :
            if planner.GenerateAchTrajToConfig(i,to) == 0 :
                print "SUCCESS"
                planner.PlayInOpenRAVE()
                
            else:
                print "FAIL"
                break
    planner.KillOpenrave()

    return

if __name__ == "__main__":
    main()
