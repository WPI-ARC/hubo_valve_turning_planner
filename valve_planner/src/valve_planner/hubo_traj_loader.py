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

# ach_map["RKP"] = 3;  // RKN in ACH
# ach_map["LKP"] = 9;  // LKN in ACH
# ach_map["REP"] = 15; // REB in ACH
# ach_map["LEP"] = 22; // LEB in ACH
# ach_map["TSY"] = 29; // WST in ACH

drchubo_joint_mapping = { 
'LSP' : 0,
'LSR' : 1,
'LSY' : 2,     
'LEP' : 3,       
'LWY' : 4,  
'LWP' : 5,    
'LWR' : 6,
'LF11' : 7,
'LF12' : 8,
'LF13' : 9,
'TSY' : 10,
'LHY' : 11,
'LHR' : 12,
'LHP' : 13,
'LKP' : 14,
'LAP' : 15,
'LAR' : 16,
'NKY' : 17,
'NK1' : 18,
'NK2' : 19,
'LF21' : 20,
'LF22' : 21,
'LF23' : 22,
'LF31' : 23,
'LF32' : 24,
'LF33' : 25,
'RSP' : 26,
'RSR' : 27,
'RSY' : 28,
'REP' : 29,
'RWY' : 30,
'RWP' : 31,
'RWR' : 32,
'RF11' : 33,
'RF12' : 34,
'RF13' : 35,
'RHY' : 36,
'RHR' : 37,
'RHP' : 38,
'RKP' : 39, 
'RAP' : 40,
'RAR' : 41,
'RF21' : 42,
'RF22' : 43, 
'RF23' : 44,  
'RF31' : 45,
'RF32' : 46,
'RF33' : 47,
'RF41' : 48, 
'RF42' : 49,
'RF43' : 50 }


class TrajectoryReader( BaseWheelTurning ):

    def __init__(self,
                 joint_mapping,
                 HuboModelPath = '../../../../drchubo/drchubo_v2/robots/drchubo_v2.robot.xml',
                 WheelModelPath = '../../models/driving_wheel_tiny.robot.xml' ):

        BaseWheelTurning.__init__( self, HuboModelPath, WheelModelPath )

        frequency = 25

        self.joint_names = joint_names
        self.joint_mapping = joint_mapping
        self.hubo_traj = None
        self.dt = 1 / float(frequency) # 20 Hz (0.05)

        print "self.dt : " + str( self.dt )

        # Ach trajectory mapping. It differs from the internal ros mapping
        # which is defined as a global parameter (joints) in the parameter server   
        self.hubo_ach_traj_joint_names = {  0 : 'RHY' ,  1 : 'RHR' ,  2 : 'RHP' ,  3 : 'RKP' ,  4 : 'RAP' ,  
                                            5 : 'RAR' ,  6 : 'LHY' ,  7 : 'LHR' ,  8 : 'LHP' ,  9 : 'LKP' , 
                                           10 : 'LAP' , 11 : 'LAR' , 12 : 'RSP' , 13 : 'RSR' , 14 : 'RSY' , 
                                           15 : 'REP' , 16 : 'RWY' , 17 : 'RWR' , 18 : 'RWP' , 19 : 'LSP' , 
                                           20 : 'LSR' , 21 : 'LSY' , 22 : 'LEP' , 23 : 'LWY' , 24 : 'LWR' , 
                                           25 : 'LWP' , 26 : 'NKY' , 27 : 'NK1' , 28 : 'NK2' , 29 : 'TSY' ,
                                           30 : 'RF1' , 31 : 'RF2' , 32 : 'RF3' , 33 : 'RF4' , 34 : 'RF5' ,  
                                           35 : 'LF1' , 36 : 'LF2' , 37 : 'LF3' , 38 : 'LF4' , 39 : 'LF5' }
        return
    
    # Loads trajectory from file and stores it in a ROS message type
    # returns False if the trajectory had not be loaded properly
    def LoadAchfile(self,fname):

        print "parsing file"

        # open the file and reads the array
        f = open(fname,'r')
        array = []
        for line in f:
            array.append([float(x) for x in line.split()])
        f.close()

        if( len(array) == 0 ):
            print "Warning : empty trajectory"
            return False

        print "filing message"

        self.hubo_traj = RaveCreateTrajectory(self.robotid.GetEnv(),'')
        self.hubo_traj.Init( self.robotid.GetActiveConfigurationSpecification() )

        for line in array: # reads all lines in the file

            # ---------------------------------------
            # Fills position buffer
            q = [0.0] * len(self.joint_mapping)

            for p in range( len(line) ):

                try:
                    i = self.joint_mapping[ self.hubo_ach_traj_joint_names[p] ]
                    #print i
                except KeyError:
                    i = None
                if i is not None:
                    q[i] = float(line[p])
                else:
                    continue

            #print q
            self.hubo_traj.Insert(self.hubo_traj.GetNumWaypoints(),q)
        
        self.execute()
        return True

    # Sends the trajectory to the actionLib
    # returns when execution is finished
    def execute(self):

        print "Play Back Trajectory!!!!"

        for i in range(self.hubo_traj.GetNumWaypoints()):
            # get the waypoint values, this holds velocites, time stamps, etc
            data=self.hubo_traj.GetWaypoint(i)
            # extract the robot joint values only
            with self.env: # have to lock environment since accessing robot
                q = self.hubo_traj.GetConfigurationSpecification().ExtractJointValues(data,self.robotid,self.robotid.GetActiveDOFIndices())
                self.robotid.SetDOFValues(q)
            time.sleep(0.1)

        return

if __name__ == "__main__":

    handles = []

    #files = ["movetraj0.txt","movetraj1.txt","movetraj7.txt","movetraj8.txt"]

    robot_name = "drchubo"
    joint_names = []
    joint_mapping = []

    player = TrajectoryReader( drchubo_joint_mapping )
    player.SetViewer(True)
    player.SetStopKeyStrokes(True)
    player.SetProblems()
    player.StartViewerAndSetValvePos( handles )
    #player.Playback()
    #player.PlaybackFiles(files)
    player.LoadAchfile("../../trajectories/open_hands.traj")
    player.KillOpenrave()

