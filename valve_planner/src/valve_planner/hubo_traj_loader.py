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

class TrajectoryReader( BaseWheelTurning ):

    def __init__(self,
                 frequency = 25,
                 joint_mapping = {},
                 HuboModelPath = '../../../../drchubo/drchubo_v2/robots/drchubo_v2.robot.xml',
                 WheelModelPath = '../../models/driving_wheel_tiny.robot.xml' ):

        BaseWheelTurning.__init__( self, HuboModelPath, WheelModelPath )

        self.joint_mapping = joint_mapping
        self.hubo_traj = None
        self.dt = 1 / float(frequency) # 20 Hz (0.05)

        self.execute_in_loop = True

        print "self.dt : " + str( self.dt )

        # Ach trajectory mapping for DRCHubo!!! It differs from the internal ros mapping
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
    
    # Gets the joint mapping supposing each joint is one dof
    def SetJointMapping(self):
        
        self.joint_mapping = {}
        for j in self.robotid.GetJoints():
            self.joint_mapping[ j.GetName() ] = j.GetDOFIndex()

        return
    
    # Loads trajectory from file
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

            for idx in range( len(line) ):

                joint_name = self.hubo_ach_traj_joint_names[idx]

                try:
                    i = self.joint_mapping[ joint_name ]
                    #print i
                except KeyError:
                    i = None
                    if joint_name == "RF1" or joint_name == "RF2" or joint_name == "LF1" :
                        print joint_name + " , value : " + str( line[idx] )
                        
                if i is not None:
                    q[i] = float(line[idx])
                else:
                    continue

            #print q
            self.hubo_traj.Insert(self.hubo_traj.GetNumWaypoints(),q)
        
        self.execute()
        return True

    # Plays the trajectory in openrave
    def execute(self):

        print "Play Back Trajectory!!!!"

        while True :

            for i in range(self.hubo_traj.GetNumWaypoints()):
                # get the waypoint values, this holds velocites, time stamps, etc
                data = self.hubo_traj.GetWaypoint(i)
                # extract the robot joint values only
                with self.env: # have to lock environment since accessing robot
                    q = self.hubo_traj.GetConfigurationSpecification().ExtractJointValues(data,self.robotid,self.robotid.GetActiveDOFIndices())
                    self.robotid.SetDOFValues(q)
                time.sleep( self.dt )

            if not self.execute_in_loop :
                break

        return

def main():

    ortraj = False
    achtraj = False
    filename = None
    
    if(len(sys.argv) >= 2):
        for index in range(1,len(sys.argv)):
            if(sys.argv[index] == "-f" and index+1<len(sys.argv)):
                filename = str(sys.argv[index+1])
            elif sys.argv[index] == "-or" :
                ortraj = True
            elif sys.argv[index] == "-ach":
                achtraj = True

    if not ortraj and not achtraj :
        print "specify the format!!!"
        return

    handles = []

    #files = ["movetraj0.txt","movetraj1.txt","movetraj7.txt","movetraj8.txt"]

    robot_name = "drchubo"

    player = TrajectoryReader()
    player.SetViewer(True)
    player.SetStopKeyStrokes(True)
    player.SetProblems()
    player.StartViewerAndSetValvePos( handles )
    #player.Playback()
    #player.PlaybackFiles(files)
    player.SetJointMapping()

    if filename is None :
        player.LoadAchfile("../../trajectories/open_hands.traj")
    else :
        player.LoadAchfile( filename )
        
    player.KillOpenrave()

    return

if __name__ == "__main__":
    main()
