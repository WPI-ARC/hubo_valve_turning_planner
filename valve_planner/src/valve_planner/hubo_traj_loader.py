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

class TrajReader( BaseWheelTurning ):

    def __init__(self,
                 HuboModelPath = '../../../../drchubo/drchubo_v2/robots/drchubo_v2.robot.xml',
                 WheelModelPath = '../../models/driving_wheel_tiny.robot.xml' ):

        BaseWheelTurning.__init__( self, HuboModelPath, WheelModelPath )
        return

    def LoadAchFormat():
        

if __name__ == "__main__":

    handles = []

    files = ["movetraj0.txt","movetraj1.txt","movetraj7.txt","movetraj8.txt"]

    player = TrajReader()
    player.SetViewer(True)
    player.SetStopKeyStrokes(True)
    player.SetProblems()
    player.StartViewerAndSetValvePos( handles )
    #player.Playback()
    player.PlaybackFiles(files)
    player.KillOpenrave()
