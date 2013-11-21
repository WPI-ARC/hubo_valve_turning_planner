# Ben Suay, RAIL
# July 2013
# Worcester Polytechnic Institute
#

import openravepy
import roslib
roslib.load_manifest("wpi_planning_utilities")
from copy import deepcopy
from numpy import *
from wpi_planning_utilities.str2num import *

def traj2ach(env,robot,traj,fname,robotJointValsOffset,robotJointVelsOffset,deltatimeOffset):

    myAchTraj=openravepy.RaveCreateTrajectory(robot.GetEnv(),'')
    config=deepcopy(traj.GetConfigurationSpecification())
    robotJointValsDOF = traj.GetConfigurationSpecification().GetGroupFromName("joint_values "+robot.GetName()).dof
    robotJointVelsDOF = traj.GetConfigurationSpecification().GetGroupFromName("joint_velocities "+robot.GetName()).dof
    
    #config.AddDeltaTimeGroup()
    myAchTraj.Init(config)

    trajLength = traj.GetNumWaypoints()
    numJoints = len(robot.GetJoints())
    
    freq = 1.0/25.0
    for i in range(trajLength):
        wp = traj.GetWaypoint(i)
        awp = deepcopy(wp)
        # print "joint values"
        q = wp[robotJointValsOffset:(robotJointValsOffset+robotJointValsDOF)]
        #print "Rave2RealHubo says"
        #print robotJointValsOffset
        #print (robotJointValsOffset+numJoints)
        
        # print q
        # print "joint velocities"
        qdot = wp[robotJointVelsOffset:(robotJointVelsOffset+robotJointVelsDOF)]
        # print qdot
        # print "deltatime"
        dt = wp[deltatimeOffset]
        # print dt
        if(dt != 0.0):
            awp[deltatimeOffset] = 0.5

        myAchTraj.Insert(i,awp)
        
        # pose['RHY'] = q[1]
        # pose['RHR'] = q[2]
        # pose['RHP'] = q[3]
        # pose['RKN'] = q[4]
        # pose['RAP'] = q[5]
        # pose['RAR'] = q[6]
        # pose['LHY'] = q[26]
        # pose['LHR'] = q[27]
        # pose['LHP'] = q[28]
        # pose['LKN'] = q[29]
        # pose['LAP'] = q[30]
        # pose['LAR'] = q[31]
        # pose['RSP'] = q[7]
        # pose['RSR'] = q[8]
        # pose['RSY'] = q[9]
        # pose['REB'] = q[10]
        # pose['RWY'] = q[11]
        # pose['RWR'] = q[13]
        # pose['RWP'] = q[12]
        # pose['LSP'] = q[19]
        # pose['LSR'] = q[20]
        # pose['LSY'] = q[21]
        # pose['LEB'] = q[22]
        # pose['LWY'] = q[23]
        # pose['LWR'] = q[25]
        # pose['LWP'] = q[24]
        # pose['NKY'] = q[14]
        # pose['NK1'] = q[15]
        # pose['NK2'] = q[16]
        # pose['WST'] = q[0]
        # pose['RF1'] = q[18]
        # pose['RF2'] = q[17]
        # pose['RF3'] = 0
        # pose['RF4'] = 0
        # pose['RF5'] = 0
        # pose['LF1'] = q[32]
        # pose['LF2'] = 0
        # pose['LF3'] = 0
        # pose['LF4'] = 0
        # pose['LF5'] = 0
        
    # openravepy.planningutils.RetimeTrajectory(myAchTraj,hastimestamps=True) # inputs: trajectory, hastimestamps.

    openravepy.planningutils.RetimeActiveDOFTrajectory(myAchTraj,robot,hastimestamps=False,maxvelmult=1,maxaccelmult=1,plannername='ParabolicTrajectoryRetimer')

    achTrajLength = myAchTraj.GetNumWaypoints()

    retimedRobotJointValsOffset = myAchTraj.GetConfigurationSpecification().GetGroupFromName("joint_values "+robot.GetName()).offset
    retimedRobotJointValsDOF = myAchTraj.GetConfigurationSpecification().GetGroupFromName("joint_values "+robot.GetName()).dof
    
    activedofs = str2num(myAchTraj.GetConfigurationSpecification().GetGroupFromName("joint_values "+robot.GetName()).name[len("joint_values "+robot.GetName()):]).astype(int)
    fRaveRetimed = open(fname+'_retimed.txt','w')
    fRaveRetimed.write(myAchTraj.serialize(0))
    fRaveRetimed.close()

    f = open(fname+'.traj','w')

    for i in range(achTrajLength):
        atwp = myAchTraj.GetWaypoint(i)
        allq = zeros(numJoints)
        aq = atwp[retimedRobotJointValsOffset:(retimedRobotJointValsOffset+retimedRobotJointValsDOF)]
        for aIdx, a in enumerate(activedofs):
            allq[a] = aq[aIdx]
            
        # No Fingers
        myAchQ = [allq[1], allq[2], allq[3], allq[4], allq[5], allq[6], allq[26], allq[27], allq[28], allq[29], allq[30], allq[31], allq[7], allq[8], allq[9], allq[10], allq[11], allq[13], allq[12], allq[19], allq[20], allq[21], allq[22], allq[23], allq[25], allq[24], allq[14], allq[15], allq[16], allq[0], 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        # print myAchQ
        f.write(' '.join([str(x) for x in myAchQ])+'\n')

    f.close()

def sameDamnThing(robot,freq,duration,hand,fname,fingerVal):
    lfv = 0.0
    rfv = 0.0
    if(hand == "LH"):
        lfv = fingerVal
    elif(hand == "RH"):
        rfv = fingerVal
    elif(hand == "BH"):
        lfv = fingerVal
        rfv = fingerVal

    # duration in seconds (How long do you want to wait until the robot closes its hands)
    # freq in hertz
    reps = duration*freq
    q = robot.GetDOFValues(range(len(robot.GetJoints())))
    f = open(fname+'.traj','w')
    for i in range(int(reps)):

        # No Fingers
        myAchQ = [q[1], q[2], q[3], q[4], q[5], q[6], q[26], q[27], q[28], q[29], q[30], q[31], q[7], q[8], q[9], q[10], q[11], q[13], q[12], q[19], q[20], q[21], q[22], q[23], q[25], q[24], q[14], q[15], q[16], q[0], rfv, rfv, 0, 0, 0, lfv, 0, 0, 0, 0]

        f.write(' '.join([str(x) for x in myAchQ])+'\n')
    f.close

def openHandsHere(robot,freq,duration,hand,fname):
    sameDamnThing(robot,freq,duration,hand,fname,-0.1)
    
def relaxHandsHere(robot,freq,duration,hand,fname):
    sameDamnThing(robot,freq,duration,hand,fname,0.0)
    
def closeHandsHere(robot,freq,duration,hand,fname):
    sameDamnThing(robot,freq,duration,hand,fname,0.1)
