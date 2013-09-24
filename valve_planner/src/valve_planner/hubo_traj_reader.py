#!/usr/bin/env python
# Bener Suay 2013 January
# benersuay@wpi.edu

#import easy to use xml parser called minidom:
from xml.dom.minidom import parseString

import rospy;
import roslib;
roslib.load_manifest('hubo_planner')

from math import *
import random
import time

from std_msgs.msg import String
from trajectory_msgs.msg import *
from hubo_robot_msgs.msg import *

import numpy

from copy import deepcopy
import sys

hubo_body_joint_names = { 
0 : 'HPY' ,  
1 : 'RHY' ,  
2 : 'LHY' ,
3 : 'RHR' ,
4 : 'LHR' ,
5 : 'RHP' , 
6 : 'LHP' ,  
7 : 'RKP' ,        
8 : 'LKP' ,  
9 : 'RAP' ,      
10 : 'LAP' ,        
11 : 'RAR' ,
12 : 'LAR' ,
13 : 'RSP' ,
14 : 'LSP' ,   
15 : 'RSR' ,        
16 : 'LSR' ,
17 : 'RSY' ,
18 : 'LSY' ,      
19 : 'REP' , 
20 : 'LEP' ,
21 : 'RWY' ,
22 : 'LWY' ,
23 : 'RWP' ,    
24 : 'LWP' ,
25 : 'HNR' ,
26 : 'HNP' 
}

hubo_joint_names = { 
0 : 'HPY' ,  
1 : 'RHY' ,  
2 : 'LHY' ,
3 : 'RHR' ,
4 : 'LHR' ,
5 : 'RHP' , 
6 : 'LHP' ,  
7 : 'RKP' ,        
8 : 'LKP' ,  
9 : 'RAP' ,      
10 : 'LAP' ,        
11 : 'RAR' ,
12 : 'LAR' ,
13 : 'RSP' ,
14 : 'LSP' ,   
15 : 'RSR' ,        
16 : 'LSR' ,
17 : 'RSY' ,
18 : 'LSY' ,      
19 : 'REP' , 
20 : 'LEP' ,
21 : 'RWY' ,
22 : 'LWY' ,
23 : 'RWP' ,    
24 : 'LWP' ,
25 : 'HNR' ,
26 : 'HNP' ,
27 : 'rightIndexKnuckle2' ,       
28 : 'rightIndexKnuckle3' ,             
29 : 'rightIndexKnuckle1' ,          
30 : 'rightMiddleKnuckle2' ,        
31 : 'rightMiddleKnuckle3' ,       
32 : 'rightMiddleKnuckle1' ,          
33 : 'rightRingKnuckle2' ,            
34 : 'rightRingKnuckle3' ,         
35 : 'rightRingKnuckle1' ,        
36 : 'rightPinkyKnuckle2' ,        
37 : 'rightPinkyKnuckle3' ,           
38 : 'rightPinkyKnuckle1' ,   
39 : 'rightThumbKnuckle2' ,      
40 : 'rightThumbKnuckle3' ,         
41 : 'rightThumbKnuckle1' ,    
42 : 'leftIndexKnuckle2' ,  
43 : 'leftIndexKnuckle3' ,  
44 : 'leftIndexKnuckle1' ,      
45 : 'leftMiddleKnuckle2' ,    
46 : 'leftMiddleKnuckle3' ,     
47 : 'leftMiddleKnuckle1' ,      
48 : 'leftRingKnuckle2' ,        
49 : 'leftRingKnuckle3' ,            
50 : 'leftRingKnuckle1' ,   
51 : 'leftPinkyKnuckle2' ,  
52 : 'leftPinkyKnuckle3' ,  
53 : 'leftPinkyKnuckle1' ,   
54 : 'leftThumbKnuckle2' ,  
55 : 'leftThumbKnuckle3' ,  
56 : 'leftThumbKnuckle1' }     


def read(fname,num_sample=-1):

    debug = False # this makes the script print out some of what it reads from file
    #open the xml file for reading:
    # print "Reading - playing: " + fname
    trajFile = open(fname,'r')

    data = trajFile.read()

    trajFile.close()

    dom = parseString(data) # Get the document object model

    # These dictionnaries will keep the information about which elements in the trajectory array correspond
    joint_pos_offsets = {}
    joint_vel_offsets = {}

    d_length = len(dom.getElementsByTagName('data')[0].firstChild.data.split(' '))
    # last element is always empty because of openrave syntax
    #d_length = d_length -1

    data_count = int(dom.getElementsByTagName('data')[0].getAttribute('count'))
    data = dom.getElementsByTagName('data')[0].firstChild.data.split(' ')
    stepsize = int(len(data)/data_count) # In "data" array we have joint angles, velocities and dTime. These values are repeated data_count times.

    if(debug):
        print "stepsize"
        print stepsize

    for t in range(len(dom.getElementsByTagName('group'))):

        # Get the value of the name attribute of this group tag
        name = dom.getElementsByTagName('group')[t].getAttribute('name')

        # It may be separated by space, make it a list
        name = name.split(' ')

        if( name[0] == 'deltatime'):
            deltatime_offset = int(dom.getElementsByTagName('group')[t].getAttribute('offset'))
            deltatime_dof = int(dom.getElementsByTagName('group')[t].getAttribute('dof'))
            if(debug):
                print "deltatime_offset"
                print deltatime_offset
                print "deltatime_dof"
                print deltatime_dof

        elif( name[0] == 'joint_velocities'):
            robot_name = name[1]
            # Check robot name, sometimes we create artificial robots like crank, valve, or driving wheel. 
            # In this script though we're only interested in the PR2 joint velocities.
            if(robot_name == 'rlhuboplus'):
                vel_offset = int(dom.getElementsByTagName('group')[t].getAttribute('offset'))
                vel_dof = int(dom.getElementsByTagName('group')[t].getAttribute('dof'))
                if(debug):
                    print "vel_offset"
                    print vel_offset
                    print "vel_dof"
                    print vel_dof

        elif( name[0] == 'joint_values'):
            robot_name = name[1]
            # Are these joint values defined for the pr2?
            # In this script we're only interested in the pr2 joint values
            if(robot_name == 'rlhuboplus'):
                jval_offset = int(dom.getElementsByTagName('group')[t].getAttribute('offset'))
                jval_dof = int(dom.getElementsByTagName('group')[t].getAttribute('dof'))

                if(debug):
                    print "jval_offset"
                    print jval_offset
                    print "jval_dof"
                    print jval_dof

                # The following block populates the two unpopulated dictionnaries we created earlier
                # E.g. {'15': '15', '21': '21', '17': '17', '16': '16', '19': '19', '173': '15', '100': '21', '178': '20', '20': '20', '99': '20', '98': '19', '179': '21', '18': '18', '177': '19', '176': '18', '175': '17', '174': '16', '95': '16', '94': '15', '97': '18', '96': '17'}
                # In the example dictionnary, 15th, 173th, and 94th elements of the "data" array belong to the value of joint 15.
                # We dynamically create / populate this because we only want to get the value of a joint
                # if and only if it's: a) one of the joints we're interested and b) it's in the data

                if(False):
                    for c in range(2):
                        print data[(c*stepsize)+jval_offset:(c*stepsize)+jval_offset+jval_dof]

                for j in range(2,len(name)):
                    if(int(name[j]) in hubo_joint_names.keys()):

                        for c in range(data_count):
                            # ind: Where, in that big array of data, will be the angle value of joint " [j]"?
                            # Note: the reasone we're adding (j-2) is because jval_offset gives us the index of the first joint's value.
                            # index should be found as follows:
                            # stepsize=(len(data)/data_count) : how long is one block of data (vals, vels, dTime, altogether)?
                            # jval_offset : offset from the beginning of one block of data for the first joint's ang value
                            # j-2 : gives us the offset for the specific joint angle we're interested
                            # 
                            # In short, we are looking for:
                            #
                            # ind = (c*stepsize)+(jval_offset)+(j-2)
                            ind = (c*stepsize)+(jval_offset)+(j-2)                            
                            joint_pos_offsets[ind] = int(name[j])

                        for c in range(data_count):
                            
                            ind = (c*stepsize)+(vel_offset)+(j-2)                            
                            joint_vel_offsets[ind] = int(name[j])
                            

    if(False):
        print joint_pos_offsets

    # One of the following variables may not exist. Let's try and return error if we fail.
    try:
        # Deltaoffset is always the last data point, which gives us the length of one single chunk of data.
        # We know that there are datacount number of chunks.

        if((num_sample < 1) or (num_sample > int(data_count))): # play full trajectory
            start_from = 0
            end_at = data_count
            skip_this_many_points = 0
        elif(num_sample == 1): # play the last point
            start_from = data_count - 1
            end_at = data_count
            skip_this_many_points = 0
        elif(num_sample == 2): # play the first and the last points
            start_from = 0
            end_at = data_count
            skip_this_many_points = data_count - 1
        else:
            start_from = data_count - (num_sample*int(floor(int(data_count)/num_sample)))
            end_at = data_count
            skip_this_many_points = int(floor(int(data_count)/num_sample))
            
        increment_by = 1 + skip_this_many_points

        # print "num_sample"
        # print num_sample
        # print "start_from"
        # print start_from
        # print "end_at"
        # print end_at
        # print "skip_this_many_points"
        # print skip_this_many_points
        # print "increment_by"
        # print increment_by

        hubo_traj = JointTrajectory()

        # Set the time stamp to now
        hubo_traj.header.stamp = rospy.Time.now()

        # Set the joint names
        hubo_joint_names_list = []
        for key, value in hubo_body_joint_names.iteritems():
            hubo_joint_names_list.append(value)

        hubo_traj.joint_names = hubo_joint_names_list
        
        tc = 0 # time_counter

        deltaT = 0.04 #float(6.0/data_count)

        for c in range(start_from,end_at,increment_by):

            current_point = JointTrajectoryPoint()
            #l_current_point.time_from_start = rospy.Duration(float(data[c*deltatime_offset]))
            #current_point.time_from_start = rospy.Duration(2.0+tc*deltaT))
            current_point.time_from_start = rospy.Duration(tc*deltaT)

            tc = tc + 1
            # Get the indices for this subset of data
            # i = c*(deltatime_offset+deltatime_dof)
            i = c*stepsize

            i_vel = i + vel_offset
            f_vel = i_vel + vel_dof

            i_jval = i + jval_offset
            f_jval = i_jval + jval_dof

            # Empty position buffers
            p_buffer = []
        
            # Fill in position buffers
            joint_id = 0
            for p in range( i_jval, f_jval ):
                if( p in joint_pos_offsets.keys() ):
                    if( joint_id in hubo_body_joint_names.keys() ):
                        p_buffer.append(float(data[p]))
                    joint_id += 1

            if(debug):
                print p_buffer

            # Empty velocity buffers
            v_buffer = []

            # Fill in velocity buffers
            v_buffer.append( (p_buffer[0]-p_buffer[1])/deltaT )
            for i in range( 1 , len(p_buffer)-1 ):
                v_buffer.append( (p_buffer[i+1]-p_buffer[i-1])/deltaT )
            v_buffer.append( (p_buffer[len(p_buffer)-1]-p_buffer[len(p_buffer)-2])/deltaT )
            
            # TODO : We do not use the velocity profile of
            # COMPS, we simply compute them by finit differenciation    
            #joint_id = 0
            #for v in range( i_vel, f_vel ):
            #    if( v in joint_vel_offsets.keys() ):
            #        if( joint_id in hubo_body_joint_names.keys() ):
            #            v_buffer.append(float(data[v]))
            #        joint_id += 1

            if(debug):
                print v_buffer

            # Empty accelerations buffers
            a_buffer = []

            # Fill in accelerations buffers
            a_factor = 10;
            a_buffer.append( a_factor*(v_buffer[0]-v_buffer[1])/deltaT )
            for i in range( 1 , len(v_buffer)-1 ):
                a_buffer.append( a_factor*(v_buffer[i+1]-v_buffer[i-1])/deltaT )
            a_buffer.append( a_factor*(v_buffer[len(v_buffer)-1]-v_buffer[len(v_buffer)-2])/deltaT )
            
            if(debug):
                print a_buffer

            # Append trajectory point
            current_point.positions = deepcopy(p_buffer)
            current_point.velocities = deepcopy(v_buffer)
            current_point.accelerations = deepcopy(a_buffer)
            #print "len(current_point.positions) : " + str(len(current_point.positions))
            #print "len(current_point.velocities) : " + str(len(current_point.velocities))
            #print "len(current_point.accelerations) : " + str(len(current_point.accelerations))

            hubo_traj.points.append(current_point)
            
        print len(hubo_traj.points)
        print hubo_traj.points[len(hubo_traj.points)-1].time_from_start.to_sec()
        # Set hubo trajectory 
        #hubo_goal = JointTrajectoryGoal()
        #hubo_goal.trajectory = hubo_traj
        return hubo_traj
                    
    except NameError, e:
        print "Error - one of the necessary variables is not found:"
        print e

    # print "Finished playing trajectory file: " + fname
    return 1


def main():
    # Initialize ROS node

    return 1

if __name__ == "__main__":
    main()
