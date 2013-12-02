#!/usr/bin/env python

# Bener Suay
# December, 2013
# benersuay@wpi.edu
# 
# Testing script that will log
# results of different planning
# attempts for valve turning.

# ROS imports
import rospy
from visualization_msgs.msg import Marker
from valve_planner_msgs.msg import *
from valve_planner_msgs.srv import *
from geometry_msgs.msg import *

# Other imports
import random
import sys

distFromRobotTorso = 0.5
clearanceFromPipe = 0.05
pipeDiam = 0.1
valveHeight = 40*0.0254

def valve_factory(marker, markerid, diam, yLoc, Type):
    marker.header.frame_id = "leftFoot"
    marker.header.stamp = rospy.get_rostime()
    marker.id = markerid
    marker.type = 3 
    marker.action = 0
    marker.pose.position.x = distFromRobotTorso - clearanceFromPipe - pipeDiam*0.5
    marker.pose.position.y = yLoc
    marker.pose.position.z = valveHeight

    if(Type == "round"):
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.707
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 0.707
    
        marker.scale.x = diam
        marker.scale.y = diam
        marker.scale.z = 0.05

    elif(Type == "lever"):
        length = diam
        marker.pose.position.z = valveHeight+length*0.5
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = length

        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 1.0

    return marker

def update_valve():

    planRequest = PlanTurningRequest()
    planRequest.Request.ValvePose = PoseStamped()

    if(True or random.randint(0,1)):
        myType = "lever"
        myDiam = 0.3302
        myLocY = random.uniform(0,0.6096)
        planRequest.Request.Hands = "LH"
        planRequest.Request.ValveType = "LL"
        planRequest.Request.ValveSize = myDiam
        planRequest.Request.ValvePose.pose.orientation.x = -0.707
        planRequest.Request.ValvePose.pose.orientation.y = 0.0
        planRequest.Request.ValvePose.pose.orientation.z = 0.0
        planRequest.Request.ValvePose.pose.orientation.w = 0.707
    else:
        myType = "round"
        myDiam = 0.4572
        myLocY = random.uniform(-0.3,0.2)
        planRequest.Request.Hands = "BH"
        planRequest.Request.ValveType = "W"
        planRequest.Request.ValveSize = myDiam*0.5
        planRequest.Request.ValvePose.pose.orientation.x = 0.0
        planRequest.Request.ValvePose.pose.orientation.y = 0.0
        planRequest.Request.ValvePose.pose.orientation.z = 0.0
        planRequest.Request.ValvePose.pose.orientation.w = 1.0

    # keep these values here for convenience
    # testValve = handle_factory(Marker(), 0, 0.3302, 0.6096, "lever")
    # midHandle = handle_factory(Marker(), 4, 0.4572, 0.0, "round")
    # rightHandle = handle_factory(Marker(), 5, 0.2286, -0.6096, "round")

    valveMarker = valve_factory(Marker(), 0, myDiam, myLocY, myType)
    planRequest.Request.ValvePose.header.frame_id = valveMarker.header.frame_id

    valveMarker.pose.position.z += random.uniform(-0.1,0)

    planRequest.Request.ValvePose.pose.position = valveMarker.pose.position

    planRequest.Request.Direction = "CW"
    print planRequest
    return [valveMarker, planRequest]
    

def marker_publisher():
    whatToDo = ["GETREADY", "TURNVALVE", "END"]
    
    
    counter = 0
    pub = rospy.Publisher('test_valve_marker', Marker)
    rospy.init_node('test_drchubo_planner')
    
    planner_client = rospy.ServiceProxy("/valve_planner/drchubo_planner/PlanningQuery", PlanTurning)
    print "Waiting for the planner..."
    planner_client.wait_for_service()
    print "Connected."
    
    while not rospy.is_shutdown():

        # canDo = [canGetReady, canTurn, canEnd]
        errors = ['', '', '']
        canDo = [False, False, False]
        
        print "Press enter to test: ",str(counter)
        sys.stdin.readline()
       
        # show the valve pose in RViz
        [valveMarker, planRequest] = update_valve()
        
        # stash the height of the valve (we will adjust it for crouching)
        myZ = planRequest.Request.ValvePose.pose.position.z

        # send the valve pose to OpenRAVE
        for i, doThis in enumerate(whatToDo):
            
            # adjust for crouching
            if(i == 1):
                valveMarker.pose.position.z += 0.05
            else:
                valveMarker.pose.position.z = myZ

            planRequest.Request.ValvePose.pose.position = valveMarker.pose.position

            pub.publish(valveMarker)

            res = None
            try:
                rospy.loginfo("Calling the planner for "+doThis)
                planRequest.Request.TaskStage = doThis
                res = planner_client.call(planRequest)
                errors[i] = res.Response.ErrorCode
                canDo[i] = (errors[i] == "NoError")
                # print "Press Enter to continue..."
                # sys.stdin.readline()
            except rospy.ServiceException, e:
                res = None
                rospy.logerr("Service call failure.")
                print e

        counter+=1
        print canDo
        print errors

if __name__ == "__main__":
    try:
        marker_publisher()
    except rospy.ROSInterruptException:
        pass
