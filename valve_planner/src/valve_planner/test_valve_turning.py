#!/usr/bin/env python

# Bener Suay
# December, 2013
# benersuay@wpi.edu
# 
# Testing script that will log
# results of different planning
# attempts for valve turning.

# Options to Enumerate:
# Size: Large Only
# Turn: False
# Hands: RH/LH for LL, BH for W

# Add Depth!!
# Add Yaw 5, 10, 15 deg


# ROS imports
import rospy
from visualization_msgs.msg import Marker
from valve_planner_msgs.msg import *
from valve_planner_msgs.srv import *
from geometry_msgs.msg import *

# Other imports
import random
import sys
import time

distFromRobotTorso = 0.5
clearanceFromPipe = 0.05
pipeDiam = 0.1
valveHeight = 40*0.0254
valveType = None
valveKnownTypes = ["lever","round"]

# Create the valve marker for the round valve or the lever
def valve_factory(marker, markerid, diam, yLoc, Type, heightDelta):

    #Setup the valve marker
    marker.header.frame_id = "leftFoot"
    marker.header.stamp = rospy.get_rostime()
    marker.id = markerid
    marker.type = 3 
    marker.action = 0

    #Set the marker's position based off of the numbers that we generated before
    marker.pose.position.x = distFromRobotTorso - clearanceFromPipe - pipeDiam*0.5
    marker.pose.position.y = yLoc
    marker.pose.position.z = valveHeight+heightDelta

    #Actually create the two valves
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
        marker.pose.position.z = valveHeight+heightDelta+length*0.5
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

    #Create a new planner request
    planRequest = PlanTurningRequest()
    planRequest.Request.ValvePose = PoseStamped()

    #Change the height by a given amount
    heightDelta = random.uniform(-0.2032, 0.2032)

    #Choose to create either a lever valve or a round valve
    #if(False or random.randint(0,1)):
    myLocY = 0.0
    myDiam = 0.0

    if valveType == "lever" :
        myDiam = 0.23
        myLocY = random.uniform(0,0.6)
        planRequest.Request.ValveType = "LL"
        planRequest.Request.ValveSize = myDiam
        planRequest.Request.ValvePose.pose.orientation.x = -0.707
        planRequest.Request.ValvePose.pose.orientation.y = 0.0
        planRequest.Request.ValvePose.pose.orientation.z = 0.0
        planRequest.Request.ValvePose.pose.orientation.w = 0.707
    elif valveType == "round" :
        myDiam = 0.4572
        myLocY = random.uniform(-0.3,0.2)
        planRequest.Request.ValveType = "W"
        planRequest.Request.ValveSize = myDiam*0.5
        planRequest.Request.ValvePose.pose.orientation.x = 0.0
        planRequest.Request.ValvePose.pose.orientation.y = 0.0
        planRequest.Request.ValvePose.pose.orientation.z = 0.0
        planRequest.Request.ValvePose.pose.orientation.w = 1.0

    #Round Y so that it can be easily tracked
    myLocY = round(myLocY, 3)

    # keep these values here for convenience
    # testValve = handle_factory(Marker(), 0, 0.3302, 0.6096, "lever")
    # midHandle = handle_factory(Marker(), 4, 0.4572, 0.0, "round")
    # rightHandle = handle_factory(Marker(), 5, 0.2286, -0.6096, "round")

    #Create the valve marker from the valve factor
    valveMarker = valve_factory(Marker(), 0, myDiam, myLocY, valveType, heightDelta)

    #Save the position and the header into the planner request
    planRequest.Request.ValvePose.header.frame_id = valveMarker.header.frame_id
    planRequest.Request.ValvePose.pose.position = valveMarker.pose.position

    print planRequest
    return [valveMarker, planRequest]
    

def marker_publisher():

    valve_type_str = "" 

    if valveType == "lever" :
        valve_type_str = "LEVER"
    elif valveType == "round" :
        valve_type_str = "ROUND"

    #Create a new logfile
    now = time.time()
    log_file_name_long = str(round(now, 0)) + "_"+valve_type_str+"_valve_planner_log_long_.csv"
    log_file_name_short = str(round(now, 0)) + "_"+valve_type_str+"_valve_planner_log_short_.csv"
    log_file_name_success = str(round(now, 0)) + "_"+valve_type_str+"_valve_planner_log_success_.csv"
    log_file_name_fail = str(round(now, 0)) + "_"+valve_type_str+"_valve_planner_log_fail_.csv"

    headerString = "Test,ID,Task Stage,Success?,Error Return,Use Global IK Seed,Valve Type,Hand,Size,Turn Amount,Fixed Turn?,Direction,Plan In Box?,Pos-X,Pos-Y,Pos-Z,Orien-X,Orien-Y,Orien-Z,Orien-W"

    log_file_long = open(log_file_name_long, "a")
    log_file_long.write(headerString)
    log_file_long.write("\n")
    log_file_long.close()

    #log_file_short = open(log_file_name_short, "a")
    #log_file_short.write(headerString)
    #log_file_short.write("\n")
    #log_file_short.close()

    log_file_success = open(log_file_name_success, "a")
    log_file_success.write(headerString)
    log_file_success.write("\n")
    log_file_success.close()

    log_file_fail = open(log_file_name_fail, "a")
    log_file_fail.write(headerString)
    log_file_fail.write("\n")
    log_file_fail.close()

    whatToDo = ["GETREADY", "TURNVALVE", "END"]
    
    #Number of Tests
    counter = 0
    ID_Counter = 0

    #Create the publisher that we will use
    pub = rospy.Publisher('test_valve_marker', Marker)
    rospy.init_node('test_drchubo_planner')
    
    #Create the client and wait for it to connect to the planner
    planner_client = rospy.ServiceProxy("/valve_planner/drchubo_planner/PlanningQuery", PlanTurning)
    print "Waiting for the planner..."
    planner_client.wait_for_service()
    print "Connected."
    

    #Run this script forever
    while not rospy.is_shutdown():

        # Set the errors and the cando's to be false since this is a new run
        # canDo = [canGetReady, canTurn, canEnd]
        errors = ['', '', '']
        canDo = [False, False, False]
        
        #Wait for an enter since Ben LOVES to hit enter
        #print "Press enter to test: ",str(counter)
        #sys.stdin.readline()
       
        # show the valve pose in RViz
        [valveMarker, planRequest] = update_valve()

        #Randomly Change the direction
        if(True or random.randint(0,1)):
            planRequest.Request.Direction = "CW"
        else:
            planRequest.Request.Direction = "CCW"

        #Randomly Select the hand based off of valve tpype
        if (planRequest.Request.ValveType == "LL"):
            if(True or random.randint(0,1)):
                planRequest.Request.Hands = "LH"
            else:
                planRequest.Request.Hands = "RH"
        elif (planRequest.Request.ValveType == "W"):
            planRequest.Request.Hands = "BH"

        #Change the turn amount
        if(True):
            planRequest.Request.TurnAmount = 30
        else:
            planRequest.Request.TurnAmount = random.randint(15,60)

        #Change the fixed turn
        if(True or random.randint(0,1)):
            planRequest.Request.FixedTurn = True
        else:
            planRequest.Request.FixedTurn = False

        #Change the plan in box
        if(True or random.randint(0,1)):
            planRequest.Request.PlanInBox = True
        else:
            planRequest.Request.PlanInBox = False


        # stash the height of the valve (we will adjust it for crouching)
        myZ = planRequest.Request.ValvePose.pose.position.z

        # send the valve pose to OpenRAVE
        for i, doThis in enumerate(whatToDo):

            planRequest.Request.IkSeed = True

            # adjust for crouching
            if(i == 1):
                valveMarker.pose.position.z += 0.05
            else:
                valveMarker.pose.position.z = myZ

            #If you updated the position you need to re-save it
            planRequest.Request.ValvePose.pose.position = valveMarker.pose.position

            #Publish the marker so that it can be seen in RVIZ
            pub.publish(valveMarker)

            #Actually call the planner here!
            res = None
            try:
                planRequest.Request.ID = ID_Counter
                rospy.loginfo("Calling the planner for "+doThis)
                planRequest.Request.TaskStage = doThis
                res = planner_client.call(planRequest)

                #Save the errors and whether or not we could plan for the given planner Request
                errors[i] = res.Response.ErrorCode
                canDo[i] = (errors[i] == "NoError")

                #print errors[i]
                #print canDo[i]

                #Wait for an enter since Ben LOVES to hit enter
                #print "Press enter to test: ",str(counter)
                #sys.stdin.readline()

                while (errors[i].find('\n') >= 0):
                    errors[i] = errors[i].replace('\n', '')

                testString = str(counter) + "," + str(ID_Counter) + "," + doThis + "," + str(canDo[i]) + "," + str(errors[i]) + "," + \
                               str(planRequest.Request.IkSeed) + "," + \
                               str(planRequest.Request.ValveType) + "," + \
                               planRequest.Request.Hands + "," + \
                               str(round(planRequest.Request.ValveSize, 3)) + "," + \
                               str(round(planRequest.Request.TurnAmount, 3)) + "," + \
                               str(planRequest.Request.FixedTurn) + "," + \
                               planRequest.Request.Direction + "," + \
                               str(planRequest.Request.PlanInBox) + "," + \
                               str(round(planRequest.Request.ValvePose.pose.position.x, 3)) + "," + \
                               str(round(planRequest.Request.ValvePose.pose.position.y, 3)) + "," + \
                               str(round(myZ, 3)) + "," + \
                               str(round(planRequest.Request.ValvePose.pose.orientation.x, 3)) + "," + \
                               str(round(planRequest.Request.ValvePose.pose.orientation.y, 3)) + "," + \
                               str(round(planRequest.Request.ValvePose.pose.orientation.z, 3)) + "," + \
                               str(round(planRequest.Request.ValvePose.pose.orientation.w, 3))

                ID_Counter += 1

         #       print testString

                #Log Things!!!
                log_file_long = open(log_file_name_long, "a")
                log_file_long.write(testString)
                log_file_long.write("\n")
                log_file_long.close()

                if (canDo[i] == False):
                    planRequest.Request.ID = ID_Counter
                    rospy.loginfo("Re-Calling the planner for "+doThis)
                    planRequest.Request.IkSeed = False
                    planRequest.Request.TaskStage = doThis
                    res = planner_client.call(planRequest)

                    #Save the errors and whether or not we could plan for the given planner Request
                    errors[i] = res.Response.ErrorCode
                    canDo[i] = (errors[i] == "NoError")

                    #if (doThis == "GETREADY" and canDo[i] == False):

                    while (errors[i].find('\n') >= 0):
                        errors[i] = errors[i].replace('\n', '')

                    testString = str(counter) + "," + str(ID_Counter) + "," + doThis + "," + str(canDo[i]) + "," + str(errors[i]) + "," + \
                                   str(planRequest.Request.IkSeed) + "," + \
                                   str(planRequest.Request.ValveType) + "," + \
                                   planRequest.Request.Hands + "," + \
                                   str(round(planRequest.Request.ValveSize, 3)) + "," + \
                                   str(round(planRequest.Request.TurnAmount, 3)) + "," + \
                                   str(planRequest.Request.FixedTurn) + "," + \
                                   planRequest.Request.Direction + "," + \
                                   str(planRequest.Request.PlanInBox) + "," + \
                                   str(round(planRequest.Request.ValvePose.pose.position.x, 3)) + "," + \
                                   str(round(planRequest.Request.ValvePose.pose.position.y, 3)) + "," + \
                                   str(round(myZ, 3)) + "," + \
                                   str(round(planRequest.Request.ValvePose.pose.orientation.x, 3)) + "," + \
                                   str(round(planRequest.Request.ValvePose.pose.orientation.y, 3)) + "," + \
                                   str(round(planRequest.Request.ValvePose.pose.orientation.z, 3)) + "," + \
                                   str(round(planRequest.Request.ValvePose.pose.orientation.w, 3))

                    ID_Counter += 1

        #            print testString

                    #Log Things!!!
                    log_file_long = open(log_file_name_long, "a")
                    log_file_long.write(testString)
                    log_file_long.write("\n")
                    log_file_long.close()

                    if (canDo[i] == False):
                        break

                else:
                    pass


        #        print testString


            except rospy.ServiceException, e:
                res = None
                rospy.logerr("Service call failure.")
                print e

        #log_file_short = open(log_file_name_short, "a")
        #log_file_short.write(testString)
        #log_file_short.write("\n")
        #log_file_short.close()

        allTrue = True
        for i in canDo:
            if (i == False):
                allTrue = False

        if (allTrue):
            log_file_success = open(log_file_name_success, "a")
            log_file_success.write(testString)
            log_file_success.write("\n")
            log_file_success.close()
        else:
            log_file_fail = open(log_file_name_fail, "a")
            log_file_fail.write(testString)
            log_file_fail.write("\n")
            log_file_fail.close()

        counter+=1
        print " "
        print whatToDo
        print canDo
        print errors
        print counter
        print "---------------------"

if __name__ == "__main__":

    valveType = "lever"

    if(len(sys.argv) >= 2):
        for index in range(1,len(sys.argv)):
            if(sys.argv[index] == "-valvetype" and index+1<len(sys.argv)):
                valveType = str(sys.argv[index+1])

    print valveType

    if valveType not in valveKnownTypes :
        print "unknown valve type"
    else:
        try:
            marker_publisher()
        except rospy.ROSInterruptException:
            pass
