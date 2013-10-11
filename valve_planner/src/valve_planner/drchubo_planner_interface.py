#!/usr/bin/python

#############################################################################
#                                                                           #
#   Calder Phillips-Grafflin -- WPI/Drexel Darpa Robotics Challenge Team    #
#                                                                           #
#   Service interface to the Hubo CBiRRT planner class                      #
#                                                                           #
#############################################################################

import subprocess

import roslib

import rospy
from std_msgs.msg import *
from sensor_msgs.msg import *
from trajectory_msgs.msg import *
from valve_planner_msgs.msg import *
from valve_planner_msgs.srv import *

#import hubo_traj_reader
import drchubo_v2_wheel_turning
import hubo_test_send_command

class HuboPlannerInterface:

    def __init__(self, path):

        self.tempIndex = 0

        self.debug = True
        self.no_planning = False
        rospy.loginfo("Starting Hubo planner interface...")
        self.path = path
        self.current_config = None

        path_to_robot = rospy.get_param("~robot_model")
        path_to_wheel = rospy.get_param("~tiny_wheel_model")
        self.read_joint_states = rospy.get_param("~read_joint_states")
        self.useIKFast = rospy.get_param("~use_ikfast")

        print "Info: Using robot model: "
        print path_to_robot
        
        print "Info: Using valve model: "
        print path_to_wheel

        print "Info: sim mode: "
        print (not self.read_joint_states)

        print "Info: IK-Fast enabled: "
        print self.useIKFast


        if( self.read_joint_states ):
            self.backend = hubo_test_send_command.HuboTestSendCommand("drchubo")
        
        
        self.planner = drchubo_v2_wheel_turning.DrcHuboV2WheelTurning( path_to_robot, path_to_wheel )
        self.planner.useIKFast = self.useIKFast
        
        # Clean-Up old trajectory files on initialization
        self.planner.RemoveFiles()

        self.planner.SetViewer(True)
        self.planner.SetStopKeyStrokes(False)

        rospy.loginfo("Loaded Hubo CBiRRT wrapper")

        # We can either read the hubo's joint state from the TF tree or assume it starts from home in OpenRAVE
        if(self.read_joint_states):
            self.RobotConfigurationClient = rospy.Subscriber("/drchubo_fullbody_interface/joint_states", JointState, self.GetRealRobotConfig)

        self.PlanRequestService = rospy.Service("drchubo_planner/PlanningQuery", PlanTurning, self.PlanRequestHandler)
        self.ExecuteRequestService = rospy.Service("drchubo_planner/ExecutionQuery", ExecuteTurning, self.ExecuteRequestHandler)

        rospy.loginfo("Service host loaded, Planner interface running")

    # Replays the last planned trajectories in openrave
    def ExecuteRequestHandler(self, req):

        print "Execute - Identifier: "
        print req.Identifier
        res = ExecuteTurningResponse()

        why = ""
        success = True

        if( req.Identifier == "PREVIEW" ):
            
            try:
                [success, why] = self.planner.trajectory.PlayInOpenRAVE()
            except:
                success = False
                why = "No trajectory to preview. You must run the planner first."
                print why

        elif( req.Identifier == "EXECUTE" ):          

            if( self.read_joint_states ):
            #if( True ):  
            
                try:

                    while self.current_config is None :
                        rospy.logwarn("Executer is waiting to recieve joint states of the robot!")

                    # Set robot at current configuration
                    self.planner.SetRobotConfiguration(self.current_config)

                    # Convert OpenRAVE format trajectory to ROS Action Lib.
                    listofq = self.planner.trajectory.GetOpenRAVETrajectory( self.planner.default_trajectory_dir ) 

                    # Check that current configuration is with in limits
                    # of acceptable distance
                    if self.planner.trajectory.IsRobotAtInitConfig() is not True :
                        print "error : robot is not at trajectory start"
                        raise

                    # TODO: Error handling for set trajectory [success, why] = set_trajectory
                    self.backend.set_trajectory(listofq, self.planner.jointDict)
                    #[success, why] = self.backend.set_trajectory()
                    #if(not success):
                    #    return why

                    # Call Action Lib. Client to play the trajectory on the robot
                    # TODO: Error handling for traj client
                    # [success, why] = self.backend.joint_traj_client()
                    self.backend.joint_traj_client()

                    # If:
                    # i)   you played the trajectory successfully, and,
                    # ii)  if the trajectory was planned for GetReady or EndTask
                    # iii) or if the trajectory was planned for any other valve type than round valve
                    #
                    # then erase the trajectory for safety purposes.
                    if( self.planner.trajectory.name == "GetReady" or \
                        self.planner.trajectory.name == "EndTask" or \
                        self.planner.trajectory.valveType != "W" ):
                        print "valve type : " + str( self.planner.trajectory.valveType )
                        print "flush trajectory!!!"
                        self.planner.trajectory = None

                except:
                    success = False
                    why = "No trajectory to execute. You must run the planner first."

            else:
                print "Info: you can't execute a trajectory in sim mode. Use preview option."

        if(not success):
            res.ErrorCode = "error : " + why
        else:
            print "no error"
            res.ErrorCode = "no error : " + why

        print res.ErrorCode
        return res

    def GetPlanResponse(self,error_code):
        # Do whatever you want to do with the error_code
        error_str = ""
        if(error_code == 0):
            # No error
            error_str = "NoError"
        else:
            error_str = str(error_code)
        res = PlanTurningResponse()
        res.Response.header = Header()
        res.Response.ErrorCode = error_str
        res.Response.Labels = "LABELS"
        return res

    def UpdateValvePose(self,req):
        valve_trans = [req.Request.ValvePose.pose.position.x, req.Request.ValvePose.pose.position.y, req.Request.ValvePose.pose.position.z]
        valve_rot = [req.Request.ValvePose.pose.orientation.x, req.Request.ValvePose.pose.orientation.y, req.Request.ValvePose.pose.orientation.z, req.Request.ValvePose.pose.orientation.w]    
        # Use the frame id that comes in from RViz and set the wheel pose
        self.h = self.planner.SetValvePoseFromQuaternionInFrame( req.Request.ValvePose.header.frame_id.strip("/"), valve_trans, valve_rot )

    def PrintReqInfo(self,req):
        print "task stage"
        print req.Request.TaskStage
        print "valve rad."
        print req.Request.ValveSize
        print "frame id"
        print req.Request.ValvePose.header.frame_id.strip("/")
        print "valve type"
        print req.Request.ValveType
        print "valve size"
        print req.Request.ValveSize
        print "hand(s)"
        print req.Request.Hands
        print "rotation direction"
        print req.Request.Direction

    # Sets the wheel location in openrave
    # Calls the planner (CiBRRT)
    # Reads the trajectories from the files
    def PlanRequestHandler(self, req):

        self.planner.ResetEnv()

        # self.PrintReqInfo(req)

        self.planner.StartViewer()

        self.UpdateValvePose(req)

        self.planner.CreateValve(req.Request.ValveSize, req.Request.ValveType)

        while (self.read_joint_states and (self.current_config is None)):
            rospy.logwarn("Planner is waiting to recieve joint states of the robot!")

        if( self.read_joint_states ):
            self.planner.SetRobotConfiguration(self.current_config)

        # Call to CBiRRT if no planning simply read the current files
        if( self.no_planning ):
            # Obsolete !!! (number of files changed)
            trajectory_files = [ 'movetraj0.txt','movetraj1.txt','movetraj2.txt','movetraj3.txt','movetraj4.txt','movetraj5.txt']
        else:
            error_code = self.planner.Plan( [], req.Request.ValveSize, req.Request.Hands, req.Request.Direction, req.Request.ValveType, req.Request.TaskStage )

        return self.GetPlanResponse(error_code)

    # NOT USED
    # Reads the trajectory from the files generated by openrave
    # then builds PlanValveTurningResponse  
    def BuildResponse( self, trajectory_files ):

        #trajectory_msgs/JointTrajectory[]
        traj_array = []

        for f in trajectory_files:
            print "read file : " + f
            traj_array.append( hubo_traj_reader.read( f ) )

        #print traj_array
        return PlanValveTurningResponse( traj_array, ["LABELS"] , "PLAN_OK" )


    def GetRealRobotConfig(self, msg):
        # Assemble a new config
        new_config = {}
        if (len(msg.name) != len(msg.position)):
            rospy.logerr("Malformed JointState!")
        else:
            for joint_index in range(len(msg.name)):
                new_config[msg.name[joint_index]] = msg.position[joint_index]
            self.current_config = new_config

            # debug #######################
            #
            # if(self.tempIndex == 200):
            #     self.tempIndex = 0
            #     print "CURRENT CONFIG"
            #     print self.current_config
            # else:
            #     self.tempIndex += 1
            ################################
    
    def Hook(self):
        print "End planner node"
        self.planner.KillOpenrave()
       

if __name__ == '__main__':
    path = subprocess.check_output("rospack find valve_planner", shell=True)
    path = path.strip("\n")
    rospy.init_node("hubo_planner_interface")
    # Maybe we will need params some day?
    huboplan = HuboPlannerInterface( path )
    rospy.on_shutdown( huboplan.Hook )
    print "Robot ready to plan, waiting for a valve pose update..."
    rospy.spin()
    

