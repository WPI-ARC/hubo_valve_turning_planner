#!/usr/bin/env python
import roslib

import sys

import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
from std_msgs.msg import *
from hubo_robot_msgs.msg import *
#from valve_planner.srv import *
from geometry_msgs.msg import *
#from ach_srvers.msg import *
from copy import *



class HuboTestSendCommand:

    def __init__(self,my_robot_name=None):
        self.traj = None
        self.hand_pub = rospy.Publisher("hand_command", Bool)
        self.default_pose = Pose()
        self.default_pose.position.x = 0.18
        self.default_pose.position.y = 0.09
        self.default_pose.position.z = 0.9
        self.default_pose.orientation.x = 0.5 
        self.default_pose.orientation.y = 0.5
        self.default_pose.orientation.z = 0.5 
        self.default_pose.orientation.w = 0.5
        self.joint_names = None
        self.joint_mapping = None
        self.robot_name = my_robot_name
        #ns = "/" + self.robot_name + "_fullbody_interface/drchubo_fullbody_feedback_node/"
        # Gets joint mapping from parameter server
        self.joint_mapping = {}
        #print "ns : " + ns + "joints"
        self.joint_names = rospy.get_param("~joints")
        for i in range(0,len(self.joint_names)):
            self.joint_names[i] = self.joint_names[i].strip( '/' )
            self.joint_mapping[ self.joint_names[i] ] = int(i)
        
        self.dt = 0.04

    def set_trajectory(self, trajectory=None, joint_dict=None, compliance=False):
 
        error = "NoError"
        success = True

        print "Joint mapping dictionary:"
        print self.joint_mapping
        print "filling message"
                       
        self.hubo_traj = JointTrajectory()
        self.hubo_traj.header.stamp = rospy.Time.now()
        self.hubo_traj.joint_names = self.joint_names
        self.hubo_traj.compliance.joint_names = []

        if(compliance):
            print "Compliance: ON"
            self.hubo_traj.compliance.compliance_kp = 1.0 # For now the value of kp doesnt matter as long as its non-zero
            self.hubo_traj.compliance.compliance_kd = 1.0 # For now the value of kd doesnt matter as long as its non-zero
            
            try:
                compliant_joints = rospy.get_param("~compliant_joints")
                for cjIdx, cj in enumerate(compliant_joints):
                    self.hubo_traj.compliance.joint_names.append(cj)
                                                             
                print "Compliant Joints: "
                print self.hubo_traj.compliance.joint_names
            except:
                error="Could not find compliant joints parameter. Is it loaded on the param server?"
                return [False, error]
        else:
            print "Compliance: OFF"
    

        t = 0.0

        for q in trajectory: # reads all lines in the file

            #print "q"
            #print q

            # Ane configuration per line
            current_point = JointTrajectoryPoint()
            current_point.time_from_start = rospy.Duration(t)

            # Advance in time by dt
            t += float( self.dt )

            # ---------------------------------------
            # Fills position buffer
            p_buffer = [0.0] * len(self.joint_mapping)
                       
            for jName, jIdx in joint_dict.iteritems():
                if (jName == "LF11"):
                    # print "caught lf11"
                    jName = "LF1"
                elif (jName == "RF11"):
                    # print "caught rf11"
                    jName = "RF1"
                elif (jName == "RF21"):
                    # print "caught rf21"
                    jName = "RF2"
                try:
                    # reverse joint dict ['joint name'] = joint val
                    urdfIndex = self.joint_mapping[ jName ]
                except KeyError:
                    urdfIndex = None

                if ((urdfIndex is not None) and (urdfIndex < len(self.joint_mapping)) ):
                    p_buffer[urdfIndex] = float(q[jIdx])
                    
                else:
                    #print "URDFINDEX not found - " + jName
                    continue
            #print len(p_buffer)

            # ---------------------------------------
            # Fills velocity buffer using finite deferencing
            v_buffer = []
            v_buffer.append( (p_buffer[0]-p_buffer[1])/self.dt )
            for i in range( 1 , len(p_buffer)-1 ):
                v_buffer.append( (p_buffer[i+1]-p_buffer[i-1])/self.dt )
            v_buffer.append( (p_buffer[len(p_buffer)-1]-p_buffer[len(p_buffer)-2])/self.dt )

            # ---------------------------------------
            # Fills acceleration buffer using finite deferencing
            a_buffer = []
            a_factor = 10;
            a_buffer.append( a_factor*(v_buffer[0]-v_buffer[1])/self.dt )
            for i in range( 1 , len(v_buffer)-1 ):
                a_buffer.append( a_factor*(v_buffer[i+1]-v_buffer[i-1])/self.dt )
            a_buffer.append( a_factor*(v_buffer[len(v_buffer)-1]-v_buffer[len(v_buffer)-2])/self.dt )

            # Appends trajectory point
            current_point.positions = deepcopy(p_buffer)
            current_point.velocities = deepcopy(v_buffer)
            current_point.accelerations = deepcopy(a_buffer)

            self.hubo_traj.points.append(current_point)
        
        return True

    def call_to_planner(self, valve_pose=None):

        rospy.wait_for_service('hubo_planner/PlanningQuery')
        try:
            planner_srv = rospy.ServiceProxy('hubo_planner/PlanningQuery', PlanValveTurning)
            valve_position = None
            if (valve_pose == None):
                valve_position = self.default_pose
            else:
                valve_position = valve_pose
                print "Default pose is:\n" + str(self.default_pose)
                print "Provided pose is:\n" + str(valve_position)

            response = planner_srv(valve_position)
            self.traj = response.trajectories
            return self.traj
        except rospy.ServiceException, e:
            self.traj = None
            print "Service call failed: %s"%e


    def usage():

        return "%s [x y]"%sys.argv[0]


    def joint_traj_client(self):
        error=None
        success=True

        if( self.hubo_traj is None ):
            error="Cannot execute empty trajectory."
            print error
            return [False, error]

        # Creates a SimpleActionClient, passing the type of action to the constructor.
        client = actionlib.SimpleActionClient('/drchubo_fullbody_controller/joint_trajectory_action', hubo_robot_msgs.msg.JointTrajectoryAction )
        
        # Waits until the action server has started
        print "waiting for action server..."
        client.wait_for_server()
        
        # Sends trajectory as a goal
        print "client started, sending trajectory!"
        res = None
        #rospy.sleep(1.0)
        self.hubo_traj.header.stamp = rospy.Time.now()
        traj_goal = JointTrajectoryGoal()
        traj_goal.trajectory = self.hubo_traj
        traj_goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(1.0)
        client.send_goal( traj_goal )

        print "Wait for result!"
        client.wait_for_result()
        res = client.get_result()
        print res

        '''
        try:
            # for all trajectories
            for t in self.traj:
            
                # Creates a goal to send to the action server.
                goal = JointTrajectoryGoal()
                t.header.stamp = rospy.Time.now()
                goal.trajectory = t

                # Sends the goal to the action server.
                client.send_goal( goal )
                client.wait_for_result()
                res = client.get_result()

                # Prints out the result of executing the action
                print "end execution of trajectory"
                #print res
            
            return res

        except rospy.ServiceException, e:
            print "Goal Sending failed : %s"%e
        '''
        return [True, error]

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('hubo_test_send_command')
        command = HuboTestSendCommand()
        traj = command.call_to_planner()
        #print traj[0]
        command.joint_traj_client()
        #rospy.spin()

    except rospy.ROSInterruptException:
        print "program interrupted before completion"

