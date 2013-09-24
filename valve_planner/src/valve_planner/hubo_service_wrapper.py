#!/usr/bin/env python
import roslib
roslib.load_manifest('hubo_planner')
roslib.load_manifest('gui')
import rospy

#tf and stuff
roslib.load_manifest("tf")
import tf
from tf.transformations import *
from transformation_helper import *
from hubo_test_send_command import *

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
from hubo_robot_msgs.msg import *
from hubo_planner.srv import *
from geometry_msgs.msg import *
from trajectory_msgs.msg import *
from gui.srv import *

class ValveService:

    def __init__(self):
        self.default_pose = Pose()
        self.default_pose.position.x = 0.18
        self.default_pose.position.y = 0.09
        self.default_pose.position.z = 0.9
        self.default_pose.orientation.x = 0.5 
        self.default_pose.orientation.y = 0.5
        self.default_pose.orientation.z = 0.5 
        self.default_pose.orientation.w = 0.5
        self.backend = HuboTestSendCommand()
        self.listener = tf.TransformListener()
        self.server = rospy.Service("update_valve_pose", updateValvePos, self.service_handler)
        rospy.loginfo("Started planner/execution server for Hubo+")
        rospy.spin()

    def service_handler(self, request):
        rospy.loginfo("Received a valve turning request")
        #self.compute_proper_correction(request.valve.pose)
        # Get the valve pose and turn it into a useful format
        raw_posestamped = request.valve
        useful_pose = self.convert_pose(raw_posestamped)
        # Call the planner
        rospy.loginfo("Calling the planner...")
        self.backend.call_to_planner(useful_pose)
        rospy.loginfo("...planner completed")
        # Execute the trajectories planned
        rospy.loginfo("Executing the trajectories...")
        self.backend.joint_traj_client()
        rospy.loginfo("...execution completed")
        # Put together the response
        response = updateValvePos._response_class()
        response.valve = raw_posestamped
        response.success_code = "Turning completed"
        return response

    def compute_proper_correction(self, rviz_pose):
        [tcor, rcor] = ComponentsFromTransform(PoseToTransform(ComposePoses(InvertPose(rviz_pose), self.default_pose)))
        print "Correct rotation correction is: " + str(rcor)

    def convert_pose(self, raw_posestamped):
        [trans,rot] = self.listener.lookupTransform("/Body_RAR", raw_posestamped.header.frame_id, rospy.Time(0))
        cur_valve_frame_pose = PoseFromTransform(TransformFromComponents(trans,rot))
        cur_valve_pose = raw_posestamped.pose
        full_valve_pose = ComposePoses(cur_valve_frame_pose, cur_valve_pose)
        return self.switch_to_openrave_orientation(full_valve_pose)

    def switch_to_openrave_orientation(self, rviz_pose):
        rot1 = quaternion_about_axis(math.pi / 2.0, (0,0,1))
        rot2 = quaternion_about_axis(math.pi / 2.0, (1,0,0))
        rfull = NormalizeQuaternion(ComposeQuaternions(rot1, rot2))
        rcomputed = [0.39215845565791163, -0.38620829332267054, -0.59065001832988195, 0.59007411032149337]
        correction = PoseFromTransform(TransformFromComponents([0.0,0.0,0.11],rcomputed))
        corrected_pose = ComposePoses(rviz_pose, correction)
        return corrected_pose

if __name__ == '__main__':
    rospy.init_node('hubo_test_send_command')
    ValveService()
