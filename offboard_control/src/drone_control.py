#! /usr/bin/env python3
from ros_client import ROSClient

import rospy
from mavros_msgs.srv import CommandTOL, SetMode
from mavros_msgs.msg import GlobalPositionTarget
from tf.transformations import quaternion_from_euler
from tf2_ros import TransformListener, TransformBroadcaster

class DroneControl:
    def __init__(self, ros_client):
        self.ros_client_ = ros_client
        self.ros_client_.init(self)

        # The setpoint publishing rate MUST be faster than 2Hz
        self.rate_ = rospy.Rate(ROS_RATE)

        self.tfBuffer_ = tf2_ros.Buffer()
        self.listener_ = tf2_ros.TransformListener(self.tfBuffer_)

    def state_cb(self, msg):
        pass

    def extended_state_cb(self, msg):
        pass

    def marker_position_cb(self, msg):
        pass

    def local_position_cb(self, msg):
        pass

    def global_position_cb(self, msg):
        pass

    def svo_position_cb(self, msg):
        pass

    def setpoint_position_cb(self, msg):
        pass

    def approachMarker(self, marker_id, distance, altitude, offset_x, offset_y):
        pass

    def takeoff(self, takeoff_altitude):
        pass

    def setGuidedMode(self):
        pass

    def arm(self):
        pass

    def land(self):
        pass

    def publishTrajectoryEndpoint(self, setpoint_pos_ENU):
        pass

    def transformToLocalFrame(self, goal_pose):
        pass

    def transformToGlobalFrame(self, goal_pose):
        pass

