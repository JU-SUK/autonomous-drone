#!/usr/bin/env python3

import rospy
from mavros_msgs.msg import State, ExtendedState
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from mavros_msgs.msg import GlobalPositionTarget
from geometry_msgs.msg import PoseArray, PoseStamped, PoseWithCovarianceStamped, TransformStamped
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

class ROSClient:

    def __init__(self):
        rospy.init_node('offboard_ctrl')
        self.avoidCollision_ = False

    def init(self, drone_control):
        self.state_sub_ = rospy.Subscriber('/mavros/state', State, drone_control.state_cb)
        self.extended_state_sub_ = rospy.Subscriber('/mavros/extended_state', ExtendedState, drone_control.extended_state_cb)
        self.marker_pos_sub_ = rospy.Subscriber('/whycon/poses', PoseArray, drone_control.marker_position_cb)
        self.local_pos_sub_ = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, drone_control.local_position_cb)
        self.global_pos_sub_ = rospy.Subscriber('/mavros/global_position/global', NavSatFix, drone_control.global_position_cb)
        self.svo_pos_sub_ = rospy.Subscriber('/svo/pose_imu', PoseWithCovarianceStamped, drone_control.svo_position_cb)
        self.setpoint_pos_sub_ = rospy.Subscriber('/trajectory/setpoint_position', PoseStamped, drone_control.setpoint_position_cb)

        self.global_setpoint_pos_pub_ = rospy.Publisher('/mavros/setpoint_position/global', GlobalPositionTarget, queue_size=10)
        self.setpoint_pos_pub_ = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.endpoint_pos_pub_ = rospy.Publisher('/trajectory/endpoint_position', PoseStamped, queue_size=10)
        self.vision_pos_pub_ = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=10)
        self.svo_cmd_pub_ = rospy.Publisher('/svo/remote_key', String, queue_size=10)
        self.ewok_cmd_pub_ = rospy.Publisher('/trajectory/command', String, queue_size=10)

        self.arming_client_ = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.land_client_ = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
        self.set_mode_client_ = rospy.ServiceProxy('/mavros/set_mode', SetMode)

    def publishTrajectoryEndpoint(self, setpoint_pos_ENU):
        if self.avoidCollision_:
            self.endpoint_pos_pub_.publish(setpoint_pos_ENU)
            rospy.spinOnce()
        else:
            rospy.loginfo("Collision avoidance has not been enabled")

    def setParam(self, key, d):
        rospy.set_param(key, d)
