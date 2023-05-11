#! /usr/bin/env python3

from __future__ import division
import rospy
import math
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from mavros_msgs.msg import State, Altitude, HomePosition, ExtendedState, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL, ParamGet,StreamRateRequest, StreamRate
from nav_msgs.mgs import Odometry
from std_msgs.msg import Float64, String, Bool
from geometry_msgs.msg import Vector3, Point
from tf.transformations import euler_from_quaternion

class DroneControl(object):
  
  def __init__(self):
    self.state = State()
