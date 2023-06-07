#! /usr/bin/env python3
import rospy
import math

import tf

# PoseStamped 메시지는 (x,y,z)와 방향을 나타내는 orientation으로 구성
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose, PoseStamped

# State : 드론의 모드 상태(offboard, mission, position 등)
# HomePosition : 시작 위치(전원 킨 상태) 위치 값
# PositionTarget : 드론이 이동하고자 하는 목표 위치를 나타내는 메시지. 드론이 이동하고자 하는 위치(x,y,z), 목표위치까지의 거리, 방향, 속도, 가속도 등이 표함
from mavros_msgs.msg import State, HomePosition,PositionTarget
# CommandBool : arming 확인
# SetMode : 모드 확인
# CommandTOL : land와 takeoff 확인
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL, SetModeRequest
# 쿼터니언(quaternion) 값을 euler 각도로 변환하기 위한 함수
from tf.transformations import euler_from_quaternion


# OffboardControl 클래스를 정의
class OffboardControl(object):

    def __init__(self):
        self.rate = rospy.Rate(30)
        # 객체가 포함할 변수
        self.current_state = State() # 현재 모드 상태 파악
        self.service_timeout = 30 # 서비스 일정 시간내에 안되는지 파악
        self.current_local_position = PoseStamped() # 드론 현재 위치 파악
        self.current_target_position = PoseStamped() # 목표 위치 설정
        self.current_target_velocity = Twist() # 드론 속도 제어

        # Subscriber
        self.state_sub = rospy.Subscriber("/mavros/state", State, self.state_cb)
        self.position_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.LocalPosition_cb)
        # Publisher
        self.local_pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)
        self.velocity_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=10)
        # Service
        rospy.loginfo('----Waiting for services to connect----')
        try:
            rospy.wait_for_service("/mavros/cmd/arming", self.service_timeout)
            rospy.wait_for_service("/mavros/set_mode", self.service_timeout)
        except rospy.ROSException as e:
            rospy.logerr('Failed to initialize service')
        self.arming_client = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        self.set_mode_client = rospy.ServiceProxy("/mavros/set_mode", SetMode)
    # 드론의 모드 등을 파악
    def state_cb(self, msg):
        prev_state = self.current_state
        self.current_state = msg

        if self.current_state.mode != prev_state.mode:
            rospy.loginfo("Current Mode : %s" % self.current_state.mode)
        if self.current_state.armed != prev_state.armed:
            rospy.loginfo("Vehicle armed : %r"% self.current_state.armed)

    # 드론의 현재 위치를 파악
    def LocalPosition_cb(self, msg):
        self.current_local_position.pose.position.x = msg.pose.position.x
        self.current_local_position.pose.position.y = msg.pose.position.y
        self.current_local_position.pose.position.z = msg.pose.position.z
        self.current_local_position.pose.orientation.x = msg.pose.orientation.x
        self.current_local_position.pose.orientation.y = msg.pose.orientation.y
        self.current_local_position.pose.orientation.z = msg.pose.orientation.z
        self.current_local_position.pose.orientation.w = msg.pose.orientation.w
    # 모드 변경
    def setMode(self, mode):
        rospy.logerr('Mode Changed')
        rate = rospy.Rate(0.5)
        try:
            response = self.set_mode_client(base_mode = 0, custom_mode = mode)
            # return response.mode_sent
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed : %s" %e)
        #######################응창님 코드 추천#############################
        # if self.current_state != "OFFBOARD":
        #     self.set_mode_client(base_mode=0, custom_mode="OFFBOARD")
        #     self.current_target_position.header.stamp = rospy.Time.now()
        #     self.current_target_position.pose.orientation.w = 1.0
        #     self.local_pos_pub.publish(self.current_target_position)
        # else:
        #     pass
        ##########################처음에 내가 한 것####################33
        # while True:
        #     if self.current_state.mode != mode:
        #         self.set_mode_client(base_mode=0, custom_mode=mode)
        #     else:
        #         break
        #     rate.sleep()
    # arming
    def setArm(self):
        rate = rospy.Rate(0.5)
        while True:
            if self.current_state.armed is not True:
                self.arming_client(True)
            else:
                break
            rate.sleep()
    def check_FCU_connection(self):
        while not self.current_state.connected:
            rospy.loginfo_throttle(1, "Wait FCU connection")
        rospy.loginfo("FCU connected")
    def currentYaw(self, local_position):
        quaternion = (local_position.pose.orientation.x,
                      local_position.pose.orientation.y,
                      local_position.pose.orientation.z,
                      local_position.pose.orientation.w)
        yaw = euler_from_quaternion(quaternion)[2]
        return yaw
    def fly_to_local(self,x, y, z):
        self.current_target_position.pose.position.x = x
        self.current_target_position.pose.position.y = y
        self.current_target_position.pose.position.z = z

        self.current_yaw = math.atan2((self.current_target_position.pose.position.y - self.current_local_position.pose.position.y), (self.current_target_position.pose.position.x - self.current_local_position.pose.position.x))
        qz = math.sin(self.current_yaw / 2.0)
        qw = math.cos(self.current_yaw / 2.0)
        self.current_target_position.pose.orientation.x = 0.0
        self.current_target_position.pose.orientation.y = 0.0
        self.current_target_position.pose.orientation.z = qz
        self.current_target_position.pose.orientation.w = qw

        while not rospy.is_shutdown() and self.distance(self.current_target_position, self.current_local_position) > 0.5:
            self.local_pos_pub.publish(self.current_target_position)
            self.rate.sleep()
        for i in range(10):
            if not rospy.is_shutdown():
                self.local_pos_pub.publish(self.current_target_position)
                self.rate.sleep()
    def distance(self, p1, p2):
        x = p1.pose.position.x - p2.pose.position.x
        y = p1.pose.position.y - p2.pose.position.y
        z = p1.pose.position.z - p2.pose.position.z
        dist = math.sqrt(x*x + y*y + z*z)
        return dist

if __name__ == "__main__":
    rospy.init_node('offboard_ros_node', anonymous=True)
    try:
        drone_control = OffboardControl()
        drone_control.check_FCU_connection()
        drone_control.setArm()
        drone_control.setMode("OFFBOARD")
        drone_control.fly_to_local(0.0, 0.0, 3.0)
        # drone_control.fly_to_local(100.0, 50.0, 3.0)
        drone_control.fly_to_local(0.0, 140.0, 3.0)
        drone_control.fly_to_local(0.0, 0.0, 3.0)
        rospy.spin()
    except rospy.ROSInterruptException as exception:
        pass


