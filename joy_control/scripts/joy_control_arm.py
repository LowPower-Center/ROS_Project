#!/usr/bin/env python

import roslib
import rospy
import cmd
import math
import sys
from geometry_msgs.msg import Twist
from control_msgs.msg import JointControllerState
from control_msgs.msg import JointTrajectoryControllerState
from std_msgs.msg import Float64
from sensor_msgs.msg import Joy
# Global constants
PI = 3.1415926535897931


####         List of topics            ####
# controllers followings:
TOPIC_J1_CMD = '/wajueji/base_to_cheshen_controller/command'
TOPIC_J2_CMD = '/wajueji/cheshen_to_03_controller/command'
TOPIC_J3_CMD = '/wajueji/t03_to_04_controller/command'
TOPIC_J4_CMD = '/wajueji/t04_to_chandou_controller/command'



class joy_Move:

    def __init__(self):
        self.linear_=7
        self.angular_=6
        self.scale_angular_=3
        self.scale_linear_=0.5
        self.j1=0
        self.j2=1
        self.j3=2
        self.j4=3
        rospy.init_node('joy_move_commander', anonymous=True)
        nodename = rospy.get_name()
        rospy.loginfo("%s started" % nodename)
        rospy.set_param("axis_linear", self.linear_)
        rospy.set_param("axis_angular", self.angular_)
        rospy.set_param("scale_angular", self.scale_angular_)
        rospy.set_param("scale_linear", self.scale_linear_)
        self.pub_j1_cmd = rospy.Publisher(TOPIC_J1_CMD, Float64, queue_size=10)
        self.pub_j2_cmd = rospy.Publisher(TOPIC_J2_CMD, Float64, queue_size=10)
        self.pub_j3_cmd = rospy.Publisher(TOPIC_J3_CMD, Float64, queue_size=10)
        self.pub_j4_cmd = rospy.Publisher(TOPIC_J4_CMD, Float64, queue_size=10)

        self.pub_move=rospy.Publisher("/cmd_vel", Twist,queue_size=1000)

        self.rate = rospy.get_param("~rate", 40)
        self.timeout_ticks = rospy.get_param("~timeout_ticks", 2)
        self.sub_joy=rospy.Subscriber("/joy", Joy,self.joy_callback)
    def joy_callback(self, joy_data):

        msg = Twist()
        msg.linear.x = joy_data.axes[self.linear_]*self.scale_linear_

        msg.angular.z = joy_data.axes[self.angular_]*self.scale_angular_
        self.pub_move.publish(msg)
        print(msg)
        self.pub_j1_cmd.publish(joy_data.axes[self.j1])
        self.pub_j2_cmd.publish(joy_data.axes[self.j2])
        self.pub_j3_cmd.publish(joy_data.axes[self.j3])
        self.pub_j4_cmd.publish(joy_data.axes[self.j4])


if __name__ == '__main__':
    try:
        joy_move = joy_Move()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

