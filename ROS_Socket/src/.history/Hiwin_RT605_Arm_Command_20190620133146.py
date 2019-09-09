#!/usr/bin/env python3
# license removed for brevity
import rospy
import os
import numpy as np
from std_msgs.msg import String
from ROS_Socket.srv import *
from ROS_Socket.msg import *
import math
import enum
pos_feedback_times = 0
mode_feedback_times = 0
msg_feedback = 1
##-----------switch define------------##
class switch(object):
    def __init__(self, value):
        self.value = value
        self.fall = False

    def __iter__(self):
        """Return the match method once, then stop"""
        yield self.match
        raise StopIteration

    def match(self, *args):
        """Indicate whether or not to enter a case suite"""
        if self.fall or not args:
            return True
        elif self.value in args: # changed for v1.5, see below
            self.fall = True
            return True
        else:
            return False

#---------strategy_client pos data------------------
def strategy_client_pos_move(x,y,z,pitch,roll,yaw):
    global pos_feedback_times

    rospy.wait_for_service('arm_pos')

    try:
        # create a handle to the add_two_ints service
        Arm_pos_client = rospy.ServiceProxy('arm_pos', arm_data)
        pos_feedback = Arm_pos_client(x,y,z,pitch,roll,yaw)
        pos_feedback_times = pos_feedback.response
        return pos_feedback_times
    except rospy.ServiceException as e:
        print ("Service call failed: %s"%e)
def usage():
     return "%s [x y]"%sys.argv[0]
##---------strategy_client mode data  20190315
def strategy_client_Arm_Mode(action,ra,vel,both):
    global mode_feedback_times

    rospy.wait_for_service('arm_mode')
    try:
        # create a handle to the add_two_ints service
        Arm_mode_client = rospy.ServiceProxy('arm_mode', arm_mode)
        mode_feedback = Arm_mode_client(ra,action,vel,both)
        mode_feedback_times = mode_feedback.mode
        return mode_feedback_times
    except rospy.ServiceException as e:
        print ("Service call failed: %s"%e)
##---------strategy_client Speed mode data
def strategy_client_Speed_Mode(Speedmode):
    global speed_mode_feedback_times

    rospy.wait_for_service('speed_mode')
    try:
        # create a handle to the add_two_ints service
        Speed_mode_client = rospy.ServiceProxy('speed_mode', speed_mode)
        speed_mode_feedback = Speed_mode_client(Speedmode)
        speed_mode_feedback_times = speed_mode_feedback.feedback
        return speed_mode_feedback_times
    except rospy.ServiceException as e:
        print ("Service call failed: %s"%e)
##---------strategy_client grip mode data
def strategy_client_grip_Mode(grip):
    global grip_mode_feedback_times

    rospy.wait_for_service('grip_mode')
    try:
        # create a handle to the add_two_ints service
        grip_mode_client = rospy.ServiceProxy('grip_mode', grip_mode)
        grip_mode_feedback = grip_mode_client(grip)
        grip_mode_feedback_times = grip_mode_feedback.feedback
        return grip_mode_feedback_times
    except rospy.ServiceException as e:
        print ("Service call failed: %s"%e)
##------------client end-------
##------------class-------
class point():
    def __init__(self,x,y,z,pitch,roll,yaw):
        self.x = x
        self.y = y
        self.z = z
        self.pitch = pitch
        self.roll = roll
        self.yaw = yaw

    #action: ptp line
    #ra : abs rel
    #grip 夾爪
    #vel speed
    #both : Ctrl_Mode
##-------------strategy end ------------
def myhook():
    print ("shutdown time!")

if __name__ == '__main__':
    argv = rospy.myargv()
    rospy.init_node('arm_ros', anonymous=True)
    rospy.spin()