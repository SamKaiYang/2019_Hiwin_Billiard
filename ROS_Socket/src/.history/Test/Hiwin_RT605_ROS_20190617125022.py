#!/usr/bin/env python3
# license removed for brevity
#策略 機械手臂 四點來回跑
import rospy
import os
import numpy as np
from std_msgs.msg import String
from ROS_Socket.srv import *
from ROS_Socket.msg import *
import math
import enum

NAME = 'strategy_server'
pos_feedback_times = 0
mode_feedback_times = 0
msg_feedback = 1
Arm_state_flag = 0
strategy_flag = 0
arm_move_times = 1

class State_Flag():
    def __init__(self,Arm,Strategy):
        self.Arm = Arm
        self.Strategy = Strategy
    def feedback(self):
        return self.Arm,self.Strategy

## ------------enum action flag ------#
class msg_feedback_Type(enum.IntEnum):
    Idle = 0
    Isbusy = 1
    Error = 2
    shutdown = 6
    ##-------------enum actionflag
class Action_Type(enum.IntEnum):
    SetVel = 0
    Delay = 1
    PtoP = 2
    Line = 3
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
class state_flag():
    def __init__(self,Arm,Strategy):
        self.Arm = Arm
        self.Strategy = Strategy
    def feedback(self):
        return self.Arm,self.Strategy
##-----------server feedback arm state----------
def Arm_state(req):
    global CurrentMissionType
    Arm.state_flag= int('%s'%req.Arm_state)
    if Arm.state_flag == 1: #表示手臂忙碌
        Strategy.state_flag = 0
        return(1)
    if Arm.state_flag == 0: #表示手臂閒置
        Strategy.state_flag = 1
        return(0)
    if Arm.state_flag == 6: #表示程式中斷
        Strategy.state_flag = 6
        return(6)
def strategy_server():
    #rospy.init_node(NAME)
    s = rospy.Service('arm_state',arm_state, Arm_state) ##server arm point data
    #rospy.spin() #spinone

##-----------server feedback arm state end----------
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
def strategy_client_Arm_Mode(action,ra,grip,vel,both):
    global mode_feedback_times

    rospy.wait_for_service('arm_mode')
    try:
        # create a handle to the add_two_ints service
        Arm_mode_client = rospy.ServiceProxy('arm_mode', arm_mode)
        mode_feedback = Arm_mode_client(grip,ra,action,vel,both)
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
    strategy_server()
    rospy.spin()