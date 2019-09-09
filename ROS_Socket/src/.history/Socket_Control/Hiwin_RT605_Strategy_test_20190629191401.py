#!/usr/bin/env python3
# license removed for brevity
#策略 機械手臂 四點來回跑
import threading
import time
import rospy
import os
import numpy as np
from std_msgs.msg import String
from ROS_Socket.srv import *
from ROS_Socket.msg import *
import math
import enum
import Hiwin_RT605_Socket as ArmTask
#import Hiwin_RT605_Arm_Command_v3 as ArmTask # 0628
from std_msgs.msg import Int32MultiArray
##----Arm state-----------
Arm_state_flag = 0
Strategy_flag = 0
Sent_data_flag = True
##----Arm status enum
class Arm_status(enum.IntEnum):
    Idle = 0
    Isbusy = 1
    Error = 2
    shutdown = 6
def callback(state):
    global Arm_state_flag,Sent_data_flag
    Arm_state_flag = state.data[0]
    Sent_data_flag = state.data[1]
def arm_state_listener():

    rospy.Subscriber("chatter", Int32MultiArray, callback)
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

##------------class-------
class point():
    def __init__(self,x,y,z,pitch,roll,yaw):
        self.x = x
        self.y = y
        self.z = z
        self.pitch = pitch
        self.roll = roll
        self.yaw = yaw
pos = point(0,36.8,11.35,-90,0,0)


##-------------------------strategy---------------------
action = 0

def Mission_Trigger():
    global action,Arm_state_flag,Sent_data_flag
    if Arm_state_flag == Arm_status.Idle and Sent_data_flag == 1:
        Sent_data_flag = 0
        Arm_state_flag = Arm_status.Isbusy
        for case in switch(action): #傳送指令給socket選擇手臂動作
            if case(0):
                pos.x = 10
                pos.y = 36.8
                pos.z = 11.35
                pos.pitch = -90
                pos.roll = 0
                pos.yaw = 0
                action = 1
                print('x: ',pos.x,' y: ',pos.y,' z: ',pos.z,' pitch: ',pos.pitch,' roll: ',pos.roll,' yaw: ',pos.yaw)
                # ArmTask.strategy_client_pos_move(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
                # ArmTask.strategy_client_Arm_Mode(2,1,0,10,2)#action,ra,grip,vel,both
                ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
                ArmTask.Arm_Mode(2,1,0,10,2)#action,ra,grip,vel,both
                break
            if case(1):
                pos.x = 10
                pos.y = 42
                pos.z = 11.35
                pos.pitch = -90
                pos.roll = 0
                pos.yaw = 0
                action = 2
                print('x: ',pos.x,' y: ',pos.y,' z: ',pos.z,' pitch: ',pos.pitch,' roll: ',pos.roll,' yaw: ',pos.yaw)
                #ArmTask.strategy_client_pos_move(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
                #ArmTask.strategy_client_Arm_Mode(2,1,0,10,2)#action,ra,grip,vel,both
                ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
                ArmTask.Arm_Mode(2,1,0,10,2)#action,ra,grip,vel,both
                break
            if case(2):
                pos.x = -10
                pos.y = 42
                pos.z = 11.35
                pos.pitch = -90
                pos.roll = 0
                pos.yaw = 0
                action = 3
                print('x: ',pos.x,' y: ',pos.y,' z: ',pos.z,' pitch: ',pos.pitch,' roll: ',pos.roll,' yaw: ',pos.yaw)
                #ArmTask.strategy_client_pos_move(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
                #ArmTask.strategy_client_Arm_Mode(2,1,0,10,2)#action,ra,grip,vel,both
                ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
                ArmTask.Arm_Mode(2,1,0,10,2)#action,ra,grip,vel,both
                break
            if case(3):
                pos.x = -10
                pos.y = 36.8
                pos.z = 11.35
                pos.pitch = -90
                pos.roll = 0
                pos.yaw = 0
                action = 4
                print('x: ',pos.x,' y: ',pos.y,' z: ',pos.z,' pitch: ',pos.pitch,' roll: ',pos.roll,' yaw: ',pos.yaw)
                #ArmTask.strategy_client_pos_move(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
                #ArmTask.strategy_client_Arm_Mode(2,1,0,10,2)#action,ra,grip,vel,both
                ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
                ArmTask.Arm_Mode(2,1,0,10,2)#action,ra,grip,vel,both
                break
            if case(4):
                pos.x = 0
                pos.y = 36.8
                pos.z = 11.35
                pos.pitch = -90
                pos.roll = 0
                pos.yaw = 0
                action = 0
                print('x: ',pos.x,' y: ',pos.y,' z: ',pos.z,' pitch: ',pos.pitch,' roll: ',pos.roll,' yaw: ',pos.yaw)
                #ArmTask.strategy_client_pos_move(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
                #ArmTask.strategy_client_Arm_Mode(2,1,0,10,2)#action,ra,grip,vel,both
                ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
                ArmTask.Arm_Mode(2,1,0,10,2)#action,ra,grip,vel,both
                break
            if case(): # default, could also just omit condition or 'if True'
                rospy.on_shutdown(myhook)
                ArmTask.rospy.on_shutdown(myhook)

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
    rospy.init_node('strategy', anonymous=True)
    GetInfoFlag = True #Test no data
    arm_state_listener()
    start_input=int(input('開始策略請按1,離開請按3 : ')) #輸入開始指令
    start_input = 1
    if start_input==1:
        while 1:
            time.sleep(0.2) #0627 最穩定 delay 0.3秒
            # my_list=[]
            # for i in range(1000000):
            #     my_list.append(i)
            Mission_Trigger()
    if start_input == 3:
        pass
    #timer.join()
    ArmTask.rospy.spin()
    rospy.spin()
