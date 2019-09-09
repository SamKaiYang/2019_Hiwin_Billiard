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
import Hiwin_RT605_Arm_Command as ArmTask
##----Arm state-----------
Arm_state_flag = 0
Strategy_flag = 0
Sent_data_flag = False
##----Arm status enum
class Arm_status(enum.IntEnum):
    Idle = 0
    Isbusy = 1
    Error = 2
    shutdown = 6
##-----------server feedback arm state----------
def Arm_state(req):
    global CurrentMissionType,Strategy_flag,Arm_state_flag
    Arm_state_flag = int('%s'%req.Arm_state)
    if Arm_state_flag  == Arm_status.Isbusy: #表示手臂忙碌
        Strategy_flag = False
        return(1)
    if Arm_state_flag  == Arm_status.Idle: #表示手臂準備
        Strategy_flag = True
        return(0)
    if Arm_state_flag  == Arm_status.shutdown: #表示程式中斷
        Strategy_flag = 6
        return(6)
##-----------server feedback Sent_flag----------
def Sent_flag(req):
    global Sent_data_flag
    Sent_data_flag = int('%s'%req.sent_flag)
    return(1)
def arm_state_server():
    #rospy.init_node(NAME)
    s = rospy.Service('arm_state',arm_state, Arm_state) ##server arm state
    a = rospy.Service('sent_flag',sent_flag,Sent_flag)
    #rospy.spin() ## spin one
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
##-----Mission 參數
GetInfoFlag = False
ExecuteFlag = False
GetKeyFlag = False
MotionSerialKey = []
MissionType_Flag = 0
MotionStep = 0
##-----手臂動作位置資訊
angle_SubCue = 0
LinePtpFlag = False
MoveFlag = False
PushBallHeight = 6
ObjAboveHeight = 10
SpeedValue = 10
MissionEndFlag = False
CurrentMissionType = 0
##---------------Enum---------------##
class ArmMotionCommand(enum.IntEnum):
    Arm_Stop = 0
    Arm_MoveToTargetUpside = 1
    Arm_MoveFowardDown = 2
    Arm_MoveVision = 3
    Arm_PushBall = 4
    Arm_LineUp = 5
    Arm_LineDown = 6
    Arm_Angle = 7
    Arm_StopPush = 8
class MissionType(enum.IntEnum):
    Get_Img = 0
    PushBall = 1
    Pushback = 2
    Mission_End = 3
##-----------switch define------------##
# class pos():
#     def __init__(self, x, y, z, pitch, roll, yaw):
#         self.x = 0
#         self.y = 36.8
#         self.z = 11.35
#         self.pitch = -90
#         self.roll = 0
#         self.yaw = 0
class Target_pos():
    def __init__(self, x, y, z, pitch, roll, yaw):
        self.x = 0
        self.y = 36.8
        self.z = 11.35
        self.pitch = -90
        self.roll = 0
        self.yaw = 0
class TargetPush_pos():
    def __init__(self, x, y, z, pitch, roll, yaw):
        self.x = 0
        self.y = 36.8
        self.z = 11.35
        self.pitch = -90
        self.roll = 0
        self.yaw = 0
class Item():
    def __init__(self,x,y,label):
        self.x = x
        self.y = y
        self.label = label

def Mission_Trigger():
    global Arm_state_flag,Sent_data_flag
    if Arm_state_flag == Arm_status.Idle and Sent_data_flag == 1:
        for case in switch(ItemNo): #傳送指令給socket選擇手臂動作
            if case(0):
                break

        print('x: ',pos.x,' y: ',pos.y,' z: ',pos.z,' pitch: ',pos.pitch,' roll: ',pos.roll,' yaw: ',pos.yaw)
        ArmTask.strategy_client_Arm_Mode(2,1,0,SpeedValue,2)#action,ra,grip,vel,both
        ArmTask.strategy_client_pos_move(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)

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
    arm_state_server()
    while 1:
        Mission_Trigger()
        if CurrentMissionType == MissionType.Mission_End:
            ArmTask.rospy.on_shutdown(myhook)
            ArmTask.rospy.spin()
    rospy.spin()
