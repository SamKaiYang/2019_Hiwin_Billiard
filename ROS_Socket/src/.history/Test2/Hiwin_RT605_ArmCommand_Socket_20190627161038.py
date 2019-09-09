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
#接收策略端命令 用Socket傳輸至控制端電腦
import socket
##多執行序
import threading
import time
import sys
import matplotlib as plot
import HiwinRA605_socket_TCPcmd as TCP
import HiwinRA605_socket_Taskcmd as Taskcmd
Socket = 0
data = '0' #設定傳輸資料初始值
Arm_feedback = 1 #假設手臂忙碌
NAME = 'socket_server'
# client_response = 0 #回傳次數初始值
# point_data_flag = False
# arm_mode_flag = False
# speed_mode_flag = False
Socket_sent_flag = False
##------------class pos-------
class point():
    def __init__(self, x, y, z, pitch, roll, yaw):
        self.x = x
        self.y = y
        self.z = z
        self.pitch = pitch
        self.roll = roll
        self.yaw = yaw
pos = point(0,36.8,11.35,-90,0,0)
##------------class socket_cmd---------
class socket_data():
    def __init__(self, grip, setvel, ra, delay, setboth, action,Speedmode):
        self.grip = grip
        self.setvel = setvel
        self.ra = ra
        self.delay = delay
        self.setboth = setboth
        self.action = action
        self.Speedmode = Speedmode
socket_cmd = socket_data(0,0,0,0,0,0,0)
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
##-----------client feedback arm state----------
class state_feedback():
    def _init_(self,ArmState,SentFlag):
        self.ArmState = ArmState
        self.SentFlag = SentFlag

# def socket_client_arm_state(Arm_state):
#     global state_feedback

#     rospy.wait_for_service('arm_state')
#     try:
#         Arm_state_client = rospy.ServiceProxy('arm_state', arm_state)
#         state_feedback = Arm_state_client(Arm_state)
#         #pos_feedback_times = pos_feedback.response
#         return state_feedback
#     except rospy.ServiceException as e:
#         print ("Service call failed: %s"%e)
# ##----------socket sent data flag-------------
# def socket_client_sent_flag(Sent_flag):
#     global sent_feedback

#     rospy.wait_for_service('sent_flag')
#     try:
#         Sent_flag_client = rospy.ServiceProxy('sent_flag', sent_flag)
#         sent_feedback = Sent_flag_client(Sent_flag)
#         #pos_feedback_times = pos_feedback.response
#         return sent_feedback
#     except rospy.ServiceException as e:
#         print ("Service call failed: %s"%e)
##-----------client feedback arm state end----------
##------------server 端-------
def point_data(x,y,z,pitch,roll,yaw): ##接收策略端傳送位姿資料
    global client_response,point_data_flag
    pos.x = x
    pos.y = y
    pos.z = z
    pos.pitch = pitch
    pos.roll = roll
    pos.yaw = yaw
    point_data_flag = True
##----------Arm Mode-------------###
def Arm_Mode(action,grip,ra,setvel,setboth): ##接收策略端傳送手臂模式資料
    global arm_mode_flag
    socket_cmd.action = action
    socket_cmd.grip = grip
    socket_cmd.ra = ra
    socket_cmd.setvel = setvel
    socket_cmd.setboth = setboth
    arm_mode_flag = True
    Socket_command()
##-------Arm Speed Mode------------###
def Speed_Mode(speedmode): ##接收策略端傳送手臂模式資料
    global speed_mode_flag
    socket_cmd.Speedmode = speedmode
    speed_mode_flag = True
# def Grip_Mode(req): ##接收策略端傳送夾爪動作資料
#     socket_cmd.grip = int('%s'%req.grip)
#     return(1)
def socket_server(): ##創建Server node
    rospy.init_node(NAME)
    # a = rospy.Service('arm_mode',arm_mode, Arm_Mode) ##server arm mode data
    # s = rospy.Service('arm_pos',arm_data, point_data) ##server arm point data
    # b = rospy.Service('speed_mode',speed_mode, Speed_Mode) ##server speed mode data
    #c = rospy.Service('grip_mode',grip_mode, Grip_Mode) ##server grip mode data
    print ("Ready to connect")
    rospy.spin() ## spin one
##------------server 端 end-------
##----------socket 封包傳輸--------------##

 ##---------------socket 傳輸手臂命令-----------------
def Socket_command():
    global arm_mode_flag,speed_mode_flag,point_data_flag
    if arm_mode_flag ==  True:
        arm_mode_flag = False
        for case in switch(socket_cmd.action):
            #-------PtP Mode--------
            if case(Taskcmd.Action_Type.PtoP):
                for case in switch(socket_cmd.setboth):
                    if case(Taskcmd.Ctrl_Mode.CTRL_POS):
                        data = TCP.SetPtoP(socket_cmd.grip,Taskcmd.RA.ABS,Taskcmd.Ctrl_Mode.CTRL_POS,pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw,socket_cmd.setvel)
                        break
                    if case(Taskcmd.Ctrl_Mode.CTRL_EULER):
                        data = TCP.SetPtoP(socket_cmd.grip,Taskcmd.RA.ABS,Taskcmd.Ctrl_Mode.CTRL_EULER,pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw,socket_cmd.setvel)
                        break
                    if case(Taskcmd.Ctrl_Mode.CTRL_BOTH):
                        data = TCP.SetPtoP(socket_cmd.grip,Taskcmd.RA.ABS,Taskcmd.Ctrl_Mode.CTRL_BOTH,pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw,socket_cmd.setvel)
                        break
                break
            #-------Line Mode--------
            if case(Taskcmd.Action_Type.Line):
                for case in switch(socket_cmd.setboth):
                    if case(Taskcmd.Ctrl_Mode.CTRL_POS):
                        data = TCP.SetLine(socket_cmd.grip,Taskcmd.RA.ABS,Taskcmd.Ctrl_Mode.CTRL_POS,pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw,socket_cmd.setvel)
                        break
                    if case(Taskcmd.Ctrl_Mode.CTRL_EULER):
                        data = TCP.SetLine(socket_cmd.grip,Taskcmd.RA.ABS,Taskcmd.Ctrl_Mode.CTRL_EULER,pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw,socket_cmd.setvel )
                        break
                    if case(Taskcmd.Ctrl_Mode.CTRL_BOTH):
                        data = TCP.SetLine(socket_cmd.grip,Taskcmd.RA.ABS,Taskcmd.Ctrl_Mode.CTRL_BOTH,pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw,socket_cmd.setvel )
                        break
                break
            #-------設定手臂速度--------
            if case(Taskcmd.Action_Type.SetVel):
                data = TCP.SetVel(socket_cmd.grip, socket_cmd.setvel)
                break
            #-------設定手臂Delay時間--------
            if case(Taskcmd.Action_Type.Delay):
                data = TCP.SetDelay(socket_cmd.grip,0)
                break
            #-------設定手臂急速&安全模式--------
            if case(Taskcmd.Action_Type.Mode):
                data = TCP.Set_SpeedMode(socket_cmd.grip,socket_cmd.Speedmode)
                break
        socket_cmd.action= 5 ##切換初始mode狀態
        Socket.send(data.encode('utf-8'))#socket傳送for python to translate str
        # Socket_sent_flag = True
        # socket_client_sent_flag(Socket_sent_flag)
##-----------socket client--------
def socket_client():
    global Socket,Arm_feedback,data,Socket_sent_flag
    try:
        Socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        Socket.connect(('192.168.0.1', 8080))#iclab 5 ＆ iclab hiwin
        #s.connect(('192.168.1.102', 8080))#iclab computerx
    except socket.error as msg:
        print(msg)
        sys.exit(1)
    print('Connection has been successful')
    print(Socket.recv(1024))

    while 1:
        feedback_str = Socket.recv(1024)
        #手臂端傳送手臂狀態
        if str(feedback_str[2]) == '48':# F 手臂為Ready狀態準備接收下一個運動指令
            Arm_feedback = 0
            socket_client_arm_state(Arm_feedback)
            #print("isbusy false")
        if str(feedback_str[2]) == '49':# T 手臂為忙碌狀態無法執行下一個運動指令
            Arm_feedback = 1
            socket_client_arm_state(Arm_feedback)
            #print("isbusy true")
        if str(feedback_str[2]) == '54':# 6 策略完成
            Arm_feedback = 6
            socket_client_arm_state(Arm_feedback)
            print("shutdown")
        #確認傳送旗標
        if str(feedback_str[4]) == '48':#回傳0 false
            #print(2222222222)
            Socket_sent_flag = False
            socket_client_sent_flag(Socket_sent_flag)
        if str(feedback_str[4]) == '49':#回傳1 true
            #print(111111111111)
            Socket_sent_flag = True
            socket_client_sent_flag(Socket_sent_flag)
    ##---------------socket 傳輸手臂命令 end-----------------
        if Arm_feedback == Taskcmd.Arm_feedback_Type.shutdown:
            break

    rospy.on_shutdown(myhook)
    Socket.close()
##-----------socket client end--------
##-------------socket 封包傳輸 end--------------##
## 多執行緒
def thread_test():
    socket_client()
## 多執行序 end
def myhook():
    print ("shutdown time!")
if __name__ == '__main__':
    socket_cmd.action = 5##切換初始mode狀態
    t = threading.Thread(target=thread_test)
    t.start() # 開啟多執行緒
    socket_server()
    t.join()