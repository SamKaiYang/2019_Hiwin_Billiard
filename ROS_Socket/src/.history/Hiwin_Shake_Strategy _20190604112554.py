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

##------yoloV3
from turtlesim.msg   import Pose    
from ROS_Socket.msg   import  ROI_array 
NAME = 'strategy_server'
pos_feedback_times = 0
mode_feedback_times = 0
msg_feedback = 1
Arm_state_flag = 0
strategy_flag = 0
arm_move_times = 1
#-------yolov3 data
obj_num = 0
object_name = ''
score = 0
min_xy = 0
max_xy = 0
pic_times = 0
ball_mid = 0
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
##-----------server feedback arm state----------
def Arm_state(req):
    global strategy_flag,Arm_state_flag,CurrentMissionType
    Arm_state_flag = int('%s'%req.Arm_state)
    if Arm_state_flag == 1: #表示手臂忙碌
        strategy_flag = 0
        return(1)
    if Arm_state_flag == 0: #表示手臂閒置
        strategy_flag = 1
        return(0)
    if Arm_state_flag == 6: #表示程式中斷
        strategy_flag = 6
        return(6)   
    if CurrentMissionType == MissionType.Mission_End:
        return('socket stop')
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
##------------------yolo_v3 stra.py-----------------
def get_obj_info_cb(data):
    global object_name,min_xy,max_xy,obj_num,score,pic_times,ball_mid,SpecifyBall_mid,CueBalll_mid
    global GetInfoFlag
    print("\n========== Detected object number = " + str(len(data.ROI_list)) + " ========== ")
    for obj_num in range(len(data.ROI_list)):
        object_name = data.ROI_list[obj_num].object_name
        score       = data.ROI_list[obj_num].score
        min_xy = np.array([data.ROI_list[obj_num].min_x, data.ROI_list[obj_num].min_y])
        max_xy = np.array([data.ROI_list[obj_num].Max_x, data.ROI_list[obj_num].Max_y])

        if(obj_num!=0):
            print("\n")
        print("----- object_" + str(obj_num) + " ----- ")
        print("object_name = " + str(object_name))
        print("score = " + str(score))
        print("min_xy = [ " +  str(min_xy) +  " ]" )
        print("max_xy = [ " +  str(max_xy) +  " ]" )

        print("mid_xy = ["+str(min_xy+max_xy)+"]")

        if  str(len(data.ROI_list)) == 2 and score >= 70:
            pic_times+=1
            if object_name == "Specify":
                SpecifyBall_mid = np.array([SpecifyBall_mid + (min_xy+max_xy)/2])
            if object_name == "Nine":
                CueBalll_mid = np.array([CueBalll_mid[1] +  (min_xy+max_xy)/2])
            if pic_times == 10:
                SpecifyBall_mid = SpecifyBall_mid/50
                CueBalll_mid = CueBalll_mid/50
                ##image to real
                # SpecifyBall_mid = SpecifyBall_mid
                # CueBalll_mid = CueBalll_mid
                GetInfoFlag = True
    pic_times+=1
##-------------------------strategy---------------------
## 球桌與球與洞口參數
UpLeft_X = -30
UpLeft_Y = 57.8
UpRight_X = 30
UpRight_Y = 57.8
DownLeft_X = -30
DownLeft_Y = 17.8
DownRight_X = 30
DownRight_Y = 17.8
Ball_radius = 2.75
HoleState = 0
Hole_X = 0
Hole_Y = 0
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
PushBallHeight = 20
ObjAboveHeight = 20
SpeedValue = 10
MissionEndFlag = False
CurrentMissionType = 0
###------------------搖飲料任務動作------
# 1.將茶夾起
# 2.移動到雪客杯上方
# 3.倒入茶 轉角度roll
# 4.鬆開夾指放置茶
# 5.按一下 按壓瓶
# 6.移至冰塊上方 進行夾取
# 7.重複上述動作兩次(冰塊兩塊)
# 8.夾取蓋子 
# 9.搖飲料roll
# 10.開蓋子

##---------------Enum---------------##
class ArmMotionCommand(enum.IntEnum):
    Arm_Stop = 0
    Arm_MoveTo_Tea_Upside = 9
    Arm_LineDownTo_Tea =10
    Arm_Catch_tea = 11
    Arm_LineUpTo_Shake = 12
    Arm_Move_Tea_To_Shake = 13
    Arm_Pour = 14
    Arm_Pour_Back = 15
    Arm_MoveTo_Press = 16
    Arm_MoveTo_Ice_Upside = 17
    Arm_LineDownTo_Ice = 18
    Arm_Move_Ice_To_Shake = 19
    Arm_MoveTo_Cover_Upside = 20
    Arm_LineDownTo_Cover = 21
    Arm_MoveToShake = 22
    Arm_LineDownTo_Shake = 23
    Arm_ShakeTea_roll = 24
    Arm_Gripper_Loose = 25
    Arm_Gripper_catch = 26
    Arm_Gripper_Home  = 27
    
    
class MissionType(enum.IntEnum):
    Get_Img = 0
    PushBall = 1
    Pushback = 2
    Mission_End = 3
##-----------switch define------------##
class pos():
    def __init__(self, x, y, z, pitch, roll, yaw):
        self.x = 0
        self.y = 50
        self.z = 20
        self.pitch = -90
        self.roll = 0
        self.yaw = 0 
class Target_pos():
    def __init__(self, x, y, z, pitch, roll, yaw):
        self.x = 0
        self.y = 50
        self.z = 20
        self.pitch = -90
        self.roll = 0
        self.yaw = 0 
class TargetPush_pos():
    def __init__(self, x, y, z, pitch, roll, yaw):
        self.x = 0
        self.y = 50
        self.z = 20
        self.pitch = -90
        self.roll = 0
        self.yaw = 0 
class Item():
    def __init__(self,x,y,label):
        self.x = x
        self.y = y
        self.label = label
def Shake_identify():
    global angle_SubCue,HoleState,SpecifyBall_mid
    ###------語音辨識
    Target_pos.x = 10
    Target_pos.y = 10
    TargetPush_pos.x = 10
    TargetPush_pos.y = 50
    angle_SubCue = 30.3
    return TargetPush_pos,Target_pos,angle_SubCue
def Mission_Trigger():
    if GetInfoFlag == True and GetKeyFlag == False and ExecuteFlag == False:
        GetInfo_Mission()
    if GetInfoFlag == False and GetKeyFlag == True and ExecuteFlag == False:
        GetKey_Mission()
    if GetInfoFlag == False and GetKeyFlag == False and ExecuteFlag == True:
        Execute_Mission()
    
    
def GetInfo_Mission():
    global GetInfoFlag,GetKeyFlag,ExecuteFlag
    Shake_identify()
    GetInfoFlag = False
    GetKeyFlag = True
    ExecuteFlag = False
def Image_Information():
     ##-- yolov3 info
    global object_name,min_xy,max_xy ,obj_num,score
    ## for 取 50張 信心值超過70%  
    ## 取出位置取平均

    
    print("----- object_" + str(obj_num) + " ----- ")
    print("object_name = " + str(object_name))
    print("score = " + str(score))
    print("min_xy = [ " +  str(min_xy) +  " ]" )
    print("max_xy = [ " +  str(max_xy) +  " ]" )



def GetKey_Mission():
    global GetInfoFlag,GetKeyFlag,ExecuteFlag,MotionKey,MotionSerialKey

    Mission = Get_MissionType()
    #MotionSerialkey = MissionItem(Mission)
    MissionItem(Mission)
    MotionSerialKey = MotionKey
    GetInfoFlag = False
    GetKeyFlag = False
    ExecuteFlag = True
def Get_MissionType():
    global MissionType_Flag,CurrentMissionType
    for case in switch(MissionType_Flag): #傳送指令給socket選擇手臂動作
        if case(0):
            Type = MissionType.PushBall
            MissionType_Flag +=1
            break
        if case(1):
            Type = MissionType.Pushback
            MissionType_Flag -=1
            break
    CurrentMissionType = Type
    return Type

###------------------搖飲料任務動作------
# 1.將茶夾起
# 2.移動到雪客杯上方
# 3.倒入茶 轉角度roll
# 4.鬆開夾指放置茶
# 5.按一下 按壓瓶
# 6.移至冰塊上方 進行夾取
# 7.重複上述動作兩次(冰塊兩塊)
# 8.夾取蓋子 
# 9.搖飲料roll
# 10.開蓋子

def MissionItem(ItemNo):
    global MotionKey
    Key_PushBallCommand = [\
        ArmMotionCommand.Arm_MoveTo_Tea_Upside,\
        ArmMotionCommand.Arm_LineDownTo_Tea,\
        ArmMotionCommand.Arm_Catch_tea,\
        ArmMotionCommand.Arm_LineUpTo_Shake,\
        ArmMotionCommand.Arm_Move_Tea_To_Shake,\
        ArmMotionCommand.Arm_Pour,\
        ArmMotionCommand.Arm_Pour_Back,\
        ArmMotionCommand.Arm_MoveTo_Press,\
        ArmMotionCommand.Arm_MoveTo_Ice_Upside,\
        ArmMotionCommand.Arm_LineDownTo_Ice,\
        ArmMotionCommand.Arm_Gripper_catch,\
        ArmMotionCommand.Arm_Move_Ice_To_Shake,\
        ArmMotionCommand.Arm_Gripper_Loose,\
        ArmMotionCommand.Arm_Gripper_Home,\
        ArmMotionCommand.Arm_MoveTo_Ice_Upside,\
        ArmMotionCommand.Arm_LineDownTo_Ice,\
        ArmMotionCommand.Arm_Gripper_catch,\
        ArmMotionCommand.Arm_Move_Ice_To_Shake,\
        ArmMotionCommand.Arm_Gripper_Loose,\
        ArmMotionCommand.Arm_Gripper_Home,\
        ArmMotionCommand.Arm_Stop
        ]
    Key_PushBackCommand = [\
        ArmMotionCommand.Arm_MoveTo_Cover_Upside,\
        ArmMotionCommand.Arm_LineDownTo_Cover,\
        ArmMotionCommand.Arm_Gripper_catch,\
        ArmMotionCommand.Arm_MoveToShake,\
        ArmMotionCommand.Arm_LineDownTo_Shake,\
        ArmMotionCommand.Arm_Gripper_Loose,\
        ArmMotionCommand.Arm_ShakeTea_roll,\
        ArmMotionCommand.Arm_Stop,\
        ]
    for case in switch(ItemNo): #傳送指令給socket選擇手臂動作
        if case(MissionType.PushBall):
            MotionKey = Key_PushBallCommand
            break
        if case(MissionType.Pushback):
            MotionKey = Key_PushBackCommand
            break
    return MotionKey
def Execute_Mission():
    global GetInfoFlag,GetKeyFlag,ExecuteFlag,MotionKey,MotionStep,MotionSerialKey,MissionEndFlag,CurrentMissionType
    global strategy_flag,Arm_state_flag
    if Arm_state_flag == 0 and strategy_flag == 1:
        strategy_flag = 0
        if MotionKey[MotionStep] == ArmMotionCommand.Arm_Stop:
            if MissionEndFlag == True:
                CurrentMissionType = MissionType.Mission_End
                GetInfoFlag = False
                GetKeyFlag = False
                ExecuteFlag = False
                print("Mission_End")
            elif CurrentMissionType == MissionType.PushBall:
                GetInfoFlag = False
                GetKeyFlag = True
                ExecuteFlag = False
                MotionStep = 0
                print("PushBall")
            else:
                GetInfoFlag = True
                GetKeyFlag = False
                ExecuteFlag = False
                MotionStep = 0
        else:
            MotionItem(MotionSerialKey[MotionStep])
            MotionStep += 1
    
def MotionItem(ItemNo):
    global angle_SubCue,SpeedValue,PushFlag,LinePtpFlag,MissionEndFlag
    SpeedValue = 10
    for case in switch(ItemNo): #傳送指令給socket選擇手臂動作
        if case(ArmMotionCommand.Arm_Stop):
            MoveFlag = False
            print("Arm_Stop")
            break
        ##----------Skake Strategy------------##
        if case(ArmMotionCommand.Arm_MoveTo_Tea_Upside):
            pos.x = 0
            pos.y = 50
            pos.z = 30
            pos.pitch = -90
            pos.roll = 0
            pos.yaw = 0
            MoveFlag = True
            LinePtpFlag = False
            print("Arm_MoveTo_Tea_Upside")
            break
        if case(ArmMotionCommand.Arm_LineDownTo_Tea):
            pos.x = 0
            pos.y = 50
            pos.z = 30
            pos.pitch = -90
            pos.roll = 0
            pos.yaw = 0
            MoveFlag = True
            LinePtpFlag = False
            print("Arm_LineDownTo_Tea")
            break
        if case(ArmMotionCommand.Arm_Catch_tea):
            pos.x = 0
            pos.y = 50
            pos.z = 30
            pos.pitch = -90
            pos.roll = 0
            pos.yaw = 0
            MoveFlag = True
            LinePtpFlag = False
            print("Arm_Catch_tea")
            break
        if case(ArmMotionCommand.Arm_LineUpTo_Shake):
            pos.x = 0
            pos.y = 50
            pos.z = 30
            pos.pitch = -90
            pos.roll = 0
            pos.yaw = 0
            MoveFlag = True
            LinePtpFlag = False
            print("Arm_LineUpTo_Shake")
            break
        if case(ArmMotionCommand.Arm_Move_Tea_To_Shake):
            pos.x = 0
            pos.y = 50
            pos.z = 30
            pos.pitch = -90
            pos.roll = 0
            pos.yaw = 0
            MoveFlag = True
            LinePtpFlag = False
            print("Arm_Move_Tea_To_Shake")
            break
        if case(ArmMotionCommand.Arm_Pour):
            pos.x = 0
            pos.y = 50
            pos.z = 30
            pos.pitch = -90
            pos.roll = 0
            pos.yaw = 0
            MoveFlag = True
            LinePtpFlag = False
            print("Arm_Pour")
            break
        if case(ArmMotionCommand.Arm_Pour_Back):
            pos.x = 0
            pos.y = 50
            pos.z = 30
            pos.pitch = -90
            pos.roll = 0
            pos.yaw = 0
            MoveFlag = True
            LinePtpFlag = False
            print("Arm_Pour_Back")
            break
        if case(ArmMotionCommand.Arm_MoveTo_Press):
            pos.x = 0
            pos.y = 50
            pos.z = 30
            pos.pitch = -90
            pos.roll = 0
            pos.yaw = 0
            MoveFlag = True
            LinePtpFlag = False
            print("Arm_MoveTo_Press")
            break
        if case(ArmMotionCommand.Arm_MoveTo_Ice_Upside):
            pos.x = 0
            pos.y = 50
            pos.z = 30
            pos.pitch = -90
            pos.roll = 0
            pos.yaw = 0
            MoveFlag = True
            LinePtpFlag = False
            print("Arm_MoveTo_Ice_Upside")
            break
        if case(ArmMotionCommand.Arm_LineDownTo_Ice):
            pos.x = 0
            pos.y = 50
            pos.z = 30
            pos.pitch = -90
            pos.roll = 0
            pos.yaw = 0
            MoveFlag = True
            LinePtpFlag = False
            print("Arm_LineDownTo_Ice")
            break
        if case(ArmMotionCommand.Arm_Move_Ice_To_Shake):
            pos.x = 0
            pos.y = 50
            pos.z = 30
            pos.pitch = -90
            pos.roll = 0
            pos.yaw = 0
            MoveFlag = True
            LinePtpFlag = False
            print("Arm_Move_Ice_To_Shake")
            break
        if case(ArmMotionCommand.Arm_MoveTo_Cover_Upside):
            pos.x = 0
            pos.y = 50
            pos.z = 30
            pos.pitch = -90
            pos.roll = 0
            pos.yaw = 0
            MoveFlag = True
            LinePtpFlag = False
            print("Arm_MoveTo_Cover_Upside")
            break
        if case(ArmMotionCommand.Arm_LineDownTo_Cover):
            pos.x = 0
            pos.y = 50
            pos.z = 30
            pos.pitch = -90
            pos.roll = 0
            pos.yaw = 0
            MoveFlag = True
            LinePtpFlag = False
            print("Arm_LineDownTo_Cover")
            break
        if case(ArmMotionCommand.Arm_MoveToShake):
            pos.x = 0
            pos.y = 50
            pos.z = 30
            pos.pitch = -90
            pos.roll = 0
            pos.yaw = 0
            MoveFlag = True
            LinePtpFlag = False
            print("Arm_MoveToShake")
            break
        if case(ArmMotionCommand.Arm_LineDownTo_Shake):
            pos.x = 0
            pos.y = 50
            pos.z = 30
            pos.pitch = -90
            pos.roll = 0
            pos.yaw = 0
            MoveFlag = True
            LinePtpFlag = False
            print("Arm_LineDownTo_Shake")
            break
        if case(ArmMotionCommand.Arm_ShakeTea_roll):
            pos.x = 0
            pos.y = 50
            pos.z = 30
            pos.pitch = -90
            pos.roll = 0
            pos.yaw = 0
            MoveFlag = True
            LinePtpFlag = False
            print("Arm_ShakeTea_roll")
            break
        if case(ArmMotionCommand.Arm_Gripper_Loose):
            pos.x = 0
            pos.y = 50
            pos.z = 30
            pos.pitch = -90
            pos.roll = 0
            pos.yaw = 0
            MoveFlag = True
            LinePtpFlag = False
            print("Arm_Gripper_Loose")
            break
        if case(ArmMotionCommand.Arm_Gripper_catch):
            pos.x = 0
            pos.y = 50
            pos.z = 30
            pos.pitch = -90
            pos.roll = 0
            pos.yaw = 0
            MoveFlag = True
            LinePtpFlag = False
            print("Arm_Gripper_catch")
            break
        if case(ArmMotionCommand.Arm_Gripper_Home):
            pos.x = 0
            pos.y = 50
            pos.z = 30
            pos.pitch = -90
            pos.roll = 0
            pos.yaw = 0
            MoveFlag = True
            LinePtpFlag = False
            print("Arm_Gripper_Home")
            break
        if case(): # default, could also just omit condition or 'if True'
            print ("something else!")
            # No need to break here, it'll stop anyway
    if MoveFlag == True:
        if LinePtpFlag == False:
            SpeedValue = 10
            print('x: ',pos.x,' y: ',pos.y,' z: ',pos.z,' pitch: ',pos.pitch,' roll: ',pos.roll,' yaw: ',pos.yaw)
            strategy_client_Arm_Mode(2,1,0,SpeedValue,2)#action,ra,grip,vel,both
            strategy_client_pos_move(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
        elif LinePtpFlag == True:
            SpeedValue = 10
            print('x: ',pos.x,' y: ',pos.y,' z: ',pos.z,' pitch: ',pos.pitch,' roll: ',pos.roll,' yaw: ',pos.yaw)
            strategy_client_Arm_Mode(3,1,0,SpeedValue,2)#action,ra,grip,vel,both
            strategy_client_pos_move(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
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
    rospy.init_node('ros_socket', anonymous=True)
    strategy_server()
    #GetInfoFlag = True
    try:
        rospy.Subscriber('/object/ROI_array', ROI_array, get_obj_info_cb)
    except rospy.ROSInterruptException:
        rospy.loginfo('error')
        pass
    while 1:
        Mission_Trigger()
        
        if CurrentMissionType == MissionType.Mission_End:
            rospy.on_shutdown(myhook)
            break
    rospy.spin()