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
import listener as listen
import enum
import argparse

def Param():
    global PushBallHeight,ObjAboveHeight
    PushBallHeight = rospy.get_param("~Push_Height")
    ObjAboveHeight = rospy.get_param("~Above_Height")
##------yoloV3
# from turtlesim.msg   import Pose    
# from ROS_Socket.msg   import  ROI_array 
NAME = 'strategy_server'
pos_feedback_times = 0
mode_feedback_times = 0
msg_feedback = 1
Arm_state_flag = 0
strategy_flag = 0
arm_move_times = 1
#-------yolov3 data
# obj_num = 0
# object_name = ''
# score = 0
# min_xy = 0
# max_xy = 0
# pic_times = 0
# ball_mid = 0
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
# def get_obj_info_cb(data):
#     global object_name,min_xy,max_xy,obj_num,score,pic_times,ball_mid,SpecifyBall_mid,CueBalll_mid
#     global GetInfoFlag
#     print("\n========== Detected object number = " + str(len(data.ROI_list)) + " ========== ")
#     for obj_num in range(len(data.ROI_list)):
#         object_name = data.ROI_list[obj_num].object_name
#         score       = data.ROI_list[obj_num].score
#         min_xy = np.array([data.ROI_list[obj_num].min_x, data.ROI_list[obj_num].min_y])
#         max_xy = np.array([data.ROI_list[obj_num].Max_x, data.ROI_list[obj_num].Max_y])

#         if(obj_num!=0):
#             print("\n")
#         print("----- object_" + str(obj_num) + " ----- ")
#         print("object_name = " + str(object_name))
#         print("score = " + str(score))
#         print("min_xy = [ " +  str(min_xy) +  " ]" )
#         print("max_xy = [ " +  str(max_xy) +  " ]" )

#         print("mid_xy = ["+str(min_xy+max_xy)+"]")
#     ##-- yolov3 info
#     ## for 取 10張 信心值超過70%  
#     ## 取出位置取平均

#         if  str(len(data.ROI_list)) == 2 and score >= 70:
#             pic_times+=1
#             if object_name == "Specify":
#                 SpecifyBall_mid = np.array([SpecifyBall_mid + (min_xy+max_xy)/2])
#             if object_name == "Nine":
#                 CueBalll_mid = np.array([CueBalll_mid[1] +  (min_xy+max_xy)/2])
#             if pic_times == 10:
#                 SpecifyBall_mid = SpecifyBall_mid/50
#                 CueBalll_mid = CueBalll_mid/50
#                 ##image to real
#                 # SpecifyBall_mid = SpecifyBall_mid
#                 # CueBalll_mid = CueBalll_mid
#                 GetInfoFlag = True
#     pic_times+=1
##-------------------------strategy---------------------
## 球桌與球與洞口參數
# UpLeft_X = -30
# UpLeft_Y = 57.8
# UpRight_X = 30
# UpRight_Y = 57.8
# DownLeft_X = -30
# DownLeft_Y = 17.8
# DownRight_X = 30
# DownRight_Y = 17.8
UpLeft_X = -11
UpLeft_Y = 72
UpRight_X = 50
UpRight_Y = 72
DownLeft_X = -11
DownLeft_Y = 43
DownRight_X = 50
DownRight_Y = 43

Ball_radius = 1.56
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
PushBallHeight = 3
ObjAboveHeight = 11
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
def Billiards_Calculation():
    global angle_SubCue,HoleState,SpecifyBall_mid
    # SpecifyBall=Item(SpecifyBall_mid[0],SpecifyBall_mid[1],0)#母球
    # CueBall=Item(CueBalll_mid[0],CueBalll_mid[1],9)#子球
    
    #test
    SpecifyBall=Item(33.5,57,0)#子球
    CueBall=Item(10.5,57,9)#母球
    if SpecifyBall.x <= ((UpLeft_X+UpRight_X)/2) and SpecifyBall.y >= ((UpLeft_Y+DownLeft_Y)/2):
        HoleState = 0
        Hole_X = UpLeft_X
        Hole_Y = UpLeft_Y
    elif SpecifyBall.x > ((UpLeft_X+UpRight_X)/2) and SpecifyBall.y >= ((UpRight_Y+DownRight_Y)/2):
        HoleState = 1
        Hole_X = UpRight_X
        Hole_Y = UpRight_Y
    elif SpecifyBall.x <= ((DownLeft_X+DownRight_X)/2) and SpecifyBall.y < ((UpLeft_Y+DownLeft_Y)/2):
        HoleState = 2
        Hole_X = DownLeft_X
        Hole_Y = DownLeft_Y
    elif SpecifyBall.x > ((DownLeft_X+DownRight_X)/2) and SpecifyBall.y < ((UpRight_Y+DownRight_Y)/2):
        HoleState = 3
        Hole_X = DownRight_X
        Hole_Y = DownLeft_Y
    Sub_Hole_X = SpecifyBall.x - Hole_X
    Sub_Hole_Y = SpecifyBall.y - Hole_Y
    
    if Sub_Hole_X == 0:
        angle_HoleSub = math.pi/2.0
    else:
        angle_HoleSub = math.atan(math.fabs(Sub_Hole_Y/Sub_Hole_X))
    if Sub_Hole_X <0.0 and Sub_Hole_Y >= 0.0:
        angle_HoleSub = math.pi -angle_HoleSub
    elif Sub_Hole_X <0.0 and Sub_Hole_Y <0.0:
        angle_HoleSub = math.pi +angle_HoleSub
    elif Sub_Hole_X >=0.0 and Sub_Hole_Y <0.0:
        angle_HoleSub = math.pi*2.0 -angle_HoleSub
    angle_HoleSub = angle_HoleSub*180/math.pi
    
    Ball_radius_X = 2*Ball_radius*math.cos(angle_HoleSub*math.pi/180)
    Ball_radius_Y = 2*Ball_radius*math.sin(angle_HoleSub*math.pi/180)
    Cub_Sub_X = CueBall.x - SpecifyBall.x + Ball_radius_X
    Cub_Sub_Y = CueBall.y - SpecifyBall.y + Ball_radius_Y

    if Cub_Sub_X ==0.0:
        angle_SubCue = math.pi/2.0
    else:
        angle_SubCue = math.atan(math.fabs(Cub_Sub_Y/Cub_Sub_X))
    
    if Cub_Sub_X < 0.0 and Cub_Sub_Y >= 0.0:
        angle_SubCue = math.pi - angle_SubCue
    elif Cub_Sub_X < 0.0 and Cub_Sub_Y < 0.0:
        angle_SubCue = math.pi + angle_SubCue
    elif Cub_Sub_X >=0.0 and Cub_Sub_Y < 0.0:
        angle_SubCue = math.pi*2.0 - angle_SubCue
    angle_SubCue = angle_SubCue*180/math.pi
    ##球直徑3.12cm
    Target_pos.x = CueBall.x + 7.5*Ball_radius*math.cos(angle_SubCue*math.pi/180)
    Target_pos.y = CueBall.y + 7.5*Ball_radius*math.sin(angle_SubCue*math.pi/180)
    TargetPush_pos.x = CueBall.x
    TargetPush_pos.y = CueBall.y

    angle_SubCue = angle_SubCue -90;#轉換手臂角度 ROLL
	##---轉換角度從指向母球至指向子球
    if angle_SubCue >= 180:
    	angle_SubCue = angle_SubCue -180
    elif angle_SubCue <= -180:
        angle_SubCue = angle_SubCue +180
    elif angle_SubCue >0 :
        angle_SubCue = angle_SubCue -180
    elif angle_SubCue <=0:
        angle_SubCue = angle_SubCue +180
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

    Billiards_Calculation()

    GetInfoFlag = False
    GetKeyFlag = True
    ExecuteFlag = False

def GetKey_Mission():
    global GetInfoFlag,GetKeyFlag,ExecuteFlag,MotionKey,MotionSerialKey

    Mission = Get_MissionType()
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
def MissionItem(ItemNo):
    global MotionKey
    Key_PushBallCommand = [\
        ArmMotionCommand.Arm_MoveToTargetUpside,\
        ArmMotionCommand.Arm_LineDown,\
        ArmMotionCommand.Arm_PushBall,\
        ArmMotionCommand.Arm_LineUp,\
        ArmMotionCommand.Arm_Stop,\
        ]
    Key_PushBackCommand = [\
        ArmMotionCommand.Arm_MoveVision,\
        ArmMotionCommand.Arm_Stop,\
        ArmMotionCommand.Arm_StopPush,\
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
    SpeedValue = 5
    for case in switch(ItemNo): #傳送指令給socket選擇手臂動作
        if case(ArmMotionCommand.Arm_Stop):
            MoveFlag = False
            print("Arm_Stop")
            break
        if case(ArmMotionCommand.Arm_StopPush):
            MoveFlag = False
            PushFlag = True #重新掃描物件
            print("Arm_StopPush")
            break
        if case(ArmMotionCommand.Arm_MoveToTargetUpside):
            pos.x = Target_pos.x
            pos.y = Target_pos.y
            pos.z = ObjAboveHeight
            pos.pitch = -90
            pos.roll = -(angle_SubCue) #RobotArm5 
            pos.yaw = 0
            MoveFlag = True
            LinePtpFlag = False
            SpeedValue = 10
            print("Arm_MoveToTargetUpside")
            break
        if case(ArmMotionCommand.Arm_LineUp):
            pos.z = ObjAboveHeight
            MoveFlag = True
            LinePtpFlag = True
            SpeedValue = 5
            print("Arm_LineUp")
            break
        if case(ArmMotionCommand.Arm_LineDown):
            pos.z = PushBallHeight
            MoveFlag = True
            LinePtpFlag = True
            SpeedValue = 5
            print("Arm_LineDown")
            break
        if case(ArmMotionCommand.Arm_PushBall):
            pos.x = TargetPush_pos.x
            pos.y = TargetPush_pos.y
            pos.z = PushBallHeight
            pos.pitch = -90
            pos.roll = -(angle_SubCue)
            pos.yaw = 0
            SpeedValue = 100   ##待測試up
            MoveFlag = True
            LinePtpFlag = False
            print("Arm_PushBall")
            break
        if case(ArmMotionCommand.Arm_MoveVision):
            pos.x = 0
            pos.y = 50
            pos.z = 20
            pos.pitch = -90
            pos.roll = 0
            pos.yaw = 0
            SpeedValue = 10
            MoveFlag = True
            LinePtpFlag = False
            ##任務結束旗標
            MissionEndFlag = True
            print("Arm_MoveVision")
            break
        if case(ArmMotionCommand.Arm_MoveFowardDown):
            pos.x = 0
            pos.y = 50
            pos.z = 20
            pos.pitch = -90
            pos.roll = 0
            pos.yaw = 0
            MoveFlag = True
            LinePtpFlag = False
            print("Arm_MoveFowardDown")
            break
        if case(): # default, could also just omit condition or 'if True'
            print ("something else!")
            # No need to break here, it'll stop anyway
    if MoveFlag == True:
        if LinePtpFlag == False:
            print('x: ',pos.x,' y: ',pos.y,' z: ',pos.z,' pitch: ',pos.pitch,' roll: ',pos.roll,' yaw: ',pos.yaw)
            strategy_client_Arm_Mode(2,1,0,SpeedValue,2)#action,ra,grip,vel,both
            strategy_client_pos_move(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
        elif LinePtpFlag == True:
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

    GetInfoFlag = True #Test no data
    
    # try:
    #     rospy.Subscriber('/object/ROI_array', ROI_array, get_obj_info_cb)
    # except rospy.ROSInterruptException:
    #     rospy.loginfo('error')
    #     pass
    Param() ##可調變參數輸入
    while 1:
        Mission_Trigger()
        
        if CurrentMissionType == MissionType.Mission_End:
            rospy.on_shutdown(myhook)
            #rospy.spin()
    rospy.spin()
