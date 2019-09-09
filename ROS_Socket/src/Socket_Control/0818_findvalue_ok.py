#!/usr/bin/env python3 
# -*- coding: utf-8 -*-                         
# license removed for brevity

# ------IMPORT FOR YOLO_PICTURE_CLIENT-----
import sys
sys.path.insert(1, "/usr/local/lib/python3.5/dist-packages/")
sys.path.insert(0, "/opt/installer/open_cv/cv_bridge/lib/python3/dist-packages/")

# import sys
# sys.path.insert(1, "/home/luca/.local/lib/python3.5/site-packages/")
# sys.path.insert(0, "/opt/installer/open_cv/cv_bridge/lib/python3/dist-packages/")
# -----------------------------------------

# -----IMPORT LIBRARIES-----
import cv2
import rospy
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from yolo_v3.srv import *
from yolo_v3.msg import ROI
from yolo_v3.msg import ROI_array
import time
from sensor_msgs.msg import Image
import os
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
from std_msgs.msg import Int32MultiArray
import tkinter
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
# --------------------------

# -----IMPORT cfg-----
from dynamic_reconfigure.server import Server
from ROS_Socket.cfg import TutorialsConfig
# --------------------

# -----Yolo_v3 PATH-----
Image_Data_Dir = os.path.dirname(os.path.realpath(__file__)) + '/Hiwin_Billiard_Image/'
current_path = os.path.dirname(os.path.abspath(__file__))
# ----------------------

# -----Yolo_v3 def-----
def call_yolo_v3_picture(inputkey):
    rospy.wait_for_service('Yolo_v3_Picture')
    yolo_v3_picture = rospy.ServiceProxy('Yolo_v3_Picture', ROI_array_srv)
    
    ROI_ARRAY = yolo_v3_picture(inputkey)
    return ROI_ARRAY

def init_yolo_picture(data):
    rospy.wait_for_service('Init_Yolo_v3_Picture')

    init_yolo_v3_picture = rospy.ServiceProxy('Init_Yolo_v3_Picture', Yolo_Picture_Init)
    OUT = init_yolo_v3_picture(data)
    return OUT.outputkey
# ---------------------

# -----Yolo_v3 class-----
class Get_image():
    
    def __init__(self):
        self.bridge = CvBridge()
        self.image = np.zeros((0,0,3), np.uint8)
        self.take_picture_counter = 0
        self.mtx = np.load(current_path + '/camera_calibration_mtx.npy')
        self.dist = np.load(current_path + '/camera_calibration_dist.npy')
        self.newcameramtx = np.load(current_path + '/camera_calibration_newcameramtx.npy')
        self.dst_roi_x, self.dst_roi_y, self.dst_roi_w, self.dst_roi_h  = np.load(current_path + '/camera_calibration_roi.npy')
        self.un_dst_img = np.zeros((0,0,3), np.uint8)
        self.image_name = ''
        rospy.Subscriber("/camera/image_color", Image, self.callback)

        if not os.path.exists(Image_Data_Dir):
            os.makedirs(Image_Data_Dir)
        time.sleep(0.5)

    def callback(self, image):
        self.cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        self.un_dst_img = cv2.undistort(self.cv_image, self.mtx, self.dist, None, self.newcameramtx)
        self.un_dst_img = self.un_dst_img[self.dst_roi_y:self.dst_roi_y+self.dst_roi_h, self.dst_roi_x:self.dst_roi_x+self.dst_roi_w]

    
    def get_image(self):
        self.image_name = Image_Data_Dir + str(self.take_picture_counter) + '.jpg'
        print("image_name", self.image_name)
        cv2.imwrite(self.image_name , self.un_dst_img)
        # cv2.imwrite(self.image_name , self.cv_image)

        self.take_picture_counter += 1
        return Image_Data_Dir + str(self.take_picture_counter - 1) + '.jpg'
# -----------------------

# -----Yolo_v3 get image-----
get_image_1 = Get_image()
# ---------------------------

##----Arm state-----------
Arm_state_flag = 0
Strategy_flag = 0
Sent_data_flag = 1
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

#----------------------strategy----------------------
#   STEP  1: 從 cfg 輸入 BMX, BMY, BMZ
#   STEP  2: 用 BMZ 取得擊球高度
#   STEP  3: 用 BMX, BMY 取得洞口相對位置
#       STEP  4: 進入 Mission_Trigger 迴圈 直到程式結束 
#           STEP  5: [case0]
#           STEP  6: 手臂移動至拍照位置
#           STEP  7: [case1]
#           STEP  8: yolo_pic 拍照 辨識 回傳 resp.Image.ROI_list ---> 座標字串
#               STEP  9: 進入 get_str_mid, 分析座標字串  
#               STEP 10: 取得球號 信心值 用信心值 大於 預期信心值 才做顯示 減少不必要的干擾 減少暫存空間的佔用 降低字串分析複雜度
#               STEP 11: 取得中心座標 母球座標 pic_M, 九號存入 pic_C, 干擾球1 存入 pic_OB_1, 干擾球2 存入pic_OB_2
#               STEP 12: 判斷九號球是否在球桌上 如果成立進入 curve, 如果不成立 重新拍照
#                   STEP 13: curve 將影像座標透過曲線擬合轉換為手臂座標
#           STEP 14: 印出 出現於 球桌桌面 上的球 其中心座標 
#           STEP 15: 進入 GetStartEnd 
#           STEP 16: 判斷球洞
#           STEP 17: 判斷是否需要kissball
#           STEP 18: 選好擊球方式
#           STEP 19: 計算擊球路徑 
#           STEP 20: 依擊球路徑判斷是否需要避障
#           STEP 21: 擊球
#           STEP 22: 回step5 
#           STEP : 
#           STEP : 
#           STEP : 
#           STEP : 
#           STEP : 
#           STEP : 
#           STEP : 
#           STEP : 
#           STEP : 
#----------------0711修正擊球的角度算式-----------------
#----------------0712選擇球洞的判斷式------------------
#----------------0713改變擊球動作流程------------------
#----------------0715改善擊球動作流程------------------
#----------------0716修正擊球位置的算式----------------
#----------------0717加入避障的算式--------------------
#----------------0718修正並加入KISS和避障的算式---------
#----------------0719加入擊球策略flag-----------------
#----------------0721two OB-----------------------------
#----------------0723加入座標轉換-------------------------

# -----BM-----
global BMX, BMY, BMZ
# 0808測試參數 會被cfg覆蓋 所以只是先定義型態
BMX = -30.7  
BMY = 35.5
BMZ = -21.6
# ------------

# -----curve fitting variable-----
global image_x, arm_x, image_y, arm_y
image_x = [974, 1238, 973]
arm_x = [0, 10, 0]
image_y = [695, 697, 432]
arm_y = [41.8, 41.8, 51.8]
global pic_M, pic_C, pic_OB_1, pic_OB_2
pic_M = [0.0, 0.0]
pic_C = [0.0, 0.0]
pic_OB_1 = [0.0, 0.0]
pic_OB_2 = [0.0, 0.0]
# --------------------------------

global action   # Mission_Trigger 中 action 初始值 讓第一次的case 從0開始
action = 0

global H1, H2, H3, H4, H5, H6
H1 = [0, 0]
H2 = [0, 0]
H3 = [0, 0]
H4 = [0, 0]
H5 = [0, 0]
H6 = [0, 0]

global gHole
gHole = [0, 0]  # 初始化 gHole

global NineOnTable  # 九號球是否在桌上 狀態
NineOnTable = 0

global billiard_radius
billiard_radius = 1.75  # billiard_radius = 1.75cm

global M
M = [0, 0]  #母球 M=(Mx,My)

global C
C = [0, 0]  #子球 C=(Cx,Cy)

global min_dif  #最小差值 （透過 check_hole 轉換 取得index值 存入num_hole） 
min_dif = 0

global num_hole # 目標洞口 洞口編號 1:左下 2:左上 3:中上 4:右上 5:右下 6:中下
num_hole = 0    # 初始值 為零

global Hole_info    # 建立一個字典 洞號 對應 洞口座標
Hole_info = {
    1: H1,
    2: H2,
    3: H3,
    4: H4,
    5: H5,
    6: H6 
}

global V, V1, V2, H_S, H_E                         #V:虛擬球,  H_S:擊球點起始位置, H_E:擊球中止位置
V = [0, 0]
V1 = [0, 0]
V2 = [0, 0]
H_S = [0, 0]
H_E = [0, 0]


global VMtoV, mvL, UVmv
global Vup, uT, n, mvL, VT
global Height
global NVMtoV, nmvL, UVnmv
global AS, ANS, Afun1, Afun2, Afun3, Afun4
global NVMtoC, nmcL, UVnmc
global KS, KNS, Kfun1, Kfun2, Kfun3, Kfun4
global VOtoV, ovL, UVov

OB_1 = [0.0, 0.0]
OB_2 = [0.0, 0.0]

# -----rqt_callback-----
def rqt_callback(config, level):
    # rospy.loginfo("""Reconfigure Request: {int_param}, {double_param},\
    #       {str_param}, {bool_param}, {size}""".format(**config))
    rospy.loginfo("""Reconfigure Request: {BMX}, {BMY}, {BMZ}""".format(**config))

    global Height
    Height = config.BMZ + 1.35
    global BMX
    BMX = config.BMX
    global BMY
    BMY = config.BMY
    global BMZ
    BMZ = config.BMZ

    # 因應 BM 找出球洞 相對位置
    global H1
    H1 = [BMX , BMY - 5.89 ]    #洞H1 左下
    global H2
    H2 = [BMX , BMY + 23.28 ]   #洞H2 左上
    global H3
    H3 = [BMX + 30.81, BMY + 23.28 ]   #洞H3 中上
    global H4
    H4 = [BMX + 61.62, BMY + 23.28 ]   #洞H4 右上
    global H5
    H5 = [BMX + 61.62, BMY - 5.89 ]    #洞H5 右下
    global H6
    H6 = [BMX + 30.81, BMY - 5.89 ]    #洞H6 中下



    
    return config
# ----------------------


# -----curve fitting-----
def curve():
    global image_x  
    I_X = np.array(image_x)
    global arm_x    
    A_X = np.array(arm_x)
    fx = np.polyfit(I_X, A_X, 1)              # x最高次方係數是1
    print(fx)
    cfx = np.poly1d(fx)                       # 產生出線性方程式
    print(cfx)

    global image_y  
    I_Y = np.array(image_y)
    global arm_y 
    A_Y = np.array(arm_y)
    fy = np.polyfit(I_Y, A_Y, 1)  
    print('fy:',fy)
    cfy = np.poly1d(fy)  
    print('cfy:',cfy) 
    print("=========================================")                     

    global M, C, OB_1, OB_2
    global pic_M, pic_C, pic_OB_1, pic_OB_2

    M[0] = cfx(pic_M[0])
    M[1] = cfy(pic_M[1])
    C[0] = cfx(pic_C[0])
    C[1] = cfy(pic_C[1])
    OB_1[0] = cfx(pic_OB_1[0])
    OB_1[1] = cfy(pic_OB_1[1])
    OB_2[0] = cfx(pic_OB_2[0])
    OB_2[1] = cfy(pic_OB_2[1])

    CamPosX = BMX + 30.7 
    CamPosY = BMY + 2 
    print('123456789CAM:',CamPosX,CamPosY)
    print(M)
    if M[0] < CamPosX-25 :
        if M[1] < CamPosY :
            M[0] = M[0] + 1.55
            M[1] = M[1] + 1.4
        if M[1] >= CamPosY and M[0] < CamPosY+5 :
            M[0] = M[0] + 1.85
            M[1] = M[1] + 1.36
        if M[1] >= CamPosY+5 and M[0] < CamPosY+10 :
            M[0] = M[0] + 0.13
            M[1] = M[1] - 0.13
        if M[1] >= CamPosY+10 and M[0] < CamPosY+15 :
            M[0] = M[0] + 0.21
            M[1] = M[1] - 0.38
        if M[1] >= CamPosY+15 and M[0] < CamPosY+20 :
            M[0] = M[0] - 0.02
            M[1] = M[1] - 0.04
        if M[1] >= CamPosY+20 :
            M[0] = M[0] - 0.02
            M[1] = M[1] - 0.04

    if M[0] >= CamPosX-25 and M[0] < CamPosX-20:
        if M[1] < CamPosY :
            M[0] = M[0] + 1.22
            M[1] = M[1] + 1.1
        if M[1] >= CamPosY and M[0] < CamPosY+5 :
            M[0] = M[0] + 1.46
            M[1] = M[1] + 0.96
        if M[1] >= CamPosY+5 and M[0] < CamPosY+10 :
            M[0] = M[0] + 1.44
            M[1] = M[1] + 1.21
        if M[1] >= CamPosY+10 and M[0] < CamPosY+15 :
            M[0] = M[0] + 1.88
            M[1] = M[1] + 0.85
        if M[1] >= CamPosY+15 and M[0] < CamPosY+20 :
            M[0] = M[0] + 2.01
            M[1] = M[1] + 0.62
        if M[1] >= CamPosY+20 :
            M[0] = M[0] + 2.01
            M[1] = M[1] + 0.62

    if M[0] >= CamPosX-20 and M[0] < CamPosX-15:
        if M[1] < CamPosY :
            M[0] = M[0] + 1.23
            M[1] = M[1] + 1.04
        if M[1] >= CamPosY and M[0] < CamPosY+5 :
            M[0] = M[0] + 1.21
            M[1] = M[1] + 0.98
        if M[1] >= CamPosY+5 and M[0] < CamPosY+10 :
            M[0] = M[0] + 0.15
            M[1] = M[1] - 0.22
        if M[1] >= CamPosY+10 and M[0] < CamPosY+15 :
            M[0] = M[0] + 0.04
            M[1] = M[1] - 0.23
        if M[1] >= CamPosY+15 and M[0] < CamPosY+20 :
            M[0] = M[0] + 0.15
            M[1] = M[1] - 0.27
        if M[1] >= CamPosY+20 :
            M[0] = M[0] + 0.15
            M[1] = M[1] - 0.27

    if M[0] >= CamPosX-15 and M[0] < CamPosX-10:
        if M[1] < CamPosY :
            M[0] = M[0] + 0.93
            M[1] = M[1] + 1.12
        if M[1] >= CamPosY and M[0] < CamPosY+5 :
            M[0] = M[0] + 0.99
            M[1] = M[1] + 0.96
        if M[1] >= CamPosY+5 and M[0] < CamPosY+10 :
            M[0] = M[0] + 1.12
            M[1] = M[1] + 0.92
        if M[1] >= CamPosY+10 and M[0] < CamPosY+15 :
            M[0] = M[0] + 1.12
            M[1] = M[1] + 0.6
        if M[1] >= CamPosY+15 and M[0] < CamPosY+20 :
            M[0] = M[0] + 1.12
            M[1] = M[1] + 0.34
        if M[1] >= CamPosY+20 :
            M[0] = M[0] + 1.12
            M[1] = M[1] + 0.34

    if M[0] >= CamPosX-10 and M[0] < CamPosX-5:
        if M[1] < CamPosY :
            M[0] = M[0] + 0.77
            M[1] = M[1] + 1.27
        if M[1] >= CamPosY and M[0] < CamPosY+5 :
            M[0] = M[0] + 0.79
            M[1] = M[1] + 1.1
        if M[1] >= CamPosY+5 and M[0] < CamPosY+10 :
            M[0] = M[0] - 0.02
            M[1] = M[1] - 0.23
        if M[1] >= CamPosY+10 and M[0] < CamPosY+15 :
            M[0] = M[0] + 0.04
            M[1] = M[1] - 0.17
        if M[1] >= CamPosY+15 and M[0] < CamPosY+20 :
            M[0] = M[0] + 0
            M[1] = M[1] - 0.32
        if M[1] >= CamPosY+20 :
            M[0] = M[0] + 0
            M[1] = M[1] - 0.32

    if M[0] >= CamPosX-5 and M[0] < CamPosX:
        if M[1] < CamPosY :
            M[0] = M[0] + 0.61
            M[1] = M[1] + 1.32
        if M[1] >= CamPosY and M[0] < CamPosY+5 :
            M[0] = M[0] + 0.61
            M[1] = M[1] + 1.23
        if M[1] >= CamPosY+5 and M[0] < CamPosY+10 :
            M[0] = M[0] + 0.65
            M[1] = M[1] + 1.08
        if M[1] >= CamPosY+10 and M[0] < CamPosY+15 :
            M[0] = M[0] + 0.55
            M[1] = M[1] + 0.81
        if M[1] >= CamPosY+15 and M[0] < CamPosY+20 :
            M[0] = M[0] + 0.57
            M[1] = M[1] + 0.51
        if M[1] >= CamPosY+20 :
            M[0] = M[0] + 0.57
            M[1] = M[1] + 0.51
        
    if M[0] >= CamPosX and M[0] < CamPosX+5:
        if M[1] < CamPosY :
            M[0] = M[0] + 0.47
            M[1] = M[1] + 1.17
        if M[1] >= CamPosY and M[0] < CamPosY+5 :
            M[0] = M[0] + 0.41
            M[1] = M[1] + 1.15
        if M[1] >= CamPosY+5 and M[0] < CamPosY+10 :
            M[0] = M[0] - 0.02
            M[1] = M[1] - 0.17
        if M[1] >= CamPosY+10 and M[0] < CamPosY+15 :
            M[0] = M[0] - 0.07
            M[1] = M[1] - 0.34
        if M[1] >= CamPosY+15 and M[0] < CamPosY+20 :
            M[0] = M[0] + 0
            M[1] = M[1] - 0.02
        if M[1] >= CamPosY+20 :
            M[0] = M[0] + 0
            M[1] = M[1] - 0.02

    if M[0] >= CamPosX+5 and M[0] < CamPosX+10:
        if M[1] < CamPosY :
            M[0] = M[0] + 0.38
            M[1] = M[1] + 1.23
        if M[1] >= CamPosY and M[0] < CamPosY+5 :
            M[0] = M[0] + 0.14
            M[1] = M[1] + 1.25
        if M[1] >= CamPosY+5 and M[0] < CamPosY+10 :
            M[0] = M[0] + 0.04
            M[1] = M[1] + 1.06
        if M[1] >= CamPosY+10 and M[0] < CamPosY+15 :
            M[0] = M[0] + 0.03
            M[1] = M[1] + 0.79
        if M[1] >= CamPosY+15 and M[0] < CamPosY+20 :
            M[0] = M[0] - 0.03
            M[1] = M[1] + 0.51
        if M[1] >= CamPosY+20 :
            M[0] = M[0] - 0.03
            M[1] = M[1]+ 0.51

    if M[0] >= CamPosX+10 and M[0] < CamPosX+15:
        if M[1] < CamPosY :
            M[0] = M[0] + 0.11
            M[1] = M[1] + 1.19
        if M[1] >= CamPosY and M[0] < CamPosY+5 :
            M[0] = M[0] + 0.02
            M[1] = M[1] + 1.19
        if M[1] >= CamPosY+5 and M[0] < CamPosY+10 :
            M[0] = M[0] - 0.08
            M[1] = M[1] - 0.21
        if M[1] >= CamPosY+10 and M[0] < CamPosY+15 :
            M[0] = M[0] - 0.17
            M[1] = M[1] - 0.15
        if M[1] >= CamPosY+15 and M[0] < CamPosY+20 :
            M[0] = M[0] - 0.04
            M[1] = M[1] - 0.23
        if M[1] >= CamPosY+20 :
            M[0] = M[0] - 0.04
            M[1] = M[1] - 0.23
        
    if M[0] >= CamPosX+15 and M[0] < CamPosX+20:
        if M[1] < CamPosY :
            M[0] = M[0] - 0.16
            M[1] = M[1] + 1.19
        if M[1] >= CamPosY and M[0] < CamPosY+5 :
            M[0] = M[0] - 0.24
            M[1] = M[1] + 1.06
        if M[1] >= CamPosY+5 and M[0] < CamPosY+10 :
            M[0] = M[0] - 0.33
            M[1] = M[1] + 0.96
        if M[1] >= CamPosY+10 and M[0] < CamPosY+15 :
            M[0] = M[0] - 0.45
            M[1] = M[1] + 0.83
        if M[1] >= CamPosY+15 and M[0] < CamPosY+20 :
            M[0] = M[0] - 0.64
            M[1] = M[1] + 0.62
        if M[1] >= CamPosY+20 :
            M[0] = M[0] - 0.64
            M[1] = M[1] + 0.62

    if M[0] >= CamPosX+20 and M[0] < CamPosX+25:
        if M[1] < CamPosY :
            M[0] = M[0] - 0.29
            M[1] = M[1] + 1.34
        if M[1] >= CamPosY and M[0] < CamPosY+5 :
            M[0] = M[0] - 0.55
            M[1] = M[1] + 1.23
        if M[1] >= CamPosY+5 and M[0] < CamPosY+10 :
            M[0] = M[0] - 0.19
            M[1] = M[1] - 0.13
        if M[1] >= CamPosY+10 and M[0] < CamPosY+15 :
            M[0] = M[0] - 0.17
            M[1] = M[1] - 0.33
        if M[1] >= CamPosY+15 and M[0] < CamPosY+20 :
            M[0] = M[0] - 0.15
            M[1] = M[1] - 0.17
        if M[1] >= CamPosY+20 :
            M[0] = M[0] - 0.15
            M[1] = M[1] - 0.17

    if M[0] >= CamPosX+25 :
        if M[1] < CamPosY :
            M[0] = M[0] - 0.81
            M[1] = M[1] + 1.46
        if M[1] >= CamPosY and M[0] < CamPosY+5 :
            M[0] = M[0] - 0.98
            M[1] = M[1] + 1.17
        if M[1] >= CamPosY+5 and M[0] < CamPosY+10 :
            M[0] = M[0] - 0.03
            M[1] = M[1] - 0.09
        if M[1] >= CamPosY+10 and M[0] < CamPosY+15 :
            M[0] = M[0] - 0.48
            M[1] = M[1] - 0.32
        if M[1] >= CamPosY+15 and M[0] < CamPosY+20 :
            M[0] = M[0] - 0.13
            M[1] = M[1] - 0.33
        if M[1] >= CamPosY+20 :
            M[0] = M[0] - 0.13
            M[1] = M[1] - 0.33

    if C[0] < CamPosX-25 :
        if C[1] < CamPosY :
            C[0] = C[0] + 1.55
            C[1] = C[1] + 1.4
        if C[1] >= CamPosY and C[0] < CamPosY+5 :
            C[0] = C[0] + 1.85
            C[1] = C[1] + 1.36
        if C[1] >= CamPosY+5 and C[0] < CamPosY+10 :
            C[0] = C[0] + 0.13
            C[1] = C[1] - 0.13
        if C[1] >= CamPosY+10 and C[0] < CamPosY+15 :
            C[0] = C[0] + 0.21
            C[1] = C[1] - 0.38
        if C[1] >= CamPosY+15 and C[0] < CamPosY+20 :
            C[0] = C[0] - 0.02
            C[1] = C[1] - 0.04
        if C[1] >= CamPosY+20 :
            C[0] = C[0] - 0.02
            C[1] = C[1] - 0.04

    if C[0] >= CamPosX-25 and C[0] < CamPosX-20:
        if C[1] < CamPosY :
            C[0] = C[0] + 1.22
            C[1] = C[1] + 1.1
        if C[1] >= CamPosY and C[0] < CamPosY+5 :
            C[0] = C[0] + 1.46
            C[1] = C[1] + 0.96
        if C[1] >= CamPosY+5 and C[0] < CamPosY+10 :
            C[0] = C[0] + 1.44
            C[1] = C[1] + 1.21
        if C[1] >= CamPosY+10 and C[0] < CamPosY+15 :
            C[0] = C[0] + 1.88
            C[1] = C[1] + 0.85
        if C[1] >= CamPosY+15 and C[0] < CamPosY+20 :
            C[0] = C[0] + 2.01
            C[1] = C[1] + 0.62
        if C[1] >= CamPosY+20 :
            C[0] = C[0] + 2.01
            C[1] = C[1] + 0.62

    if C[0] >= CamPosX-20 and C[0] < CamPosX-15:
        if C[1] < CamPosY :
            C[0] = C[0] + 1.23
            C[1] = C[1] + 1.04
        if C[1] >= CamPosY and C[0] < CamPosY+5 :
            C[0] = C[0] + 1.21
            C[1] = C[1] + 0.98
        if C[1] >= CamPosY+5 and C[0] < CamPosY+10 :
            C[0] = C[0] + 0.15
            C[1] = C[1] - 0.22
        if C[1] >= CamPosY+10 and C[0] < CamPosY+15 :
            C[0] = C[0] + 0.04
            C[1] = C[1] - 0.23
        if C[1] >= CamPosY+15 and C[0] < CamPosY+20 :
            C[0] = C[0] + 0.15
            C[1] = C[1] - 0.27
        if C[1] >= CamPosY+20 :
            C[0] = C[0] + 0.15
            C[1] = C[1] - 0.27

    if C[0] >= CamPosX-15 and C[0] < CamPosX-10:
        if C[1] < CamPosY :
            C[0] = C[0] + 0.93
            C[1] = C[1] + 1.12
        if C[1] >= CamPosY and C[0] < CamPosY+5 :
            C[0] = C[0] + 0.99
            C[1] = C[1] + 0.96
        if C[1] >= CamPosY+5 and C[0] < CamPosY+10 :
            C[0] = C[0] + 1.12
            C[1] = C[1] + 0.92
        if C[1] >= CamPosY+10 and C[0] < CamPosY+15 :
            C[0] = C[0] + 1.12
            C[1] = C[1] + 0.6
        if C[1] >= CamPosY+15 and C[0] < CamPosY+20 :
            C[0] = C[0] + 1.12
            C[1] = C[1] + 0.34
        if C[1] >= CamPosY+20 :
            C[0] = C[0] + 1.12
            C[1] = C[1] + 0.34

    if C[0] >= CamPosX-10 and C[0] < CamPosX-5:
        if C[1] < CamPosY :
            C[0] = C[0] + 0.77
            C[1] = C[1] + 1.27
        if C[1] >= CamPosY and C[0] < CamPosY+5 :
            C[0] = C[0] + 0.79
            C[1] = C[1] + 1.1
        if C[1] >= CamPosY+5 and C[0] < CamPosY+10 :
            C[0] = C[0] - 0.02
            C[1] = C[1] - 0.23
        if C[1] >= CamPosY+10 and C[0] < CamPosY+15 :
            C[0] = C[0] + 0.04
            C[1] = C[1] - 0.17
        if C[1] >= CamPosY+15 and C[0] < CamPosY+20 :
            C[0] = C[0] + 0
            C[1] = C[1] - 0.32
        if C[1] >= CamPosY+20 :
            C[0] = C[0] + 0
            C[1] = C[1] - 0.32

    if C[0] >= CamPosX-5 and C[0] < CamPosX:
        if C[1] < CamPosY :
            C[0] = C[0] + 0.61
            C[1] = C[1] + 1.32
        if C[1] >= CamPosY and C[0] < CamPosY+5 :
            C[0] = C[0] + 0.61
            C[1] = C[1] + 1.23
        if C[1] >= CamPosY+5 and C[0] < CamPosY+10 :
            C[0] = C[0] + 0.65
            C[1] = C[1] + 1.08
        if C[1] >= CamPosY+10 and C[0] < CamPosY+15 :
            C[0] = C[0] + 0.55
            C[1] = C[1] + 0.81
        if C[1] >= CamPosY+15 and C[0] < CamPosY+20 :
            C[0] = C[0] + 0.57
            C[1] = C[1] + 0.51
        if C[1] >= CamPosY+20 :
            C[0] = C[0] + 0.57
            C[1] = C[1] + 0.51
        
    if C[0] >= CamPosX and C[0] < CamPosX+5:
        if C[1] < CamPosY :
            C[0] = C[0] + 0.47
            C[1] = C[1] + 1.17
        if C[1] >= CamPosY and C[0] < CamPosY+5 :
            C[0] = C[0] + 0.41
            C[1] = C[1] + 1.15
        if C[1] >= CamPosY+5 and C[0] < CamPosY+10 :
            C[0] = C[0] - 0.02
            C[1] = C[1] - 0.17
        if C[1] >= CamPosY+10 and C[0] < CamPosY+15 :
            C[0] = C[0] - 0.07
            C[1] = C[1] - 0.34
        if C[1] >= CamPosY+15 and C[0] < CamPosY+20 :
            C[0] = C[0] + 0
            C[1] = C[1] - 0.02
        if C[1] >= CamPosY+20 :
            C[0] = C[0] + 0
            C[1] = C[1] - 0.02

    if C[0] >= CamPosX+5 and C[0] < CamPosX+10:
        if C[1] < CamPosY :
            C[0] = C[0] + 0.38
            C[1] = C[1] + 1.23
        if C[1] >= CamPosY and C[0] < CamPosY+5 :
            C[0] = C[0] + 0.14
            C[1] = C[1] + 1.25
        if C[1] >= CamPosY+5 and C[0] < CamPosY+10 :
            C[0] = C[0] + 0.04
            C[1] = C[1] + 1.06
        if C[1] >= CamPosY+10 and C[0] < CamPosY+15 :
            C[0] = C[0] + 0.03
            C[1] = C[1] + 0.79
        if C[1] >= CamPosY+15 and C[0] < CamPosY+20 :
            C[0] = C[0] - 0.03
            C[1] = C[1] + 0.51
        if C[1] >= CamPosY+20 :
            C[0] = C[0] - 0.03
            C[1] = C[1]+ 0.51

    if C[0] >= CamPosX+10 and C[0] < CamPosX+15:
        if C[1] < CamPosY :
            C[0] = C[0] + 0.11
            C[1] = C[1] + 1.19
        if C[1] >= CamPosY and C[0] < CamPosY+5 :
            C[0] = C[0] + 0.02
            C[1] = C[1] + 1.19
        if C[1] >= CamPosY+5 and C[0] < CamPosY+10 :
            C[0] = C[0] - 0.08
            C[1] = C[1] - 0.21
        if C[1] >= CamPosY+10 and C[0] < CamPosY+15 :
            C[0] = C[0] - 0.17
            C[1] = C[1] - 0.15
        if C[1] >= CamPosY+15 and C[0] < CamPosY+20 :
            C[0] = C[0] - 0.04
            C[1] = C[1] - 0.23
        if C[1] >= CamPosY+20 :
            C[0] = C[0] - 0.04
            C[1] = C[1] - 0.23
        
    if C[0] >= CamPosX+15 and C[0] < CamPosX+20:
        if C[1] < CamPosY :
            C[0] = C[0] - 0.16
            C[1] = C[1] + 1.19
        if C[1] >= CamPosY and C[0] < CamPosY+5 :
            C[0] = C[0] - 0.24
            C[1] = C[1] + 1.06
        if C[1] >= CamPosY+5 and C[0] < CamPosY+10 :
            C[0] = C[0] - 0.33
            C[1] = C[1] + 0.96
        if C[1] >= CamPosY+10 and C[0] < CamPosY+15 :
            C[0] = C[0] - 0.45
            C[1] = C[1] + 0.83
        if C[1] >= CamPosY+15 and C[0] < CamPosY+20 :
            C[0] = C[0] - 0.64
            C[1] = C[1] + 0.62
        if C[1] >= CamPosY+20 :
            C[0] = C[0] - 0.64
            C[1] = C[1] + 0.62

    if C[0] >= CamPosX+20 and C[0] < CamPosX+25:
        if C[1] < CamPosY :
            C[0] = C[0] - 0.29
            C[1] = C[1] + 1.34
        if C[1] >= CamPosY and C[0] < CamPosY+5 :
            C[0] = C[0] - 0.55
            C[1] = C[1] + 1.23
        if C[1] >= CamPosY+5 and C[0] < CamPosY+10 :
            C[0] = C[0] - 0.19
            C[1] = C[1] - 0.13
        if C[1] >= CamPosY+10 and C[0] < CamPosY+15 :
            C[0] = C[0] - 0.17
            C[1] = C[1] - 0.33
        if C[1] >= CamPosY+15 and C[0] < CamPosY+20 :
            C[0] = C[0] - 0.15
            C[1] = C[1] - 0.17
        if C[1] >= CamPosY+20 :
            C[0] = C[0] - 0.15
            C[1] = C[1] - 0.17

    if C[0] >= CamPosX+25 :
        if C[1] < CamPosY :
            C[0] = C[0] - 0.81
            C[1] = C[1] + 1.46
        if C[1] >= CamPosY and C[0] < CamPosY+5 :
            C[0] = C[0] - 0.98
            C[1] = C[1] + 1.17
        if C[1] >= CamPosY+5 and C[0] < CamPosY+10 :
            C[0] = C[0] - 0.03
            C[1] = C[1] - 0.09
        if C[1] >= CamPosY+10 and C[0] < CamPosY+15 :
            C[0] = C[0] - 0.48
            C[1] = C[1] - 0.32
        if C[1] >= CamPosY+15 and C[0] < CamPosY+20 :
            C[0] = C[0] - 0.13
            C[1] = C[1] - 0.33
        if C[1] >= CamPosY+20 :
            C[0] = C[0] - 0.13
            C[1] = C[1] - 0.33

    if OB_1[0] < CamPosX-25 :
        if OB_1[1] < CamPosY :
            OB_1[0] = OB_1[0] + 1.55
            OB_1[1] = OB_1[1] + 1.4
        if OB_1[1] >= CamPosY and OB_1[0] < CamPosY+5 :
            OB_1[0] = OB_1[0] + 1.85
            OB_1[1] = OB_1[1] + 1.36
        if OB_1[1] >= CamPosY+5 and OB_1[0] < CamPosY+10 :
            OB_1[0] = OB_1[0] + 0.13
            OB_1[1] = OB_1[1] - 0.13
        if OB_1[1] >= CamPosY+10 and OB_1[0] < CamPosY+15 :
            OB_1[0] = OB_1[0] + 0.21
            OB_1[1] = OB_1[1] - 0.38
        if OB_1[1] >= CamPosY+15 and OB_1[0] < CamPosY+20 :
            OB_1[0] = OB_1[0] - 0.02
            OB_1[1] = OB_1[1] - 0.04
        if OB_1[1] >= CamPosY+20 :
            OB_1[0] = OB_1[0] - 0.02
            OB_1[1] = OB_1[1] - 0.04

    if OB_1[0] >= CamPosX-25 and OB_1[0] < CamPosX-20:
        if OB_1[1] < CamPosY :
            OB_1[0] = OB_1[0] + 1.22
            OB_1[1] = OB_1[1] + 1.1
        if OB_1[1] >= CamPosY and OB_1[0] < CamPosY+5 :
            OB_1[0] = OB_1[0] + 1.46
            OB_1[1] = OB_1[1] + 0.96
        if OB_1[1] >= CamPosY+5 and OB_1[0] < CamPosY+10 :
            OB_1[0] = OB_1[0] + 1.44
            OB_1[1] = OB_1[1] + 1.21
        if OB_1[1] >= CamPosY+10 and OB_1[0] < CamPosY+15 :
            OB_1[0] = OB_1[0] + 1.88
            OB_1[1] = OB_1[1] + 0.85
        if OB_1[1] >= CamPosY+15 and OB_1[0] < CamPosY+20 :
            OB_1[0] = OB_1[0] + 2.01
            OB_1[1] = OB_1[1] + 0.62
        if OB_1[1] >= CamPosY+20 :
            OB_1[0] = OB_1[0] + 2.01
            OB_1[1] = OB_1[1] + 0.62

    if OB_1[0] >= CamPosX-20 and OB_1[0] < CamPosX-15:
        if OB_1[1] < CamPosY :
            OB_1[0] = OB_1[0] + 1.23
            OB_1[1] = OB_1[1] + 1.04
        if OB_1[1] >= CamPosY and OB_1[0] < CamPosY+5 :
            OB_1[0] = OB_1[0] + 1.21
            OB_1[1] = OB_1[1] + 0.98
        if OB_1[1] >= CamPosY+5 and OB_1[0] < CamPosY+10 :
            OB_1[0] = OB_1[0] + 0.15
            OB_1[1] = OB_1[1] - 0.22
        if OB_1[1] >= CamPosY+10 and OB_1[0] < CamPosY+15 :
            OB_1[0] = OB_1[0] + 0.04
            OB_1[1] = OB_1[1] - 0.23
        if OB_1[1] >= CamPosY+15 and OB_1[0] < CamPosY+20 :
            OB_1[0] = OB_1[0] + 0.15
            OB_1[1] = OB_1[1] - 0.27
        if OB_1[1] >= CamPosY+20 :
            OB_1[0] = OB_1[0] + 0.15
            OB_1[1] = OB_1[1] - 0.27

    if OB_1[0] >= CamPosX-15 and OB_1[0] < CamPosX-10:
        if OB_1[1] < CamPosY :
            OB_1[0] = OB_1[0] + 0.93
            OB_1[1] = OB_1[1] + 1.12
        if OB_1[1] >= CamPosY and OB_1[0] < CamPosY+5 :
            OB_1[0] = OB_1[0] + 0.99
            OB_1[1] = OB_1[1] + 0.96
        if OB_1[1] >= CamPosY+5 and OB_1[0] < CamPosY+10 :
            OB_1[0] = OB_1[0] + 1.12
            OB_1[1] = OB_1[1] + 0.92
        if OB_1[1] >= CamPosY+10 and OB_1[0] < CamPosY+15 :
            OB_1[0] = OB_1[0] + 1.12
            OB_1[1] = OB_1[1] + 0.6
        if OB_1[1] >= CamPosY+15 and OB_1[0] < CamPosY+20 :
            OB_1[0] = OB_1[0] + 1.12
            OB_1[1] = OB_1[1] + 0.34
        if OB_1[1] >= CamPosY+20 :
            OB_1[0] = OB_1[0] + 1.12
            OB_1[1] = OB_1[1] + 0.34

    if OB_1[0] >= CamPosX-10 and OB_1[0] < CamPosX-5:
        if OB_1[1] < CamPosY :
            OB_1[0] = OB_1[0] + 0.77
            OB_1[1] = OB_1[1] + 1.27
        if OB_1[1] >= CamPosY and OB_1[0] < CamPosY+5 :
            OB_1[0] = OB_1[0] + 0.79
            OB_1[1] = OB_1[1] + 1.1
        if OB_1[1] >= CamPosY+5 and OB_1[0] < CamPosY+10 :
            OB_1[0] = OB_1[0] - 0.02
            OB_1[1] = OB_1[1] - 0.23
        if OB_1[1] >= CamPosY+10 and OB_1[0] < CamPosY+15 :
            OB_1[0] = OB_1[0] + 0.04
            OB_1[1] = OB_1[1] - 0.17
        if OB_1[1] >= CamPosY+15 and OB_1[0] < CamPosY+20 :
            OB_1[0] = OB_1[0] + 0
            OB_1[1] = OB_1[1] - 0.32
        if OB_1[1] >= CamPosY+20 :
            OB_1[0] = OB_1[0] + 0
            OB_1[1] = OB_1[1] - 0.32

    if OB_1[0] >= CamPosX-5 and OB_1[0] < CamPosX:
        if OB_1[1] < CamPosY :
            OB_1[0] = OB_1[0] + 0.61
            OB_1[1] = OB_1[1] + 1.32
        if OB_1[1] >= CamPosY and OB_1[0] < CamPosY+5 :
            OB_1[0] = OB_1[0] + 0.61
            OB_1[1] = OB_1[1] + 1.23
        if OB_1[1] >= CamPosY+5 and OB_1[0] < CamPosY+10 :
            OB_1[0] = OB_1[0] + 0.65
            OB_1[1] = OB_1[1] + 1.08
        if OB_1[1] >= CamPosY+10 and OB_1[0] < CamPosY+15 :
            OB_1[0] = OB_1[0] + 0.55
            OB_1[1] = OB_1[1] + 0.81
        if OB_1[1] >= CamPosY+15 and OB_1[0] < CamPosY+20 :
            OB_1[0] = OB_1[0] + 0.57
            OB_1[1] = OB_1[1] + 0.51
        if OB_1[1] >= CamPosY+20 :
            OB_1[0] = OB_1[0] + 0.57
            OB_1[1] = OB_1[1] + 0.51
        
    if OB_1[0] >= CamPosX and OB_1[0] < CamPosX+5:
        if OB_1[1] < CamPosY :
            OB_1[0] = OB_1[0] + 0.47
            OB_1[1] = OB_1[1] + 1.17
        if OB_1[1] >= CamPosY and OB_1[0] < CamPosY+5 :
            OB_1[0] = OB_1[0] + 0.41
            OB_1[1] = OB_1[1] + 1.15
        if OB_1[1] >= CamPosY+5 and OB_1[0] < CamPosY+10 :
            OB_1[0] = OB_1[0] - 0.02
            OB_1[1] = OB_1[1] - 0.17
        if OB_1[1] >= CamPosY+10 and OB_1[0] < CamPosY+15 :
            OB_1[0] = OB_1[0] - 0.07
            OB_1[1] = OB_1[1] - 0.34
        if OB_1[1] >= CamPosY+15 and OB_1[0] < CamPosY+20 :
            OB_1[0] = OB_1[0] + 0
            OB_1[1] = OB_1[1] - 0.02
        if OB_1[1] >= CamPosY+20 :
            OB_1[0] = OB_1[0] + 0
            OB_1[1] = OB_1[1] - 0.02

    if OB_1[0] >= CamPosX+5 and OB_1[0] < CamPosX+10:
        if OB_1[1] < CamPosY :
            OB_1[0] = OB_1[0] + 0.38
            OB_1[1] = OB_1[1] + 1.23
        if OB_1[1] >= CamPosY and OB_1[0] < CamPosY+5 :
            OB_1[0] = OB_1[0] + 0.14
            OB_1[1] = OB_1[1] + 1.25
        if OB_1[1] >= CamPosY+5 and OB_1[0] < CamPosY+10 :
            OB_1[0] = OB_1[0] + 0.04
            OB_1[1] = OB_1[1] + 1.06
        if OB_1[1] >= CamPosY+10 and OB_1[0] < CamPosY+15 :
            OB_1[0] = OB_1[0] + 0.03
            OB_1[1] = OB_1[1] + 0.79
        if OB_1[1] >= CamPosY+15 and OB_1[0] < CamPosY+20 :
            OB_1[0] = OB_1[0] - 0.03
            OB_1[1] = OB_1[1] + 0.51
        if OB_1[1] >= CamPosY+20 :
            OB_1[0] = OB_1[0] - 0.03
            OB_1[1] = OB_1[1]+ 0.51

    if OB_1[0] >= CamPosX+10 and OB_1[0] < CamPosX+15:
        if OB_1[1] < CamPosY :
            OB_1[0] = OB_1[0] + 0.11
            OB_1[1] = OB_1[1] + 1.19
        if OB_1[1] >= CamPosY and OB_1[0] < CamPosY+5 :
            OB_1[0] = OB_1[0] + 0.02
            OB_1[1] = OB_1[1] + 1.19
        if OB_1[1] >= CamPosY+5 and OB_1[0] < CamPosY+10 :
            OB_1[0] = OB_1[0] - 0.08
            OB_1[1] = OB_1[1] - 0.21
        if OB_1[1] >= CamPosY+10 and OB_1[0] < CamPosY+15 :
            OB_1[0] = OB_1[0] - 0.17
            OB_1[1] = OB_1[1] - 0.15
        if OB_1[1] >= CamPosY+15 and OB_1[0] < CamPosY+20 :
            OB_1[0] = OB_1[0] - 0.04
            OB_1[1] = OB_1[1] - 0.23
        if OB_1[1] >= CamPosY+20 :
            OB_1[0] = OB_1[0] - 0.04
            OB_1[1] = OB_1[1] - 0.23
        
    if OB_1[0] >= CamPosX+15 and OB_1[0] < CamPosX+20:
        if OB_1[1] < CamPosY :
            OB_1[0] = OB_1[0] - 0.16
            OB_1[1] = OB_1[1] + 1.19
        if OB_1[1] >= CamPosY and OB_1[0] < CamPosY+5 :
            OB_1[0] = OB_1[0] - 0.24
            OB_1[1] = OB_1[1] + 1.06
        if OB_1[1] >= CamPosY+5 and OB_1[0] < CamPosY+10 :
            OB_1[0] = OB_1[0] - 0.33
            OB_1[1] = OB_1[1] + 0.96
        if OB_1[1] >= CamPosY+10 and OB_1[0] < CamPosY+15 :
            OB_1[0] = OB_1[0] - 0.45
            OB_1[1] = OB_1[1] + 0.83
        if OB_1[1] >= CamPosY+15 and OB_1[0] < CamPosY+20 :
            OB_1[0] = OB_1[0] - 0.64
            OB_1[1] = OB_1[1] + 0.62
        if OB_1[1] >= CamPosY+20 :
            OB_1[0] = OB_1[0] - 0.64
            OB_1[1] = OB_1[1] + 0.62

    if OB_1[0] >= CamPosX+20 and OB_1[0] < CamPosX+25:
        if OB_1[1] < CamPosY :
            OB_1[0] = OB_1[0] - 0.29
            OB_1[1] = OB_1[1] + 1.34
        if OB_1[1] >= CamPosY and OB_1[0] < CamPosY+5 :
            OB_1[0] = OB_1[0] - 0.55
            OB_1[1] = OB_1[1] + 1.23
        if OB_1[1] >= CamPosY+5 and OB_1[0] < CamPosY+10 :
            OB_1[0] = OB_1[0] - 0.19
            OB_1[1] = OB_1[1] - 0.13
        if OB_1[1] >= CamPosY+10 and OB_1[0] < CamPosY+15 :
            OB_1[0] = OB_1[0] - 0.17
            OB_1[1] = OB_1[1] - 0.33
        if OB_1[1] >= CamPosY+15 and OB_1[0] < CamPosY+20 :
            OB_1[0] = OB_1[0] - 0.15
            OB_1[1] = OB_1[1] - 0.17
        if OB_1[1] >= CamPosY+20 :
            OB_1[0] = OB_1[0] - 0.15
            OB_1[1] = OB_1[1] - 0.17

    if OB_1[0] >= CamPosX+25 :
        if OB_1[1] < CamPosY :
            OB_1[0] = OB_1[0] - 0.81
            OB_1[1] = OB_1[1] + 1.46
        if OB_1[1] >= CamPosY and OB_1[0] < CamPosY+5 :
            OB_1[0] = OB_1[0] - 0.98
            OB_1[1] = OB_1[1] + 1.17
        if OB_1[1] >= CamPosY+5 and OB_1[0] < CamPosY+10 :
            OB_1[0] = OB_1[0] - 0.03
            OB_1[1] = OB_1[1] - 0.09
        if OB_1[1] >= CamPosY+10 and OB_1[0] < CamPosY+15 :
            OB_1[0] = OB_1[0] - 0.48
            OB_1[1] = OB_1[1] - 0.32
        if OB_1[1] >= CamPosY+15 and OB_1[0] < CamPosY+20 :
            OB_1[0] = OB_1[0] - 0.13
            OB_1[1] = OB_1[1] - 0.33
        if OB_1[1] >= CamPosY+20 :
            OB_1[0] = OB_1[0] - 0.13
            OB_1[1] = OB_1[1] - 0.33
    
    if OB_2[0] < CamPosX-25 :
        if OB_2[1] < CamPosY :
            OB_2[0] = OB_2[0] + 1.55
            OB_2[1] = OB_2[1] + 1.4
        if OB_2[1] >= CamPosY and OB_2[0] < CamPosY+5 :
            OB_2[0] = OB_2[0] + 1.85
            OB_2[1] = OB_2[1] + 1.36
        if OB_2[1] >= CamPosY+5 and OB_2[0] < CamPosY+10 :
            OB_2[0] = OB_2[0] + 0.13
            OB_2[1] = OB_2[1] - 0.13
        if OB_2[1] >= CamPosY+10 and OB_2[0] < CamPosY+15 :
            OB_2[0] = OB_2[0] + 0.21
            OB_2[1] = OB_2[1] - 0.38
        if OB_2[1] >= CamPosY+15 and OB_2[0] < CamPosY+20 :
            OB_2[0] = OB_2[0] - 0.02
            OB_2[1] = OB_2[1] - 0.04
        if OB_2[1] >= CamPosY+20 :
            OB_2[0] = OB_2[0] - 0.02
            OB_2[1] = OB_2[1] - 0.04

    if OB_2[0] >= CamPosX-25 and OB_2[0] < CamPosX-20:
        if OB_2[1] < CamPosY :
            OB_2[0] = OB_2[0] + 1.22
            OB_2[1] = OB_2[1] + 1.1
        if OB_2[1] >= CamPosY and OB_2[0] < CamPosY+5 :
            OB_2[0] = OB_2[0] + 1.46
            OB_2[1] = OB_2[1] + 0.96
        if OB_2[1] >= CamPosY+5 and OB_2[0] < CamPosY+10 :
            OB_2[0] = OB_2[0] + 1.44
            OB_2[1] = OB_2[1] + 1.21
        if OB_2[1] >= CamPosY+10 and OB_2[0] < CamPosY+15 :
            OB_2[0] = OB_2[0] + 1.88
            OB_2[1] = OB_2[1] + 0.85
        if OB_2[1] >= CamPosY+15 and OB_2[0] < CamPosY+20 :
            OB_2[0] = OB_2[0] + 2.01
            OB_2[1] = OB_2[1] + 0.62
        if OB_2[1] >= CamPosY+20 :
            OB_2[0] = OB_2[0] + 2.01
            OB_2[1] = OB_2[1] + 0.62

    if OB_2[0] >= CamPosX-20 and OB_2[0] < CamPosX-15:
        if OB_2[1] < CamPosY :
            OB_2[0] = OB_2[0] + 1.23
            OB_2[1] = OB_2[1] + 1.04
        if OB_2[1] >= CamPosY and OB_2[0] < CamPosY+5 :
            OB_2[0] = OB_2[0] + 1.21
            OB_2[1] = OB_2[1] + 0.98
        if OB_2[1] >= CamPosY+5 and OB_2[0] < CamPosY+10 :
            OB_2[0] = OB_2[0] + 0.15
            OB_2[1] = OB_2[1] - 0.22
        if OB_2[1] >= CamPosY+10 and OB_2[0] < CamPosY+15 :
            OB_2[0] = OB_2[0] + 0.04
            OB_2[1] = OB_2[1] - 0.23
        if OB_2[1] >= CamPosY+15 and OB_2[0] < CamPosY+20 :
            OB_2[0] = OB_2[0] + 0.15
            OB_2[1] = OB_2[1] - 0.27
        if OB_2[1] >= CamPosY+20 :
            OB_2[0] = OB_2[0] + 0.15
            OB_2[1] = OB_2[1] - 0.27

    if OB_2[0] >= CamPosX-15 and OB_2[0] < CamPosX-10:
        if OB_2[1] < CamPosY :
            OB_2[0] = OB_2[0] + 0.93
            OB_2[1] = OB_2[1] + 1.12
        if OB_2[1] >= CamPosY and OB_2[0] < CamPosY+5 :
            OB_2[0] = OB_2[0] + 0.99
            OB_2[1] = OB_2[1] + 0.96
        if OB_2[1] >= CamPosY+5 and OB_2[0] < CamPosY+10 :
            OB_2[0] = OB_2[0] + 1.12
            OB_2[1] = OB_2[1] + 0.92
        if OB_2[1] >= CamPosY+10 and OB_2[0] < CamPosY+15 :
            OB_2[0] = OB_2[0] + 1.12
            OB_2[1] = OB_2[1] + 0.6
        if OB_2[1] >= CamPosY+15 and OB_2[0] < CamPosY+20 :
            OB_2[0] = OB_2[0] + 1.12
            OB_2[1] = OB_2[1] + 0.34
        if OB_2[1] >= CamPosY+20 :
            OB_2[0] = OB_2[0] + 1.12
            OB_2[1] = OB_2[1] + 0.34

    if OB_2[0] >= CamPosX-10 and OB_2[0] < CamPosX-5:
        if OB_2[1] < CamPosY :
            OB_2[0] = OB_2[0] + 0.77
            OB_2[1] = OB_2[1] + 1.27
        if OB_2[1] >= CamPosY and OB_2[0] < CamPosY+5 :
            OB_2[0] = OB_2[0] + 0.79
            OB_2[1] = OB_2[1] + 1.1
        if OB_2[1] >= CamPosY+5 and OB_2[0] < CamPosY+10 :
            OB_2[0] = OB_2[0] - 0.02
            OB_2[1] = OB_2[1] - 0.23
        if OB_2[1] >= CamPosY+10 and OB_2[0] < CamPosY+15 :
            OB_2[0] = OB_2[0] + 0.04
            OB_2[1] = OB_2[1] - 0.17
        if OB_2[1] >= CamPosY+15 and OB_2[0] < CamPosY+20 :
            OB_2[0] = OB_2[0] + 0
            OB_2[1] = OB_2[1] - 0.32
        if OB_2[1] >= CamPosY+20 :
            OB_2[0] = OB_2[0] + 0
            OB_2[1] = OB_2[1] - 0.32

    if OB_2[0] >= CamPosX-5 and OB_2[0] < CamPosX:
        if OB_2[1] < CamPosY :
            OB_2[0] = OB_2[0] + 0.61
            OB_2[1] = OB_2[1] + 1.32
        if OB_2[1] >= CamPosY and OB_2[0] < CamPosY+5 :
            OB_2[0] = OB_2[0] + 0.61
            OB_2[1] = OB_2[1] + 1.23
        if OB_2[1] >= CamPosY+5 and OB_2[0] < CamPosY+10 :
            OB_2[0] = OB_2[0] + 0.65
            OB_2[1] = OB_2[1] + 1.08
        if OB_2[1] >= CamPosY+10 and OB_2[0] < CamPosY+15 :
            OB_2[0] = OB_2[0] + 0.55
            OB_2[1] = OB_2[1] + 0.81
        if OB_2[1] >= CamPosY+15 and OB_2[0] < CamPosY+20 :
            OB_2[0] = OB_2[0] + 0.57
            OB_2[1] = OB_2[1] + 0.51
        if OB_2[1] >= CamPosY+20 :
            OB_2[0] = OB_2[0] + 0.57
            OB_2[1] = OB_2[1] + 0.51
        
    if OB_2[0] >= CamPosX and OB_2[0] < CamPosX+5:
        if OB_2[1] < CamPosY :
            OB_2[0] = OB_2[0] + 0.47
            OB_2[1] = OB_2[1] + 1.17
        if OB_2[1] >= CamPosY and OB_2[0] < CamPosY+5 :
            OB_2[0] = OB_2[0] + 0.41
            OB_2[1] = OB_2[1] + 1.15
        if OB_2[1] >= CamPosY+5 and OB_2[0] < CamPosY+10 :
            OB_2[0] = OB_2[0] - 0.02
            OB_2[1] = OB_2[1] - 0.17
        if OB_2[1] >= CamPosY+10 and OB_2[0] < CamPosY+15 :
            OB_2[0] = OB_2[0] - 0.07
            OB_2[1] = OB_2[1] - 0.34
        if OB_2[1] >= CamPosY+15 and OB_2[0] < CamPosY+20 :
            OB_2[0] = OB_2[0] + 0
            OB_2[1] = OB_2[1] - 0.02
        if OB_2[1] >= CamPosY+20 :
            OB_2[0] = OB_2[0] + 0
            OB_2[1] = OB_2[1] - 0.02

    if OB_2[0] >= CamPosX+5 and OB_2[0] < CamPosX+10:
        if OB_2[1] < CamPosY :
            OB_2[0] = OB_2[0] + 0.38
            OB_2[1] = OB_2[1] + 1.23
        if OB_2[1] >= CamPosY and OB_2[0] < CamPosY+5 :
            OB_2[0] = OB_2[0] + 0.14
            OB_2[1] = OB_2[1] + 1.25
        if OB_2[1] >= CamPosY+5 and OB_2[0] < CamPosY+10 :
            OB_2[0] = OB_2[0] + 0.04
            OB_2[1] = OB_2[1] + 1.06
        if OB_2[1] >= CamPosY+10 and OB_2[0] < CamPosY+15 :
            OB_2[0] = OB_2[0] + 0.03
            OB_2[1] = OB_2[1] + 0.79
        if OB_2[1] >= CamPosY+15 and OB_2[0] < CamPosY+20 :
            OB_2[0] = OB_2[0] - 0.03
            OB_2[1] = OB_2[1] + 0.51
        if OB_2[1] >= CamPosY+20 :
            OB_2[0] = OB_2[0] - 0.03
            OB_2[1] = OB_2[1]+ 0.51

    if OB_2[0] >= CamPosX+10 and OB_2[0] < CamPosX+15:
        if OB_2[1] < CamPosY :
            OB_2[0] = OB_2[0] + 0.11
            OB_2[1] = OB_2[1] + 1.19
        if OB_2[1] >= CamPosY and OB_2[0] < CamPosY+5 :
            OB_2[0] = OB_2[0] + 0.02
            OB_2[1] = OB_2[1] + 1.19
        if OB_2[1] >= CamPosY+5 and OB_2[0] < CamPosY+10 :
            OB_2[0] = OB_2[0] - 0.08
            OB_2[1] = OB_2[1] - 0.21
        if OB_2[1] >= CamPosY+10 and OB_2[0] < CamPosY+15 :
            OB_2[0] = OB_2[0] - 0.17
            OB_2[1] = OB_2[1] - 0.15
        if OB_2[1] >= CamPosY+15 and OB_2[0] < CamPosY+20 :
            OB_2[0] = OB_2[0] - 0.04
            OB_2[1] = OB_2[1] - 0.23
        if OB_2[1] >= CamPosY+20 :
            OB_2[0] = OB_2[0] - 0.04
            OB_2[1] = OB_2[1] - 0.23
        
    if OB_2[0] >= CamPosX+15 and OB_2[0] < CamPosX+20:
        if OB_2[1] < CamPosY :
            OB_2[0] = OB_2[0] - 0.16
            OB_2[1] = OB_2[1] + 1.19
        if OB_2[1] >= CamPosY and OB_2[0] < CamPosY+5 :
            OB_2[0] = OB_2[0] - 0.24
            OB_2[1] = OB_2[1] + 1.06
        if OB_2[1] >= CamPosY+5 and OB_2[0] < CamPosY+10 :
            OB_2[0] = OB_2[0] - 0.33
            OB_2[1] = OB_2[1] + 0.96
        if OB_2[1] >= CamPosY+10 and OB_2[0] < CamPosY+15 :
            OB_2[0] = OB_2[0] - 0.45
            OB_2[1] = OB_2[1] + 0.83
        if OB_2[1] >= CamPosY+15 and OB_2[0] < CamPosY+20 :
            OB_2[0] = OB_2[0] - 0.64
            OB_2[1] = OB_2[1] + 0.62
        if OB_2[1] >= CamPosY+20 :
            OB_2[0] = OB_2[0] - 0.64
            OB_2[1] = OB_2[1] + 0.62

    if OB_2[0] >= CamPosX+20 and OB_2[0] < CamPosX+25:
        if OB_2[1] < CamPosY :
            OB_2[0] = OB_2[0] - 0.29
            OB_2[1] = OB_2[1] + 1.34
        if OB_2[1] >= CamPosY and OB_2[0] < CamPosY+5 :
            OB_2[0] = OB_2[0] - 0.55
            OB_2[1] = OB_2[1] + 1.23
        if OB_2[1] >= CamPosY+5 and OB_2[0] < CamPosY+10 :
            OB_2[0] = OB_2[0] - 0.19
            OB_2[1] = OB_2[1] - 0.13
        if OB_2[1] >= CamPosY+10 and OB_2[0] < CamPosY+15 :
            OB_2[0] = OB_2[0] - 0.17
            OB_2[1] = OB_2[1] - 0.33
        if OB_2[1] >= CamPosY+15 and OB_2[0] < CamPosY+20 :
            OB_2[0] = OB_2[0] - 0.15
            OB_2[1] = OB_2[1] - 0.17
        if OB_2[1] >= CamPosY+20 :
            OB_2[0] = OB_2[0] - 0.15
            OB_2[1] = OB_2[1] - 0.17

    if OB_2[0] >= CamPosX+25 :
        if OB_2[1] < CamPosY :
            OB_2[0] = OB_2[0] - 0.81
            OB_2[1] = OB_2[1] + 1.46
        if OB_2[1] >= CamPosY and OB_2[0] < CamPosY+5 :
            OB_2[0] = OB_2[0] - 0.98
            OB_2[1] = OB_2[1] + 1.17
        if OB_2[1] >= CamPosY+5 and OB_2[0] < CamPosY+10 :
            OB_2[0] = OB_2[0] - 0.03
            OB_2[1] = OB_2[1] - 0.09
        if OB_2[1] >= CamPosY+10 and OB_2[0] < CamPosY+15 :
            OB_2[0] = OB_2[0] - 0.48
            OB_2[1] = OB_2[1] - 0.32
        if OB_2[1] >= CamPosY+15 and OB_2[0] < CamPosY+20 :
            OB_2[0] = OB_2[0] - 0.13
            OB_2[1] = OB_2[1] - 0.33
        if OB_2[1] >= CamPosY+20 :
            OB_2[0] = OB_2[0] - 0.13
            OB_2[1] = OB_2[1] - 0.33

    # if C[0] > -0.1 :
    #     C[0] = C[0] - ( C[0]-(-0.1)) * ( 0.9 / 29 ) 
    # if C[0] < -0.1 :
    #     C[0] = C[0] + (-0.1 - C[0] ) * ( 1.61 / 29 )  
    # if C[1] > 43.3 :
    #     C[1] = C[1] + 1.3 #( C[1]- 43.3 ) * ( 1.5 / 12.5)    
    # if C[1] < 43.3 :
    #     C[1] = C[1] + 2.68 #( 43.3 - C[1] ) *  (2.26 / 10.5)


    # if OB_1[0] > -0.1 :
    #     OB_1[0] = OB_1[0] - ( OB_1[0]-(-0.1)) * ( 1.91 / 29 )  
    # if M[0] < -0.1 :
    #     OB_1[0] = OB_1[0] + (-0.1 - OB_1[0] ) * ( 1.61 / 29 ) 
    # if OB_1[1] > 43.3 :
    #     OB_1[1] = OB_1[1] + 1.3#( OB_1[1]- 43.3 ) * ( 1.5 / 12.5)    
    # if OB_1[1] < 43.3 :
    #     OB_1[1] = OB_1[1] + 2.68#( 43.3 - OB_1[1] ) *  (2.26 / 10.5)


    # if OB_2[0] > -0.1 :
    #     OB_2[0] = OB_2[0] - ( OB_2[0]-(-0.1)) * ( 1.91 / 29 ) 
    # if OB_2[0] < -0.1 :
    #     OB_2[0] = OB_2[0] + (-0.1 - OB_2[0] ) * ( 1.61 / 29 )  
    # if OB_2[1] > 43.3 :
    #     OB_2[1] = OB_2[1] + 1.3 #( OB_2[1]- 43.3 ) * ( 1.5 / 12.5)    
    # if OB_2[1] < 43.3 :
    #     OB_2[1] = OB_2[1] + 2.68 #( 43.3 - OB_2[1] ) *  (2.26 / 10.5)

def get_str_mid(): #一個球的座標中心
    #input :one ball mid coordination as string type ---> temp_str
    #output:mid coordination for the ball ---> temp_list = [s_obj_name, f_score, MID_X, MID_Y]
    dict_ball = {   'zero' : 0, 
                    'one'  : 1,
                    'two'  : 2,
                    'three': 3,
                    'four' : 4,
                    'five' : 5,
                    'six'  : 6,
                    'seven': 7,
                    'eight': 8, 
                    'nine' : 9}

    global temp_str

    s_obj_name = ''
    s_score = ''
    s_min_x = ''
    s_Max_x = ''
    s_min_y = ''
    s_Max_y = ''
        
    score_vel_beg = int(temp_str.find('object_name: "')) + int(len('object_name: "')) # 取得球號
    score_vel_end = temp_str.find('"',score_vel_beg)
    for index in range(score_vel_beg, score_vel_end) :
        s_obj_name += temp_str[index]

    score_vel = int(temp_str.find('score: ')) + int(len("score: ")) # 取得信心值
    for index in range(score_vel,score_vel + 5) :
        s_score += temp_str[index]
    f_score = float(s_score)
    
    if(f_score >= 0.7): #信心值大於預期目標 才做分析 取得該球中心座標
        # -----取中心座標-----
        min_x_beg = int(temp_str.find('min_x: ')) + int(len("min_x: "))
        min_x_end = temp_str.find('.0',min_x_beg) 
        for index in range(min_x_beg,min_x_end) :
            s_min_x += temp_str[index]
        f_min_x = float(s_min_x)

        Max_x_beg = int(temp_str.find('Max_x: ')) + int(len("Max_x: "))
        Max_x_end = temp_str.find('.0',Max_x_beg) 
        for index in range(Max_x_beg,Max_x_end) :
            s_Max_x += temp_str[index]
        f_Max_x = float(s_Max_x)

        min_y_beg = int(temp_str.find('min_y: ')) + int(len("min_y: "))
        min_y_end = temp_str.find('.0',min_y_beg) 
        for index in range(min_y_beg,min_y_end) :
            s_min_y += temp_str[index]
        f_min_y = float(s_min_y)
        
        Max_y_beg = int(temp_str.find('Max_y: ')) + int(len("Max_y: "))
        Max_y_end = temp_str.find('.0',Max_y_beg) 
        for index in range(Max_y_beg,Max_y_end) :
            s_Max_y += temp_str[index]
        f_Max_y = float(s_Max_y)

        MID_X = float(f_min_x + f_Max_x)/2
        MID_Y = float(f_min_y + f_Max_y)/2
        # --------------------

        global temp_list
        temp_list = [s_obj_name, f_score, MID_X, MID_Y]
        global pic_M
        if( int(dict_ball[s_obj_name] ) == 0 ):
            pic_M = [MID_X, MID_Y]
        global pic_C
        if( int(dict_ball[s_obj_name] ) == 9 ):
            pic_C = [MID_X, MID_Y]
        
        global pic_OB_1, pic_OB_2
        if ((int(dict_ball[s_obj_name] ) != 0) and (int(dict_ball[s_obj_name] ) != 9) and pic_OB_1 == [0.0, 0.0]):
            pic_OB_1[0] = MID_X
            pic_OB_1[1] = MID_Y
        elif ((int(dict_ball[s_obj_name] ) != 0) and (int(dict_ball[s_obj_name] ) != 9) and pic_OB_2 == [0.0, 0.0]):
            pic_OB_2[0] = MID_X
            pic_OB_2[1] = MID_Y
        
        
        global pic_C, NineOnTable
        if(int(dict_ball[s_obj_name])==9):
            pic_C = [MID_X, MID_Y]
            if (pic_C == [0.0, 0.0]) :
                NineOnTable = 0
            else :
                NineOnTable = 1

        for i in range(2,9): #check if score achieve, it will be record on Array_2D
            if( int(dict_ball[s_obj_name] ) == i): # check number of the ball
                for index in range(len(temp_list)): #put value in, one by one
                    global Array_2D
                    Array_2D[int(dict_ball[s_obj_name])][index] = temp_list[index] 
    else:
        pass

    

def GetStartEnd():
    global action
    global H1,H2,H3,H4,H5,H6
    global gHole
    
    check_hole = [] #local variable 存每個洞口 跟 子球 母球的 角度差值

    global Hole_info
    Hole_info = {
        1: H1,    
        2: H2,    
        3: H3,    
        4: H4,    
        5: H5,
        6: H6 
        }
    
    global min_dif, num_hole
    global V, H_S, H_E, M, C ,OB_1 ,OB_2
    global VT
    global Height,BMZ
    global KS, KNS, Kfun1, Kfun2, Kfun3, Kfun4
    global AS, ANS, Afun1, Afun2, Afun3, Afun4

    Height = BMZ + 1.35

    Kfun1 = 0.0
    Kfun2 = 0.0
    Kfun3 = 0.0
    Kfun4 = 0.0
    Afun1 = 0.0
    Afun2 = 0.0
    Afun3 = 0.0
    Afun4 = 0.0


    #---------------------計算母到子和母到洞的夾角----------------------------------------

    VMtoC= np.array([C[0]-M[0],C[1]-M[1]]).reshape(2, 1)        #母到子的向量
    
    Vmh1 = np.array([H1[0]-M[0],H1[1]-M[1]]).reshape(2, 1)      #母到洞的向量
    Tmh1 = Vmh1.transpose()                                     #轉置
    h1dot = np.dot(Tmh1, VMtoC)                                #內積
    Lmc = np.linalg.norm(VMtoC)                                 #MC向量長
    Lmh1 = np.linalg.norm(Vmh1)                                 #MH向量長
    Dmh1 = np.rad2deg(np.arccos(h1dot/(Lmc*Lmh1)))             #兩向量夾角
    
    Vmh2 = np.array([H2[0]-M[0],H2[1]-M[1]]).reshape(2, 1)
    Tmh2 = Vmh2.transpose()
    h2dot = np.dot(Tmh2, VMtoC)
    Lmh2 = np.linalg.norm(Vmh2)
    Dmh2 = np.rad2deg(np.arccos(h2dot/(Lmc*Lmh2)))             

    Vmh3 = np.array([H3[0]-M[0],H3[1]-M[1]]).reshape(2, 1)
    Tmh3 = Vmh3.transpose()
    h3dot = np.dot(Tmh3, VMtoC)
    Lmh3 = np.linalg.norm(Vmh3)
    Dmh3 = np.rad2deg(np.arccos(h3dot/(Lmc*Lmh3)))     

    Vmh4 = np.array([H4[0]-M[0],H4[1]-M[1]]).reshape(2, 1)
    Tmh4 = Vmh4.transpose()
    h4dot = np.dot(Tmh4, VMtoC)
    Lmh4 = np.linalg.norm(Vmh4)
    Dmh4 = np.rad2deg(np.arccos(h4dot/(Lmc*Lmh4)))

    Vmh5 = np.array([H5[0]-M[0],H5[1]-M[1]]).reshape(2, 1)
    Tmh5 = Vmh5.transpose()
    h5dot = np.dot(Tmh5, VMtoC)
    Lmh5 = np.linalg.norm(Vmh5)
    Dmh5 = np.rad2deg(np.arccos(h5dot/(Lmc*Lmh5)))                      

    Vmh6 = np.array([H6[0]-M[0],H6[1]-M[1]]).reshape(2, 1)
    Tmh6 = Vmh6.transpose()
    h6dot = np.dot(Tmh6, VMtoC)
    Lmh6 = np.linalg.norm(Vmh6)
    Dmh6 = np.rad2deg(np.arccos(h6dot/(Lmc*Lmh6)))    

    
    check_hole = ['0', 
                Dmh1,
                Dmh2, 
                Dmh3,
                Dmh4,
                Dmh5,
                Dmh6 ]

    min_dif = min(check_hole[1:7])       #確認最小差值

    num_hole = check_hole.index(min_dif) #差值最小洞號 為目標洞口 取得目標洞口 index值 存放置 num_hole #前面要做一個 字典 存 各個洞口座標
    print("目標洞口是: ",num_hole,"號洞口")
    gHole = Hole_info.get(num_hole)
    print("目標洞口座標是: ", gHole)
    
    print('M',M)
    print('C',C)    
    print('OB_1',OB_1)
    print('OB_2',OB_2)


    #----------------母球與子球連線上有球(用KISS)的判斷式----OB_1--------------------
    NVMtoC= np.array([C[1]-M[1] ,(C[0]-M[0])* -1]) #-------母到子球的向量(V)的法向量(NV)
    nmcL = np.linalg.norm(NVMtoC)                  #-------NVMtoC的長度
    UVnmc = NVMtoC / nmcL                          #-------向量(NVMtoC)除完長度(nmcl)後即為單位向量(UVnmc)
    kissball = 0                                   #-------kissball用旗標

    KS = (M[1]-C[1])/(M[0]-C[0])                   #-------母球子球斜率 K(Kissball)S（斜率）--------------
    KNS = -1/KS                                    #-------母球子球斜率 K(Kissball)N(垂直)S（斜率）--------------
    if M[1] > C[1] :                               #-------母球及子球兩側的斜線方程式（用法向量向兩側延伸）-----------------
        Kfun1 = KS * (OB_1[0]-(M[0]+2 * (billiard_radius) * UVnmc[0])) - (OB_1[1]-(M[1]+2 * (billiard_radius) * UVnmc[1]))
        Kfun2 = KS * (OB_1[0]-(M[0]-2 * (billiard_radius) * UVnmc[0])) - (OB_1[1]-(M[1]-2 * (billiard_radius) * UVnmc[1]))
        if KS<0:
            Kfun1 = Kfun1 * -1
            Kfun2 = Kfun2 * -1    #斜率正時點帶入大於0即在右側小於0在左側---斜率負時相反故乘一個負號
        if Kfun1 > 0 and Kfun2 < 0 :
            if M[0] > C[0] :                             #----------------過母球球心及子球球心的斜線方程式-----------------
                Kfun3 = KNS * (OB_1[0]-M[0]) - (OB_1[1]-M[1])
                Kfun4 = KNS * (OB_1[0]-C[0]) - (OB_1[1]-C[1])
                if KNS<=0:
                    Kfun3 = Kfun3 * -1
                    Kfun4 = Kfun4 * -1
                if Kfun3 < 0 and Kfun4 > 0 :
                    print('1Use kiss ball(OB_1)')
                    kissball = 1
                    MOB_1C()
            if M[0] < C[0] :
                Kfun3 = KNS * (OB_1[0]-M[0]) - (OB_1[1]-M[1])
                Kfun4 = KNS * (OB_1[0]-C[0]) - (OB_1[1]-C[1])
                if KNS<0:
                    Kfun3 = Kfun3 * -1
                    Kfun4 = Kfun4 * -1
                if Kfun3 > 0 and Kfun4 < 0 :
                    print('2Use kiss ball(OB_1)')
                    kissball = 1
                    MOB_1C()

    if M[1] < C[1] :
        Kfun1 = KS * (OB_1[0]-(M[0]+2 * (billiard_radius) * UVnmc[0])) - (OB_1[1]-(M[1]+2 * (billiard_radius) * UVnmc[1]))
        Kfun2 = KS * (OB_1[0]-(M[0]-2 * (billiard_radius) * UVnmc[0])) - (OB_1[1]-(M[1]-2 * (billiard_radius) * UVnmc[1]))
        if KS<=0:
            Kfun1 = Kfun1 * -1
            Kfun2 = Kfun2 * -1
        if Kfun1 < 0 and Kfun2 > 0 :
            if M[0] > C[0] :
                Kfun3 = KNS * (OB_1[0]-M[0]) - (OB_1[1]-M[1])
                Kfun4 = KNS * (OB_1[0]-C[0]) - (OB_1[1]-C[1])
            if KNS<0:
                Kfun3 = Kfun3 * -1
                Kfun4 = Kfun4 * -1
                if Kfun3 < 0 and Kfun4 > 0 :
                    print('3Use kiss ball(OB_1)')
                    kissball = 1
                    MOB_1C()
            if M[0] < C[0] :
                Kfun3 = KNS * (OB_1[0]-M[0]) - (OB_1[1]-M[1])
                Kfun4 = KNS * (OB_1[0]-C[0]) - (OB_1[1]-C[1])
                if KNS<0:
                    Kfun3 = Kfun3 * -1
                    Kfun4 = Kfun4 * -1
                if Kfun3 > 0 and Kfun4 < 0 :
                    print('4Use kiss ball(OB_1)')
                    kissball = 1
                    MOB_1C()


    #----------------母球與子球連線上有球(用KISS)的判斷式----OB_2--------------------
    if M[1] > C[1] :                                     #----------------母球及子球兩側的斜線方程式-----------------
        Kfun1 = KS * (OB_2[0]-(M[0]+2 * (billiard_radius) * UVnmc[0])) - (OB_2[1]-(M[1]+2 * (billiard_radius) * UVnmc[1]))
        Kfun2 = KS * (OB_2[0]-(M[0]-2 * (billiard_radius) * UVnmc[0])) - (OB_2[1]-(M[1]-2 * (billiard_radius) * UVnmc[1]))
        if KS<0:
            Kfun1 = Kfun1 * -1
            Kfun2 = Kfun2 * -1
        if Kfun1 > 0 and Kfun2 < 0 :
            if M[0] > C[0] :                             #----------------過母球球心及子球球心的斜線方程式-----------------
                Kfun3 = KNS * (OB_2[0]-M[0]) - (OB_2[1]-M[1])
                Kfun4 = KNS * (OB_2[0]-C[0]) - (OB_2[1]-C[1])
                if KNS<=0:
                    Kfun3 = Kfun3 * -1
                    Kfun4 = Kfun4 * -1
                if Kfun3 < 0 and Kfun4 > 0 :
                    print('5Use kiss ball(OB_2')
                    kissball = 1
                    MOB_2C()
            if M[0] < C[0] :
                Kfun3 = KNS * (OB_2[0]-M[0]) - (OB_2[1]-M[1])
                Kfun4 = KNS * (OB_2[0]-C[0]) - (OB_2[1]-C[1])
                if KNS<0:
                    Kfun3 = Kfun3 * -1
                    Kfun4 = Kfun4 * -1
                if Kfun3 > 0 and Kfun4 < 0 :
                    print('6Use kiss ball(OB_2)')
                    kissball = 1
                    MOB_2C()

    if M[1] < C[1] :
        Kfun1 = KS * (OB_2[0]-(M[0]+2 * (billiard_radius) * UVnmc[0])) - (OB_2[1]-(M[1]+2 * (billiard_radius) * UVnmc[1]))
        Kfun2 = KS * (OB_2[0]-(M[0]-2 * (billiard_radius) * UVnmc[0])) - (OB_2[1]-(M[1]-2 * (billiard_radius) * UVnmc[1]))
        if KS<0:
                    Kfun1 = Kfun1 * -1
                    Kfun2 = Kfun2 * -1
        if Kfun1 < 0 and Kfun2 > 0 :
            if M[0] > C[0] :
                Kfun3 = KNS * (OB_2[0]-M[0]) - (OB_2[1]-M[1])
                Kfun4 = KNS * (OB_2[0]-C[0]) - (OB_2[1]-C[1])
            if KNS<0:
                Kfun3 = Kfun3 * -1
                Kfun4 = Kfun4 * -1
                if Kfun3 < 0 and Kfun4 > 0 :
                    print('7Use kiss ball(OB_2)')
                    kissball = 1
                    MOB_2C()
            if M[0] < C[0] :
                Kfun3 = KNS * (OB_2[0]-M[0]) - (OB_2[1]-M[1])
                Kfun4 = KNS * (OB_2[0]-C[0]) - (OB_2[1]-C[1])
                if KNS<0:
                    Kfun3 = Kfun3 * -1
                    Kfun4 = Kfun4 * -1
                if Kfun3 > 0 and Kfun4 < 0 :
                    print('8Use kiss ball(OB_2)')
                    kissball = 1
                    MOB_2C()



    if kissball == 0:
        #---------------計算子球後的虛擬球球心位置------------
        

        VCtoH= np.array([gHole[0]-C[0],gHole[1]-C[1]])    #從C指向H的向量                       
        chL = np.linalg.norm(VCtoH)                       #CH向量的長度
        UVch = VCtoH / chL                                #CH向量的單位向量 
        #C_hit = [ (C[0]- r *UVch[0]), (C[1]- r *UVch[1]) ]   #C的擊球點 = 往反方向延伸一個r的長度 
        V = [ (C[0]-2 * (billiard_radius) * UVch[0]), (C[1]-2 * (billiard_radius) * UVch[1]) ]    #V的球心 = 往反方向延伸兩個r的長度 
        print('V :', V,'\n')

        
        #---------------計算母球後的擊球軌跡------------
        VMtoV= np.array([V[0]-M[0],V[1]-M[1]])                 #從M指向V的向量   ##不確定可不可以不加.reshape(2, 1)                    
        mvL = np.linalg.norm(VMtoV)                            #MV向量的長度
        UVmv = VMtoV / mvL                                     #MV向量的單位向量 
        H_S = [ (M[0]-3 * (billiard_radius) * UVmv[0]), (M[1]-3 * (billiard_radius) * UVmv[1]) ]       #M擊球開始點 = 往反方向延伸4個r的長度
        H_E = [ ((V[0])+(M[0]))/2, ((V[1])+(M[1]))/2 ]         #M擊球結束點 = M和V的球心中點
        print('H_S :', H_S,'\n''H_E :',H_E,'\n','M:',M)
        
        #---------------計算擊球的球桿角度--------------
        VHSE= np.array([H_E[0]-H_S[0],H_E[1]-H_S[1]]) 
        Vup = np.array([0, 1]).reshape(2, 1)
        uT = Vup.transpose()
        n = np.dot(uT, VHSE)
        HSEL = np.linalg.norm(VHSE)
        VT = float(np.rad2deg(np.arccos(n/HSEL)))
        if(H_E[0]>H_S[0]):                                     #0716若子球在母球右側(即超過180度)須乘負號修正
            VT = VT * -1       
        print ('一般擊球')      
        print ('一般擊球DEGREE :',VT)


        ##------------------------避障--OB_1----------------------------
        #--------------------超出球桌範圍-------------------------
        if H_S[0] >= 26.6 or H_S[0] <= -27.6 or H_S[1] >= 55.5 or H_S[1] <= 33.5 :
            Height = BMZ + 2.8
            print ('--------------上升------------避障-----------------')


        #---------------------擊球起始點有球-----------------------
        NVMtoV= np.array([V[1]-M[1] ,(V[0]-M[0])* -1])   #法向量運算
        print('NVM_TO_V',NVMtoV)
        nmvL = np.linalg.norm(NVMtoV)                    #向量長
        print('nmvL',nmvL)
        UVnmv = NVMtoV / nmvL                            #單位法向量
        print('UVnmv',UVnmv)

        AS = (M[1]-V[1])/(M[0]-V[0])
        print('as',AS)
        ANS = -1/AS
        print('ANS',ANS)
        if M[1] > V[1] :                                    #----------------母球兩側各加一個r的斜線方程式-----------------
            print('M', M,'V', V,'OB_1', OB_1)
            Afun1 = AS * (OB_1[0]-(M[0]+2 * (billiard_radius) * UVnmv[0])) - (OB_1[1]-(M[1]+2 * (billiard_radius) * UVnmv[1]))
            Afun2 = AS * (OB_1[0]-(M[0]-2 * (billiard_radius) * UVnmv[0])) - (OB_1[1]-(M[1]-2 * (billiard_radius) * UVnmv[1]))
            if AS<0:
                Afun1 = Afun1 * -1
                Afun2 = Afun2 * -1
            if Afun1 > 0 and Afun2 < 0 :
                if M[0] > V[0] :                            #----------------過母球球心及其後方5倍r距離的斜線方程式-----------------
                    Afun3 = ANS * (OB_1[0]-M[0]) - (OB_1[1]-M[1])
                    Afun4 = ANS * (OB_1[0]-(M[0]-5 * (billiard_radius) * UVmv[0])) - (OB_1[1]-(M[1]-5 * (billiard_radius) * UVmv[1]))
                    if ANS<0:
                        Afun3 = Afun3 * -1
                        Afun4 = Afun4 * -1
                    if Afun3 > 0 and Afun4 < 0 :
                        print('1MoveBall')
                        MoveBall()
                if M[0] < V[0] :
                    Afun3 = ANS * (OB_1[0]-M[0]) - (OB_1[1]-M[1])
                    Afun4 = ANS * (OB_1[0]-(M[0]-5 * (billiard_radius) * UVmv[0])) - (OB_1[1]-(M[1]-5 * (billiard_radius) * UVmv[1]))
                    if ANS<0:
                        Afun3 = Afun3 * -1
                        Afun4 = Afun4 * -1
                    if Afun3 < 0 and Afun4 > 0 :
                        print('2MoveBall')
                        MoveBall()
        if M[1] < V[1] :
            print('M', M,'V', V,'OB_1', OB_1)
            Afun1 = AS * (OB_1[0]-(M[0]+2 * (billiard_radius) * UVnmv[0])) - (OB_1[1]-(M[1]+2 * (billiard_radius) * UVnmv[1]))
            Afun2 = AS * (OB_1[0]-(M[0]-2 * (billiard_radius) * UVnmv[0])) - (OB_1[1]-(M[1]-2 * (billiard_radius) * UVnmv[1]))
            if AS<0:
                Afun1 = Afun1 * -1
                Afun2 = Afun2 * -1
            if Afun1 < 0 and Afun2 > 0 :
                if M[0] > V[0] :
                    Afun3 = ANS * (OB_1[0]-M[0]) - (OB_1[1]-M[1])
                    Afun4 = ANS * (OB_1[0]-(M[0]-5 * (billiard_radius) * UVmv[0])) - (OB_1[1]-(M[1]-5 * (billiard_radius) * UVmv[1]))
                    if ANS<0:
                        Afun3 = Afun3 * -1
                        Afun4 = Afun4 * -1
                    if Afun3 > 0 and Afun4 < 0 :
                        print('3MoveBall')
                        MoveBall()
                if M[0] < V[0] :
                    Afun3 = ANS * (OB_1[0]-M[0]) - (OB_1[1]-M[1])
                    Afun4 = ANS * (OB_1[0]-(M[0]-5 * (billiard_radius) * UVmv[0])) - (OB_1[1]-(M[1]-5 * (billiard_radius) * UVmv[1]))
                    if ANS<0:
                        Afun3 = Afun3 * -1
                        Afun4 = Afun4 * -1
                    if Afun3 < 0 and Afun4 > 0 :
                        print('4MoveBall')
                        MoveBall()


        ##------------------------避障--OB_2----------------------------
        #---------------------擊球起始點有球-----------------------
        if M[1] > V[1] :                                    #----------------母球兩側各加一個r的斜線方程式-----------------
            Afun1 = AS * (OB_2[0]-(M[0]+2 * (billiard_radius) * UVnmv[0])) - (OB_2[1]-(M[1]+2 * (billiard_radius) * UVnmv[1]))
            Afun2 = AS * (OB_2[0]-(M[0]-2 * (billiard_radius) * UVnmv[0])) - (OB_2[1]-(M[1]-2 * (billiard_radius) * UVnmv[1]))
            if AS<0:
                Afun1 = Afun1 * -1
                Afun2 = Afun2 * -1
            if Afun1 > 0 and Afun2 < 0 :
                if M[0] > V[0] :                            #----------------過母球球心及其後方5倍r距離的斜線方程式-----------------
                    Afun3 = ANS * (OB_2[0]-M[0]) - (OB_2[1]-M[1])
                    Afun4 = ANS * (OB_2[0]-(M[0]-5 * (billiard_radius) * UVmv[0])) - (OB_2[1]-(M[1]-5 * (billiard_radius) * UVmv[1]))
                    if ANS<0:
                        Afun3 = Afun3 * -1
                        Afun4 = Afun4 * -1
                    if Afun3 > 0 and Afun4 < 0 :
                        print('5MoveBall')
                        MoveBall()
                if M[0] < V[0] :
                    Afun3 = ANS * (OB_2[0]-M[0]) - (OB_2[1]-M[1])
                    Afun4 = ANS * (OB_2[0]-(M[0]-5 * (billiard_radius) * UVmv[0])) - (OB_2[1]-(M[1]-5 * (billiard_radius) * UVmv[1]))
                    if ANS<0:
                        Afun3 = Afun3 * -1
                        Afun4 = Afun4 * -1
                    if Afun3 < 0 and Afun4 > 0 :
                        print('6MoveBall')
                        MoveBall()
        if M[1] < V[1] :
            Afun1 = AS * (OB_2[0]-(M[0]+2 * (billiard_radius) * UVnmv[0])) - (OB_2[1]-(M[1]+2 * (billiard_radius) * UVnmv[1]))
            Afun2 = AS * (OB_2[0]-(M[0]-2 * (billiard_radius) * UVnmv[0])) - (OB_2[1]-(M[1]-2 * (billiard_radius) * UVnmv[1]))
            if AS<0:
                Afun1 = Afun1 * -1
                Afun2 = Afun2 * -1
            if Afun1 < 0 and Afun2 > 0 :
                if M[0] > V[0] :
                    Afun3 = ANS * (OB_2[0]-M[0]) - (OB_2[1]-M[1])
                    Afun4 = ANS * (OB_2[0]-(M[0]-5 * (billiard_radius) * UVmv[0])) - (OB_2[1]-(M[1]-5 * (billiard_radius) * UVmv[1]))
                    if ANS<0:
                        Afun3 = Afun3 * -1
                        Afun4 = Afun4 * -1
                    if Afun3 > 0 and Afun4 < 0 :
                        print('7MoveBall')
                        MoveBall()
                if M[0] < V[0] :
                    Afun3 = ANS * (OB_2[0]-M[0]) - (OB_2[1]-M[1])
                    Afun4 = ANS * (OB_2[0]-(M[0]-5 * (billiard_radius) * UVmv[0])) - (OB_2[1]-(M[1]-5 * (billiard_radius) * UVmv[1]))
                    if ANS<0:
                        Afun3 = Afun3 * -1
                        Afun4 = Afun4 * -1
                    if Afun3 < 0 and Afun4 > 0 :
                        print('8MoveBall')
                        MoveBall()


#-------0719有干擾球時轉45度在打-------------------------
def MoveBall():
    global VT, H_S, H_E
    global Height,BMZ
    global V, H_S, H_E, M, C ,OB_1 ,OB_2
    global AS, ANS, Afun1, Afun2, Afun3, Afun4

    Height = BMZ + 1.35

    Afun1 = 0.0
    Afun2 = 0.0
    Afun3 = 0.0
    Afun4 = 0.0

    Vhse= np.array([H_E[0]-H_S[0],H_E[1]-H_S[1]])                 #從M指向V的向量   ##不確定可不可以不加.reshape(2, 1)                    
    hseL = np.linalg.norm(Vhse)                            #MV向量的長度
    UVhse = Vhse / hseL   

    NVhse= np.array([H_E[1]-H_S[1] ,(H_E[0]-H_S[0])* -1])   #法向量運算
    nhseL = np.linalg.norm(NVhse)
    UVnhse = NVhse / nhseL                            #單位法向量9
    
    #------------------------------------順時針轉45度---------------------------------------------------------
    H_S[0] = M[0]-4 * (billiard_radius) * np.cos(45)*UVhse[0] + 4 * (billiard_radius) * np.cos(45)*UVnhse[0]
    H_S[1] = M[1]-4 * (billiard_radius) * np.cos(45)*UVhse[1] + 4 * (billiard_radius) * np.cos(45)*UVnhse[1]
    H_E[0] = M[0]+4 * (billiard_radius) * np.cos(45)*UVhse[0] - 4 * (billiard_radius) * np.cos(45)*UVnhse[0]
    H_E[1] = M[1]+4 * (billiard_radius) * np.cos(45)*UVhse[1] - 4 * (billiard_radius) * np.cos(45)*UVnhse[1]
    VT = VT + 45

    # if ( M[0]>?? and M[1]<V[1] ) or ( M[0]<?? and M[1]>V[1] ) or ( M[1]>?? and M[0]>V[0]) or (M[1]<?? and M[0]<V[0]):
    #     H_S[0] = M[0]+4 * (billiard_radius) * np.cos(45)*UVhse[0] - 4 * (billiard_radius) * np.cos(45)*UVnhse[0]
    #     H_S[1] = M[1]+4 * (billiard_radius) * np.cos(45)*UVhse[1] - 4 * (billiard_radius) * np.cos(45)*UVnhse[1]
    #     H_E[0] = M[0]-4 * (billiard_radius) * np.cos(45)*UVhse[0] + 4 * (billiard_radius) * np.cos(45)*UVnhse[0]
    #     H_E[1] = M[1]-4 * (billiard_radius) * np.cos(45)*UVhse[1] + 4 * (billiard_radius) * np.cos(45)*UVnhse[1]
    #     VT = VT - 45
    

    ##------------------------避障--OB_1----------------------------
    #--------------------超出球桌範圍-------------------------
    if H_S[0] >= 26.6 or H_S[0] <= -27.6 or H_S[1] >= 55.5 or H_S[1] <= 33.5 :
        Height = BMZ + 2.8
        print ('--------------上升------------避障-----------------')
    if H_E[0] >= 29.5  or H_E[0] <= -30.5  or H_E[1] >= 58.5 or H_E[1] <= 30.5 :
        Height = BMZ + 2.8
    #     print ('--------------上升------------避障-----------------')

    #---------------------擊球起始點有球-----------------------
    AS = (H_S[1]-H_E[1])/(H_S[0]-H_E[0])
    ANS = -1/AS
    if H_S[1] > H_E[1] :                                    #----------------母球兩側各加一個r的斜線方程式-----------------
        Afun1 = AS * (OB_1[0]-(M[0]+2 * (billiard_radius) * UVnhse[0])) - (OB_1[1]-(M[1]+2 * (billiard_radius) * UVnhse[1]))
        Afun2 = AS * (OB_1[0]-(M[0]-2 * (billiard_radius) * UVnhse[0])) - (OB_1[1]-(M[1]-2 * (billiard_radius) * UVnhse[1]))
        if AS<=0:
            Afun1 = Afun1 * -1
            Afun2 = Afun2 * -1
        if Afun1 > 0 and Afun2 < 0 :
            if H_S[0] > H_E[0] :                            #----------------過母球球心及其後方5倍r距離的斜線方程式-----------------
                Afun3 = ANS * (OB_1[0]-M[0]) - (OB_1[1]-M[1])
                Afun4 = ANS * (OB_1[0]-(M[0]-5 * (billiard_radius) * UVhse[0])) - (OB_1[1]-(M[1]-5 * (billiard_radius) * UVhse[1]))
                if ANS<0:
                    Afun3 = Afun3 * -1
                    Afun4 = Afun4 * -1
                if Afun3 > 0 and Afun4 < 0 :
                    print('MoveBall')
                    MoveBall()
            if H_S[0] <H_E[0] :
                Afun3 = ANS * (OB_1[0]-M[0]) - (OB_1[1]-M[1])
                Afun4 = ANS * (OB_1[0]-(M[0]-5 * (billiard_radius) * UVhse[0])) - (OB_1[1]-(M[1]-5 * (billiard_radius) * UVhse[1]))
                if ANS<0:
                    Afun3 = Afun3 * -1
                    Afun4 = Afun4 * -1
                if Afun3 < 0 and Afun4 > 0 :
                    print('MoveBall')
                    MoveBall()
    if H_S[1] < H_E[1] :
        Afun1 = AS * (OB_1[0]-(M[0]+2 * (billiard_radius) * UVnhse[0])) - (OB_1[1]-(M[1]+2 * (billiard_radius) * UVnhse[1]))
        Afun2 = AS * (OB_1[0]-(M[0]-2 * (billiard_radius) * UVnhse[0])) - (OB_1[1]-(M[1]-2 * (billiard_radius) * UVnhse[1]))
        if AS<0:
            Afun1 = Afun1 * -1
            Afun2 = Afun2 * -1
        if Afun1 < 0 and Afun2 > 0 :
            if H_S[0] > H_E[0] :
                Afun3 = ANS * (OB_1[0]-M[0]) - (OB_1[1]-M[1])
                Afun4 = ANS * (OB_1[0]-(M[0]-5 * (billiard_radius) * UVhse[0])) - (OB_1[1]-(M[1]-5 * (billiard_radius) * UVhse[1]))
                if ANS<0:
                    Afun3 = Afun3 * -1
                    Afun4 = Afun4 * -1
                if Afun3 > 0 and Afun4 < 0 :
                    print('MoveBall')
                    MoveBall()
            if H_S[0] < H_E[0] :
                Afun3 = ANS * (OB_1[0]-M[0]) - (OB_1[1]-M[1])
                Afun4 = ANS * (OB_1[0]-(M[0]-5 * (billiard_radius) * UVhse[0])) - (OB_1[1]-(M[1]-5 * (billiard_radius) * UVhse[1]))
                if ANS<0:
                    Afun3 = Afun3 * -1
                    Afun4 = Afun4 * -1
                if Afun3 < 0 and Afun4 > 0 :
                    print('MoveBall')
                    MoveBall()


    ##------------------------避障--OB_2----------------------------
    #---------------------擊球起始點有球-----------------------
    if M[1] > V[1] :                                    #----------------母球兩側各加一個r的斜線方程式-----------------
        Afun1 = AS * (OB_2[0]-(M[0]+2 * (billiard_radius) * UVnhse[0])) - (OB_2[1]-(M[1]+2 * (billiard_radius) * UVnhse[1]))
        Afun2 = AS * (OB_2[0]-(M[0]-2 * (billiard_radius) * UVnhse[0])) - (OB_2[1]-(M[1]-2 * (billiard_radius) * UVnhse[1]))
        if AS<0:
            Afun1 = Afun1 * -1
            Afun2 = Afun2 * -1
        if Afun1 > 0 and Afun2 < 0 :
            if M[0] > V[0] :                            #----------------過母球球心及其後方5倍r距離的斜線方程式-----------------
                Afun3 = ANS * (OB_2[0]-M[0]) - (OB_2[1]-M[1])
                Afun4 = ANS * (OB_2[0]-(M[0]-5 * (billiard_radius) * UVhse[0])) - (OB_2[1]-(M[1]-5 * (billiard_radius) * UVhse[1]))
                if ANS<0:
                    Afun3 = Afun3 * -1
                    Afun4 = Afun4 * -1
                if Afun3 > 0 and Afun4 < 0 :
                    print('MoveBall')
                    MoveBall()
            if M[0] < V[0] :
                Afun3 = ANS * (OB_2[0]-M[0]) - (OB_2[1]-M[1])
                Afun4 = ANS * (OB_2[0]-(M[0]-5 * (billiard_radius) * UVhse[0])) - (OB_2[1]-(M[1]-5 * (billiard_radius) * UVhse[1]))
                if ANS<0:
                    Afun3 = Afun3 * -1
                    Afun4 = Afun4 * -1
                if Afun3 < 0 and Afun4 > 0 :
                    print('MoveBall')
                    MoveBall()
    if M[1] < V[1] :
        Afun1 = AS * (OB_2[0]-(M[0]+2 * (billiard_radius) * UVnhse[0])) - (OB_2[1]-(M[1]+2 * (billiard_radius) * UVnhse[1]))
        Afun2 = AS * (OB_2[0]-(M[0]-2 * (billiard_radius) * UVnhse[0])) - (OB_2[1]-(M[1]-2 * (billiard_radius) * UVnhse[1]))
        if AS<0:
            Afun1 = Afun1 * -1
            Afun2 = Afun2 * -1
        if Afun1 <0 and Afun2 > 0 :
            if M[0] > V[0] :
                Afun3 = ANS * (OB_2[0]-M[0]) - (OB_2[1]-M[1])
                Afun4 = ANS * (OB_2[0]-(M[0]-5 * (billiard_radius) * UVhse[0])) - (OB_2[1]-(M[1]-5 * (billiard_radius) * UVhse[1]))
                if ANS<0:
                    Afun3 = Afun3 * -1
                    Afun4 = Afun4 * -1
                if Afun3 > 0 and Afun4 < 0 :
                    print('MoveBall')
                    MoveBall()
            if M[0] < V[0] :
                Afun3 = ANS * (OB_2[0]-M[0]) - (OB_2[1]-M[1])
                Afun4 = ANS * (OB_2[0]-(M[0]-5 * (billiard_radius) * UVhse[0])) - (OB_2[1]-(M[1]-5 * (billiard_radius) * UVhse[1]))
                if ANS<0:
                    Afun3 = Afun3 * -1
                    Afun4 = Afun4 * -1
                if Afun3 < 0 and Afun4 > 0 :
                    print('MoveBall')
                    MoveBall()    
    print ('避障打母球') 
    print('H_S', H_S)
    print('H_E', H_E)
    print ('Degree after rotate :',VT)


def MOB_1C():
    global action
    global gHole
    global min_dif, num_hole
    # global Hole_info
    global V, H_S, H_E
    global M, C, OB_1, OB_2 ,VT
    global Height,BMZ
    global AS, ANS, Afun1, Afun2, Afun3, Afun4

    Afun1 = 0.0
    Afun2 = 0.0
    Afun3 = 0.0
    Afun4 = 0.0

    #------------從洞口(gHole)推算子球(C)的虛擬球1(V1)-------------------
    print ('干擾球(OB_1)打子球')
    VCtoH= np.array([gHole[0]-C[0],gHole[1]-C[1]])                  #從C指向H的向量                       
    chL = np.linalg.norm(VCtoH)                                     #CH向量的長度
    UVch = VCtoH / chL                                              #CH向量的單位向量  
    V1 = [ (C[0]-2 * (billiard_radius) * UVch[0]), (C[1]-2 * (billiard_radius) * UVch[1]) ]                 #V的球心 = 往反方向延伸兩個r的長度 
    print('V1 :', V1,'\n')

    #------------從虛擬球1(V1)推算干擾球(OB_1)的虛擬球2(V2)-------------------
    VOtoV= np.array([V1[0]-OB_1[0],V1[1]-OB_1[1]])                        #從指向的向量                       
    ovL = np.linalg.norm(VOtoV)                                     #向量的長度
    UVov = VOtoV / ovL                                              #向量的單位向量 
    V2 = [ (C[0]-2 * (billiard_radius) * UVov[0]), (C[1]-2 * (billiard_radius) * UVov[1]) ]                 #V的球心 = 往反方向延伸兩個r的長度 
    print('V2 :', V2,'\n')

    #------------計算母球(M)到干擾球(OB_1)的虛擬球2(V2)擊球軌跡-------------------
    VMtoV= np.array([V2[0]-M[0],V2[1]-M[1]])                        #從指向的向量   ##不確定可不可以不加.reshape(2, 1)                    
    mvL = np.linalg.norm(VMtoV)                                     #向量的長度
    UVmv = VMtoV / mvL                                              #向量的單位向量 
    H_S = [ (M[0]-3 * (billiard_radius) * UVmv[0]), (M[1]-3 * (billiard_radius) * UVmv[1]) ]                #M擊球開始點 = 往反方向延伸兩個r的長度
    H_E = [ ((V2[0])+(M[0]))/2, ((V2[1])+(M[1]))/2 ]                #M擊球結束點 = M和V的球心中點
    print('H_S :', H_S,'\n''H_E :',H_E,'\n')


    VHSE= np.array([H_E[0]-H_S[0],H_E[1]-H_S[1]]) 
    Vup = np.array([0, 1]).reshape(2, 1)
    uT = Vup.transpose()
    n = np.dot(uT, VHSE)
    HSEL = np.linalg.norm(VHSE)
    VT = float(np.rad2deg(np.arccos(n/HSEL)))
    if(H_E[0]>=H_S[0]):                                     #0716若子球在母球右側(即超過180度)須乘負號修正
        VT = VT * -1
    print ('OB_1擊球DEGREE :',VT)
    

     ##------------------------避障----------------------------
     #--------------------超出球桌範圍-------------------------
    if H_S[0] >= 26.6 or H_S[0] <= -27.6 or H_S[1] >= 55.5 or H_S[1] <= 33.5 :
        Height = BMZ + 2.8
        print ('--------------上升------------避障-----------------')


     #---------------------擊球起始點有球-----------------------
    NVMtoV= np.array([V2[1]-M[1] ,(V2[0]-M[0])* -1])   #法向量運算
    nmvL = np.linalg.norm(NVMtoV)
    UVnmv = NVMtoV / nmvL                            #單位法向量

   
    AS = (M[1]-V[1])/(M[0]-V[0])
    ANS = -1/AS
    if M[1] > V[1] :                                    #----------------母球兩側各加一個r的斜線方程式-----------------
        Afun1 = AS * (OB_2[0]-(M[0]+2 * (billiard_radius) * UVnmv[0])) - (OB_2[1]-(M[1]+2 * (billiard_radius) * UVnmv[1]))
        Afun2 = AS * (OB_2[0]-(M[0]-2 * (billiard_radius) * UVnmv[0])) - (OB_2[1]-(M[1]-2 * (billiard_radius) * UVnmv[1]))
        if AS<0:
            Afun1 = Afun1 * -1
            Afun2 = Afun2 * -1
        if Afun1 > 0 and Afun2 < 0 :
            if M[0] > V[0] :                            #----------------過母球球心及其後方5倍r距離的斜線方程式-----------------
                Afun3 = ANS * (OB_2[0]-M[0]) - (OB_2[1]-M[1])
                Afun4 = ANS * (OB_2[0]-(M[0]-5 * (billiard_radius) * UVmv[0])) - (OB_2[1]-(M[1]-5 * (billiard_radius) * UVmv[1]))
                if ANS<0:
                    Afun3 = Afun3 * -1
                    Afun4 = Afun4 * -1
                if Afun3 > 0 and Afun4 < 0 :
                    print('MoveBall')
                    MoveBall()
            if M[0] < V[0] :
                Afun3 = ANS * (OB_2[0]-M[0]) - (OB_2[1]-M[1])
                Afun4 = ANS * (OB_2[0]-(M[0]-5 * (billiard_radius) * UVmv[0])) - (OB_2[1]-(M[1]-5 * (billiard_radius) * UVmv[1]))
                if ANS<0:
                    Afun3 = Afun3 * -1
                    Afun4 = Afun4 * -1
                if Afun3 < 0 and Afun4 > 0 :
                    print('MoveBall')
                    MoveBall()
    if M[1] < V[1] :
        Afun1 = AS * (OB_2[0]-(M[0]+2 * (billiard_radius) * UVnmv[0])) - (OB_2[1]-(M[1]+2 * (billiard_radius) * UVnmv[1]))
        Afun2 = AS * (OB_2[0]-(M[0]-2 * (billiard_radius) * UVnmv[0])) - (OB_2[1]-(M[1]-2 * (billiard_radius) * UVnmv[1]))
        if AS<0:
            Afun1 = Afun1 * -1
            Afun2 = Afun2 * -1
        if Afun1 < 0 and Afun2 > 0 :
            if M[0] > V[0] :
                Afun3 = ANS * (OB_2[0]-M[0]) - (OB_2[1]-M[1])
                Afun4 = ANS * (OB_2[0]-(M[0]-5 * (billiard_radius) * UVmv[0])) - (OB_2[1]-(M[1]-5 * (billiard_radius) * UVmv[1]))
                if ANS<0:
                    Afun3 = Afun3 * -1
                    Afun4 = Afun4 * -1
                if Afun3 > 0 and Afun4 < 0 :
                    print('MoveBall')
                    MoveBall()
            if M[0] < V[0] :
                Afun3 = ANS * (OB_2[0]-M[0]) - (OB_2[1]-M[1])
                Afun4 = ANS * (OB_2[0]-(M[0]-5 * (billiard_radius) * UVmv[0])) - (OB_2[1]-(M[1]-5 * (billiard_radius) * UVmv[1]))
                if ANS<0:
                    Afun3 = Afun3 * -1
                    Afun4 = Afun4 * -1
                if Afun3 < 0 and Afun4 > 0 :
                    print('MoveBall')
                    MoveBall()

def MOB_2C():
    global action
    global gHole
    global min_dif, num_hole
    # global Hole_info
    global V, H_S, H_E, M, C ,OB_1 ,OB_2
    global VT
    global Height,BMZ
    global V1, V2
    global AS, ANS, Afun1, Afun2, Afun3, Afun4

    Afun1 = 0.0
    Afun2 = 0.0
    Afun3 = 0.0
    Afun4 = 0.0
    

    #------------從洞口(gHole)推算子球(C)的虛擬球1(V1)-------------------
    VCtoH= np.array([gHole[0]-C[0],gHole[1]-C[1]])                  #從C指向H的向量                       
    chL = np.linalg.norm(VCtoH)                                     #CH向量的長度
    UVch = VCtoH / chL                                              #CH向量的單位向量  
    V1 = [ (C[0]-2 * (billiard_radius) * UVch[0]), (C[1]-2 * (billiard_radius) * UVch[1]) ]                 #V的球心 = 往反方向延伸兩個r的長度 
    print('V1 :', V1,'\n')

    #------------從虛擬球1(V1)推算干擾球(OB_2)的虛擬球2(V2)-------------------
    print ('干擾球(OB_2)打子球')
    VOtoV= np.array([V1[0]-OB_2[0],V1[1]-OB_2[1]])                        #從指向的向量                       
    ovL = np.linalg.norm(VOtoV)                                     #向量的長度
    UVov = VOtoV / ovL                                              #向量的單位向量 
    V2 = [ (C[0]-2 * (billiard_radius) * UVov[0]), (C[1]-2 * (billiard_radius) * UVov[1]) ]                 #V的球心 = 往反方向延伸兩個r的長度 
    print('V2 :', V2,'\n')

    #------------計算母球(M)到干擾球(OB_2)的虛擬球2(V2)擊球軌跡-------------------
    VMtoV= np.array([V2[0]-M[0],V2[1]-M[1]])                        #從指向的向量   ##不確定可不可以不加.reshape(2, 1)                    
    mvL = np.linalg.norm(VMtoV)                                     #向量的長度
    UVmv = VMtoV / mvL                                              #向量的單位向量 
    H_S = [ (M[0]-3 * (billiard_radius) * UVmv[0]), (M[1]-3 * (billiard_radius) * UVmv[1]) ]                #M擊球開始點 = 往反方向延伸兩個r的長度
    H_E = [ ((V2[0])+(M[0]))/2, ((V2[1])+(M[1]))/2 ]                #M擊球結束點 = M和V的球心中點
    print('H_S :', H_S,'\n''H_E :',H_E,'\n')


    VHSE= np.array([H_E[0]-H_S[0],H_E[1]-H_S[1]]) 
    Vup = np.array([0, 1]).reshape(2, 1)
    uT = Vup.transpose()
    n = np.dot(uT, VHSE)
    HSEL = np.linalg.norm(VHSE)
    VT = float(np.rad2deg(np.arccos(n/HSEL)))
    if(H_E[0]>H_S[0]):                                     #0716若子球在母球右側(即超過180度)須乘負號修正
        VT = VT * -1
    print ('OB_2擊球DEGREE :',VT)
    

     ##------------------------避障----------------------------
     #--------------------超出球桌範圍-------------------------
    if H_S[0] >= 26.6 or H_S[0] <= -27.6 or H_S[1] >= 55.5 or H_S[1] <= 33.5 :
        Height = BMZ + 2.8
        print ('--------------上升------------避障-----------------')


     #---------------------yolo_client點有球-----------------------
    NVMtoV= np.array([V2[1]-M[1] ,(V2[0]-M[0])* -1])   #法向量運算
    nmvL = np.linalg.norm(NVMtoV)
    UVnmv = NVMtoV / nmvL                            #單位法向量


    AS = (M[1]-V[1])/(M[0]-V[0])
    ANS = -1/AS
    if M[1] > V[1] :                                    #----------------母球兩側各加一個r的斜線方程式-----------------
        Afun1 = AS * (OB_1[0]-(M[0]+2 * (billiard_radius) * UVnmv[0])) - (OB_1[1]-(M[1]+2 * (billiard_radius) * UVnmv[1]))
        Afun2 = AS * (OB_1[0]-(M[0]-2 * (billiard_radius) * UVnmv[0])) - (OB_1[1]-(M[1]-2 * (billiard_radius) * UVnmv[1]))
        if AS<0:
            Afun1 = Afun1 * -1
            Afun2 = Afun2 * -1
        if Afun1 > 0 and Afun2 < 0 :
            if M[0] > V[0] :                            #----------------過母球球心及其後方5倍r距離的斜線方程式-----------------
                Afun3 = ANS * (OB_1[0]-M[0]) - (OB_1[1]-M[1])
                Afun4 = ANS * (OB_1[0]-(M[0]-5 * (billiard_radius) * UVmv[0])) - (OB_1[1]-(M[1]-5 * (billiard_radius) * UVmv[1]))
                if ANS<0:
                    Afun3 = Afun3 * -1
                    Afun4 = Afun4 * -1
                if Afun3 > 0 and Afun4 < 0 :
                    print('MoveBall')
                    MoveBall()
            if M[0] < V[0] :
                Afun3 = ANS * (OB_1[0]-M[0]) - (OB_1[1]-M[1])
                Afun4 = ANS * (OB_1[0]-(M[0]-5 * (billiard_radius) * UVmv[0])) - (OB_1[1]-(M[1]-5 * (billiard_radius) * UVmv[1]))
                if ANS<0:
                    Afun3 = Afun3 * -1
                    Afun4 = Afun4 * -1
                if Afun3 < 0 and Afun4 > 0 :
                    print('MoveBall')
                    MoveBall()
    if M[1] < V[1] :
        Afun1 = AS * (OB_1[0]-(M[0]+2 * (billiard_radius) * UVnmv[0])) - (OB_1[1]-(M[1]+2 * (billiard_radius) * UVnmv[1]))
        Afun2 = AS * (OB_1[0]-(M[0]-2 * (billiard_radius) * UVnmv[0])) - (OB_1[1]-(M[1]-2 * (billiard_radius) * UVnmv[1]))
        if AS<0:
            Afun1 = Afun1 * -1
            Afun2 = Afun2 * -1
        if Afun1 < 0 and Afun2 > 0 :
            if M[0] > V[0] :
                Afun3 = ANS * (OB_1[0]-M[0]) - (OB_1[1]-M[1])
                Afun4 = ANS * (OB_1[0]-(M[0]-5 * (billiard_radius) * UVmv[0])) - (OB_1[1]-(M[1]-5 * (billiard_radius) * UVmv[1]))
                if ANS<0:
                    Afun3 = Afun3 * -1
                    Afun4 = Afun4 * -1
                if Afun3 > 0 and Afun4 < 0 :
                    print('MoveBall')
                    MoveBall()
            if M[0] < V[0] :
                Afun3 = ANS * (OB_1[0]-M[0]) - (OB_1[1]-M[1])
                Afun4 = ANS * (OB_1[0]-(M[0]-5 * (billiard_radius) * UVmv[0])) - (OB_1[1]-(M[1]-5 * (billiard_radius) * UVmv[1]))
                if ANS<0:
                    Afun3 = Afun3 * -1
                    Afun4 = Afun4 * -1
                if Afun3 < 0 and Afun4 > 0 :
                    print('MoveBall')
                    MoveBall()

def yolo_pic():
    init_resp = init_yolo_picture(get_image_1.get_image())
    # init_resp = init_yolo_picture("/home/iclab/catkin_ws/src/yolo_v3/scripts/15.jpg")
    time.sleep(2.5)
    resp = call_yolo_v3_picture(1)
    # print(resp.Image.ROI_list)  #印出 回傳的座標字串
    global pic_OB_1,pic_OB_2
    pic_OB_1 = [0.0, 0.0]
    pic_OB_2 = [0.0, 0.0]
    
    for i in range(len(resp.Image.ROI_list)): # 分析座標字串
        global temp_list
        temp_list = ['', 0.0, 0.0, 0.0]
        global temp_str
        # temp_str = str('') #initial temp_str as empty string
        temp_str = str(resp.Image.ROI_list[i])
        get_str_mid ()
        curve()
    
    global Array_2D
    print("==================================================================")
    print("Array_2D:")
    print(Array_2D)
    print("==================================================================")
    print("pic_M")
    print(pic_M)
    print("==================================================================")
    print("pic_C")
    print(pic_C)
    print("==================================================================")


def Mission_Trigger():
    global action,M,C
    global gHole
    global H1,H2,H3,H4,H5,H6
    global min_dif, num_hole
    # global Hole_info
    global V, H_S, H_E
    global VT
    global Height,BMZ

    global Array_2D
    Array_2D = [['0', 0.0, 0.0, 0.0] for i in range(9)] # two-dimension 4*9 to be all zero
    
    if Arm_state_flag == Arm_status.Idle and Sent_data_flag == 1:
        #Sent_data_flag = 0
        #Arm_state_flag = Arm_status.Isbusy
        
        for case in switch(action): #傳送指令給socket選擇手臂動作
            if case(0):            #---------------------------拍照位置---------------------------
                pos.x = BMX + 30.7
                pos.y = BMY + 2
                pos.z = BMZ + 34
                pos.pitch = -90 
                pos.roll = 0  
                pos.yaw = 0 
                ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
                ArmTask.Arm_Mode(2,1,0,100,2)#action,ra,grip,vel,both
                print('x: ',pos.x,' y: ',pos.y,' z: ',pos.z,' pitch: ',pos.pitch,' roll: ',pos.roll,' yaw: ',pos.yaw)
                action = 1
                break
            if case(1):             #------------------------擊球起始點上方--------------------------
                time.sleep(0.8)

                global NineOnTable 
                NineOnTable = 0
                yolo_pic()
                if(NineOnTable == 0):
                    break

                GetStartEnd()
                pos.x = ((H_S[0]-1)/2)+1
                pos.y = ((H_S[1]-36.05)/2)+36.05
                pos.z = 0
                pos.pitch = -90 
                pos.roll = 0  
                pos.yaw = VT
                ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
                ArmTask.Arm_Mode(2,1,0,95,2)#action,ra,grip,vel,both
                print('x: ',pos.x,' y: ',pos.y,' z: ',pos.z,' pitch: ',pos.pitch,' roll: ',pos.roll,' yaw: ',pos.yaw)
                action = 2
                break
            if case(2):                #---------------------------------------------
                pos.x = H_S[0]
                pos.y = H_S[1]
                pos.z = -10
                pos.pitch = -90 
                pos.roll = 0
                pos.yaw = VT 
                ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
                ArmTask.Arm_Mode(3,1,0,90,2)#action,ra,grip,vel,both
                print('x: ',pos.x,' y: ',pos.y,' z: ',pos.z,' pitch: ',pos.pitch,' roll: ',pos.roll,' yaw: ',pos.yaw)
                action = 3
                break
            if case(3):                #------------------------擊球起始點---------------------
                pos.x = H_S[0]
                pos.y = H_S[1]
                pos.z = Height
                pos.pitch = -90 
                pos.roll = 0
                pos.yaw = VT 
                ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
                ArmTask.Arm_Mode(3,1,0,15,2)#action,ra,grip,vel,both
                print('x: ',pos.x,' y: ',pos.y,' z: ',pos.z,' pitch: ',pos.pitch,' roll: ',pos.roll,' yaw: ',pos.yaw)
                action = 4
                break
            if case(4):                  #------------------------擊球-----------------------
                VMtoH= np.array([gHole[0]-M[0],gHole[1]-M[1]])
                mhL = np.linalg.norm(VMtoH)
                VMtoC= np.array([C[0]-M[0],C[1]-M[1]]).reshape(2, 1)
                Lmc = np.linalg.norm(VMtoC)

                if mhL >30 :
                    if Lmc >20 :
                        hitspeed = 100
                        print('hitspeed 100')
                    elif Lmc <=20 :
                        hitspeed = 100
                        print('hitspeed 100')
                elif mhL <= 30 :
                    if Lmc >20 :
                        hitspeed = 100
                        print('hitspeed 100')
                    elif Lmc <=20 :
                        hitspeed = 100
                        print('hitspeed 100')
                pos.x = H_E[0]
                pos.y = H_E[1]
                pos.z = Height
                pos.pitch = -90 
                pos.roll = 0 
                pos.yaw = VT
                ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
                ArmTask.Arm_Mode(3,1,0,hitspeed,2)#action,ra,grip,vel,both
                print('x: ',pos.x,' y: ',pos.y,' z: ',pos.z,' pitch: ',pos.pitch,' roll: ',pos.roll,' yaw: ',pos.yaw)
                action = 5
                break
            if case(5):                  #-----------------------擊球結束點上方--------------------------------
                pos.x = H_E[0]
                pos.y = H_E[1]
                pos.z = 0
                pos.pitch = -90 
                pos.roll = 0  
                pos.yaw = VT 
                ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
                ArmTask.Arm_Mode(3,1,0,85,2)#action,ra,grip,vel,both
                print('x: ',pos.x,' y: ',pos.y,' z: ',pos.z,' pitch: ',pos.pitch,' roll: ',pos.roll,' yaw: ',pos.yaw)
                action = 6
                break
            if case(6):                   #--------------------拍照位置----------------------------------------
                pos.x = BMX + 30.7
                pos.y = BMY + 2
                pos.z = BMZ + 34
                pos.pitch = -90 
                pos.roll = 0  
                pos.yaw = 0 
                ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
                ArmTask.Arm_Mode(2,1,0,95,2)#action,ra,grip,vel,both
                print('x: ',pos.x,' y: ',pos.y,' z: ',pos.z,' pitch: ',pos.pitch,' roll: ',pos.roll,' yaw: ',pos.yaw)
                action = 0
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
    srv = Server(TutorialsConfig, rqt_callback)
    GetInfoFlag = True #Test no data
    arm_state_listener()
    start_input=int(input('開始策略請按1,離開請按3 : ')) #輸入開始指令
    #start_input = 1
    if start_input==1:  
        ArmTask.Speed_Mode(1)
        ArmTask.Arm_Mode(4,1,0,10,2)  
        while 1:
            time.sleep(0.1) #0628 最穩定 delay 0.2秒
            # 加入cnt
            # my_list=[]
            # for i in range(1000000):
            #     my_list.append(i)
            Mission_Trigger()
    if start_input == 3:
        pass
    #timer.join()
    ArmTask.rospy.spin()
    rospy.spin()


