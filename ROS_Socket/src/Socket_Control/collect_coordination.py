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
# image_x = [ 177, 162.5, 162.5, 168, 178.5, 616, 607, 607.5, 611, 622, 1335, 1345.5, 1350, 1348, 1345.5, 1817.5, 1831, 1833, 1839, 1838, 
#             308.5, 599.5, 447.5, 184.5, 756.5, 1161.5, 1327, 1484, 1639, 1767.5, 1788, 
#             1084, 1261.5, 1428, 1584, 1715, 1845, 
#             1093.5, 1271, 1440.5, 1594.5, 1737, 1863.5, 
#             1097, 1279, 1446.5, 1602.5, 1739.5, 1867.5, 
#             1099, 1282, 1447.5, 1602, 1741.5, 1865, 
#             1097.5, 1277, 1440, 1598, 1734.5, 1858.5, 
#             129, 240.5, 378.5, 524, 685, 855.5, 
#             120, 231.5, 364, 516.5, 680, 854.5, 
#             112.5, 227, 363.5, 514, 672.5, 846.5, 
#             104, 221, 350.5, 500.5, 661.5, 836, 
#             131, 243, 374, 521.5, 684, 852] #from0~9, without 1 影像中的X值
# arm_x = [   -28, -28, -28, -28, -28, -12, -12, -12, -12, -12, 9, 9, 9, 9, 9, 26, 26, 26, 26, 26, 
#             -27.5, -22.5, -17.5, -12.5, -7.5, 4, 9, 14, 19, 24, 24, 
#             2, 7, 12, 17, 22, 27, 
#             2, 7, 12, 17, 22, 27, 
#             2, 7, 12, 17, 22, 27, 
#             2, 7, 12, 17, 22, 27, 
#             2, 7, 12, 17, 22, 27, 
#             -30, -25, -20, -15, -10, -5, 
#             -30, -25, -20, -15, -10, -5, 
#             -30, -25, -20, -15, -10, -5, 
#             -30, -25, -20, -15, -10, -5, 
#             -30, -25, -20, -15, -10, -5, 
#             ] #from0~9, without 1 手臂的X值
# image_y = [ 1103.5, 923.5, 742.5, 553.5, 398.5, 1147, 947, 775, 565, 363, 1092.5, 922, 746.5, 569, 400, 1099.5, 455, 948.5, 791.5, 635, 
#             1176.5, 1207, 1197, 1168, 1219.5, 1196, 1195.5, 1186.5, 1170, 1155, 1005.5, 
#             1108, 1106, 1101, 1090.5, 1079, 1069, 
#             930, 929, 924, 924, 921.5, 921.5, 
#             749, 948, 751, 956, 757, 757, 
#             566, 572, 570.5, 584.5, 594.5, 603, 
#             390.5, 399.5, 406.5, 426.5, 438, 455, 
#             451, 438, 413.5, 411, 395, 389, 
#             592, 583, 577, 573, 569, 562, 
#             744.5, 744, 748, 748.5, 754, 744, 
#             909, 914, 920, 928.5, 930.5, 932, 
#             1044.5, 1051.5, 1067, 1080.5, 1088.5, 1094] #from0~9, without 1 影像中的Y值
# arm_y = [   33, 39, 45, 51, 56, 33, 39, 44, 50, 56, 35, 40, 45, 50, 55, 34, 39, 44, 49, 55, 
#             31.5, 31.5, 31.5, 31.5, 31.5, 32, 32, 32, 32, 32, 37,
#             35, 35, 35, 35, 35, 35, 
#             40, 40, 40, 40, 40, 40, 
#             45, 45, 45, 45, 45, 45, 
#             50, 50, 50, 50, 50, 50, 
#             55, 55, 55, 55, 55, 55, 
#             55, 55, 55, 55, 55, 55, 
#             50, 50, 50, 50, 50, 50, 
#             45, 45, 45, 45, 45, 45, 
#             40, 40, 40, 40, 40, 40, 
#             35, 35, 35, 35, 35, 35 ] #from0~9, without 1 手臂的Y值
global pic_M, pic_C, pic_OB_1, pic_OB_2
pic_M = [0.0, 0.0]
pic_C = [0.0, 0.0]
pic_OB_1 = [0.0, 0.0]
pic_OB_2 = [0.0, 0.0]
# --------------------------------
global i_dont_know
i_dont_know = 0
print(i_dont_know)
global action   # Mission_Trigger 中 action 初始值 讓第一次的case 從0開始
action = 0

global num_step
num_step = 0

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
    Height = config.BMZ + 1.3
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
    fx = np.polyfit(I_X, A_X, 2)              # x最高次方係數是1
    cfx = np.poly1d(fx)                       # 產生出線性方程式

    global image_y  
    I_Y = np.array(image_y)
    global arm_y 
    A_Y = np.array(arm_y)
    fy = np.polyfit(I_Y, A_Y, 2)  
    cfy = np.poly1d(fy)                        

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
# -----------------------

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
        
        # global pic_OB_1, pic_OB_2
        # if ((int(dict_ball[s_obj_name] ) != 0) and (int(dict_ball[s_obj_name] ) != 9) and pic_OB_1 == [0.0, 0.0]):
            # pic_OB_1[0] = MID_X
            # pic_OB_1[1] = MID_Y
        # elif ((int(dict_ball[s_obj_name] ) != 0) and (int(dict_ball[s_obj_name] ) != 9) and pic_OB_2 == [0.0, 0.0]):
            # pic_OB_2[0] = MID_X
            # pic_OB_2[1] = MID_Y
        
        
        # global pic_C, NineOnTable
        # if(int(dict_ball[s_obj_name])==9):
            # pic_C = [MID_X, MID_Y]
            # if (pic_C == [0.0, 0.0]) :
                # NineOnTable = 0
            # else :
                # NineOnTable = 1
        # SaveDirectory = os.getcwd()
        SaveDirectory = '/home/iclab/Desktop'
        image_X_path = SaveDirectory +'/image_X'+'.txt'
        image_X_file = open(image_X_path, 'a')
        image_Y_path = SaveDirectory +'/image_Y'+'.txt'
        image_Y_file = open(image_Y_path, 'a')

        for i in range(0,10): #check if score achieve, it will be record on Array_2D
            if( int(dict_ball[s_obj_name] ) == i): # check number of the ball
                for index in range(len(temp_list)): #put value in, one by one
                    global Array_2D
                    Array_2D[int(dict_ball[s_obj_name])][index] = temp_list[index] 
                global i_dont_know
                if i_dont_know > 2 :
                    print(i_dont_know)
                    image_X_file.write(str(temp_list[2])+', ')
                    image_Y_file.write(str(temp_list[3])+', ')
    else:
        pass

    
'''
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

    Height = BMZ + 1.3

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

    Vmh6 = np.array([H2[0]-M[0],H2[1]-M[1]]).reshape(2, 1)
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

    min_dif = min(check_hole[1:6])       #確認最小差值

    num_hole = check_hole.index(min_dif) #差值最小洞號 為目標洞口 取得目標洞口 index值 存放置 num_hole #前面要做一個 字典 存 各個洞口座標
    print("目標洞口是: ",num_hole,"號洞口")
    gHole = Hole_info.get(num_hole)
    print("目標洞口座標是: ", gHole)
    
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
        if KS<=0:
            Kfun1 = Kfun1 * -1
            Kfun2 = Kfun2 * -1    #斜率正時點帶入大於0即在右側小於0在左側---斜率負時相反故乘一個負號
        if Kfun1 >= 0 and Kfun2 <= 0 :
            if M[0] > C[0] :                             #----------------過母球球心及子球球心的斜線方程式-----------------
                Kfun3 = KNS * (OB_1[0]-M[0]) - (OB_1[1]-M[1])
                Kfun4 = KNS * (OB_1[0]-C[0]) - (OB_1[1]-C[1])
                if KNS<=0:
                    Kfun3 = Kfun3 * -1
                    Kfun4 = Kfun4 * -1
                if Kfun3 <= 0 and Kfun4 >= 0 :
                    print('Use kiss ball(OB_1)')
                    kissball = 1
                    MOB_1C()
            if M[0] <= C[0] :
                Kfun3 = KNS * (OB_1[0]-M[0]) - (OB_1[1]-M[1])
                Kfun4 = KNS * (OB_1[0]-C[0]) - (OB_1[1]-C[1])
                if KNS<=0:
                    Kfun3 = Kfun3 * -1
                    Kfun4 = Kfun4 * -1
                if Kfun3 >= 0 and Kfun4 <= 0 :
                    print('Use kiss ball(OB_1)')
                    kissball = 1
                    MOB_1C()

    if M[1] <= C[1] :
        Kfun1 = KS * (OB_1[0]-(M[0]+2 * (billiard_radius) * UVnmc[0])) - (OB_1[1]-(M[1]+2 * (billiard_radius) * UVnmc[1]))
        Kfun2 = KS * (OB_1[0]-(M[0]-2 * (billiard_radius) * UVnmc[0])) - (OB_1[1]-(M[1]-2 * (billiard_radius) * UVnmc[1]))
        if KS<=0:
            Kfun1 = Kfun1 * -1
            Kfun2 = Kfun2 * -1
        if Kfun1 <= 0 and Kfun2 >= 0 :
            if M[0] > C[0] :
                Kfun3 = KNS * (OB_1[0]-M[0]) - (OB_1[1]-M[1])
                Kfun4 = KNS * (OB_1[0]-C[0]) - (OB_1[1]-C[1])
            if KNS<=0:
                Kfun3 = Kfun3 * -1
                Kfun4 = Kfun4 * -1
                if Kfun3 <= 0 and Kfun4 >= 0 :
                    print('Use kiss ball(OB_1)')
                    kissball = 1
                    MOB_1C()
            if M[0] <= C[0] :
                Kfun3 = KNS * (OB_1[0]-M[0]) - (OB_1[1]-M[1])
                Kfun4 = KNS * (OB_1[0]-C[0]) - (OB_1[1]-C[1])
                if KNS<=0:
                    Kfun3 = Kfun3 * -1
                    Kfun4 = Kfun4 * -1
                if Kfun3 >= 0 and Kfun4 <= 0 :
                    print('Use kiss ball(OB_1)')
                    kissball = 1
                    MOB_1C()


    #----------------母球與子球連線上有球(用KISS)的判斷式----OB_2--------------------
    if M[1] > C[1] :                                     #----------------母球及子球兩側的斜線方程式-----------------
        Kfun1 = KS * (OB_2[0]-(M[0]+2 * (billiard_radius) * UVnmc[0])) - (OB_2[1]-(M[1]+2 * (billiard_radius) * UVnmc[1]))
        Kfun2 = KS * (OB_2[0]-(M[0]-2 * (billiard_radius) * UVnmc[0])) - (OB_2[1]-(M[1]-2 * (billiard_radius) * UVnmc[1]))
        if KS<=0:
            Kfun1 = Kfun1 * -1
            Kfun2 = Kfun2 * -1
        if Kfun1 >= 0 and Kfun2 <= 0 :
            if M[0] > C[0] :                             #----------------過母球球心及子球球心的斜線方程式-----------------
                Kfun3 = KNS * (OB_2[0]-M[0]) - (OB_2[1]-M[1])
                Kfun4 = KNS * (OB_2[0]-C[0]) - (OB_2[1]-C[1])
                if KNS<=0:
                    Kfun3 = Kfun3 * -1
                    Kfun4 = Kfun4 * -1
                if Kfun3 <= 0 and Kfun4 >= 0 :
                    print('Use kiss ball(OB_2')
                    kissball = 1
                    MOB_2C()
            if M[0] <= C[0] :
                Kfun3 = KNS * (OB_2[0]-M[0]) - (OB_2[1]-M[1])
                Kfun4 = KNS * (OB_2[0]-C[0]) - (OB_2[1]-C[1])
                if KNS<=0:
                    Kfun3 = Kfun3 * -1
                    Kfun4 = Kfun4 * -1
                if Kfun3 >= 0 and Kfun4 <= 0 :
                    print('Use kiss ball(OB_2)')
                    kissball = 1
                    MOB_2C()

    if M[1] <= C[1] :
        Kfun1 = KS * (OB_2[0]-(M[0]+2 * (billiard_radius) * UVnmc[0])) - (OB_2[1]-(M[1]+2 * (billiard_radius) * UVnmc[1]))
        Kfun2 = KS * (OB_2[0]-(M[0]-2 * (billiard_radius) * UVnmc[0])) - (OB_2[1]-(M[1]-2 * (billiard_radius) * UVnmc[1]))
        if KS<=0:
                    Kfun1 = Kfun1 * -1
                    Kfun2 = Kfun2 * -1
        if Kfun1 <= 0 and Kfun2 >= 0 :
            if M[0] > C[0] :
                Kfun3 = KNS * (OB_2[0]-M[0]) - (OB_2[1]-M[1])
                Kfun4 = KNS * (OB_2[0]-C[0]) - (OB_2[1]-C[1])
            if KNS<=0:
                Kfun3 = Kfun3 * -1
                Kfun4 = Kfun4 * -1
                if Kfun3 <= 0 and Kfun4 >= 0 :
                    print('Use kiss ball(OB_2)')
                    kissball = 1
                    MOB_2C()
            if M[0] <= C[0] :
                Kfun3 = KNS * (OB_2[0]-M[0]) - (OB_2[1]-M[1])
                Kfun4 = KNS * (OB_2[0]-C[0]) - (OB_2[1]-C[1])
                if KNS<=0:
                    Kfun3 = Kfun3 * -1
                    Kfun4 = Kfun4 * -1
                if Kfun3 >= 0 and Kfun4 <= 0 :
                    print('Use kiss ball(OB_2)')
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
        print('OB_1',OB_1)
        print('OB_2',OB_2)
        
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
        #if H_S[0] >= 31.5 or H_S[0] <= -27 or H_S[1] >= 62 or H_S[1] <= 35.5 :
        if H_S[0] >= 28 or H_S[0] <= -28 or H_S[1] >= 56 or H_S[1] <= 33 :
            Height = BMZ + 2.6
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
            print('Afun1',Afun1, 'Afun2',Afun2)
            if AS<=0:
                Afun1 = Afun1 * -1
                Afun2 = Afun2 * -1
            if Afun1 >= 0 and Afun2 <= 0 :
                if M[0] > V[0] :                            #----------------過母球球心及其後方5倍r距離的斜線方程式-----------------
                    Afun3 = ANS * (OB_1[0]-M[0]) - (OB_1[1]-M[1])
                    Afun4 = ANS * (OB_1[0]-(M[0]-5 * (billiard_radius) * UVmv[0])) - (OB_1[1]-(M[1]-5 * (billiard_radius) * UVmv[1]))
                    print('Afun3',Afun3, 'Afun4',Afun4)
                    if ANS<=0:
                        Afun3 = Afun3 * -1
                        Afun4 = Afun4 * -1
                    if Afun3 >= 0 and Afun4 <= 0 :
                        print('MoveBall')
                        MoveBall()
                if M[0] <= V[0] :
                    Afun3 = ANS * (OB_1[0]-M[0]) - (OB_1[1]-M[1])
                    Afun4 = ANS * (OB_1[0]-(M[0]-5 * (billiard_radius) * UVmv[0])) - (OB_1[1]-(M[1]-5 * (billiard_radius) * UVmv[1]))
                    print('Afun3',Afun3, 'Afun4',Afun4)
                    if ANS<=0:
                        Afun3 = Afun3 * -1
                        Afun4 = Afun4 * -1
                    if Afun3 <= 0 and Afun4 >= 0 :
                        print('MoveBall')
                        MoveBall()
        if M[1] <= V[1] :
            print('M', M,'V', V,'OB_1', OB_1)
            Afun1 = AS * (OB_1[0]-(M[0]+2 * (billiard_radius) * UVnmv[0])) - (OB_1[1]-(M[1]+2 * (billiard_radius) * UVnmv[1]))
            Afun2 = AS * (OB_1[0]-(M[0]-2 * (billiard_radius) * UVnmv[0])) - (OB_1[1]-(M[1]-2 * (billiard_radius) * UVnmv[1]))
            print('Afun1',Afun1, 'Afun2',Afun2)
            if AS<=0:
                Afun1 = Afun1 * -1
                Afun2 = Afun2 * -1
            if Afun1 <= 0 and Afun2 >= 0 :
                if M[0] > V[0] :
                    Afun3 = ANS * (OB_1[0]-M[0]) - (OB_1[1]-M[1])
                    Afun4 = ANS * (OB_1[0]-(M[0]-5 * (billiard_radius) * UVmv[0])) - (OB_1[1]-(M[1]-5 * (billiard_radius) * UVmv[1]))
                    print('Afun3',Afun3, 'Afun4',Afun4)
                    if ANS<=0:
                        Afun3 = Afun3 * -1
                        Afun4 = Afun4 * -1
                    if Afun3 >= 0 and Afun4 <= 0 :
                        print('MoveBall')
                        MoveBall()
                if M[0] <= V[0] :
                    Afun3 = ANS * (OB_1[0]-M[0]) - (OB_1[1]-M[1])
                    Afun4 = ANS * (OB_1[0]-(M[0]-5 * (billiard_radius) * UVmv[0])) - (OB_1[1]-(M[1]-5 * (billiard_radius) * UVmv[1]))
                    print('Afun3',Afun3, 'Afun4',Afun4)
                    if ANS<=0:
                        Afun3 = Afun3 * -1
                        Afun4 = Afun4 * -1
                    if Afun3 <= 0 and Afun4 >= 0 :
                        print('MoveBall')
                        MoveBall()


        ##------------------------避障--OB_2----------------------------
        #---------------------擊球起始點有球-----------------------
        if M[1] > V[1] :                                    #----------------母球兩側各加一個r的斜線方程式-----------------
            Afun1 = AS * (OB_2[0]-(M[0]+2 * (billiard_radius) * UVnmv[0])) - (OB_2[1]-(M[1]+2 * (billiard_radius) * UVnmv[1]))
            Afun2 = AS * (OB_2[0]-(M[0]-2 * (billiard_radius) * UVnmv[0])) - (OB_2[1]-(M[1]-2 * (billiard_radius) * UVnmv[1]))
            if AS<=0:
                Afun1 = Afun1 * -1
                Afun2 = Afun2 * -1
            if Afun1 >= 0 and Afun2 <= 0 :
                if M[0] > V[0] :                            #----------------過母球球心及其後方5倍r距離的斜線方程式-----------------
                    Afun3 = ANS * (OB_2[0]-M[0]) - (OB_2[1]-M[1])
                    Afun4 = ANS * (OB_2[0]-(M[0]-5 * (billiard_radius) * UVmv[0])) - (OB_2[1]-(M[1]-5 * (billiard_radius) * UVmv[1]))
                    if ANS<=0:
                        Afun3 = Afun3 * -1
                        Afun4 = Afun4 * -1
                    if Afun3 >= 0 and Afun4 <= 0 :
                        print('MoveBall')
                        MoveBall()
                if M[0] <= V[0] :
                    Afun3 = ANS * (OB_2[0]-M[0]) - (OB_2[1]-M[1])
                    Afun4 = ANS * (OB_2[0]-(M[0]-5 * (billiard_radius) * UVmv[0])) - (OB_2[1]-(M[1]-5 * (billiard_radius) * UVmv[1]))
                    if ANS<=0:
                        Afun3 = Afun3 * -1
                        Afun4 = Afun4 * -1
                    if Afun3 <= 0 and Afun4 >= 0 :
                        print('MoveBall')
                        MoveBall()
        if M[1] <= V[1] :
            Afun1 = AS * (OB_2[0]-(M[0]+2 * (billiard_radius) * UVnmv[0])) - (OB_2[1]-(M[1]+2 * (billiard_radius) * UVnmv[1]))
            Afun2 = AS * (OB_2[0]-(M[0]-2 * (billiard_radius) * UVnmv[0])) - (OB_2[1]-(M[1]-2 * (billiard_radius) * UVnmv[1]))
            if AS<=0:
                Afun1 = Afun1 * -1
                Afun2 = Afun2 * -1
            if Afun1 <= 0 and Afun2 >= 0 :
                if M[0] > V[0] :
                    Afun3 = ANS * (OB_2[0]-M[0]) - (OB_2[1]-M[1])
                    Afun4 = ANS * (OB_2[0]-(M[0]-5 * (billiard_radius) * UVmv[0])) - (OB_2[1]-(M[1]-5 * (billiard_radius) * UVmv[1]))
                    if ANS<=0:
                        Afun3 = Afun3 * -1
                        Afun4 = Afun4 * -1
                    if Afun3 >= 0 and Afun4 <= 0 :
                        print('MoveBall')
                        MoveBall()
                if M[0] <= V[0] :
                    Afun3 = ANS * (OB_2[0]-M[0]) - (OB_2[1]-M[1])
                    Afun4 = ANS * (OB_2[0]-(M[0]-5 * (billiard_radius) * UVmv[0])) - (OB_2[1]-(M[1]-5 * (billiard_radius) * UVmv[1]))
                    if ANS<=0:
                        Afun3 = Afun3 * -1
                        Afun4 = Afun4 * -1
                    if Afun3 <= 0 and Afun4 >= 0 :
                        print('MoveBall')
                        MoveBall()

#-------0719有干擾球時轉45度在打-------------------------
def MoveBall():
    global VT, H_S, H_E
    global Height,BMZ
    global V, H_S, H_E, M, C ,OB_1 ,OB_2
    global AS, ANS, Afun1, Afun2, Afun3, Afun4

    Height = BMZ + 1.3

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
    
    H_S[0] = M[0]-4 * (billiard_radius) * np.cos(45)*UVhse[0] + 4 * (billiard_radius) * np.cos(45)*UVnhse[0]
    H_S[1] = M[1]-4 * (billiard_radius) * np.cos(45)*UVhse[1] + 4 * (billiard_radius) * np.cos(45)*UVnhse[1]
    H_E[0] = M[0]+4 * (billiard_radius) * np.cos(45)*UVhse[0] - 4 * (billiard_radius) * np.cos(45)*UVnhse[0]
    H_E[1] = M[1]+4 * (billiard_radius) * np.cos(45)*UVhse[1] - 4 * (billiard_radius) * np.cos(45)*UVnhse[1]


    VT = VT + 45    
    ##------------------------避障--OB_1----------------------------
    #--------------------超出球桌範圍-------------------------
    #if H_S[0] >= 31.5 or H_S[0] <= -27 or H_S[1] >= 62 or H_S[1] <= 35.5 :
    if H_S[0] >= 28 or H_S[0] <= -28 or H_S[1] >= 56 or H_S[1] <= 33 :
        Height = BMZ + 2.6
        print ('--------------上升------------避障-----------------')


    #---------------------擊球起始點有球-----------------------
    AS = (H_S[1]-H_E[1])/(H_S[0]-H_E[0])
    ANS = -1/AS
    if H_S[1] > H_E[1] :                                    #----------------母球兩側各加一個r的斜線方程式-----------------
        Afun1 = AS * (OB_1[0]-(M[0]+2 * (billiard_radius) * UVnhse[0])) - (OB_1[1]-(M[1]+2 * (billiard_radius) * UVnhse[1]))
        Afun2 = AS * (OB_1[0]-(M[0]-2 * (billiard_radius) * UVnhse[0])) - (OB_1[1]-(M[1]-2 * (billiard_radius) * UVnhse[1]))
        if AS<=0:
            Afun1 = Afun1 * -1
            Afun2 = Afun2 * -1
        if Afun1 >= 0 and Afun2 <= 0 :
            if H_S[0] > H_E[0] :                            #----------------過母球球心及其後方5倍r距離的斜線方程式-----------------
                Afun3 = ANS * (OB_1[0]-M[0]) - (OB_1[1]-M[1])
                Afun4 = ANS * (OB_1[0]-(M[0]-5 * (billiard_radius) * UVhse[0])) - (OB_1[1]-(M[1]-5 * (billiard_radius) * UVhse[1]))
                if ANS<=0:
                    Afun3 = Afun3 * -1
                    Afun4 = Afun4 * -1
                if Afun3 >= 0 and Afun4 <= 0 :
                    print('MoveBall')
                    MoveBall()
            if H_S[0] <= H_E[0] :
                Afun3 = ANS * (OB_1[0]-M[0]) - (OB_1[1]-M[1])
                Afun4 = ANS * (OB_1[0]-(M[0]-5 * (billiard_radius) * UVhse[0])) - (OB_1[1]-(M[1]-5 * (billiard_radius) * UVhse[1]))
                if ANS<=0:
                    Afun3 = Afun3 * -1
                    Afun4 = Afun4 * -1
                if Afun3 <= 0 and Afun4 >= 0 :
                    print('MoveBall')
                    MoveBall()
    if H_S[1] <= H_E[1] :
        Afun1 = AS * (OB_1[0]-(M[0]+2 * (billiard_radius) * UVnhse[0])) - (OB_1[1]-(M[1]+2 * (billiard_radius) * UVnhse[1]))
        Afun2 = AS * (OB_1[0]-(M[0]-2 * (billiard_radius) * UVnhse[0])) - (OB_1[1]-(M[1]-2 * (billiard_radius) * UVnhse[1]))
        if AS<=0:
            Afun1 = Afun1 * -1
            Afun2 = Afun2 * -1
        if Afun1 <= 0 and Afun2 >= 0 :
            if H_S[0] > H_E[0] :
                Afun3 = ANS * (OB_1[0]-M[0]) - (OB_1[1]-M[1])
                Afun4 = ANS * (OB_1[0]-(M[0]-5 * (billiard_radius) * UVhse[0])) - (OB_1[1]-(M[1]-5 * (billiard_radius) * UVhse[1]))
                if ANS<=0:
                    Afun3 = Afun3 * -1
                    Afun4 = Afun4 * -1
                if Afun3 >= 0 and Afun4 <= 0 :
                    print('MoveBall')
                    MoveBall()
            if H_S[0] <= H_E[0] :
                Afun3 = ANS * (OB_1[0]-M[0]) - (OB_1[1]-M[1])
                Afun4 = ANS * (OB_1[0]-(M[0]-5 * (billiard_radius) * UVhse[0])) - (OB_1[1]-(M[1]-5 * (billiard_radius) * UVhse[1]))
                if ANS<=0:
                    Afun3 = Afun3 * -1
                    Afun4 = Afun4 * -1
                if Afun3 <= 0 and Afun4 >= 0 :
                    print('MoveBall')
                    MoveBall()


    ##------------------------避障--OB_2----------------------------
    #---------------------擊球起始點有球-----------------------
    if M[1] > V[1] :                                    #----------------母球兩側各加一個r的斜線方程式-----------------
        Afun1 = AS * (OB_2[0]-(M[0]+2 * (billiard_radius) * UVnhse[0])) - (OB_2[1]-(M[1]+2 * (billiard_radius) * UVnhse[1]))
        Afun2 = AS * (OB_2[0]-(M[0]-2 * (billiard_radius) * UVnhse[0])) - (OB_2[1]-(M[1]-2 * (billiard_radius) * UVnhse[1]))
        if AS<=0:
            Afun1 = Afun1 * -1
            Afun2 = Afun2 * -1
        if Afun1 >= 0 and Afun2 <= 0 :
            if M[0] > V[0] :                            #----------------過母球球心及其後方5倍r距離的斜線方程式-----------------
                Afun3 = ANS * (OB_2[0]-M[0]) - (OB_2[1]-M[1])
                Afun4 = ANS * (OB_2[0]-(M[0]-5 * (billiard_radius) * UVhse[0])) - (OB_2[1]-(M[1]-5 * (billiard_radius) * UVhse[1]))
                if ANS<=0:
                    Afun3 = Afun3 * -1
                    Afun4 = Afun4 * -1
                if Afun3 >= 0 and Afun4 <= 0 :
                    print('MoveBall')
                    MoveBall()
            if M[0] <= V[0] :
                Afun3 = ANS * (OB_2[0]-M[0]) - (OB_2[1]-M[1])
                Afun4 = ANS * (OB_2[0]-(M[0]-5 * (billiard_radius) * UVhse[0])) - (OB_2[1]-(M[1]-5 * (billiard_radius) * UVhse[1]))
                if ANS<=0:
                    Afun3 = Afun3 * -1
                    Afun4 = Afun4 * -1
                if Afun3 <= 0 and Afun4 >= 0 :
                    print('MoveBall')
                    MoveBall()
    if M[1] <= V[1] :
        Afun1 = AS * (OB_2[0]-(M[0]+2 * (billiard_radius) * UVnhse[0])) - (OB_2[1]-(M[1]+2 * (billiard_radius) * UVnhse[1]))
        Afun2 = AS * (OB_2[0]-(M[0]-2 * (billiard_radius) * UVnhse[0])) - (OB_2[1]-(M[1]-2 * (billiard_radius) * UVnhse[1]))
        if AS<=0:
            Afun1 = Afun1 * -1
            Afun2 = Afun2 * -1
        if Afun1 <= 0 and Afun2 >= 0 :
            if M[0] > V[0] :
                Afun3 = ANS * (OB_2[0]-M[0]) - (OB_2[1]-M[1])
                Afun4 = ANS * (OB_2[0]-(M[0]-5 * (billiard_radius) * UVhse[0])) - (OB_2[1]-(M[1]-5 * (billiard_radius) * UVhse[1]))
                if ANS<=0:
                    Afun3 = Afun3 * -1
                    Afun4 = Afun4 * -1
                if Afun3 >= 0 and Afun4 <= 0 :
                    print('MoveBall')
                    MoveBall()
            if M[0] <= V[0] :
                Afun3 = ANS * (OB_2[0]-M[0]) - (OB_2[1]-M[1])
                Afun4 = ANS * (OB_2[0]-(M[0]-5 * (billiard_radius) * UVhse[0])) - (OB_2[1]-(M[1]-5 * (billiard_radius) * UVhse[1]))
                if ANS<=0:
                    Afun3 = Afun3 * -1
                    Afun4 = Afun4 * -1
                if Afun3 <= 0 and Afun4 >= 0 :
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
    #if H_S[0] >= 31.5 or H_S[0] <= -27 or H_S[1] >= 62 or H_S[1] <= 35.5 :
    if H_S[0] >= 28 or H_S[0] <= -28 or H_S[1] >= 56 or H_S[1] <= 33 :
        Height = BMZ + 2.6
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
        if AS<=0:
            Afun1 = Afun1 * -1
            Afun2 = Afun2 * -1
        if Afun1 >= 0 and Afun2 <= 0 :
            if M[0] > V[0] :                            #----------------過母球球心及其後方5倍r距離的斜線方程式-----------------
                Afun3 = ANS * (OB_2[0]-M[0]) - (OB_2[1]-M[1])
                Afun4 = ANS * (OB_2[0]-(M[0]-5 * (billiard_radius) * UVmv[0])) - (OB_2[1]-(M[1]-5 * (billiard_radius) * UVmv[1]))
                if ANS<=0:
                    Afun3 = Afun3 * -1
                    Afun4 = Afun4 * -1
                if Afun3 >= 0 and Afun4 <= 0 :
                    print('MoveBall')
                    MoveBall()
            if M[0] <= V[0] :
                Afun3 = ANS * (OB_2[0]-M[0]) - (OB_2[1]-M[1])
                Afun4 = ANS * (OB_2[0]-(M[0]-5 * (billiard_radius) * UVmv[0])) - (OB_2[1]-(M[1]-5 * (billiard_radius) * UVmv[1]))
                if ANS<=0:
                    Afun3 = Afun3 * -1
                    Afun4 = Afun4 * -1
                if Afun3 <= 0 and Afun4 >= 0 :
                    print('MoveBall')
                    MoveBall()
    if M[1] <= V[1] :
        Afun1 = AS * (OB_2[0]-(M[0]+2 * (billiard_radius) * UVnmv[0])) - (OB_2[1]-(M[1]+2 * (billiard_radius) * UVnmv[1]))
        Afun2 = AS * (OB_2[0]-(M[0]-2 * (billiard_radius) * UVnmv[0])) - (OB_2[1]-(M[1]-2 * (billiard_radius) * UVnmv[1]))
        if AS<=0:
            Afun1 = Afun1 * -1
            Afun2 = Afun2 * -1
        if Afun1 <= 0 and Afun2 >= 0 :
            if M[0] > V[0] :
                Afun3 = ANS * (OB_2[0]-M[0]) - (OB_2[1]-M[1])
                Afun4 = ANS * (OB_2[0]-(M[0]-5 * (billiard_radius) * UVmv[0])) - (OB_2[1]-(M[1]-5 * (billiard_radius) * UVmv[1]))
                if ANS<=0:
                    Afun3 = Afun3 * -1
                    Afun4 = Afun4 * -1
                if Afun3 >= 0 and Afun4 <= 0 :
                    print('MoveBall')
                    MoveBall()
            if M[0] <= V[0] :
                Afun3 = ANS * (OB_2[0]-M[0]) - (OB_2[1]-M[1])
                Afun4 = ANS * (OB_2[0]-(M[0]-5 * (billiard_radius) * UVmv[0])) - (OB_2[1]-(M[1]-5 * (billiard_radius) * UVmv[1]))
                if ANS<=0:
                    Afun3 = Afun3 * -1
                    Afun4 = Afun4 * -1
                if Afun3 <= 0 and Afun4 >= 0 :
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
    #if H_S[0] >= 31.5 or H_S[0] <= -27 or H_S[1] >= 62 or H_S[1] <= 35.5 :
    if H_S[0] >= 28 or H_S[0] <= -28 or H_S[1] >= 56 or H_S[1] <= 33 :
        Height = BMZ + 2.5
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
        if AS<=0:
            Afun1 = Afun1 * -1
            Afun2 = Afun2 * -1
        if Afun1 >= 0 and Afun2 <= 0 :
            if M[0] > V[0] :                            #----------------過母球球心及其後方5倍r距離的斜線方程式-----------------
                Afun3 = ANS * (OB_1[0]-M[0]) - (OB_1[1]-M[1])
                Afun4 = ANS * (OB_1[0]-(M[0]-5 * (billiard_radius) * UVmv[0])) - (OB_1[1]-(M[1]-5 * (billiard_radius) * UVmv[1]))
                if ANS<=0:
                    Afun3 = Afun3 * -1
                    Afun4 = Afun4 * -1
                if Afun3 >= 0 and Afun4 <= 0 :
                    print('MoveBall')
                    MoveBall()
            if M[0] <= V[0] :
                Afun3 = ANS * (OB_1[0]-M[0]) - (OB_1[1]-M[1])
                Afun4 = ANS * (OB_1[0]-(M[0]-5 * (billiard_radius) * UVmv[0])) - (OB_1[1]-(M[1]-5 * (billiard_radius) * UVmv[1]))
                if ANS<=0:
                    Afun3 = Afun3 * -1
                    Afun4 = Afun4 * -1
                if Afun3 <= 0 and Afun4 >= 0 :
                    print('MoveBall')
                    MoveBall()
    if M[1] <= V[1] :
        Afun1 = AS * (OB_1[0]-(M[0]+2 * (billiard_radius) * UVnmv[0])) - (OB_1[1]-(M[1]+2 * (billiard_radius) * UVnmv[1]))
        Afun2 = AS * (OB_1[0]-(M[0]-2 * (billiard_radius) * UVnmv[0])) - (OB_1[1]-(M[1]-2 * (billiard_radius) * UVnmv[1]))
        if AS<=0:
            Afun1 = Afun1 * -1
            Afun2 = Afun2 * -1
        if Afun1 <= 0 and Afun2 >= 0 :
            if M[0] > V[0] :
                Afun3 = ANS * (OB_1[0]-M[0]) - (OB_1[1]-M[1])
                Afun4 = ANS * (OB_1[0]-(M[0]-5 * (billiard_radius) * UVmv[0])) - (OB_1[1]-(M[1]-5 * (billiard_radius) * UVmv[1]))
                if ANS<=0:
                    Afun3 = Afun3 * -1
                    Afun4 = Afun4 * -1
                if Afun3 >= 0 and Afun4 <= 0 :
                    print('MoveBall')
                    MoveBall()
            if M[0] <= V[0] :
                Afun3 = ANS * (OB_1[0]-M[0]) - (OB_1[1]-M[1])
                Afun4 = ANS * (OB_1[0]-(M[0]-5 * (billiard_radius) * UVmv[0])) - (OB_1[1]-(M[1]-5 * (billiard_radius) * UVmv[1]))
                if ANS<=0:
                    Afun3 = Afun3 * -1
                    Afun4 = Afun4 * -1
                if Afun3 <= 0 and Afun4 >= 0 :
                    print('MoveBall')
                    MoveBall()
'''

def yolo_pic():
    init_resp = init_yolo_picture(get_image_1.get_image())
    # init_resp = init_yolo_picture("/home/iclab/catkin_ws/src/yolo_v3/scripts/15.jpg")
    time.sleep(3)
    resp = call_yolo_v3_picture(1)
    # print(resp.Image.ROI_list)  #印出 回傳的座標字串
    global i_dont_know
    i_dont_know = i_dont_know + 1

    for i in range(len(resp.Image.ROI_list)): # 分析座標字串
        global temp_list
        temp_list = ['', 0.0, 0.0, 0.0]
        global temp_str
        # temp_str = str('') #initial temp_str as empty string
        temp_str = str(resp.Image.ROI_list[i])
        get_str_mid ()
        # curve()
    
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
    # time.sleep(15)

def camera_state():
    pos.x = -0.1
    pos.y = 36.8
    pos.z = 10
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 0 
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(2,1,0,50,2)#action,ra,grip,vel,both
    # print('x: ',pos.x,' y: ',pos.y,' z: ',pos.z,' pitch: ',pos.pitch,' roll: ',pos.roll,' yaw: ',pos.yaw)

def take_pic():
    time.sleep(1)
    # global NineOnTable 
    # NineOnTable = 0
    yolo_pic()
    # if(NineOnTable == 0):
    #     break
    #GetStartEnd()
    pos.x = -0.1
    pos.y = 36.8
    pos.z = 10
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 0
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(2,1,0,50,2)#action,ra,grip,vel,both
    # print('x: ',pos.x,' y: ',pos.y,' z: ',pos.z,' pitch: ',pos.pitch,' roll: ',pos.roll,' yaw: ',pos.yaw)

# 20190813 BMZ = -23.25 為了方便四個腳 所以用 -21.40 --->  pos.z = BMZ  + 1 . 8 5
def place_0():
    pos.x = BMX + 12.18
    pos.y = BMY + 2.86
    pos.z = BMZ + 1.85
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(3,1,0,5,2)#action,ra,grip,vel,both

def place_0_above():
    pos.x = BMX + 12.18
    pos.y = BMY + 2.86
    pos.z = BMZ + 15
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(2,1,0,50,2)#action,ra,grip,vel,both

def place_0_above_back():
    pos.x = BMX + 12.18
    pos.y = BMY + 2.86
    pos.z = BMZ + 15
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(3,1,0,15,2)#action,ra,grip,vel,both

def place_1():
    pos.x = BMX + 12.18 + 6
    pos.y = BMY + 2.86
    pos.z = BMZ + 1.85
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(3,1,0,5,2)#action,ra,grip,vel,both

def place_1_above():
    pos.x = BMX + 12.18 + 6
    pos.y = BMY + 2.86
    pos.z = BMZ + 15
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(2,1,0,50,2)#action,ra,grip,vel,both

def place_1_above_back():
    pos.x = BMX + 12.18 + 6
    pos.y = BMY + 2.86
    pos.z = BMZ + 15
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(3,1,0,15,2)#action,ra,grip,vel,both

def place_2():
    pos.x = BMX + 12.18 + 12
    pos.y = BMY + 2.86
    pos.z = BMZ + 1.85
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(3,1,0,5,2)#action,ra,grip,vel,both

def place_2_above():
    pos.x = BMX + 12.18 + 12
    pos.y = BMY + 2.86
    pos.z = BMZ + 15
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(2,1,0,50,2)#action,ra,grip,vel,both

def place_2_above_back():
    pos.x = BMX + 12.18 + 12
    pos.y = BMY + 2.86
    pos.z = BMZ + 15
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(3,1,0,15,2)#action,ra,grip,vel,both

def place_3():
    pos.x = BMX + 12.18 + 18
    pos.y = BMY + 2.86
    pos.z = BMZ + 1.85
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(3,1,0,5,2)#action,ra,grip,vel,both

def place_3_above():
    pos.x = BMX + 12.18 + 18
    pos.y = BMY + 2.86
    pos.z = BMZ + 15
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(2,1,0,50,2)#action,ra,grip,vel,both

def place_3_above_back():
    pos.x = BMX + 12.18 + 18
    pos.y = BMY + 2.86
    pos.z = BMZ + 15
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(3,1,0,15,2)#action,ra,grip,vel,both

def place_4():
    pos.x = BMX + 12.18 + 24
    pos.y = BMY + 2.86
    pos.z = BMZ + 1.85
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(3,1,0,5,2)#action,ra,grip,vel,both

def place_4_above():
    pos.x = BMX + 12.18 + 24
    pos.y = BMY + 2.86
    pos.z = BMZ + 15
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(2,1,0,50,2)#action,ra,grip,vel,both

def place_4_above_back():
    pos.x = BMX + 12.18 + 24
    pos.y = BMY + 2.86
    pos.z = BMZ + 15
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(3,1,0,15,2)#action,ra,grip,vel,both

def place_5():
    pos.x = BMX + 12.18 + 30
    pos.y = BMY + 2.86
    pos.z = BMZ + 1.85
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(3,1,0,5,2)#action,ra,grip,vel,both

def place_5_above():
    pos.x = BMX + 12.18 + 30
    pos.y = BMY + 2.86
    pos.z = BMZ + 15
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(2,1,0,50,2)#action,ra,grip,vel,both

def place_5_above_back():
    pos.x = BMX + 12.18 + 30
    pos.y = BMY + 2.86
    pos.z = BMZ + 15
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(3,1,0,15,2)#action,ra,grip,vel,both

def place_6():
    pos.x = BMX + 12.18 + 36
    pos.y = BMY + 2.86
    pos.z = BMZ + 1.85
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(3,1,0,5,2)#action,ra,grip,vel,both

def place_6_above():
    pos.x = BMX + 12.18 + 36
    pos.y = BMY + 2.86
    pos.z = BMZ + 15
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(2,1,0,50,2)#action,ra,grip,vel,both

def place_6_above_back():
    pos.x = BMX + 12.18 + 36
    pos.y = BMY + 2.86
    pos.z = BMZ + 15
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(3,1,0,15,2)#action,ra,grip,vel,both

def place_7():
    pos.x = BMX + 12.18 + 42
    pos.y = BMY + 2.86
    pos.z = BMZ + 1.85
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(3,1,0,5,2)#action,ra,grip,vel,both

def place_7_above():
    pos.x = BMX + 12.18 + 42
    pos.y = BMY + 2.86
    pos.z = BMZ + 15
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(2,1,0,50,2)#action,ra,grip,vel,both

def place_7_above_back():
    pos.x = BMX + 12.18 + 42
    pos.y = BMY + 2.86
    pos.z = BMZ + 15
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(3,1,0,15,2)#action,ra,grip,vel,both

def place_8():
    pos.x = BMX + 12.18 + 42
    pos.y = BMY + 2.86 + 6
    pos.z = BMZ + 1.85
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(3,1,0,5,2)#action,ra,grip,vel,both

def place_8_above():
    pos.x = BMX + 12.18 + 42
    pos.y = BMY + 2.86 + 6
    pos.z = BMZ + 15
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(2,1,0,50,2)#action,ra,grip,vel,both

def place_8_above_back():
    pos.x = BMX + 12.18 + 42
    pos.y = BMY + 2.86 + 6
    pos.z = BMZ + 15
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(3,1,0,15,2)#action,ra,grip,vel,both
    
def place_9():
    pos.x = BMX + 12.18 + 36
    pos.y = BMY + 2.86 + 6
    pos.z = BMZ + 1.85
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(3,1,0,5,2)#action,ra,grip,vel,both

def place_9_above():
    pos.x = BMX + 12.18 + 36
    pos.y = BMY + 2.86 + 6
    pos.z = BMZ + 15
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(2,1,0,50,2)#action,ra,grip,vel,both

def place_9_above_back():
    pos.x = BMX + 12.18 + 36
    pos.y = BMY + 2.86 + 6
    pos.z = BMZ + 15
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(3,1,0,15,2)#action,ra,grip,vel,both

def place_10():
    pos.x = BMX + 12.18 + 30
    pos.y = BMY + 2.86 + 6
    pos.z = BMZ + 1.85
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(3,1,0,5,2)#action,ra,grip,vel,both

def place_10_above():
    pos.x = BMX + 12.18 + 30
    pos.y = BMY + 2.86 + 6
    pos.z = BMZ + 15
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(2,1,0,50,2)#action,ra,grip,vel,both

def place_10_above_back():
    pos.x = BMX + 12.18 + 30
    pos.y = BMY + 2.86 + 6
    pos.z = BMZ + 15
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(3,1,0,15,2)#action,ra,grip,vel,both

def place_11():
    pos.x = BMX + 12.18 + 24
    pos.y = BMY + 2.86 + 6
    pos.z = BMZ + 1.85
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(3,1,0,5,2)#action,ra,grip,vel,both

def place_11_above():
    pos.x = BMX + 12.18 + 24
    pos.y = BMY + 2.86 + 6
    pos.z = BMZ + 15
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(2,1,0,50,2)#action,ra,grip,vel,both

def place_11_above_back():
    pos.x = BMX + 12.18 + 24
    pos.y = BMY + 2.86 + 6
    pos.z = BMZ + 15
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(3,1,0,15,2)#action,ra,grip,vel,both

def place_12():
    pos.x = BMX + 12.18 + 18
    pos.y = BMY + 2.86 + 6
    pos.z = BMZ + 1.85
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(3,1,0,5,2)#action,ra,grip,vel,both

def place_12_above():
    pos.x = BMX + 12.18 + 18
    pos.y = BMY + 2.86 + 6
    pos.z = BMZ + 15
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(2,1,0,50,2)#action,ra,grip,vel,both

def place_12_above_back():
    pos.x = BMX + 12.18 + 18
    pos.y = BMY + 2.86 + 6
    pos.z = BMZ + 15
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(3,1,0,15,2)#action,ra,grip,vel,both

def place_13():
    pos.x = BMX + 12.18 + 12
    pos.y = BMY + 2.86 + 6
    pos.z = BMZ + 1.85
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(3,1,0,5,2)#action,ra,grip,vel,both

def place_13_above():
    pos.x = BMX + 12.18 + 12
    pos.y = BMY + 2.86 + 6
    pos.z = BMZ + 15
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(2,1,0,50,2)#action,ra,grip,vel,both

def place_13_above_back():
    pos.x = BMX + 12.18 + 12
    pos.y = BMY + 2.86 + 6
    pos.z = BMZ + 15
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(3,1,0,15,2)#action,ra,grip,vel,both

def place_14():
    pos.x = BMX + 12.18 + 6
    pos.y = BMY + 2.86 + 6
    pos.z = BMZ + 1.85
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(3,1,0,5,2)#action,ra,grip,vel,both

def place_14_above():
    pos.x = BMX + 12.18 + 6
    pos.y = BMY + 2.86 + 6
    pos.z = BMZ + 15
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(2,1,0,50,2)#action,ra,grip,vel,both

def place_14_above_back():
    pos.x = BMX + 12.18 + 6
    pos.y = BMY + 2.86 + 6
    pos.z = BMZ + 15
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(3,1,0,15,2)#action,ra,grip,vel,both

def place_15():
    pos.x = BMX + 12.18 
    pos.y = BMY + 2.86 + 6
    pos.z = BMZ + 1.85
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(3,1,0,5,2)#action,ra,grip,vel,both

def place_15_above():
    pos.x = BMX + 12.18 
    pos.y = BMY + 2.86 + 6
    pos.z = BMZ + 15
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(2,1,0,50,2)#action,ra,grip,vel,both

def place_15_above_back():
    pos.x = BMX + 12.18 
    pos.y = BMY + 2.86 + 6
    pos.z = BMZ + 15
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(3,1,0,15,2)#action,ra,grip,vel,both

def place_16():
    pos.x = BMX + 12.18 
    pos.y = BMY + 2.86 + 12
    pos.z = BMZ + 1.85
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(3,1,0,5,2)#action,ra,grip,vel,both

def place_16_above():
    pos.x = BMX + 12.18 
    pos.y = BMY + 2.86 + 12
    pos.z = BMZ + 15
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(2,1,0,50,2)#action,ra,grip,vel,both

def place_16_above_back():
    pos.x = BMX + 12.18 
    pos.y = BMY + 2.86 + 12
    pos.z = BMZ + 15
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(3,1,0,15,2)#action,ra,grip,vel,both

def place_17():
    pos.x = BMX + 12.18 + 6
    pos.y = BMY + 2.86 + 12
    pos.z = BMZ + 1.85
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(3,1,0,5,2)#action,ra,grip,vel,both

def place_17_above():
    pos.x = BMX + 12.18 + 6
    pos.y = BMY + 2.86 + 12
    pos.z = BMZ + 15
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(2,1,0,50,2)#action,ra,grip,vel,both

def place_17_above_back():
    pos.x = BMX + 12.18 + 6
    pos.y = BMY + 2.86 + 12
    pos.z = BMZ + 15
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(3,1,0,15,2)#action,ra,grip,vel,both

def place_18():
    pos.x = BMX + 12.18 + 12
    pos.y = BMY + 2.86 + 12
    pos.z = BMZ + 1.85
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(3,1,0,5,2)#action,ra,grip,vel,both

def place_18_above():
    pos.x = BMX + 12.18 + 12
    pos.y = BMY + 2.86 + 12
    pos.z = BMZ + 15
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(2,1,0,50,2)#action,ra,grip,vel,both

def place_18_above_back():
    pos.x = BMX + 12.18 + 12
    pos.y = BMY + 2.86 + 12
    pos.z = BMZ + 15
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(3,1,0,15,2)#action,ra,grip,vel,both

def place_19():
    pos.x = BMX + 12.18 + 18
    pos.y = BMY + 2.86 + 12
    pos.z = BMZ + 1.85
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(3,1,0,5,2)#action,ra,grip,vel,both

def place_19_above():
    pos.x = BMX + 12.18 + 18
    pos.y = BMY + 2.86 + 12
    pos.z = BMZ + 15
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(2,1,0,50,2)#action,ra,grip,vel,both

def place_19_above_back():
    pos.x = BMX + 12.18 + 18
    pos.y = BMY + 2.86 + 12
    pos.z = BMZ + 15
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(3,1,0,15,2)#action,ra,grip,vel,both

def place_20():
    pos.x = BMX + 12.18 + 24
    pos.y = BMY + 2.86 + 12
    pos.z = BMZ + 1.85
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(3,1,0,5,2)#action,ra,grip,vel,both

def place_20_above():
    pos.x = BMX + 12.18 + 24
    pos.y = BMY + 2.86 + 12
    pos.z = BMZ + 15
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(2,1,0,50,2)#action,ra,grip,vel,both

def place_20_above_back():
    pos.x = BMX + 12.18 + 24
    pos.y = BMY + 2.86 + 12
    pos.z = BMZ + 15
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(3,1,0,15,2)#action,ra,grip,vel,both

def place_21():
    pos.x = BMX + 12.18 + 30
    pos.y = BMY + 2.86 + 12
    pos.z = BMZ + 1.85
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(3,1,0,5,2)#action,ra,grip,vel,both

def place_21_above():
    pos.x = BMX + 12.18 + 30
    pos.y = BMY + 2.86 + 12
    pos.z = BMZ + 15
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(2,1,0,50,2)#action,ra,grip,vel,both

def place_21_above_back():
    pos.x = BMX + 12.18 + 30
    pos.y = BMY + 2.86 + 12
    pos.z = BMZ + 15
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(3,1,0,15,2)#action,ra,grip,vel,both

def place_22():
    pos.x = BMX + 12.18 + 36
    pos.y = BMY + 2.86 + 12
    pos.z = BMZ + 1.85
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(3,1,0,5,2)#action,ra,grip,vel,both

def place_22_above():
    pos.x = BMX + 12.18 + 36
    pos.y = BMY + 2.86 + 12
    pos.z = BMZ + 15
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(2,1,0,50,2)#action,ra,grip,vel,both

def place_22_above_back():
    pos.x = BMX + 12.18 + 36
    pos.y = BMY + 2.86 + 12
    pos.z = BMZ + 15
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(3,1,0,15,2)#action,ra,grip,vel,both

def place_23():
    pos.x = BMX + 12.18 + 42
    pos.y = BMY + 2.86 + 12
    pos.z = BMZ + 1.85
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(3,1,0,5,2)#action,ra,grip,vel,both

def place_23_above():
    pos.x = BMX + 12.18 + 42
    pos.y = BMY + 2.86 + 12
    pos.z = BMZ + 15
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(2,1,0,50,2)#action,ra,grip,vel,both

def place_23_above_back():
    pos.x = BMX + 12.18 + 42
    pos.y = BMY + 2.86 + 12
    pos.z = BMZ + 15
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(3,1,0,15,2)#action,ra,grip,vel,both

def place_24():
    pos.x = BMX + 12.18 + 42
    pos.y = BMY + 2.86 + 18
    pos.z = BMZ + 1.85
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(3,1,0,5,2)#action,ra,grip,vel,both

def place_24_above():
    pos.x = BMX + 12.18 + 42
    pos.y = BMY + 2.86 + 18
    pos.z = BMZ + 15
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(2,1,0,50,2)#action,ra,grip,vel,both

def place_24_above_back():
    pos.x = BMX + 12.18 + 42
    pos.y = BMY + 2.86 + 18
    pos.z = BMZ + 15
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(3,1,0,15,2)#action,ra,grip,vel,both

def place_25():
    pos.x = BMX + 12.18 + 36
    pos.y = BMY + 2.86 + 18
    pos.z = BMZ + 1.85
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(3,1,0,5,2)#action,ra,grip,vel,both

def place_25_above():
    pos.x = BMX + 12.18 + 36
    pos.y = BMY + 2.86 + 18
    pos.z = BMZ + 15
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(2,1,0,50,2)#action,ra,grip,vel,both

def place_25_above_back():
    pos.x = BMX + 12.18 + 36
    pos.y = BMY + 2.86 + 18
    pos.z = BMZ + 15
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(3,1,0,15,2)#action,ra,grip,vel,both

def place_26():
    pos.x = BMX + 12.18 + 30
    pos.y = BMY + 2.86 + 18
    pos.z = BMZ + 1.85
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(3,1,0,5,2)#action,ra,grip,vel,both

def place_26_above():
    pos.x = BMX + 12.18 + 30
    pos.y = BMY + 2.86 + 18
    pos.z = BMZ + 15
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(2,1,0,50,2)#action,ra,grip,vel,both

def place_26_above_back():
    pos.x = BMX + 12.18 + 30
    pos.y = BMY + 2.86 + 18
    pos.z = BMZ + 15
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(3,1,0,15,2)#action,ra,grip,vel,both

def place_27():
    pos.x = BMX + 12.18 + 24
    pos.y = BMY + 2.86 + 18
    pos.z = BMZ + 1.85
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(3,1,0,5,2)#action,ra,grip,vel,both

def place_27_above():
    pos.x = BMX + 12.18 + 24
    pos.y = BMY + 2.86 + 18
    pos.z = BMZ + 15
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(2,1,0,50,2)#action,ra,grip,vel,both

def place_27_above_back():
    pos.x = BMX + 12.18 + 24
    pos.y = BMY + 2.86 + 18
    pos.z = BMZ + 15
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(3,1,0,15,2)#action,ra,grip,vel,both

def place_28():
    pos.x = BMX + 12.18 + 18
    pos.y = BMY + 2.86 + 18
    pos.z = BMZ + 1.85
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(3,1,0,5,2)#action,ra,grip,vel,both

def place_28_above():
    pos.x = BMX + 12.18 + 18
    pos.y = BMY + 2.86 + 18
    pos.z = BMZ + 15
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(2,1,0,50,2)#action,ra,grip,vel,both

def place_28_above_back():
    pos.x = BMX + 12.18 + 18
    pos.y = BMY + 2.86 + 18
    pos.z = BMZ + 15
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(3,1,0,15,2)#action,ra,grip,vel,both

def place_29():
    pos.x = BMX + 12.18 + 12
    pos.y = BMY + 2.86 + 18
    pos.z = BMZ + 1.85
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(3,1,0,5,2)#action,ra,grip,vel,both

def place_29_above():
    pos.x = BMX + 12.18 + 12
    pos.y = BMY + 2.86 + 18
    pos.z = BMZ + 15
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(2,1,0,50,2)#action,ra,grip,vel,both

def place_29_above_back():
    pos.x = BMX + 12.18 + 12
    pos.y = BMY + 2.86 + 18
    pos.z = BMZ + 15
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(3,1,0,15,2)#action,ra,grip,vel,both

def place_30():
    pos.x = BMX + 12.18 + 6
    pos.y = BMY + 2.86 + 18
    pos.z = BMZ + 1.85
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(3,1,0,5,2)#action,ra,grip,vel,both

def place_30_above():
    pos.x = BMX + 12.18 + 6
    pos.y = BMY + 2.86 + 18
    pos.z = BMZ + 15
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(2,1,0,50,2)#action,ra,grip,vel,both

def place_30_above_back():
    pos.x = BMX + 12.18 + 6
    pos.y = BMY + 2.86 + 18
    pos.z = BMZ + 15
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(3,1,0,15,2)#action,ra,grip,vel,both

def place_31():
    pos.x = BMX + 12.18 
    pos.y = BMY + 2.86 + 18
    pos.z = BMZ + 1.85
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(3,1,0,5,2)#action,ra,grip,vel,both

def place_31_above():
    pos.x = BMX + 12.18 
    pos.y = BMY + 2.86 + 18
    pos.z = BMZ + 15
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(2,1,0,50,2)#action,ra,grip,vel,both

def place_31_above_back():
    pos.x = BMX + 12.18 
    pos.y = BMY + 2.86 + 18
    pos.z = BMZ + 15
    pos.pitch = -90 
    pos.roll = 0  
    pos.yaw = 5
    ArmTask.point_data(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    ArmTask.Arm_Mode(3,1,0,15,2)#action,ra,grip,vel,both
    

def Mission_Trigger():
    global num_step
    global action,M,C
    global gHole
    global H1,H2,H3,H4,H5,H6
    global min_dif, num_hole
    # global Hole_info
    global V, H_S, H_E
    global Height,BMZ
    global Array_2D
    Array_2D = [['0', 0.0, 0.0, 0.0] for i in range(10)] # two-dimension 4*10 to be all zero
    
    if Arm_state_flag == Arm_status.Idle and Sent_data_flag == 1:
        #Sent_data_flag = 0
        #Arm_state_flag = Arm_status.Isbusy
        print('123456789123456789')
        

        for case in switch(action): #傳送指令給socket選擇手臂動作
                if case(0):           
                    for step in switch(num_step):
                        if step(0):
                            yolo_pic() # for not to read zero and write in txt
                            yolo_pic() # for not to read zero and write in txt
                            yolo_pic() # for not to read zero and write in txt
                            print(action, num_step)
                            place_0_above()
                            num_step = 1
                            break
                        if step(1):
                            print(action, num_step)
                            place_0()
                            num_step = 2
                            break
                        if step(2):
                            print(action, num_step)
                            place_0()
                            num_step = 3
                            break
                        if step(3):
                            print(action, num_step)
                            place_0_above()
                            num_step = 4
                            break
                        if step(4):
                            print(action, num_step)
                            camera_state()
                            num_step = 5
                            break
                        if step(5):
                            print(action, num_step)
                            take_pic()
                            num_step = 0
                            action = 1
                            break
                    break
                if case(1):             
                    for step in switch(num_step):
                        if step(0):
                            print(action, num_step)
                            place_0_above()
                            num_step = 1
                            break
                        if step(1):
                            print(action, num_step)
                            place_0()
                            num_step = 2
                            break
                        if step(2):
                            print(action, num_step)
                            place_1()
                            num_step = 3
                            break
                        if step(3):
                            print(action, num_step)
                            place_1_above()
                            num_step = 4
                            break
                        if step(4):
                            print(action, num_step)
                            camera_state()
                            num_step = 5
                            break
                        if step(5):
                            print(action, num_step)
                            take_pic()
                            num_step = 0
                            action = 2
                            break
                    break
                if case(2):             
                    for step in switch(num_step):
                        if step(0):
                            print(action, num_step)
                            place_1_above()
                            num_step = 1
                            break
                        if step(1):
                            print(action, num_step)
                            place_1()
                            num_step = 2
                            break
                        if step(2):
                            print(action, num_step)
                            place_2()
                            num_step = 3
                            break
                        if step(3):
                            print(action, num_step)
                            place_2_above()
                            num_step = 4
                            break
                        if step(4):
                            print(action, num_step)
                            camera_state()
                            num_step = 5
                            break
                        if step(5):
                            print(action, num_step)
                            take_pic()
                            num_step = 0
                            action = 3
                            break
                    break
                if case(3):             
                    for step in switch(num_step):
                        if step(0):
                            print(action, num_step)
                            place_2_above()
                            num_step = 1
                            break
                        if step(1):
                            print(action, num_step)
                            place_2()
                            num_step = 2
                            break
                        if step(2):
                            print(action, num_step)
                            place_3()
                            num_step = 3
                            break
                        if step(3):
                            print(action, num_step)
                            place_3_above()
                            num_step = 4
                            break
                        if step(4):
                            print(action, num_step)
                            camera_state()
                            num_step = 5
                            break
                        if step(5):
                            print(action, num_step)
                            take_pic()
                            num_step = 0
                            action = 4
                            break
                    break
                if case(4):             
                    for step in switch(num_step):
                        if step(0):
                            print(action, num_step)
                            place_3_above()
                            num_step = 1
                            break
                        if step(1):
                            print(action, num_step)
                            place_3()
                            num_step = 2
                            break
                        if step(2):
                            print(action, num_step)
                            place_4()
                            num_step = 3
                            break
                        if step(3):
                            print(action, num_step)
                            place_4_above()
                            num_step = 4
                            break
                        if step(4):
                            print(action, num_step)
                            camera_state()
                            num_step = 5
                            break
                        if step(5):
                            print(action, num_step)
                            take_pic()
                            num_step = 0
                            action = 5
                            break
                    break
                if case(5):             
                    for step in switch(num_step):
                        if step(0):
                            print(action, num_step)
                            place_4_above()
                            num_step = 1
                            break
                        if step(1):
                            print(action, num_step)
                            place_4()
                            num_step = 2
                            break
                        if step(2):
                            print(action, num_step)
                            place_5()
                            num_step = 3
                            break
                        if step(3):
                            print(action, num_step)
                            place_5_above()
                            num_step = 4
                            break
                        if step(4):
                            print(action, num_step)
                            camera_state()
                            num_step = 5
                            break
                        if step(5):
                            print(action, num_step)
                            take_pic()
                            num_step = 0
                            action = 6
                            break
                    break
                if case(6):             
                    for step in switch(num_step):
                        if step(0):
                            print(action, num_step)
                            place_5_above()
                            num_step = 1
                            break
                        if step(1):
                            print(action, num_step)
                            place_5()
                            num_step = 2
                            break
                        if step(2):
                            print(action, num_step)
                            place_6()
                            num_step = 3
                            break
                        if step(3):
                            print(action, num_step)
                            place_6_above()
                            num_step = 4
                            break
                        if step(4):
                            print(action, num_step)
                            camera_state()
                            num_step = 5
                            break
                        if step(5):
                            print(action, num_step)
                            take_pic()
                            num_step = 0
                            action = 7
                            break
                    break
                if case(7):             
                    for step in switch(num_step):
                        if step(0):
                            print(action, num_step)
                            place_6_above()
                            num_step = 1
                            break
                        if step(1):
                            print(action, num_step)
                            place_6()
                            num_step = 2
                            break
                        if step(2):
                            print(action, num_step)
                            place_7()
                            num_step = 3
                            break
                        if step(3):
                            print(action, num_step)
                            place_7_above()
                            num_step = 4
                            break
                        if step(4):
                            print(action, num_step)
                            camera_state()
                            num_step = 5
                            break
                        if step(5):
                            print(action, num_step)
                            take_pic()
                            num_step = 0
                            action = 8
                            break
                    break
                if case(8):             
                    for step in switch(num_step):
                        if step(0):
                            print(action, num_step)
                            place_7_above()
                            num_step = 1
                            break
                        if step(1):
                            print(action, num_step)
                            place_7()
                            num_step = 2
                            break
                        if step(2):
                            print(action, num_step)
                            place_8()
                            num_step = 3
                            break
                        if step(3):
                            print(action, num_step)
                            place_8_above()
                            num_step = 4
                            break
                        if step(4):
                            print(action, num_step)
                            camera_state()
                            num_step = 5
                            break
                        if step(5):
                            print(action, num_step)
                            take_pic()
                            num_step = 0
                            action = 9
                            break
                    break
                if case(9):             
                    for step in switch(num_step):
                        if step(0):
                            print(action, num_step)
                            place_8_above()
                            num_step = 1
                            break
                        if step(1):
                            print(action, num_step)
                            place_8()
                            num_step = 2
                            break
                        if step(2):
                            print(action, num_step)
                            place_9()
                            num_step = 3
                            break
                        if step(3):
                            print(action, num_step)
                            place_9_above()
                            num_step = 4
                            break
                        if step(4):
                            print(action, num_step)
                            camera_state()
                            num_step = 5
                            break
                        if step(5):
                            print(action, num_step)
                            take_pic()
                            num_step = 0
                            action = 10
                            break
                    break
                if case(10):             
                    for step in switch(num_step):
                        if step(0):
                            print(action, num_step)
                            place_9_above()
                            num_step = 1
                            break
                        if step(1):
                            print(action, num_step)
                            place_9()
                            num_step = 2
                            break
                        if step(2):
                            print(action, num_step)
                            place_10()
                            num_step = 3
                            break
                        if step(3):
                            print(action, num_step)
                            place_10_above()
                            num_step = 4
                            break
                        if step(4):
                            print(action, num_step)
                            camera_state()
                            num_step = 5
                            break
                        if step(5):
                            print(action, num_step)
                            take_pic()
                            num_step = 0
                            action = 11
                            break
                    break
                if case(11):             
                    for step in switch(num_step):
                        if step(0):
                            print(action, num_step)
                            place_10_above()
                            num_step = 1
                            break
                        if step(1):
                            print(action, num_step)
                            place_10()
                            num_step = 2
                            break
                        if step(2):
                            print(action, num_step)
                            place_11()
                            num_step = 3
                            break
                        if step(3):
                            print(action, num_step)
                            place_11_above()
                            num_step = 4
                            break
                        if step(4):
                            print(action, num_step)
                            camera_state()
                            num_step = 5
                            break
                        if step(5):
                            print(action, num_step)
                            take_pic()
                            num_step = 0
                            action = 12
                            break
                    break
                if case(12):             
                    for step in switch(num_step):
                        if step(0):
                            print(action, num_step)
                            place_11_above()
                            num_step = 1
                            break
                        if step(1):
                            print(action, num_step)
                            place_11()
                            num_step = 2
                            break
                        if step(2):
                            print(action, num_step)
                            place_12()
                            num_step = 3
                            break
                        if step(3):
                            print(action, num_step)
                            place_12_above()
                            num_step = 4
                            break
                        if step(4):
                            print(action, num_step)
                            camera_state()
                            num_step = 5
                            break
                        if step(5):
                            print(action, num_step)
                            take_pic()
                            num_step = 0
                            action = 13
                            break
                    break
                if case(13):             
                    for step in switch(num_step):
                        if step(0):
                            print(action, num_step)
                            place_12_above()
                            num_step = 1
                            break
                        if step(1):
                            print(action, num_step)
                            place_12()
                            num_step = 2
                            break
                        if step(2):
                            print(action, num_step)
                            place_13()
                            num_step = 3
                            break
                        if step(3):
                            print(action, num_step)
                            place_13_above()
                            num_step = 4
                            break
                        if step(4):
                            print(action, num_step)
                            camera_state()
                            num_step = 5
                            break
                        if step(5):
                            print(action, num_step)
                            take_pic()
                            num_step = 0
                            action = 14
                            break
                    break
                if case(14):             
                    for step in switch(num_step):
                        if step(0):
                            print(action, num_step)
                            place_13_above()
                            num_step = 1
                            break
                        if step(1):
                            print(action, num_step)
                            place_13()
                            num_step = 2
                            break
                        if step(2):
                            print(action, num_step)
                            place_14()
                            num_step = 3
                            break
                        if step(3):
                            print(action, num_step)
                            place_14_above()
                            num_step = 4
                            break
                        if step(4):
                            print(action, num_step)
                            camera_state()
                            num_step = 5
                            break
                        if step(5):
                            print(action, num_step)
                            take_pic()
                            num_step = 0
                            action = 15
                            break
                    break
                if case(15):             
                    for step in switch(num_step):
                        if step(0):
                            print(action, num_step)
                            place_14_above()
                            num_step = 1
                            break
                        if step(1):
                            print(action, num_step)
                            place_14()
                            num_step = 2
                            break
                        if step(2):
                            print(action, num_step)
                            place_15()
                            num_step = 3
                            break
                        if step(3):
                            print(action, num_step)
                            place_15_above()
                            num_step = 4
                            break
                        if step(4):
                            print(action, num_step)
                            camera_state()
                            num_step = 5
                            break
                        if step(5):
                            print(action, num_step)
                            take_pic()
                            num_step = 0
                            action = 16
                            break
                    break
                if case(16):             
                    for step in switch(num_step):
                        if step(0):
                            print(action, num_step)
                            place_15_above()
                            num_step = 1
                            break
                        if step(1):
                            print(action, num_step)
                            place_15()
                            num_step = 2
                            break
                        if step(2):
                            print(action, num_step)
                            place_16()
                            num_step = 3
                            break
                        if step(3):
                            print(action, num_step)
                            place_16_above()
                            num_step = 4
                            break
                        if step(4):
                            print(action, num_step)
                            camera_state()
                            num_step = 5
                            break
                        if step(5):
                            print(action, num_step)
                            take_pic()
                            num_step = 0
                            action = 17
                            break
                    break
                if case(17):             
                    for step in switch(num_step):
                        if step(0):
                            print(action, num_step)
                            place_16_above()
                            num_step = 1
                            break
                        if step(1):
                            print(action, num_step)
                            place_16()
                            num_step = 2
                            break
                        if step(2):
                            print(action, num_step)
                            place_17()
                            num_step = 3
                            break
                        if step(3):
                            print(action, num_step)
                            place_17_above()
                            num_step = 4
                            break
                        if step(4):
                            print(action, num_step)
                            camera_state()
                            num_step = 5
                            break
                        if step(5):
                            print(action, num_step)
                            take_pic()
                            num_step = 0
                            action = 18
                            break
                    break
                if case(18):             
                    for step in switch(num_step):
                        if step(0):
                            print(action, num_step)
                            place_17_above()
                            num_step = 1
                            break
                        if step(1):
                            print(action, num_step)
                            place_17()
                            num_step = 2
                            break
                        if step(2):
                            print(action, num_step)
                            place_18()
                            num_step = 3
                            break
                        if step(3):
                            print(action, num_step)
                            place_18_above()
                            num_step = 4
                            break
                        if step(4):
                            print(action, num_step)
                            camera_state()
                            num_step = 5
                            break
                        if step(5):
                            print(action, num_step)
                            take_pic()
                            num_step = 0
                            action = 19
                            break
                    break
                if case(19):             
                    for step in switch(num_step):
                        if step(0):
                            print(action, num_step)
                            place_18_above()
                            num_step = 1
                            break
                        if step(1):
                            print(action, num_step)
                            place_18()
                            num_step = 2
                            break
                        if step(2):
                            print(action, num_step)
                            place_19()
                            num_step = 3
                            break
                        if step(3):
                            print(action, num_step)
                            place_19_above()
                            num_step = 4
                            break
                        if step(4):
                            print(action, num_step)
                            camera_state()
                            num_step = 5
                            break
                        if step(5):
                            print(action, num_step)
                            take_pic()
                            num_step = 0
                            action = 20
                            break
                    break
                if case(20):             
                    for step in switch(num_step):
                        if step(0):
                            print(action, num_step)
                            place_19_above()
                            num_step = 1
                            break
                        if step(1):
                            print(action, num_step)
                            place_19()
                            num_step = 2
                            break
                        if step(2):
                            print(action, num_step)
                            place_20()
                            num_step = 3
                            break
                        if step(3):
                            print(action, num_step)
                            place_20_above()
                            num_step = 4
                            break
                        if step(4):
                            print(action, num_step)
                            camera_state()
                            num_step = 5
                            break
                        if step(5):
                            print(action, num_step)
                            take_pic()
                            num_step = 0
                            action = 21
                            break
                    break
                if case(21):             
                    for step in switch(num_step):
                        if step(0):
                            print(action, num_step)
                            place_20_above()
                            num_step = 1
                            break
                        if step(1):
                            print(action, num_step)
                            place_20()
                            num_step = 2
                            break
                        if step(2):
                            print(action, num_step)
                            place_21()
                            num_step = 3
                            break
                        if step(3):
                            print(action, num_step)
                            place_21_above()
                            num_step = 4
                            break
                        if step(4):
                            print(action, num_step)
                            camera_state()
                            num_step = 5
                            break
                        if step(5):
                            print(action, num_step)
                            take_pic()
                            num_step = 0
                            action = 22
                            break
                    break
                if case(22):             
                    for step in switch(num_step):
                        if step(0):
                            print(action, num_step)
                            place_21_above()
                            num_step = 1
                            break
                        if step(1):
                            print(action, num_step)
                            place_21()
                            num_step = 2
                            break
                        if step(2):
                            print(action, num_step)
                            place_22()
                            num_step = 3
                            break
                        if step(3):
                            print(action, num_step)
                            place_22_above()
                            num_step = 4
                            break
                        if step(4):
                            print(action, num_step)
                            camera_state()
                            num_step = 5
                            break
                        if step(5):
                            print(action, num_step)
                            take_pic()
                            num_step = 0
                            action = 23
                            break
                    break
                if case(23):             
                    for step in switch(num_step):
                        if step(0):
                            print(action, num_step)
                            place_22_above()
                            num_step = 1
                            break
                        if step(1):
                            print(action, num_step)
                            place_22()
                            num_step = 2
                            break
                        if step(2):
                            print(action, num_step)
                            place_23()
                            num_step = 3
                            break
                        if step(3):
                            print(action, num_step)
                            place_23_above()
                            num_step = 4
                            break
                        if step(4):
                            print(action, num_step)
                            camera_state()
                            num_step = 5
                            break
                        if step(5):
                            print(action, num_step)
                            take_pic()
                            num_step = 0
                            action = 24
                            break
                    break
                if case(24):             
                    for step in switch(num_step):
                        if step(0):
                            print(action, num_step)
                            place_23_above()
                            num_step = 1
                            break
                        if step(1):
                            print(action, num_step)
                            place_23()
                            num_step = 2
                            break
                        if step(2):
                            print(action, num_step)
                            place_24()
                            num_step = 3
                            break
                        if step(3):
                            print(action, num_step)
                            place_24_above()
                            num_step = 4
                            break
                        if step(4):
                            print(action, num_step)
                            camera_state()
                            num_step = 5
                            break
                        if step(5):
                            print(action, num_step)
                            take_pic()
                            num_step = 0
                            action = 25
                            break
                    break
                if case(25):             
                    for step in switch(num_step):
                        if step(0):
                            print(action, num_step)
                            place_24_above()
                            num_step = 1
                            break
                        if step(1):
                            print(action, num_step)
                            place_24()
                            num_step = 2
                            break
                        if step(2):
                            print(action, num_step)
                            place_25()
                            num_step = 3
                            break
                        if step(3):
                            print(action, num_step)
                            place_25_above()
                            num_step = 4
                            break
                        if step(4):
                            print(action, num_step)
                            camera_state()
                            num_step = 5
                            break
                        if step(5):
                            print(action, num_step)
                            take_pic()
                            num_step = 0
                            action = 26
                            break
                    break
                if case(26):             
                    for step in switch(num_step):
                        if step(0):
                            print(action, num_step)
                            place_25_above()
                            num_step = 1
                            break
                        if step(1):
                            print(action, num_step)
                            place_25()
                            num_step = 2
                            break
                        if step(2):
                            print(action, num_step)
                            place_26()
                            num_step = 3
                            break
                        if step(3):
                            print(action, num_step)
                            place_26_above()
                            num_step = 4
                            break
                        if step(4):
                            print(action, num_step)
                            camera_state()
                            num_step = 5
                            break
                        if step(5):
                            print(action, num_step)
                            take_pic()
                            num_step = 0
                            action = 27
                            break
                    break
                if case(27):             
                    for step in switch(num_step):
                        if step(0):
                            print(action, num_step)
                            place_26_above()
                            num_step = 1
                            break
                        if step(1):
                            print(action, num_step)
                            place_26()
                            num_step = 2
                            break
                        if step(2):
                            print(action, num_step)
                            place_27()
                            num_step = 3
                            break
                        if step(3):
                            print(action, num_step)
                            place_27_above()
                            num_step = 4
                            break
                        if step(4):
                            print(action, num_step)
                            camera_state()
                            num_step = 5
                            break
                        if step(5):
                            print(action, num_step)
                            take_pic()
                            num_step = 0
                            action = 28
                            break
                    break
                if case(28):             
                    for step in switch(num_step):
                        if step(0):
                            print(action, num_step)
                            place_27_above()
                            num_step = 1
                            break
                        if step(1):
                            print(action, num_step)
                            place_27()
                            num_step = 2
                            break
                        if step(2):
                            print(action, num_step)
                            place_28()
                            num_step = 3
                            break
                        if step(3):
                            print(action, num_step)
                            place_28_above()
                            num_step = 4
                            break
                        if step(4):
                            print(action, num_step)
                            camera_state()
                            num_step = 5
                            break
                        if step(5):
                            print(action, num_step)
                            take_pic()
                            num_step = 0
                            action = 29
                            break
                    break
                if case(29):             
                    for step in switch(num_step):
                        if step(0):
                            print(action, num_step)
                            place_28_above()
                            num_step = 1
                            break
                        if step(1):
                            print(action, num_step)
                            place_28()
                            num_step = 2
                            break
                        if step(2):
                            print(action, num_step)
                            place_29()
                            num_step = 3
                            break
                        if step(3):
                            print(action, num_step)
                            place_29_above()
                            num_step = 4
                            break
                        if step(4):
                            print(action, num_step)
                            camera_state()
                            num_step = 5
                            break
                        if step(5):
                            print(action, num_step)
                            take_pic()
                            num_step = 0
                            action = 30
                            break
                    break
                if case(30):             
                    for step in switch(num_step):
                        if step(0):
                            print(action, num_step)
                            place_29_above()
                            num_step = 1
                            break
                        if step(1):
                            print(action, num_step)
                            place_29()
                            num_step = 2
                            break
                        if step(2):
                            print(action, num_step)
                            place_30()
                            num_step = 3
                            break
                        if step(3):
                            print(action, num_step)
                            place_30_above()
                            num_step = 4
                            break
                        if step(4):
                            print(action, num_step)
                            camera_state()
                            num_step = 5
                            break
                        if step(5):
                            print(action, num_step)
                            take_pic()
                            num_step = 0
                            action = 31
                            break
                    break
                if case(31):             
                    for step in switch(num_step):
                        if step(0):
                            print(action, num_step)
                            place_30_above()
                            num_step = 1
                            break
                        if step(1):
                            print(action, num_step)
                            place_30()
                            num_step = 2
                            break
                        if step(2):
                            print(action, num_step)
                            place_31()
                            num_step = 3
                            break
                        if step(3):
                            print(action, num_step)
                            place_31_above()
                            num_step = 4
                            break
                        if step(4):
                            print(action, num_step)
                            camera_state()
                            num_step = 5
                            break
                        if step(5):
                            print(action, num_step)
                            take_pic()
                            num_step = 0
                            action = 32
                            break
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
        ArmTask.Arm_Mode(4,1,0,2,2)  
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
