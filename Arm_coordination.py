0#!/usr/bin/env python3 
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
import rospy
import numpy as np
import time
from sensor_msgs.msg import Image
import os
import threading
import time
import rospy
import os
import numpy as np
from std_msgs.msg import String
import math
import enum
from std_msgs.msg import Int32MultiArray
import tkinter
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
# -----------------------------------
# -------------------
# 20190813 BMX,BMY
BMX = -31.28
BMY = 35.14
# -------------------
global ball
ball = [[0.0, 0.0] for i in range(10)]
def count():
    # 顺时针 左上開始 到左下 2 3 4 5 6 7 8 9
    A = 5*math.sin(5)
    B = 5*math.cos(5)
    ball[2][0] = ball[4][0] - 2*B
    ball[2][1] = ball[4][1] - 2*A

    ball[3][0] = ball[4][0] - B
    ball[3][1] = ball[4][1] - A

    ball[5][0] = ball[4][0] + B
    ball[5][1] = ball[4][1] + A

    ball[6][0] = ball[4][0] + B + A
    ball[6][1] = ball[4][1] + A - B

    ball[7][0] = ball[4][0] + A
    ball[7][1] = ball[4][1] - B

    ball[8][0] = ball[4][0] - B + A
    ball[8][1] = ball[4][1] - A - B

    ball[9][0] = ball[4][0] - 2*B + A
    ball[9][1] = ball[4][1] - 2*A - B

def write():
    SaveDirectory = '/home/iclab/Desktop'
    Arm_X_path = SaveDirectory +'/Arm_X'+'.txt'
    Arm_Y_path = SaveDirectory +'/Arm_Y'+'.txt'
    Arm_X_file = open(Arm_X_path, 'a')
    Arm_Y_file = open(Arm_Y_path, 'a')
    for index in range (9,1,-1):
        Arm_X_file.write(str(ball[index][0])+', ')
        print(str(ball[index][0])+', ')
        Arm_Y_file.write(str(ball[index][1])+', ')
for times in range(0,32):
    if times == 0:
        ball[4][0] = BMX + 12.18
        ball[4][1] = BMY + 2.86
        count()
        write()
    if times == 1:
        ball[4][0] = BMX + 12.18 + 6
        ball[4][1] = BMY + 2.86
        count()
        write()
    if times == 2:
        ball[4][0] = BMX + 12.18 + 12
        ball[4][1] = BMY + 2.86
        count()
        write()
    if times == 3:
        ball[4][0] = BMX + 12.18 + 18
        ball[4][1] = BMY + 2.86
        count()
        write()
    if times == 4:
        ball[4][0] = BMX + 12.18 + 24
        ball[4][1] = BMY + 2.86
        count()
        write() 
    if times == 5:
        ball[4][0] = BMX + 12.18 + 30
        ball[4][1] = BMY + 2.86
        count()
        write()   
    if times == 6:
        ball[4][0] = BMX + 12.18 + 36
        ball[4][1] = BMY + 2.86
        count()
        write() 
    if times == 7:
        ball[4][0] = BMX + 12.18 + 42
        ball[4][1] = BMY + 2.86
        count()
        write()   
    if times == 8:
        ball[4][0] = BMX + 12.18 + 42
        ball[4][1] = BMY + 2.86 + 6
        count()
        write()
    if times == 9:
        ball[4][0] = BMX + 12.18 + 36
        ball[4][1] = BMY + 2.86 + 6
        count()
        write()
    if times == 10:
        ball[4][0] = BMX + 12.18 + 30
        ball[4][1] = BMY + 2.86 + 6
        count()
        write()
    if times == 11:
        ball[4][0] = BMX + 12.18 + 24
        ball[4][1] = BMY + 2.86 + 6
        count()
        write()
    if times == 12:
        ball[4][0] = BMX + 12.18 + 18
        ball[4][1] = BMY + 2.86 + 6
        count()
        write() 
    if times == 13:
        ball[4][0] = BMX + 12.18 + 12
        ball[4][1] = BMY + 2.86 + 6
        count()
        write()   
    if times == 14:
        ball[4][0] = BMX + 12.18 + 6
        ball[4][1] = BMY + 2.86 + 6
        count()
        write() 
    if times == 15:
        ball[4][0] = BMX + 12.18
        ball[4][1] = BMY + 2.86 + 6
        count()
        write()   
    if times == 16:
        ball[4][0] = BMX + 12.18
        ball[4][1] = BMY + 2.86 + 12
        count()
        write()
    if times == 17:
        ball[4][0] = BMX + 12.18 + 6
        ball[4][1] = BMY + 2.86 + 12
        count()
        write()
    if times == 18:
        ball[4][0] = BMX + 12.18 + 12
        ball[4][1] = BMY + 2.86 + 12
        count()
        write()
    if times == 19:
        ball[4][0] = BMX + 12.18 + 18
        ball[4][1] = BMY + 2.86 + 12
        count()
        write()
    if times == 20:
        ball[4][0] = BMX + 12.18 + 24
        ball[4][1] = BMY + 2.86 + 12
        count()
        write() 
    if times == 21:
        ball[4][0] = BMX + 12.18 + 30
        ball[4][1] = BMY + 2.86 + 12
        count()
        write()   
    if times == 22:
        ball[4][0] = BMX + 12.18 + 36
        ball[4][1] = BMY + 2.86 + 12
        count()
        write() 
    if times == 23:
        ball[4][0] = BMX + 12.18 + 42
        ball[4][1] = BMY + 2.86 + 12
        count()
        write()   
    if times == 24:
        ball[4][0] = BMX + 12.18 + 42
        ball[4][1] = BMY + 2.86 + 18
        count()
        write()
    if times == 25:
        ball[4][0] = BMX + 12.18 + 36
        ball[4][1] = BMY + 2.86 + 18
        count()
        write()
    if times == 26:
        ball[4][0] = BMX + 12.18 + 30
        ball[4][1] = BMY + 2.86 + 18
        count()
        write()
    if times == 27:
        ball[4][0] = BMX + 12.18 + 24
        ball[4][1] = BMY + 2.86 + 18
        count()
        write()
    if times == 28:
        ball[4][0] = BMX + 12.18 + 18
        ball[4][1] = BMY + 2.86 + 18
        count()
        write() 
    if times == 29:
        ball[4][0] = BMX + 12.18 + 12
        ball[4][1] = BMY + 2.86 + 18
        count()
        write()   
    if times == 30:
        ball[4][0] = BMX + 12.18 + 6
        ball[4][1] = BMY + 2.86 + 18
        count()
        write() 
    if times == 31:
        ball[4][0] = BMX + 12.18
        ball[4][1] = BMY + 2.86 + 18
        count()
        write()   
