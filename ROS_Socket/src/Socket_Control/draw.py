#!/usr/bin/env python3                          
# license removed for brevity

import threading
import time
import rospy
import os
import numpy as np
from std_msgs.msg import String
import math
import enum
#import Hiwin_RT605_Socket as ArmTask
from std_msgs.msg import Int32MultiArray

#畫圖import
import tkinter
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches


r = 1.75         #r=1.75cm
M = [20, 45]     #母球 M=(Mx,My)
C = [0, 35]
OB_1 = [21, 47]
OB_2 = [10, 40]




def draw():
    global r
    global M, C, OB_1, OB_2
    theta = np.arange(0, 2*np.pi, 0.01)

    draw_Mx = M[0] + r * np.cos(theta)
    draw_My = M[1] + r * np.sin(theta)

    draw_Cx = C[0] + r * np.cos(theta)
    draw_Cy = C[1] + r * np.sin(theta)

    draw_OB_1x = OB_1[0] + r * np.cos(theta)
    draw_OB_1y = OB_1[1] + r * np.sin(theta)

    draw_OB_2x = OB_2[0] + r * np.cos(theta)
    draw_OB_2y = OB_2[1] + r * np.sin(theta)

    fig = plt.figure() 
    axes = fig.add_subplot(111) 

    axes.add_patch(patches.Rectangle((-27, 25), 59, 28))# ((x,y), width, height)
    #plt.plot([H_S[0],H_E[0],[H_S[1],H_E[1]],'k--')
    axes.plot(draw_Mx, draw_My, 'w') #母球用黑色 表示
    axes.plot(draw_Cx, draw_Cy, 'y') #子球用紅色 表示
    axes.plot(draw_OB_1x, draw_OB_1y, 'b') #干擾球用紅色 表示
    axes.plot(draw_OB_2x, draw_OB_2y, 'b') #干擾球用紅色 表示

    axes.axis('equal')
    plt.title('billiard')

    plt.ion()

    ...

    plt.pause(1500) 
    plt.close()
if __name__ == '__main__': #輸入開始指令
    while 1:
            draw()

    
    rospy.spin()
