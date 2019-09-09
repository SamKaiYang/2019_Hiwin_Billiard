#!/usr/bin/env python3
# license removed for brevity
import sys
import os
import numpy as np
import rospy
import matplotlib as plot
from std_msgs.msg import String
from ROS_Socket.srv import *
import Hiwin_socket_TCPcmd as TCP
##------------talker-------

def talker(Arm_state):
    pub = rospy.Publisher('chatter', String, queue_size=10) #publisher 初始化
    rospy.init_node('ros_strategy', anonymous=True) #node initial
    rate = rospy.Rate(10) # 10hz
    if not rospy.is_shutdown(): #當rospy尚未關閉時
        if Arm_state == 0:
            Arm_state = 1
            Arm_State_feedback = 1
            Arm_state_str = "Arm State %s" % Arm_State_feedback
            rospy.loginfo(Arm_state_str)
            pub.publish(Arm_state_str)
            #pub.publish（255.0,255.0,255.0,128.0） # colorRGBA
            rate.sleep()
##------------talker end-------

    #     talker()
    # except rospy.ROSInterruptException:
    #     pass