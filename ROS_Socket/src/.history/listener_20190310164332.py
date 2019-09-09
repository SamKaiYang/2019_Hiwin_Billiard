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

##------------listener subscriber-------
def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    
def listener():
    rospy.init_node('ros_socket', anonymous=True)

    rospy.Subscriber("chatter", String, callback)
    rospy.spin()
##------------listener subscriber end-------