#!/usr/bin/env python3
# license removed for brevity
import rospy
import os
from std_msgs.msg import String
from ROS_Socket.srv import *
from ROS_Socket.msg import *
import listener as listen
import enum
import Hiwin_ros_Strategy as ros_strategy
import Hiwin_socket_ros
import Hiwin_socket_Taskcmd
import Hiwin_socket_TCPcmd

if __name__ == '__main__':
    cmd = 'python3 Hiwin_ros_Strategy.py'
    os.system(cmd)
    socket = 'python3 Hiwin_socket_ros.py'
    os.system(socket)
    