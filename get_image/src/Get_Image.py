#!/usr/bin/env python3
import sys
sys.path.insert(1, "/usr/local/lib/python3.5/dist-packages/")
sys.path.insert(0, "/opt/installer/open_cv/cv_bridge/lib/python3/dist-packages/")
# import sys
# sys.path.insert(1, "/home/luca/.local/lib/python3.5/site-packages/")
# sys.path.insert(0, "/opt/installer/open_cv/cv_bridge/lib/python3/dist-packages/")
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image

from datetime import datetime
import cv2
from cv_bridge import CvBridge, CvBridgeError
import time 
import argparse
import numpy as np
import os

parser = argparse.ArgumentParser()
parser.add_argument('--Object_Name', type=str, default='.', help='Class name of training object.')
FLAGS = parser.parse_args()

Object_Name = FLAGS.Object_Name
current_path = os.path.dirname(os.path.abspath(__file__))
print("current_path.split(" ") : ", str(datetime.now()))

execute_time = str(datetime.now())
execute_time = execute_time.split(' ')[0] + '_' + execute_time.split(' ')[1]
Train_Data_Dir = os.path.dirname(os.path.realpath(__file__)) + '/Training_Data/' + \
     execute_time + '_' + Object_Name + '/'

class Get_image():
    def __init__(self):
            rospy.init_node('get_image_from_FLIR', anonymous=True)
            self.bridge = CvBridge()
            self.image = np.zeros((0,0,3), np.uint8)
            self.take_picture_counter = 0
            self.mtx = np.load(current_path + '/camera_calibration_mtx.npy')
            self.dist = np.load(current_path + '/camera_calibration_dist.npy')
            self.newcameramtx = np.load(current_path + '/camera_calibration_newcameramtx.npy')
            self.dst_roi_x, self.dst_roi_y, self.dst_roi_w, self.dst_roi_h  = np.load(current_path + '/camera_calibration_roi.npy')

            rospy.Subscriber("/camera/image_color", Image, self.callback)

            if not os.path.exists(Train_Data_Dir):
                os.makedirs(Train_Data_Dir)
            rospy.spin()

    def callback(self, image):
        
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
            un_dst_img = cv2.undistort(self.cv_image, self.mtx, self.dist, None, self.newcameramtx)
            un_dst_img = un_dst_img[self.dst_roi_y:self.dst_roi_y+self.dst_roi_h, self.dst_roi_x:self.dst_roi_x+self.dst_roi_w]
            pass
        except CvBridgeError as e:
            print(e)
        
        cv2.namedWindow("result", cv2.WINDOW_NORMAL)
        cv2.imshow("result", un_dst_img)
        self.get_image(un_dst_img)
        # cv2.imshow("result", self.cv_image)
        # self.get_image(self.cv_image)
        cv2.waitKey(1)
        
    def get_image(self, crop_image):
        if cv2.waitKey(33) & 0xFF == ord('s'):
            name = str(Train_Data_Dir + execute_time + '_' + Object_Name + '_' + str(self.take_picture_counter+1) + ".jpg")
            cv2.imwrite(name,crop_image)
            print("[Save] ", name)
            self.take_picture_counter += 1
        else:
            pass

if __name__ == '__main__':
    listener = Get_image()
    cv2.destroyAllWindows()
    pass
