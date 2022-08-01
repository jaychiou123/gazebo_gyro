#!/usr/bin/env python
import sys
import cv2
import rospy
import rogata_library as rgt
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np



def img_call(ros_img,object_name_list):
	cv_img = bridge.imgmsg_to_cv2(ros_img,'mono8')
        rgt.track_dynamic_objects(cv_img,object_name_list)



if __name__ == '__main__':
	try:
		rospy.init_node("Dynamic Object Tracker")
                bridge       = CvBridge()
                object_numb  = len(sys.argv)
                object_list  = [sys.argv[i] for i in range(2,object_numb)]
		cam_sub      = rospy.Subscriber(sys.argv[1],Image,img_call,object_list)
		rospy.spin()
	except rospy.ROSInterruptException:
		rospy.loginfo("Dynamic Object Tracker  Node not working")

