#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
import message_filters
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError

class joint_angles:
	
    def __init__(self):
		rospy.init_node('image_processing', anonymous=True)

        self.robot_joint2_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10)
        self.robot_joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
        self.robot_joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)

        self.joint_angles_pub = rospy.Publisher("estimated_joint_angles", Float64MultiArray, queue_size=10)

        self.image1_sub = message_filters.Subscriber()

    # Publish data
    def move(self):

        t0 = rospy.get_time()

        while not rospy.is_shutdown():
            cur_time = np.array([rospy.get_time()])-t0
            angle2 = np.pi/2 * np.sin(cur_time * np.pi/15)
            angle3 = np.pi/2 * np.sin(cur_time * np.pi/18)
            angle4 = np.pi/2 * np.sin(cur_time * np.pi/20)

            joint2 = Float64()
            joint2.data = angle2
            joint3 = Float64()
            joint3.data = angle3
            joint4 = Float64()
            joint4.data = angle4

            self.robot_joint2_pub.publish(joint2)
            self.robot_joint3_pub.publish(joint3)
            self.robot_joint4_pub.publish(joint4)

            self.rate.sleep()