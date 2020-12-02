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
import math
from math import sin, cos
import sympy as sp

class kinematics:

    def __init__(self):
        rospy.init_node('kinematics', anonymous=True)

        self.estimated_joint_angles = rospy.Subscriber("estimated_joint_angles", Float64MultiArray, self.estimated_joint_angle_callback, queue_size=1)
        self.fk_estimated_pub = rospy.Publisher("fk_estimated_end_effector", Float64MultiArray, queue_size=1)

        self.joint2_sub = message_filters.Subscriber("/robot/joint2_position_controller/command", Float64, queue_size=1)
        self.joint3_sub = message_filters.Subscriber("/robot/joint3_position_controller/command", Float64, queue_size=1)
        self.joint4_sub = message_filters.Subscriber("/robot/joint4_position_controller/command", Float64, queue_size=1)
        self.synced_sub = message_filters.ApproximateTimeSynchronizer([self.joint2_sub, self.joint3_sub, self.joint4_sub], 1, 1, allow_headerless=True)
        self.synced_sub.registerCallback(self.actual_joint_angle_callback)
        self.fk_actual_pub = rospy.Publisher("fk_actual_end_effector", Float64MultiArray, queue_size=1)

        self.rate = rospy.Rate(60)
    
    def estimated_joint_angle_callback(self, angles_msg):
        joint1_angle, joint2_angle, joint3_angle, joint4_angle = angles_msg.data

        fk_msg = Float64MultiArray()
        fk_msg.data = self.calculate_FK(joint1_angle, joint2_angle, joint3_angle, joint4_angle)
        self.fk_estimated_pub.publish(fk_msg)

    def actual_joint_angle_callback(self, joint2_msg, joint3_msg, joint4_msg):
        joint1_angle = 0
        joint2_angle = joint2_msg.data
        joint3_angle = joint3_msg.data
        joint4_angle = joint4_msg.data

        fk_msg = Float64MultiArray()
        fk_msg.data = self.calculate_FK(joint1_angle, joint2_angle, joint3_angle, joint4_angle)
        self.fk_actual_pub.publish(fk_msg)

    def calculate_FK(self, theta_1, theta_2, theta_3, theta_4):
        h_0_1 = self.create_HTM_matrix(theta_1 + np.pi/2, np.pi/2, 0,   2.5)
        h_1_2 = self.create_HTM_matrix(theta_2 + np.pi/2, np.pi/2, 0,   0  )
        h_2_3 = self.create_HTM_matrix(theta_3,           -np.pi/2, 3.5, 0  )
        h_3_4 = self.create_HTM_matrix(theta_4,           0,       3,   0  )

        h_0_4 = h_0_1 @ h_1_2 @ h_2_3 @ h_3_4

        return h_0_4[:3, -1].reshape(-1)

    def create_HTM_matrix(self, theta, alpha, r, d):
        return np.array([[np.cos(theta), -np.sin(theta) * np.cos(alpha),  np.sin(theta) * np.sin(alpha), r * np.cos(theta)],
                         [np.sin(theta),  np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), r * np.sin(theta)],
                         [0,              np.sin(alpha),                  np.cos(alpha),                 d                ],
                         [0,              0,                              0,                             1                ]
                        ])

# call the class
def main(args):
    fk = kinematics()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)