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
from kinematics_sympy import evaluate_jacobian

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

        self.target_sub = message_filters.Subscriber("/target_sphere_coords", Float64MultiArray, queue_size=1)
        self.end_effector_sub = message_filters.Subscriber("/red_sphere_coords", Float64MultiArray, queue_size=1)
        self.synced_control_sub = message_filters.ApproximateTimeSynchronizer([self.target_sub, self.end_effector_sub], 1, 1, allow_headerless=True)
        self.synced_control_sub.registerCallback(self.control_callback)
        self.robot_joint1_pub = rospy.Publisher("/robot/joint1_position_controller/command", Float64, queue_size=10)
        self.robot_joint2_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10)
        self.robot_joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
        self.robot_joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)
        
        self.control_thetas = np.array([0,0,0,0])
        self.previous_time = rospy.get_time()
        self.previous_error = np.array([0,0,0])
        self.error_i = np.array([0,0,0])

        self.rate = rospy.Rate(60)
    
    def estimated_joint_angle_callback(self, angles_msg):
        joint1_angle, joint2_angle, joint3_angle, joint4_angle = angles_msg.data

        fk_msg = Float64MultiArray()
        fk_msg.data = self.calculate_FK(joint1_angle, joint2_angle, joint3_angle, joint4_angle)
        self.fk_estimated_pub.publish(fk_msg)

    def control_callback(self, target_msg, end_effector_msg):
        self.control_closed(np.array(target_msg.data), np.array(end_effector_msg.data))
        joint1_angle, joint2_angle, joint3_angle, joint4_angle = self.control_thetas

        joint1 = Float64()
        joint1.data = joint1_angle
        joint2 = Float64()
        joint2.data = joint2_angle
        joint3 = Float64()
        joint3.data = joint3_angle
        joint4 = Float64()
        joint4.data = joint4_angle

        self.robot_joint1_pub.publish(joint1)
        self.robot_joint2_pub.publish(joint2)
        self.robot_joint3_pub.publish(joint3)
        self.robot_joint4_pub.publish(joint4)

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
    
    def control_closed(self, target_xyz_pos, end_effector_xyz_pos):
        K_p = np.eye(3) * 0.2 # 0.2
        K_d = np.eye(3) * 0.15 # 0.2
        K_i = np.eye(3) * 1e-5   # 1e-5

        cur_time = rospy.get_time()
        dt = cur_time - self.previous_time
        self.previous_time = cur_time

        self.error = target_xyz_pos - end_effector_xyz_pos
        self.error_i = self.error_i + self.error * dt
        self.error_d = (self.error - self.previous_error) / dt
        pid_output = K_p.dot(self.error.T) + K_d.dot(self.error_d.T) # + K_i.dot(self.error_i.T)

        J_inv = np.linalg.pinv(evaluate_jacobian(self.control_thetas[0], self.control_thetas[1], self.control_thetas[2], self.control_thetas[3]))
        
        print("Control thetas: " + str(self.control_thetas))
        q_desired_d = J_inv.dot(pid_output)
        print("q_desired_d: " + str(q_desired_d))
        q_desired = self.control_thetas + (q_desired_d * dt)
        print("q_desired: " + str(q_desired))
        print("---")

        self.control_thetas = q_desired
        self.previous_error = self.error

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