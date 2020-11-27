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

class joint_angles:

    def __init__(self):
        rospy.init_node('image_processing', anonymous=True)

        self.robot_joint2_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10)
        self.robot_joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
        self.robot_joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)
        self.vec_bg_pub = rospy.Publisher("/vec_bg", Float64MultiArray, queue_size=1)

        # Centre coords from camera 1
        self.yz_sub = message_filters.Subscriber("/yz_centre_coords", Float64MultiArray, queue_size=1)
        # Centre coords from camera 2
        self.xz_sub = message_filters.Subscriber("/xz_centre_coords", Float64MultiArray, queue_size=1)
        self.synced_sub = message_filters.ApproximateTimeSynchronizer([self.yz_sub, self.xz_sub], 1, 1, allow_headerless=True)
        self.synced_sub.registerCallback(self.callback)

        self.joint_angles_pub = rospy.Publisher("estimated_joint_angles", Float64MultiArray, queue_size=1)

        self.rate = rospy.Rate(30)
        self.move()

    def callback(self, yz_coords, xz_coords):
        # Calculate joint angles

        joint_angles_msg = Float64MultiArray()
        joint_angles_msg.data = self.calculate_angles(yz_coords.data, xz_coords.data)
        self.joint_angles_pub.publish(joint_angles_msg)

    def calculate_angles(self, yz_coords, xz_coords):
        blue_z = max([xz_coords[1], yz_coords[1]])
        green_z = max([xz_coords[3], yz_coords[3]])
        green_z = min(blue_z, green_z)

        blue_coords = np.array([xz_coords[0], yz_coords[0], blue_z])
        green_coords = np.array([xz_coords[2], yz_coords[2], green_z])
        red_coords = np.array([xz_coords[4], yz_coords[4], max([xz_coords[5], yz_coords[5]])])

        if 0 in blue_coords.tolist():
            print("BLUE OBSTRUCTED")
        if 0 in green_coords.tolist():
            print("GREEN OBSTRUCTED")

        vec_YB = np.array([0, 0, -1])
        vec_BG = blue_coords - green_coords

        vec_bg_msg = Float64MultiArray()
        vec_bg_msg.data = vec_BG
        self.vec_bg_pub.publish(vec_bg_msg)

        joint2_angle = np.arctan2(vec_BG[1], vec_BG[2])
        # Select value for special case where y and z are both 0 (atan2 technically undefined in this case)
        # Numpy returns 0, we arbitrarily choose pi/2 as opposed to -pi/2
        # if joint2_angle == 0:
        #     joint2_angle = math.pi / 2

        print("Blue: " + str(blue_coords))
        print("Green: " + str(green_coords))
        print("Red: " + str(red_coords))
        print("Vec YB: " + str(vec_YB)) 
        print("Vec BG: " + str(vec_BG))
        print("Angle 2: " + str(joint2_angle))
        #print("Angle 3: " + str(joint3_angle))
        print("---")        

        return np.array([joint2_angle])

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
            # self.robot_joint4_pub.publish(joint4)

            self.rate.sleep()

# call the class
def main(args):
    ja = joint_angles()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)