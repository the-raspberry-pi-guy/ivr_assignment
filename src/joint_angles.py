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

        # Centre coords from camera 1
        self.yz_sub = message_filters.Subscriber("/yz_centre_coords", Float64MultiArray, queue_size=10)
        # Centre coords from camera 2
        self.xz_sub = message_filters.Subscriber("/xz_centre_coords", Float64MultiArray, queue_size=10)
        self.synced_sub = message_filters.ApproximateTimeSynchronizer([self.yz_sub, self.xz_sub], 10, 1, allow_headerless=True)
        self.synced_sub.registerCallback(self.callback)

        self.joint_angles_pub = rospy.Publisher("estimated_joint_angles", Float64MultiArray, queue_size=10)

        self.rate = rospy.Rate(30)
        self.move()

    def callback(self, yz_coords, xz_coords):
        # Calculate joint angles

        joint_angles_msg = Float64MultiArray()
        joint_angles_msg.data = self.calculate_angles(yz_coords.data, xz_coords.data)
        self.joint_angles_pub.publish(joint_angles_msg)

    def calculate_angles(self, yz_coords, xz_coords):
        blue_coords = np.array([xz_coords[0], yz_coords[0], np.mean([xz_coords[1], yz_coords[1]])])
        green_coords = np.array([xz_coords[2], yz_coords[2], np.mean([xz_coords[3], yz_coords[3]])])
        red_coords = np.array([xz_coords[4], yz_coords[4], np.mean([xz_coords[5], yz_coords[5]])])

        vec_yellow_blue = np.array([0, 0, 1])
        vec_blue_green = blue_coords - green_coords

        vec_y_world = np.array([0,1,0])
        vec_x_world = np.array([1,0,0])

        vec_x_rotated_axis = np.cross(vec_y_world, vec_blue_green)

        print("Blue: " + str(blue_coords))
        print("Green: " + str(green_coords))
        print("Red: " + str(red_coords))
        print("Vec Blue-Green:" + str(vec_blue_green))
        print("Vec X Rotated Axis: " + str(vec_x_rotated_axis))
        print("---")        

        joint3_angle = np.arctan2(vec_x_rotated_axis[2], vec_x_rotated_axis[0])

        return np.array([joint3_angle])

    def angle_between_vecs(self, vec1, vec2):
        unit_vec1 = vec1 / np.linalg.norm(vec1)
        unit_vec2 = vec2 / np.linalg.norm(vec2)
        dot_prod = np.dot(unit_vec1, unit_vec2)
        return np.arccos(dot_prod)

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

            #self.robot_joint2_pub.publish(joint2)
            #self.robot_joint3_pub.publish(joint3)
            #self.robot_joint4_pub.publish(joint4)

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