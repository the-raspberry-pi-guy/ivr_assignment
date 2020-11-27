#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  # Defines publisher and subscriber
  def __init__(self):
    # initialize the node named image_processing
    rospy.init_node('image_processing', anonymous=True)
    # initialize a publisher to send images from camera2 to a topic named image_topic2
    self.image_pub2 = rospy.Publisher("image_topic2",Image, queue_size = 1)
    # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
    self.image_sub2 = rospy.Subscriber("/camera2/robot/image_raw",Image,self.callback2)
    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()
    self.rate = rospy.Rate(30)

    # Publisher for centre coords - [BlueX, BlueZ, GreenX, GreenZ, RedX, RedZ]
    self.xz_plane_pub = rospy.Publisher("xz_centre_coords", Float64MultiArray, queue_size=1)

    self.obst_pub = rospy.Publisher("/camera2/obst_pub", Float64, queue_size=1)

    self.last_blue_x = 0
    self.last_blue_z = 0
    self.last_green_x = 0
    self.last_green_z = 0
    self.last_red_x = 0
    self.last_red_z = 0 

  # Recieve data, process it, and publish
  def callback2(self,data):
    # Recieve the image
    try:
      self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    # Find centres of each joint
    red_x, red_z = self.detect_colour(self.cv_image1, (0,0,100), (0,0,255)) # Detect red
    blue_x, blue_z = self.detect_colour(self.cv_image1, (100,0,0), (255,0,0)) # Detect blue
    green_x, green_z = self.detect_colour(self.cv_image1, (0,100,0), (0,255,0)) # Detect green

    # Camera 1: get X and Z
    centre_msg = Float64MultiArray()
    centre_msg.data = np.zeros(6)

    obst_msg = Float64()

    if blue_x is not None and blue_z is not None:
      self.last_blue_x = blue_x
      self.last_blue_z = blue_z 
      centre_msg.data[0] = blue_x
      centre_msg.data[1] = blue_z
    else:
      obst_msg.data = 200
      self.obst_pub.publish(obst_msg)
      centre_msg.data[0] = self.last_blue_x
      centre_msg.data[1] = self.last_blue_z

    if green_x is not None and green_z is not None:
      self.last_green_x = green_x
      self.last_green_z = green_z
      centre_msg.data[2] = green_x
      centre_msg.data[3] = green_z
    else:
      obst_msg.data = 300
      self.obst_pub.publish(obst_msg)
      centre_msg.data[2] = self.last_green_x
      centre_msg.data[3] = self.last_green_z

    if red_x is not None and red_z is not None:
      self.last_red_x = red_x
      self.last_red_z = red_z
      centre_msg.data[4] = red_x
      centre_msg.data[5] = red_z
    else:
      obst_msg.data = 400
      self.obst_pub.publish(obst_msg)
      centre_msg.data[4] = self.last_red_x
      centre_msg.data[5] = self.last_red_z

    self.xz_plane_pub.publish(centre_msg)

    im1=cv2.imshow('Camera 2 - XZ Plane', self.cv_image1)
    cv2.waitKey(1)

  def detect_colour(self, image, lower, upper):
    mask = cv2.inRange(image, lower, upper)

    kernel = np.ones((5,5), np.uint8)
    mask = cv2.dilate(mask, kernel, iterations=3)

    M = cv2.moments(mask)
    try:
      cx = int(M['m10'] / M['m00'])
      cy = int(M['m01'] / M['m00'])
    except ZeroDivisionError:
      return (None, None)

    return (cx, cy)

# call the class
def main(args):
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)
