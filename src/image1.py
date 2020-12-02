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
  def __init__(self):
    rospy.init_node('image_processing_camera_1', anonymous=True)

    self.bridge = CvBridge()
    self.rate = rospy.Rate(30)

    self.image_sub1 = rospy.Subscriber("/camera1/robot/image_raw",Image,self.callback)

    # Publisher for centre coords - [BlueY, BlueZ, GreenY, GreenZ, RedY, RedZ]
    self.yz_plane_pub = rospy.Publisher("yz_centre_coords", Float64MultiArray, queue_size=1)
    # Publisher for sphere coords [SphereY, SphereZ]
    self.target_sphere_yz_pub = rospy.Publisher("target_sphere_yz_centre_coords", Float64MultiArray, queue_size=1)

    self.last_blue_y = 0
    self.last_blue_z = 0
    self.last_green_y = 0
    self.last_green_z = 0
    self.last_red_y = 0
    self.last_red_z = 0 

  # Recieve data from camera 1, process it, and publish
  def callback(self,data):
    try:
      self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    # Find centres of each joint
    red_y, red_z = self.detect_colour(self.cv_image1, (0,0,100), (0,0,255))
    blue_y, blue_z = self.detect_colour(self.cv_image1, (100,0,0), (255,0,0))
    green_y, green_z = self.detect_colour(self.cv_image1, (0,100,0), (0,255,0))
    
    # crop image to search target in so we can use a wider range of RGB values when masking for orange blobs
    cropped_target_detection_image = self.cv_image1[0:470, :]
    target_sphere_y, target_sphere_z = self.detect_target_sphere(cropped_target_detection_image, (0,45,100), (15,150,255))

    # Camera 1: get Y and Z
    centre_msg = Float64MultiArray()
    centre_msg.data = np.zeros(6)

    if blue_y is not None and blue_z is not None:
      self.last_blue_y = blue_y
      self.last_blue_z = blue_z 
      centre_msg.data[0] = blue_y
      centre_msg.data[1] = blue_z
    else:
      centre_msg.data[0] = self.last_blue_y
      centre_msg.data[1] = self.last_blue_z

    if green_y is not None and green_z is not None:
      self.last_green_y = green_y
      self.last_green_z = green_z
      centre_msg.data[2] = green_y
      centre_msg.data[3] = green_z
    else:
      centre_msg.data[2] = self.last_green_y
      centre_msg.data[3] = self.last_green_z

    if red_y is not None and red_z is not None:
      self.last_red_y = red_y
      self.last_red_z = red_z
      centre_msg.data[4] = red_y
      centre_msg.data[5] = red_z
    else:
      centre_msg.data[4] = self.last_red_y
      centre_msg.data[5] = self.last_red_z

    self.yz_plane_pub.publish(centre_msg)

    if target_sphere_y is not None and target_sphere_z is not None:
      target_sphere_msg = Float64MultiArray()
      target_sphere_msg.data = [target_sphere_y, target_sphere_z]
      self.target_sphere_yz_pub.publish(target_sphere_msg)

    im1=cv2.imshow('Camera 1 - YZ Plane', self.cv_image1)
    cv2.waitKey(1)

  def detect_target_sphere(self, image, lower, upper):
    mask = cv2.inRange(image, lower, upper)

    kernel = np.ones((5,5), np.uint8)
    mask = cv2.dilate(mask, kernel, iterations=3)

    contours, _ = cv2.findContours(mask, mode=cv2.RETR_EXTERNAL, method=cv2.CHAIN_APPROX_SIMPLE)

    target_contour = max(contours, key=lambda contour: len(cv2.approxPolyDP(contour, 0.001*cv2.arcLength(contour, True), True)), default=None)
    M = cv2.moments(target_contour)

    try:
      cx = int(M['m10'] / M['m00'])
      cy = int(M['m01'] / M['m00'])
    except ZeroDivisionError:
      return (None, None)

    return (cx, cy)

  def pixel2meters(self, image):
    yellow_coords = self.detect_colour(image, (0,100,100), (0,255,255))
    blue_coords = self.detect_colour(image, (100,0,0), (255,0,0))    
    pixel_distance = np.linalg.norm(np.array(blue_coords) - np.array(yellow_coords))
    return 2.5 / pixel_distance # we know distance from yellow to blue joint is 2.5 meters

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
