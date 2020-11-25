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
    # initialize a publisher to send images from camera1 to a topic named image_topic1
    self.image_pub1 = rospy.Publisher("image_topic1",Image, queue_size = 1)
    # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
    self.image_sub1 = rospy.Subscriber("/camera1/robot/image_raw",Image,self.callback1)
    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()
    self.rate = rospy.Rate(30)

    # initialize a publisher to send joints' angular position to the robot
    self.robot_joint2_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10)
    self.robot_joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
    self.robot_joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)

    self.move()


  # Recieve data from camera 1, process it, and publish
  def callback1(self,data):
    # Recieve the image
    try:
      self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    
    # Uncomment if you want to save the image
    #cv2.imwrite('image_copy.png', cv_image)

    red_centre = self.detect_colour(self.cv_image1, (0,0,100), (0,0,255)) # Detect red
    blue_centre = self.detect_colour(self.cv_image1, (100,0,0), (255,0,0)) # Detect blue
    green_centre = self.detect_colour(self.cv_image1, (0,100,0), (0,255,0)) # Detect green
    rospy.loginfo("Red centre: " + str(red_centre))
    rospy.loginfo("Blue centre: " + str(blue_centre))
    rospy.loginfo("Green centre: " + str(green_centre))

    im1=cv2.imshow('window1', self.cv_image1)
    cv2.waitKey(1)
    # Publish the results
    try: 
      self.image_pub1.publish(self.bridge.cv2_to_imgmsg(self.cv_image1, "bgr8"))
    except CvBridgeError as e:
      print(e)

  def detect_colour(self, image, lower, upper):
    mask = cv2.inRange(image, lower, upper)

    kernel = np.ones((5,5), np.uint8)
    mask = cv2.dilate(mask, kernel, iterations=3)

    M = cv2.moments(mask)
    try:
      cx = int(M['m10'] / M['m00'])
      cy = int(M['m01'] / M['m00'])
    except ZeroDivisionError:
      return None

    return np.array([cx, cy])

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


