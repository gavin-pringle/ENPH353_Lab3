#!/usr/bin/env python3

## @package enph353_ros_lab
#
# A node script for simple proportional control line following.

from __future__ import print_function

import roslib
roslib.load_manifest('my_package')
import sys
from geometry_msgs.msg import Twist
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

## Class that subscribes to image stream and publishes velocity messages
#
#  Uses OpenCV to detect line and follow it
class image_converter:

  ## Constructor that declares which topics are being published to and subscribed to
  def __init__(self):
    self.drive_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/rrbot/camera1/image_raw",Image,self.callback, queue_size=3)

  ## Callback function that publishes velocity commands to the drive topic
  def callback(self,data):
    # convert image to a format compatible with OpenCV
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    # Using OpenCV to process the image from the robot's camera into a binnarized blurred image
    threshold = 131
    _, img_bin = cv2.threshold(cv_image, threshold, 255, cv2.THRESH_BINARY)
    img_blur = cv2.blur(img_bin,(5,5)) 

    imgWidth = 800
    lineHeight = 670
    sum = 0
    numBlack = 0
    for i in range(imgWidth):
      (b, g, r) = img_blur[lineHeight, i]
      if b == 0:
        sum += i
        numBlack += 1
    if numBlack == 0:
      numBlack = 1
    # Find the x coordinate of where the line is on the screen
    coord = sum // numBlack
    # Find the error of how far the robot is pointed away from the line
    error = coord - 400

    move = Twist()

    # Move forward at a constant speed of 1
    move.linear.x = 1
    # Turn to face toward the line
    move.angular.z = -1 * error // 100
    
    # publish drive commands
    try:
      self.drive_pub.publish(move)
    except CvBridgeError as e:
      print(e)


## Main function that keeps the simulation running until it is stopped by a user
def main(args):
  ic = image_converter()
  # name of node is image_converter
  rospy.init_node('robot_driver', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
