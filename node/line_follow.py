#!/usr/bin/env python3

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

class image_converter:

  def __init__(self):
    self.drive_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/rrbot/camera1/image_raw",Image,self.callback, queue_size=3)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

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
    coord = sum // numBlack
    error = coord - 400

    move = Twist()

    move.linear.x = 1
    move.angular.z = -1 * error // 100

    try:
      self.drive_pub.publish(move)
    except CvBridgeError as e:
      print(e)

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
