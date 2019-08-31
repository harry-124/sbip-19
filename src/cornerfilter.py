#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('sbip')
import sys
import rospy
import numpy as np
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def getcorner(self):
    mask_red = cv2.inRange(self.cv_image,np.array([0,0,100]),np.array([50,50,250]))
    kernel =np.ones((10,10),np.uint8)
    dilation = cv2.dilate(mask_red,kernel,iterations = 1)
    cv2.imshow('asd',dilation)
    _,cnts,_ = cv2.findContours(dilation, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
      for c in cnts:
        M = cv2.moments(c)
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        points.append([cX,cY])

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_3",Image)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("usb_cam/image_raw",Image,self.callback)
  def callback(self,data):
    try:
      self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      self.getcorner()
    except CvBridgeError as e:
      print(e)
    cv2.waitKey(3)  
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
