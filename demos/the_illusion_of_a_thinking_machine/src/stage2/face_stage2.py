#!/usr/bin/env python

import os
import sys
import argparse

import rospy
from std_msgs.msg import String
import cv2
import cv_bridge

from sensor_msgs.msg import (
    Image,
)

import baxter_interface

from baxter_interface import CHECK_VERSION



def baxter_sleep():
    """
    Send the image located at the specified path to the head
    display on Baxter.

    @param path: path to the image file to load and send
    """
    img = cv2.imread("$YOUT_BAXTER_WORKSPACE/src/my_baxter/BaxterFace/sleep1.png")
    msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
    pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=1)
    pub.publish(msg)
    # Sleep to allow for image to be published.
    rospy.sleep(0.5)
    img = cv2.imread("$YOUT_BAXTER_WORKSPACE/src/my_baxter/BaxterFace/sleep2.png")
    msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
    pub.publish(msg)
    rospy.sleep(0.17)
    img = cv2.imread("$YOUT_BAXTER_WORKSPACE/src/my_baxter/BaxterFace/sleep3.png")
    msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
    pub.publish(msg)
    rospy.sleep(0.17)
    img = cv2.imread("$YOUT_BAXTER_WORKSPACE/src/my_baxter/BaxterFace/sleep4.png")
    msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
    pub.publish(msg)
    rospy.sleep(0.17)

def baxter_s2s():
    img = cv2.imread("$YOUT_BAXTER_WORKSPACE/src/my_baxter/BaxterFace/sleep2smile.png")
    msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
    pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=1)
    pub.publish(msg)
    # Sleep to allow for image to be published.
    rospy.sleep(0.17)

def baxter_smile():
    img = cv2.imread("$YOUT_BAXTER_WORKSPACE/src/my_baxter/BaxterFace/smile1.png")
    msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
    pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=1)
    pub.publish(msg)
    # Sleep to allow for image to be published.
    rospy.sleep(2)
    img = cv2.imread("$YOUT_BAXTER_WORKSPACE/src/my_baxter/BaxterFace/smile2.png")
    msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
    pub.publish(msg)
    rospy.sleep(0.17)
    img = cv2.imread("$YOUT_BAXTER_WORKSPACE/src/my_baxter/BaxterFace/smile3.png")
    msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
    pub.publish(msg)
    rospy.sleep(0.17)

def baxter_lookLeft():
    img = cv2.imread("$YOUT_BAXTER_WORKSPACE/src/my_baxter/BaxterFace/lookLeft.png")
    msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
    pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=1)
    pub.publish(msg)

def baxter_lookRight():
    img = cv2.imread("$YOUT_BAXTER_WORKSPACE/src/my_baxter/BaxterFace/lookRight.png")
    msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
    pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=1)
    pub.publish(msg)

def baxter_talk():
    img = cv2.imread("$YOUT_BAXTER_WORKSPACE/src/my_baxter/BaxterFace/smile1.png")
    msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
    pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=1)
    pub.publish(msg)
    rospy.sleep(0.17)
    img = cv2.imread("$YOUT_BAXTER_WORKSPACE/src/my_baxter/BaxterFace/talk2.png")
    msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
    pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=1)
    pub.publish(msg)
    rospy.sleep(0.17)
    img = cv2.imread("$YOUT_BAXTER_WORKSPACE/src/my_baxter/BaxterFace/talk1.png")
    msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
    pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=1)
    pub.publish(msg)
    rospy.sleep(0.6)
    img = cv2.imread("$YOUT_BAXTER_WORKSPACE/src/my_baxter/BaxterFace/smile1.png")
    msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
    pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=1)
    pub.publish(msg)
    rospy.sleep(0.4)

def baxter_suprise():
    img = cv2.imread("$YOUT_BAXTER_WORKSPACE/src/my_baxter/BaxterFace/surprise1.png")
    msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
    pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=1)
    pub.publish(msg)
    rospy.sleep(1)
    img = cv2.imread("$YOUT_BAXTER_WORKSPACE/src/my_baxter/BaxterFace/surprise2.png")
    msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
    pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=1)
    pub.publish(msg)
    rospy.sleep(0.17)
    img = cv2.imread("$YOUT_BAXTER_WORKSPACE/src/my_baxter/BaxterFace/surprise1.png")
    msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
    pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=1)
    pub.publish(msg)
    rospy.sleep(1)

def callback(msg):
    rospy.loginfo(msg)
    baxter_smile()
    count = 18
    while count > 0:
        baxter_talk()
        count = count - 1
        if count%2 == 0:
            rospy.sleep(0.7)
    baxter_suprise()
    baxter_suprise()
    baxter_suprise()
    baxter_suprise()

def main():
    rospy.init_node('rsdk_xdisplay_stage2_image', anonymous=True)
    # rospy.Subscriber("face_s1_manager",String, callback)
    rospy.Subscriber("stage2_start",String,callback)
    rospy.spin()

if __name__ == '__main__':
    sys.exit(main())
