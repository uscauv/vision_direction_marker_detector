#!/usr/bin/env python

import os

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

if __name__ == '__main__':
    pub = rospy.Publisher('/camera/bottom', Image, queue_size=10)
    rospy.init_node('direction_marker_sample_pub')

    cv_images = []
    for f in os.listdir("samples/"):
        if f.endswith(".jpg"):
            cv_images.append(cv2.imread('samples/' + f))

    bridge = CvBridge()

    rate = rospy.Rate(30)  # 30hz

    image_index = 0
    while not rospy.is_shutdown():
        pub.publish(bridge.cv2_to_imgmsg(cv_images[image_index], "bgr8"))
        rospy.loginfo('published')
        image_index += 1
        if image_index == len(cv_images):
            image_index = 0
        rate.sleep()