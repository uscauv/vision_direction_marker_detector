#!/usr/bin/env python

import rospy

from vision_common.msg import Target, Targets
import direction_marker_detector

def on_image()

def direction_marker_node():
    pub = rospy.Publisher('/vision/direction_marker/target', Target)
    rospy.init_node('direction_marker')
    rate = rospy.Rate(33)  # 33Hz
    while not rospy.is_shutdown():
        rects = direction_marker_detector.find()
        target = Target()
        target.x =
        pub.publish