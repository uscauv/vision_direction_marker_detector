#!/usr/bin/env python

import rospy

from vision_common.msg import Target, Targets
import direction_marker_detector


class DirectionMarkerNode():

    def __init__(self):
        self.pub = rospy.Publisher('/vision/direction_marker/target', Target, queue_size=10)
        rospy.init_node('direction_marker')

    def on_image(img):
        rospy.loginfo('got image')
        pass

if __name__ == '__main__':
    node = DirectionMarkerNode()
    rospy.loginfo('created')
    rospy.spin()