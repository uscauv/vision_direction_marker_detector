#!/usr/bin/env python

import rospy

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from vision_common.msg import Target, Targets
import direction_marker_detector


class DirectionMarkerNode():
    def __init__(self):
        self.targets_pub = rospy.Publisher('/vision/direction_marker/targets', Targets, queue_size=10)
        self.target_pub = rospy.Publisher('/vision/direction_marker/target', Target, queue_size=10)
        self.bin_pub = rospy.Publisher('/vision/direction_marker/img/bin', Image, queue_size=10)
        self.result_img_pub = rospy.Publisher('/vision/direction_marker/img/result', Image, queue_size=10)
        rospy.init_node('direction_marker')

        self.bridge = CvBridge()

        rospy.Subscriber('/camera/bottom', Image, self.on_image)

        self.bridge = CvBridge()

    def on_image(self, img_msg):
        rospy.loginfo('got image')
        img = self.bridge.imgmsg_to_cv2(img_msg, 'bgr8')

        intermediary_imgs = {}
        results = direction_marker_detector.find(img, output_images=intermediary_imgs)

        self.bin_pub.publish(self.bridge.cv2_to_imgmsg(intermediary_imgs['bin'], "mono8"))
        self.result_img_pub.publish(self.bridge.cv2_to_imgmsg(intermediary_imgs['result'], "bgr8"))

        if len(results) > 0:
            targets_msg = Targets()
            for rect in results:
                target = Target()
                target.header.stamp = rospy.Time.now()
                target.header.frame_id = 'direction_marker'
                target.x = rect[0][0]
                target.y = rect[0][1]
                targets_msg.targets.append(target)

            targets_msg.header.stamp = rospy.Time.now()
            self.target_pub.publish(targets_msg.targets[0])
            self.targets_pub.publish(targets_msg)


if __name__ == '__main__':
    node = DirectionMarkerNode()
    rospy.loginfo('created')
    rospy.spin()