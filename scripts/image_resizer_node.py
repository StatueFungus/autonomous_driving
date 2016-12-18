#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

NODE_NAME = "image_resizer_node"
SUB_TOPIC = "image"
PUB_TOPIC = "image_preproc_resized"
QUEUE_SIZE = 1
DEFAULT_HEIGHT = 480
DEFAULT_WIDTH = 640


class ImageReziserNode:

    def __init__(self, node_name, sub_topic, pub_topic):
        self.bridge = CvBridge()
        
        self.image_pub = rospy.Publisher(pub_topic, Image, queue_size=QUEUE_SIZE)

        rospy.init_node(node_name, anonymous=True)

        self.image_sub = rospy.Subscriber(sub_topic, Image, self.callback)

        rospy.spin()

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

        height = rospy.get_param("/autonomous_driving/image_resizer_node/height", DEFAULT_HEIGHT)
        width = rospy.get_param("/autonomous_driving/image_resizer_node/width", DEFAULT_WIDTH)
        cv_image = cv2.resize(cv_image, (width, height), 0, 0, 0)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            rospy.logerr(e)


def main():
    try:
        ImageReziserNode(NODE_NAME, SUB_TOPIC, PUB_TOPIC)
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down node %s", NODE_NAME)

if __name__ == '__main__':
    main()
