#!/usr/bin/env python
# -*- coding: utf-8 -*-

from imagepreprocessing import ImagePreparator
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

NODE_NAME = "canny_node"
SUB_TOPIC = "image"
PUB_TOPIC = "image_preproc_canny"
QUEUE_SIZE = 1


class CannyNode:

    def __init__(self, sub_topic, pub_topic):
        self.bridge = CvBridge()
        self.img_prep = ImagePreparator()
        self.image_sub = rospy.Subscriber(sub_topic, Image, self.callback)
        self.image_pub = rospy.Publisher(pub_topic, Image, queue_size=QUEUE_SIZE)
        rospy.spin()

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "mono8")
        except CvBridgeError as e:
            rospy.logerr(e)

        threshold_low = rospy.get_param("/autonomous_driving/canny_node/threshold_low", 50)
        threshold_high = rospy.get_param("/autonomous_driving/canny_node/threshold_high", 150)
        aperture = rospy.get_param("/autonomous_driving/canny_node/aperture", 3)
        canny = self.img_prep.edge_detection(cv_image, threshold_low, threshold_high, aperture)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(canny, "mono8"))
        except CvBridgeError as e:
            rospy.logerr(e)


def main():
    # Initialisiere den Knoten
    rospy.init_node(NODE_NAME, anonymous=True)
    try:
        CannyNode(SUB_TOPIC, PUB_TOPIC)
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down node %s", NODE_NAME)

if __name__ == '__main__':
    main()
