#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from cv_bridge import CvBridge, CvBridgeError

NODE_NAME = "object_detection_node"
SUB_TOPIC = "image_raw"
PUB_STEERING_TOPIC = "objectcontroller/steering"
PUB_THROTTLE_TOPIC = "objectcontroller/throttle"
QUEUE_SIZE = 10


class ObjectDetectionNode:
    def __init__(self, sub_topic, pub_steering_topic, pub_throttle_topic):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(sub_topic, Image, self.callback)
        rospy.spin()

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
			
	rospy.loginfo("Implemented in Lane_Detection_Node.py")

def main():
    # Initialisiere den Knoten
    rospy.init_node(NODE_NAME, anonymous=True)
    try:
        ObjectDetectionNode(SUB_TOPIC, PUB_STEERING_TOPIC, PUB_THROTTLE_TOPIC)
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down node %s", NODE_NAME)

if __name__ == '__main__':
    main()
