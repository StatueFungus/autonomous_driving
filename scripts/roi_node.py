#!/usr/bin/env python
# -*- coding: utf-8 -*-

from imagepreprocessing import ImagePreparator
import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

NODE_NAME = "roi_node"
SUB_TOPIC = "image"
PUB_TOPIC = "image_preproc_roi"
QUEUE_SIZE = 10

class RoiNode:

	def __init__(self, sub_topic, pub_topic):
		self.bridge = CvBridge()
		self.img_prep = ImagePreparator() 
		self.image_sub = rospy.Subscriber(sub_topic, Image, self.callback)
		self.image_pub = rospy.Publisher(pub_topic, Image, queue_size=QUEUE_SIZE)
		rospy.spin()

	def callback(self, data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			rospy.logerr(e)

		above_value = rospy.get_param("/autonomous_driving/roi_node/above", 0)
		below_value = rospy.get_param("/autonomous_driving/roi_node/below", 0)
		roi = self.img_prep.define_roi(cv_image, above_value, below_value)

		try:
			self.image_pub.publish(self.bridge.cv2_to_imgmsg(roi, "bgr8"))
		except CvBridgeError as e:
			rospy.logerr(e)


def main():
	# Initialisiere den Knoten
	rospy.init_node(NODE_NAME, anonymous=True)
	try:
		ld_node = RoiNode(SUB_TOPIC, PUB_TOPIC)
	except KeyboardInterrupt:
		rospy.loginfo("Shutting down node %s", NODE_NAME)

if __name__ == '__main__':
	main()

