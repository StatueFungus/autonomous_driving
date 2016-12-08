#!/usr/bin/env python
# -*- coding: utf-8 -*-

from detectionlib import ImagePreparator
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

NODE_NAME = "image_grayscale"
SUB_TOPIC = "image_ipm"
PUB_TOPIC = "debug_grayscale_image"

class GrayscaleNode:

	def __init__(self, sub_topic, pub_topic):
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber(sub_topic, Image, self.callback)
		self.image_pub = rospy.Publisher(pub_topic, Image, queue_size=10)
		rospy.spin()

	def callback(self, data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			rospy.logerr(e)

		image_prep = ImagePreparator(cv_image)
		image_prep.grayscale()

		try:
			self.image_pub.publish(self.bridge.cv2_to_imgmsg(image_prep.image, "mono8"))
		except CvBridgeError as e:
			rospy.logerr(e)


def main():
	# Initialisiere den Knoten
	rospy.init_node(NODE_NAME, anonymous=True)
	try:
		ld_node = GrayscaleNode(SUB_TOPIC, PUB_TOPIC)
	except KeyboardInterrupt:
		rospy.loginfo("Shutting down node %s", NODE_NAME)

if __name__ == '__main__':
	main()

