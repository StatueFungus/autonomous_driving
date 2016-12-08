#!/usr/bin/env python
# -*- coding: utf-8 -*-

from imagepreprocessing import ImagePreparator
import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

NODE_NAME = "grayscale_node"
SUB_TOPIC = "image"
PUB_TOPIC = "image_preproc_grayscale"
QUEUE_SIZE = 10

class GrayscaleNode:

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

		gray = self.img_prep.grayscale(cv_image)

		try:
			self.image_pub.publish(self.bridge.cv2_to_imgmsg(gray, "mono8"))
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

