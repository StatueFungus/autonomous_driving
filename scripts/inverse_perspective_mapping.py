#!/usr/bin/env python
# -*- coding: utf-8 -*-

from cameralib import Camera
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import rospy
import cv2
import numpy as np

NODE_NAME = "inverse_perspective_mapping_node"
SUB_TOPIC = "image"
PUB_TOPIC = "image_preproc_ipm"


class InversePerspectiveMappingNode:

	def __init__(self, sub_topic, pub_topic):

		self.camera = Camera(h=20, aperture=90)
		self.bridge = CvBridge()
		self.horizon_y = self._calculate_horizon()

		self.image_sub = rospy.Subscriber(sub_topic, Image, self.callback)
		self.image_pub = rospy.Publisher(pub_topic, Image, queue_size=10)

		rospy.spin()
	
	def callback(self, data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			rospy.logerr(e)

		p1_w, p2_w, p3_w, p4_w = self._calculate_world_coordinates()
		

		rect= np.array([
		        [0, self.horizon_y],
		        [639, self.horizon_y],
		        [639, 479],
		        [0, 479]
		], dtype = "float32")

		p1_new, p2_new, p3_new, p4_new = self._calculate_destination_points(p1_w, p2_w, p3_w, p4_w)

		dst = np.array([
		        [p1_new[0], p1_new[1]],
		        [p2_new[0], p2_new[1]],
		        [p3_new[0], p3_new[1]],
		        [p4_new[0], p4_new[1]]
		], dtype = "float32")

		M = cv2.getPerspectiveTransform(rect, dst)
		warped = cv2.warpPerspective(cv_image, M, (640, 480))

		try:
			self.image_pub.publish(self.bridge.cv2_to_imgmsg(warped, "bgr8"))
		except CvBridgeError as e:
			rospy.logerr(e)

	def _calculate_horizon(self):
		return rospy.get_param("/horizon_y", 265) # horizont muss abhaengig vom neigungswinkel von der kamera berechnet werden

	def _calculate_world_coordinates(self):
		p1 = self.camera.image_to_world_coordinates(self.horizon_y, 0)
		p2 = self.camera.image_to_world_coordinates(self.horizon_y, 639)
		p3 = self.camera.image_to_world_coordinates(479, 639)
		p4 = self.camera.image_to_world_coordinates(479, 0)

		return p1, p2, p3, p4

	def _calculate_destination_points(self, p1_w, p2_w, p3_w, p4_w):
		max_y = max(p1_w[0], p2_w[0], p3_w[0], p4_w[0])
		y_factor = 480.0 / max_y

		min_x = min(p1_w[1], p2_w[1], p3_w[1], p4_w[1])
		max_x = max(p1_w[1], p2_w[1], p3_w[1], p4_w[1])

		range_x = max_x - min_x
		x_factor = 640.0 / range_x

		p1_new = ((p1_w[1] - min_x) * x_factor, 479 - p1_w[0] * y_factor)
		p2_new = ((p2_w[1] - min_x) * x_factor, 479 - p2_w[0] * y_factor)
		p3_new = ((p3_w[1] - min_x) * x_factor, 479 - p3_w[0] * y_factor)
		p4_new = ((p4_w[1] - min_x) * x_factor, 479 - p4_w[0] * y_factor)

		return p1_new, p2_new, p3_new, p4_new

def main():

	rospy.init_node(NODE_NAME, anonymous=True)
	try: 
		ipm_node = InversePerspectiveMappingNode(SUB_TOPIC, PUB_TOPIC)
	except KeyboardInterrupt:
		rospy.loginfo("Shutting down node %s", NODE_NAME)

if __name__ == '__main__':
	main()