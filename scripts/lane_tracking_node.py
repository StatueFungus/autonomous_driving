#!/usr/bin/env python
# -*- coding: utf-8 -*-

from lanedetection import LaneModel
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from cv_bridge import CvBridge, CvBridgeError

NODE_NAME = "lane_tracking"
SUB_TOPIC = "image"
PUB_TOPIC = "debug_image"
PUB_SETPOINT_TOPIC = "setpoint"
PUB_STATE_TOPIC = "state"		

class LaneTrackingNode:

	def __init__(self, sub_topic, pub_topic, pub_setpoint_topic, pub_state_topic):
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber(sub_topic, Image, self.callback)
		self.image_pub = rospy.Publisher(pub_topic, Image, queue_size=10)
		self.setpoint_pub = rospy.Publisher(pub_setpoint_topic, Float64, queue_size=10)
		self.state_pub = rospy.Publisher(pub_state_topic, Float64, queue_size=10)
		self.lane_model = LaneModel(40, 1, 425)
		rospy.spin()

	def callback(self, data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
			heigth, width, _ = cv_image.shape
		except CvBridgeError as e:
			rospy.logerr(e)
		
		self.lane_model.update_segments(cv_image.copy())
		self.lane_model.draw_segments(cv_image)
		state_point_x = self.lane_model.state_point_x()
		#state_point_x = None

		try:
			self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
			self.setpoint_pub.publish(0.0)
			if state_point_x:
				self.state_pub.publish(state_point_x - int(width/2))
		except CvBridgeError as e:
			rospy.logerr(e)

def main():
	# Initialisiere den Knoten
	rospy.init_node(NODE_NAME, anonymous=True)
	try:
		ld_node = LaneTrackingNode(SUB_TOPIC, PUB_TOPIC, PUB_SETPOINT_TOPIC, PUB_STATE_TOPIC)
	except KeyboardInterrupt:
		rospy.loginfo("Shutting down node %s", NODE_NAME)

if __name__ == '__main__':
	main()

	
