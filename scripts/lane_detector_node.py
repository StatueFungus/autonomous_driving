#!/usr/bin/env python
# -*- coding: utf-8 -*-

from detectionlib import ImagePreparator
from detectionlib import LineFilter
from detectionlib import LaneDetector
from detectionlib import Visualizer
from cameralib import Camera
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from cv_bridge import CvBridge, CvBridgeError

NODE_NAME = "lane_tracking"
SUB_TOPIC = "image"
PUB_TOPIC = "debug_image"
PUB_SETPOINT_TOPIC = "setpoint"
PUB_STATE_TOPIC = "state"

class LaneModel:
	def __init__(self, intersection_points_count, intersection_point_distance):
		
		self.intersection_points_count = intersection_points_count
		self.intersection_point_distance = intersection_point_distance
		self.intersection_points = []
		self._initialize_intersection_points()

	def _initialize_intersection_points(self):
		for i in range(intersection_points_count):
			self.intersection_points[i] = LanePoint()

class LanePoint:
	def __init__(self, y_offset):
		self.initialized = False
		self.y_offset = y_offset			

class LaneDetectorNode:

	def __init__(self, sub_topic, pub_topic, pub_setpoint_topic, pub_state_topic):
		self.line_filter = LineFilter()
		self.lane_detector = LaneDetector()
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber(sub_topic, Image, self.callback)
		self.image_pub = rospy.Publisher(pub_topic, Image, queue_size=10)
		self.setpoint_pub = rospy.Publisher(pub_setpoint_topic, Float64, queue_size=10)
		self.state_pub = rospy.Publisher(pub_state_topic, Float64, queue_size=10)

		self.left_point = None
		self.right_point = None
		self.line_distance = 40
		rospy.spin()

	def callback(self, data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			rospy.logerr(e)

		print "---------------------"

		vis = Visualizer(cv_image)
		
		#schwarze linien auf dem canny um aeussere raender zu entfernen (muss auf jeden fall dynamisch sein)
		#cv2.line(img_prep2.image, (32 + int((p1[1] - min_x) * x_factor), 479 - int(p1[0] * y_factor)), (int((p4[1] - min_x) * x_factor), 479 - int(p4[0] * y_factor)), (0,0,0), 2)
		#cv2.line(img_prep2.image, (-33 + int((p2[1] - min_x) * x_factor), 479 - int(p2[0] * y_factor)), (int((p3[1] - min_x) * x_factor), 479 - int(p3[0] * y_factor)), (0,0,0), 2)

		state_point_x = None

		for i in range(1):
			z = 10 * i
			arr = np.array(np.nonzero(cv_image[425 - z])[0])
			left_points, right_points =  [arr[arr<320], arr[~(arr<320)]]
			left_points = left_points[::-1] # invertiere liste 
			
			print left_points, right_points	
			self.left_point, self.right_point = self._find_lane_points(left_points,right_points)
			print self.left_point, self.right_point
			
			if self.left_point != None and self.right_point != None:
				new_line_distance = self.right_point - self.left_point
				if abs(new_line_distance - self.line_distance) < int(self.line_distance * 0.55):
					self.line_distance = new_line_distance
					print self.line_distance
				vis.draw_line((0, 425 - z), (639, 425 - z), (255,0,0), 1) # test linie auf canny
				cv2.circle(cv_image, (self.left_point, 425 - z), 3, (0,255,0), 2) # linker test punkt
				cv2.circle(cv_image, (self.right_point, 425 - z), 3, (0,255,0), 2) # rechter test punkt
				state_point_x = self.left_point + int((self.right_point - self.left_point) / 2.0)
				cv2.circle(cv_image, (state_point_x, 425 - z), 3, (0,0,255), 2) # test mitte
				cv2.line(cv_image, (319,0), (319,479), (255,0,0), 1) # test blickrichtung
			
			vis.draw_line((0, 425 - z), (639, 425 - z), (255,0,0), 1) # test linie auf canny

		try:
			self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
			self.setpoint_pub.publish(0.0)
			if state_point_x != None:
				self.state_pub.publish(state_point_x - 320)
		except CvBridgeError as e:
			rospy.logerr(e)

	def _find_lane_points(self, left_points, right_points):

		left_points_score = {}
		for idx, p in enumerate(left_points):
			left_points_score[p] = 0
			if len(left_points) > idx + 1 and left_points[idx + 1] in range(p - 3, p):
				print  "%s: Moegliche linke Spur" % p
				left_points_score[p] += 1
			if any(right_point in right_points for right_point in range(p + self.line_distance - 2, p + self.line_distance + 3)):
				print "%s: Hat rechte Gegenspur" % p
				left_points_score[p] += 5
			if self.left_point != None and p in range(self.left_point - 4, self.left_point + 5):
				print "%s: Liegt in der Naehe vom vorherigen Punkt" % p
				left_points_score[p] += 3

		print "#####"

		right_points_score = {}
		for idx, p in enumerate(right_points):
			right_points_score[p] = 0
			if len(right_points) > idx + 1 and right_points[idx + 1] in range(p, p + 5):
				print  "%s: Moegliche rechte Spur" % p
				right_points_score[p] += 1
			if any(left_point in left_points for left_point in range(p - self.line_distance - 3, p - self.line_distance + 2)):
				print "%s: Hat linke Gegenspur" % p
				right_points_score[p] += 5
			if self.right_point != None and p in range(self.right_point - 4, self.right_point + 5):
				print "%s: Liegt in der Naehe vom vorherigen Punkt" % p
				right_points_score[p] += 3

		print left_points_score, right_points_score

		if left_points != []:
			left_candidate = max(left_points_score, key=left_points_score.get)
		else:
			left_candidate = self.left_point
		if right_points != []:
			right_candidate = max(right_points_score, key=right_points_score.get)
		else:
			right_candidate = self.right_point

		if left_candidate != None and right_candidate != None and self.line_distance != None:
			if abs(abs(left_candidate - right_candidate) - self.line_distance) >= int(self.line_distance * 0.54):	
				if left_points_score[left_candidate] == right_points_score[right_candidate]:
					return left_candidate, right_candidate

				if left_points_score[left_candidate] > right_points_score[right_candidate]:
					return left_candidate, left_candidate + self.line_distance
				else:
					return right_candidate - self.line_distance, right_candidate

		return left_candidate, right_candidate




def main():
	# Initialisiere den Knoten
	rospy.init_node(NODE_NAME, anonymous=True)
	try:
		ld_node = LaneDetectorNode(SUB_TOPIC, PUB_TOPIC, PUB_SETPOINT_TOPIC, PUB_STATE_TOPIC)
	except KeyboardInterrupt:
		rospy.loginfo("Shutting down node %s", NODE_NAME)

if __name__ == '__main__':
	main()

	
