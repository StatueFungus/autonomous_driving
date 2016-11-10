#!/usr/bin/python
from detectionlib import ImagePreparator
from detectionlib import LineFilter
from detectionlib import LaneDetector
from detectionlib import Visualizer
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

NODE_NAME = "lane_detection"
SUB_TOPIC = "image_raw"
PUB_TOPIC = "debug_image"

class LaneDetectionNode:

	def __init__(self, sub_topic, pub_topic):
		self.line_filter = LineFilter()
		self.lane_detector = LaneDetector()
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber(sub_topic, Image, self.callback)
		self.image_pub = rospy.Publisher(pub_topic, Image, queue_size=1)
		rospy.spin()

	def callback(self, data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			rospy.loginfo(e)

		# Aufbereitung des Bilder
		img_prep = ImagePreparator(cv_image.copy())
		img_prep.define_roi(0.5)
		img_prep.grayscale()
		#img_prep.adaptive_hist_equalization()
		img_prep.blur((3,3), 0)
		# Canny oder Threshold benutzen
		img_prep.global_threshold(165, 255)
		#img_prep.canny(1, 250, 3) # (1, 250, 3) oder (50, 150, 3)
		
		# Entdecke Linien
		lines = self.lane_detector.houghlines_p(img_prep.image, 50, 10, 20) # (50, 10, 20) oder (100, 1, 10)
		
		# Filter korrekte Linien
		flines = self.line_filter.filter(lines, 10, 50)
		
		vis = Visualizer(cv_image)
		vis.draw_lines(flines, (0,255,0), 2)
		
		# Publish Bild mit den gezeichneten Linien
		try:
			self.image_pub.publish(self.bridge.cv2_to_imgmsg(vis.image, "bgr8"))
		except CvBridgeError as e:
			rospy.loginfo(e)

def main():
	# Initialisiere den Knoten
	rospy.init_node(NODE_NAME, anonymous=True)
	try:
		ld_node = LaneDetectionNode(SUB_TOPIC, PUB_TOPIC)
	except KeyboardInterrupt:
		rospy.loginfo("Shutting down")

if __name__ == '__main__':
	main()

	
