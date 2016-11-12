#!/usr/bin/python
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

NODE_NAME = "image_resizer"
SUB_TOPIC = "image_raw"
PUB_TOPIC = "image_resized"
DEFAULT_WIDTH = 640
DEFAULT_HEIGHT = 480

class ImageResizerNode:

	def __init__(self, sub_topic, pub_topic, width=DEFAULT_WIDTH, height=DEFAULT_HEIGHT):
		self.width = width
		self.height = height
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber(sub_topic, Image, self.callback)
		self.image_pub = rospy.Publisher(pub_topic, Image, queue_size=10)
		rospy.spin()

	def callback(self, data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			rospy.logerr(e)

		img_heigth = cv_image.shape[0]
		img_width = cv_image.shape[1]

		if img_width!=self.width or img_heigth!=self.height:
			cv_image = cv2.resize(cv_image, (self.width, self.height), 0, 0, 0)

		try:
			self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
		except CvBridgeError as e:
			rospy.logerr(e)

def main():
	# Initialisiere den Knoten
	rospy.init_node(NODE_NAME, anonymous=True)
	try:
		ld_node = ImageResizerNode(SUB_TOPIC, PUB_TOPIC)
	except KeyboardInterrupt:
		rospy.loginfo("Shutting down node %s", NODE_NAME)

if __name__ == '__main__':
	main()