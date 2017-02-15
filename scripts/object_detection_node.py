#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from cv_bridge import CvBridge, CvBridgeError

NODE_NAME = "object_detection_node"
SUB_TOPIC = "image"
PUB_STEERING_TOPIC = "objectcontroller/steering"
PUB_THROTTLE_TOPIC = "objectcontroller/throttle"
QUEUE_SIZE = 10

DEFAULT_BASE_THROTTLE = 0.5
DEFAULT_HEIGHT_SCALE_FACTOR = 0.5
DEFAULT_B_CALC_STEERING = False
DEBUG_FLAG = False
waitValue = 1


class ObjectDetectionNode:
    def __init__(self, sub_topic, pub_steering_topic, pub_throttle_topic):
        self.bridge = CvBridge()
        self.base_throttle = rospy.get_param("/autonomous_driving/object_detection_node/base_throttle", DEFAULT_BASE_THROTTLE)
        self.b_calc_steering = rospy.get_param("/autonomous_driving/object_detection_node/b_calc_steering", DEFAULT_B_CALC_STEERING)
        self.debug_flag = rospy.get_param("/autonomous_driving/object_detection_node/debug_flag", DEBUG_FLAG)
        self.image_sub = rospy.Subscriber(sub_topic, Image, self.callback)
        self.steering_pub = rospy.Publisher(pub_steering_topic, Float64, queue_size=1)
        self.throttle_pub = rospy.Publisher(pub_throttle_topic, Float64, queue_size=1)
        self.roadmask = None
        rospy.spin()

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
			
			
	def correct_gamma(image, gamma=1.0):
		# build a lookup table mapping the pixel values [0, 255] to
		# their adjusted gamma values
		invGamma = 1.0 / gamma
		table = np.array([((i / 255.0) ** invGamma) * 255
			for i in np.arange(0, 256)]).astype("uint8")
 
		# apply gamma correction using the lookup table
		return cv2.LUT(image, table)
	
	def cleanContours(contours):
		for idx in xrange(len(contours)-1, -1, -1):
			area = cv2.contourArea(contours[idx])
			if area < 12: #or area > 1000:
				del contours[idx]
		return contours

	videoHeight, videoWidth , _ = cv_image.shape
	
	copyFrame = cv_image[(videoHeight*DEFAULT_HEIGHT_SCALE_FACTOR) : videoHeight, videoWidth*0.0: videoWidth*1.0] # Crop from x, y, w, h -> 100, 200, 300, 400
	videoHeight, videoWidth , _ = copyFrame.shape

	## blur the image to remove noise
	copyFrame = cv2.GaussianBlur(copyFrame, (5,5),3)

	## change the color system from BGR to HSV
	hsv = cv2.cvtColor(copyFrame, cv2.COLOR_BGR2HSV)
	
	## range of yellow color in HSV
	lower_color = np.array([38,175,115]) # 35#200 #100
	upper_color = np.array([70,240,240])
	
	## Threshold the HSV image to get only yellow colors
	hsvMask = cv2.inRange(hsv, lower_color, upper_color)
	
	## threshold the gamma corrected (makes the image darker) image to get bright spots of the yellow object
	gamma2 = correct_gamma(copyFrame,0.32)
	hsvMask2 = cv2.inRange(cv2.cvtColor(gamma2, cv2.COLOR_BGR2HSV), lower_color, upper_color) # 0.32
	## threshold the gamma corrected (makes the image less darker then the previous one) image to get darker spots of the yellow object
	gamma3 = correct_gamma(copyFrame,0.50)#35#50
	hsvMask3 = cv2.inRange(cv2.cvtColor(gamma3, cv2.COLOR_BGR2HSV), lower_color, upper_color)
	 
	## add all hsv masks toghter
	theMask = hsvMask + hsvMask2 
	theMask = theMask + hsvMask3
	
	## find contours in the thresholded mask image
	contours, hierarchy = cv2.findContours(theMask.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
	
	## remove all small contours
	contours = cleanContours(contours)

	## get the south point of the contour
	minDistance = 99999;
	centerX = 0
	if contours is not None:
		for cnt in contours:
			## get the moments and draw the contour
			mom = cv2.moments(cnt)
			y = int(mom['m01']/mom['m00'] + (np.sqrt(cv2.contourArea(cnt)/np.pi)))
				
			## calculate distance
			distance = videoHeight - y
			if minDistance > distance:
				minDistance = distance
				centerX = int(mom['m10']/mom['m00'])
	
	## publish steering and throttle based on distance
	if minDistance < videoHeight:
                self.bLastFrameObjectDetected = True
		if self.b_calc_steering is True:
			## Calculate Steering
			deviation = (1.0 - ((centerX) / float(videoWidth))) + 0.5
			if deviation > 0.7:
				deviation = 0.7
			## Always steer to left
			self.steering_pub.publish(-deviation)
                        ## Get Throttle
			self.throttle_pub.publish(self.base_throttle * 0.9)
		else:
			## hard stop
			self.throttle_pub.publish(-1.0)
	

def main():
    # Initialisiere den Knoten
    rospy.init_node(NODE_NAME, anonymous=True)
    try:
        ObjectDetectionNode(SUB_TOPIC, PUB_STEERING_TOPIC, PUB_THROTTLE_TOPIC)
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down node %s", NODE_NAME)

if __name__ == '__main__':
    main()
