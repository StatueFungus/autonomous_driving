#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from cv_bridge import CvBridge, CvBridgeError

NODE_NAME = "object_detection_node"
SUB_TOPIC = "image"
PUB_STEERING_TOPIC = "obj_steering"
PUB_THROTTLE_TOPIC = "obj_throttle"
QUEUE_SIZE = 10
# valid default value only for resolution of 320x240
DEFAULT_BREAKING_DISTANCE = 100

waitValue = 1

## cv2.namedWindow("original")
## cv2.moveWindow("original", 650,350)

## cv2.namedWindow("theMask")
## cv2.moveWindow("theMask",390,350)
## cv2.namedWindow("hsvMask")
## cv2.moveWindow("hsvMask",390,50)
## cv2.namedWindow("hsvMask2")
## cv2.moveWindow("hsvMask2",720,50)
## cv2.namedWindow("hsvMask3")
## cv2.moveWindow("hsvMask3",1050,50)
## cv2.namedWindow("gamma2")
## cv2.moveWindow("gamma2",1,50)
## cv2.namedWindow("gamma3")
## cv2.moveWindow("gamma3",1,350)

class ObjectDetectionNode:
    def __init__(self, sub_topic, pub_steering_topic, pub_throttle_topic):
        self.bridge = CvBridge()
        self.breaking_distance = rospy.get_param("/object_detection_node/breaking_distance", DEFAULT_BREAKING_DISTANCE)
        self.image_sub = rospy.Subscriber(sub_topic, Image, self.callback)
        self.steering_pub = rospy.Publisher(pub_steering_topic, Float64, queue_size=1)
        self.throttle_pub = rospy.Publisher(pub_throttle_topic, Float64, queue_size=1)
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
			#print(area)
			#print("______________________________")
			if area < 12: #or area > 1000:
				del contours[idx]
		return contours
	
	
	## resize the Image from 640*480 to 320*240
	#resizedImage = cv2.resize(cv_image.copy(),None,fx=0.5, fy=0.5, interpolation = cv2.INTER_CUBIC)
	
	#videoHeight, videoWidth , _ = resizedImage.shape
	videoHeight, videoWidth , _ = cv_image.shape
	
	## create a blank Image for the Mask
	#copyFrame = np.zeros(resizedImage.shape, np.uint8)
	copyFrame = np.zeros(cv_image.shape, np.uint8)
	## img[y: y + h, x: x + w]
	## crop the Image and store it to the blank image
	#copyFrame[(videoHeight/2) : videoHeight, 0: videoWidth] = resizedImage[(videoHeight/2) : videoHeight, 0: videoWidth] # Crop from x, y, w, h -> 100, 200, 300, 400
	copyFrame[(videoHeight/2) : videoHeight, 0: videoWidth] = cv_image[(videoHeight/2) : videoHeight, 0: videoWidth] # Crop from x, y, w, h -> 100, 200, 300, 400
	## blur the image to remove noise
	##Knoten
	copyFrame = cv2.bilateralFilter(copyFrame, 5, 90,40)
	copyFrame = cv2.GaussianBlur(copyFrame, (5,5),3)

	## change the color system from BGR to HSV
	hsv = cv2.cvtColor(copyFrame, cv2.COLOR_BGR2HSV)
	
	## range of yellow color in HSV
	lower_color = np.array([38,175,105]) # 35#200 #100
	upper_color = np.array([70,240,240])
	
	## Threshold the HSV image to get only yellow colors
	hsvMask = cv2.inRange(hsv, lower_color, upper_color)
	## close small holes an remove noise

	
	#equ = cv2.equalizeHist(copyFrame)
	## threshold the gamma corrected (makes the image darker) image to get bright spots of the yellow object
	gamma2 = correct_gamma(copyFrame,0.20)
	hsvMask2 = cv2.inRange(cv2.cvtColor(gamma2, cv2.COLOR_BGR2HSV), lower_color, upper_color) # 0.32
	## threshold the gamma corrected (makes the image less darker then the previous one) image to get darker spots of the yellow object
	gamma3 = correct_gamma(copyFrame,0.50)#35#50
	hsvMask3 = cv2.inRange(cv2.cvtColor(gamma3, cv2.COLOR_BGR2HSV), lower_color, upper_color)
	
	## add all hsv masks toghter
	theMask = hsvMask + hsvMask2 
	theMask = theMask + hsvMask3
	
	## find contours in the thresholded mask image
	contours, hierarchy = cv2.findContours(theMask.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
	
	#cv2.drawContours(resizedImage, contours, -1, (0,0,255),1)
	## remove all small contours
	contours = cleanContours(contours)
	
	## draw all contours
	#cv2.drawContours(resizedImage, contours, -1, (0,255,0), 3)

	## get the south point of the contour
	minDistance = 99999;
	centerX = 0
	if contours is not None:
		for cnt in contours:
			## get the moments and draw the contour
			mom = cv2.moments(cnt)
			#cv2.circle(resizedImage , (int(mom['m10']/mom['m00']) , int(mom['m01']/mom['m00'])),int(np.sqrt(cv2.contourArea(cnt)/np.pi)+0.5),(0,0,255),2)
			#cv2.circle(resizedImage, (int(mom['m10']/mom['m00']) , int(mom['m01']/mom['m00'] + (np.sqrt(cv2.contourArea(cnt)/np.pi)))), 1, (20,0,255),2)
			cv2.circle(cv_image, (int(mom['m10']/mom['m00']) , int(mom['m01']/mom['m00'] + (np.sqrt(cv2.contourArea(cnt)/np.pi)))), 1, (20,0,255),2)
			
			## calculate the distance from the car to the object in pixels
			distance = videoHeight - int(mom['m01']/mom['m00'] + (np.sqrt(cv2.contourArea(cnt)/np.pi)))
			#print("Distance: " + str(distance) + " pixel")
			if minDistance > distance:
				minDistance = distance
				centerX = int(mom['m10']/mom['m00'])
	
	#cv2.imshow("original", resizedImage)
	## cv2.imshow("original", cv_image)
	## cv2.imshow("theMask", theMask)
	## cv2.imshow("hsvMask", hsvMask)
	## cv2.imshow("hsvMask2", hsvMask2)
	## cv2.imshow("hsvMask3", hsvMask3)
	#cv2.imshow("gamma3",gamma3)
	#cv2.imshow("gamma2",gamma2)

	# publish steering and throttle based on distance
	if minDistance < self.breaking_distance/10.0:
		# hard stop
		self.throttle_pub.publish(0.0)
		self.steering_pub.publish(0.0)
	elif minDistance < self.breaking_distance:
		# maximum throttle is 0.5
		self.throttle_pub.publish(float(minDistance/(self.breaking_distance*2.0)))
		halfVideoWidth = videoWidth * 0.5
		deviation = (centerX - halfVideoWidth) / halfVideoWidth
		if deviation > 0.0:
			self.steering_pub.publish(1.0 - deviation)
		else:
			self.steering_pub.publish(-(1.0 + deviation))
	

	key = cv2.waitKey(waitValue)
        
	## if key & 0xFF == ord('p'):
		## if(waitValue == 0):
			## waitValue = 10
		## else:
			## waitValue = 0
	## if key & 0xFF == ord('q'):
		## print("q pressed")
	

def main():
    # Initialisiere den Knoten
    rospy.init_node(NODE_NAME, anonymous=True)
    try:
        ObjectDetectionNode(SUB_TOPIC, PUB_STEERING_TOPIC, PUB_THROTTLE_TOPIC)
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down node %s", NODE_NAME)

if __name__ == '__main__':
    main()
