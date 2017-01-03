#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

NODE_NAME = "object_detection_node"
SUB_TOPIC = "image"
PUB_TOPIC = ""
QUEUE_SIZE = 10

waitValue = 1

cv2.namedWindow("original")
cv2.moveWindow("original", 10,50)

#cv2.namedWindow("canny")
#cv2.moveWindow("canny",600,50)
#cv2.namedWindow("equ")
#cv2.moveWindow("equ",600,380)


class ObjectDetectionNode:
    def __init__(self, sub_topic, pub_topic):
        self.bridge = CvBridge()

        self.image_sub = rospy.Subscriber(sub_topic, Image, self.callback)
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
	resizedImage = cv2.resize(cv_image.copy(),None,fx=0.5, fy=0.5, interpolation = cv2.INTER_CUBIC)
	
	videoHeight, videoWidth , _ = resizedImage.shape
	
	## create a blank Image for the Mask
	copyFrame = np.zeros(resizedImage.shape, np.uint8)
	## img[y: y + h, x: x + w]
	## crop the Image and store it to the blank image
	copyFrame[(videoHeight/2) : videoHeight, 0: videoWidth] = resizedImage[(videoHeight/2) : videoHeight, 0: videoWidth] # Crop from x, y, w, h -> 100, 200, 300, 400
	
	## blur the image to remove noise
	copyFrame = cv2.bilateralFilter(copyFrame, 5, 90,40)
	copyFrame = cv2.GaussianBlur(copyFrame, (5,5),3)

	## change the color system from BGR to HSV
	hsv = cv2.cvtColor(copyFrame, cv2.COLOR_BGR2HSV)
	
	## range of yellow color in HSV
	lower_color = np.array([23,85,100]) # 15 geht mehr ins orange.. 20 ist noch gelblich
	upper_color = np.array([45,255,255])
	
	## Threshold the HSV image to get only yellow colors
	hsvMask = cv2.inRange(hsv, lower_color, upper_color)
	## close small holes an remove noise
	hsvMask = cv2.morphologyEx(hsvMask, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(2,2)))
	
	#equ = cv2.equalizeHist(copyFrame)
	## threshold the gamma corrected (makes the image darker) image to get bright spots of the yellow object
	hsvMask2 = cv2.inRange(cv2.cvtColor(correct_gamma(copyFrame,0.32), cv2.COLOR_BGR2HSV), lower_color, upper_color)
	## threshold the gamma corrected (makes the image brighter) image to get dark spots of the yellow object
	hsvMask3 = cv2.inRange(cv2.cvtColor(correct_gamma(copyFrame,1.95), cv2.COLOR_BGR2HSV), lower_color, upper_color)
	
	## add all hsv masks toghter
	theMask = hsvMask + hsvMask2 
	theMask = theMask + hsvMask3

	## find contours in the thresholded mask image
	contours, hierarchy = cv2.findContours(theMask.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
	## remove all small contours
	contours = cleanContours(contours)
	
	## draw all contours
	#cv2.drawContours(resizedImage, contours, -1, (0,255,0), 3)
	
	## get the south point of the contour
	if contours is not None:
		for cnt in contours:
			## get the moments and draw the contour
			mom = cv2.moments(cnt)
			#cv2.circle(resizedImage , (int(mom['m10']/mom['m00']) , int(mom['m01']/mom['m00'])),int(np.sqrt(cv2.contourArea(cnt)/np.pi)+0.5),(0,0,255),2)
			cv2.circle(resizedImage, (int(mom['m10']/mom['m00']) , int(mom['m01']/mom['m00'] + (np.sqrt(cv2.contourArea(cnt)/np.pi)))), 1, (20,255,20),2)
			
			## calculate the distance from the car to the object in pixels
			print("Distance: " + str(videoHeight - int(mom['m01']/mom['m00'] + (np.sqrt(cv2.contourArea(cnt)/np.pi)))) + " pixel")
	
	
	
	cv2.imshow("original", resizedImage)
	cv2.imshow("theMask", theMask)

	
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
        ObjectDetectionNode(SUB_TOPIC, PUB_TOPIC)
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down node %s", NODE_NAME)

if __name__ == '__main__':
    main()
