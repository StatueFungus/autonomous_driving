#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from cv_bridge import CvBridge, CvBridgeError
from imagepreprocessing import ImagePreparator
import random

NODE_NAME = "object_detection_node"
SUB_TOPIC = "image"
PUB_STEERING_TOPIC = "objectcontroller/steering"
PUB_THROTTLE_TOPIC = "objectcontroller/throttle"
QUEUE_SIZE = 10

DEFAULT_BASE_THROTTLE = 0.5
DEFAULT_HEIGHT_SCALE_FACTOR = 0.62
DEFAULT_B_CALC_STEERING = False
SEGMENTATION_FREQUENCY = 3
DEBUG_FLAG = False
waitValue = 1
arrayContainer = []

pointColor = (0,0,0)

## cv2.namedWindow("original")
## cv2.moveWindow("original", 650,350)
## cv2.namedWindow("regionOfInterest")

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
		self.img_prep = ImagePreparator()
		self.base_throttle = rospy.get_param("/autonomous_driving/object_detection_node/base_throttle", DEFAULT_BASE_THROTTLE)
		self.b_calc_steering = rospy.get_param("/autonomous_driving/object_detection_node/b_calc_steering", DEFAULT_B_CALC_STEERING)
		self.debug_flag = rospy.get_param("/autonomous_driving/object_detection_node/debug_flag", DEBUG_FLAG)
		self.image_sub = rospy.Subscriber(sub_topic, Image, self.callback)
		self.steering_pub = rospy.Publisher(pub_steering_topic, Float64, queue_size=1)
		self.throttle_pub = rospy.Publisher(pub_throttle_topic, Float64, queue_size=1)
		self.roadmask = None
		self.bLastFrameObjectDetected = False
		self.breakFlag = False
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
		#cv_image = cv2.resize(cv_image.copy(),None,fx=0.5, fy=0.5, interpolation = cv2.INTER_CUBIC)
	
		#videoHeight, videoWidth , _ = resizedImage.shape
		videoHeight, videoWidth , _ = cv_image.shape
		
			##now = rospy.get_rostime()
	
		## create a blank Image for the Mask
		#copyFrame = np.zeros(resizedImage.shape, np.uint8)
		#copyFrame = np.zeros(cv_image.shape, np.uint8)
		## img[y: y + h, x: x + w]
		## crop the Image and store it to the blank image
		#copyFrame[(videoHeight/2) : videoHeight, 0: videoWidth] = resizedImage[(videoHeight/2) : videoHeight, 0: videoWidth] # Crop from x, y, w, h -> 100, 200, 300, 400
		copyFrame = cv_image[(videoHeight*DEFAULT_HEIGHT_SCALE_FACTOR) : videoHeight*0.85, videoWidth*0.1: videoWidth*0.9] # Crop from x, y, w, h -> 100, 200, 300, 400
		videoHeight, videoWidth , _ = copyFrame.shape
		
			#if self.roadmask is None:
			## Region of Interest by creating mask
			#self.roadmask = np.ones(copyFrame.shape, dtype=np.uint8) * 255
			#vh, vw, _ = copyFrame.shape
			#roi_corners = np.array([[(0,0), (vw*0.3,0), (vw*0.1,vh), (0,vh)]], dtype=np.int32)
			#cv2.fillPoly(self.roadmask, roi_corners, (0,0,0))
			#roi_corners = np.array([[(vw,0), (vw*0.7,0),  (vw*0.9,vh), (vw,vh)]], dtype=np.int32)
			#cv2.fillPoly(self.roadmask, roi_corners, (0,0,0))
		#else:
			## apply the mask
			#copyFrame = cv2.bitwise_and(copyFrame, self.roadmask)
			#cv2.imshow("regionOfInterest", copyFrame)
		
		# grayscale
		cv2.imshow("orgi", copyFrame)
		gray = self.img_prep.grayscale(copyFrame)
	
			# blur
		blurred = self.img_prep.blur(gray, (5, 5), 0)
	
			# canny
		canny = self.img_prep.edge_detection(blurred, 50, 150, 3)
		canny2 = cv2.cvtColor(canny, cv2.COLOR_GRAY2BGR)
		#blank = np.zeros(canny2.shape, np.uint8)
		#cv2.circle(canny2 , (videoWidth/2,videoHeight/2), 1 ,(0,255,0),2)
		idxRows = videoHeight -1
		decrement = 10
		#arrayContainer[0].append([1,2,3,4])
		#arrayContainer.append([])
		while idxRows > 0 :
		#	print(str(idxRows) + " von " + str(videoHeight))
			for idxCols in range(0,videoWidth):
				#print canny[idxRows,idxCols]
				if canny[idxRows,idxCols] == 255:
					#print("weises pixel bei [" + str(idxRows) + ", " + str(idxCols) + "] gefunden")
					#cv2.circle(canny2 , (idxCols,idxRows), 1 ,(0,0,255),2)
					#koordsStr = str(idxCols) + ", " + str(idxRows)
					#cv2.putText(blank,koordsStr,(idxCols,idxRows), cv2.FONT_HERSHEY_PLAIN, 1,(255,255,255)) #,2,cv2.LINE_AA)
					#print("ContainerGroesse " + str(len(arrayContainer)))
					if len(arrayContainer) is not 0:
						
						for y in range(0, len(arrayContainer)): # geht die linien durch
							
							for x in range(0, len(arrayContainer[y])): # geht die koords in den linien durch
								# vergleiche die X werte und die Y werte um falsche werte durch schlangenfoermige linien zu bekommen
								# und auch falsche werte durch kurven zu bekommen
								if abs(arrayContainer[y][x][1] - idxCols) < 30 and abs(arrayContainer[y][x][0] - idxRows) < 30:
									arrayContainer[y].append([idxRows,idxCols])
									self.breakFlag = True
									break
							if self.breakFlag is True:
								self.breakFlag = False
								break
							
							if y is (len(arrayContainer)-1):
								arrayContainer.append([])
								for i in range(0,len(arrayContainer)):
									if len(arrayContainer[i]) is 0:
										arrayContainer[i].append([idxRows, idxCols])
							#Hier muss noch eine neue linie erstellt werden wenn der weisse punkt zu keiner anderen linie hinzuegefuegt werden kann
					else:
						print("Erster Pkt save")
						arrayContainer.append([])
						arrayContainer[0].append([idxRows,idxCols])
					
			idxRows = idxRows - decrement
			if idxRows < videoHeight/1.5 and decrement < 30:
				decrement = decrement + SEGMENTATION_FREQUENCY
		
		
		for indexLinien in range(0,len(arrayContainer)):
			r = random.randint(0,255)
			g = random.randint(0,255)
			b = random.randint(0,255)
			pointColor = (b,g,r)
			print("Length: " + str(len(arrayContainer)))
			print("Der Median: " + str(arrayContainer[indexLinien][int(len(arrayContainer[indexLinien])/2+0.5)][1]))
			for indexKoords in range(0,len(arrayContainer[indexLinien])):
				print("Linie " + str(indexLinien) + ": X: " + str(arrayContainer[indexLinien][indexKoords][1]) + " Y: " + str(arrayContainer[indexLinien][indexKoords][0]))
				cv2.circle(canny2 , (arrayContainer[indexLinien][indexKoords][1],arrayContainer[indexLinien][indexKoords][0]), 1 ,pointColor,2)
		print("---------------------------------------- \n ")
		del arrayContainer[:]
		
		cv2.imshow("Canny", canny2)
		#cv2.imshow("Blank", blank)
		## blur the image to remove noise
		##Knoten
		## Bilateral Filter: Heavy Performance hit!
		#copyFrame = cv2.bilateralFilter(copyFrame, 5, 90,40)
		copyFrame = cv2.GaussianBlur(copyFrame, (5,5),3)
	
		## change the color system from BGR to HSV
		hsv = cv2.cvtColor(copyFrame, cv2.COLOR_BGR2HSV)
		
		## range of yellow color in HSV
		lower_color = np.array([38,175,115]) # 35#200 #100
		upper_color = np.array([70,240,240])
		
		## Threshold the HSV image to get only yellow colors
		hsvMask = cv2.inRange(hsv, lower_color, upper_color)
		## close small holes an remove noise
		
		#equ = cv2.equalizeHist(copyFrame)
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
		#cv2.drawContours(resizedImage, contours, -1, (0,0,255),1)
		## remove all small contours
		contours = cleanContours(contours)
		
		## draw all contours
		#cv2.drawContours(cv_image, contours, -1, (0,255,0), 3)
	
		## get the south point of the contour
		minDistance = 99999;
		centerX = 0
		if contours is not None:
			for cnt in contours:
				## get the moments and draw the contour
				mom = cv2.moments(cnt)
				#cv2.circle(resizedImage , (int(mom['m10']/mom['m00']) , int(mom['m01']/mom['m00'])),int(np.sqrt(cv2.contourArea(cnt)/np.pi)+0.5),(0,0,255),2)
				#cv2.circle(resizedImage, (int(mom['m10']/mom['m00']) , int(mom['m01']/mom['m00'] + (np.sqrt(cv2.contourArea(cnt)/np.pi)))), 1, (20,0,255),2)
				#x = int(mom['m10']/mom['m00'])
				y = int(mom['m01']/mom['m00'] + (np.sqrt(cv2.contourArea(cnt)/np.pi)))
				#cv2.circle(cv_image, (int(x), int(videoHeight/DEFAULT_HEIGHT_SCALE_FACTOR + y)), 1, (20,0,255),2)
				
				## calculate the distance from the car to the object in pixels
				distance = videoHeight - y
				if self.debug_flag:
					print("Distance: " + str(distance) + " pixel")
				if minDistance > distance:
					minDistance = distance
					centerX = int(mom['m10']/mom['m00'])
		
		##cv2.imshow("original", resizedImage)
		#cv2.imshow("original", cv_image)
		if self.debug_flag:
			## cv2.imshow("croppedFrameBlurred", copyFrame)
			## cv2.imshow("theMask", theMask)
			## cv2.imshow("hsvMask", hsvMask)
			## cv2.imshow("hsvMask2", hsvMask2)
			## cv2.imshow("hsvMask3", hsvMask3)
			key = cv2.waitKey(waitValue)
		#cv2.imshow("gamma3",gamma3)
		#cv2.imshow("gamma2",gamma2)
	
		#if minDistance < videoHeight/10.0:
			# hard stop
			#self.throttle_pub.publish(-1.0)
		#elif minDistance < videoHeight:
			# maximum throttle is 0.5
			#self.throttle_pub.publish(float(minDistance/(videoHeight*2.0)))
	
		## publish steering and throttle based on distance
		if minDistance < videoHeight:
			self.bLastFrameObjectDetected = True
			if self.b_calc_steering is True:
				## Calculate Steering
				deviation = (1.0 - ((centerX) / float(videoWidth))) + 0.5
				if deviation > 1.0:
					deviation = 1.0
				## Always steer to left
				self.steering_pub.publish(-deviation)
				## Get Throttle
				self.throttle_pub.publish(self.base_throttle * 0.9)
			else:
				## hard stop
				self.throttle_pub.publish(-1.0)
				self.throttle_pub.publish(0.0)
				self.throttle_pub.publish(-1.0)
		else:
			if self.bLastFrameObjectDetected is True:
				self.steering_pub.publish(0.0)
				self.bLastFrameObjectDetected = False
				self.throttle_pub.publish(self.base_throttle)
		
		#end = rospy.get_rostime()
		#rospy.loginfo("Milliseconds    %s", str((end - now)/1000000.0))
	
			
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
