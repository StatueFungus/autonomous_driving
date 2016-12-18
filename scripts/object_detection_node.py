#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from matplotlib import pyplot as plt

NODE_NAME = "object_detection_node"
SUB_TOPIC = "image"
PUB_TOPIC = ""
QUEUE_SIZE = 10

waitValue = 2

cv2.namedWindow("bild")
cv2.moveWindow("bild", 10,50)
## cv2.namedWindow("mask")
## cv2.moveWindow("mask",600,50)
## cv2.namedWindow("equ")
## cv2.moveWindow("equ",600,380)


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
			
	def drawMatches(img1, kp1, img2, kp2, matches):
    
		## My own implementation of cv2.drawMatches as OpenCV 2.4.9
		## does not have this function available but it's supported in
		## OpenCV 3.0.0

		## This function takes in two images with their associated 
		## keypoints, as well as a list of DMatch data structure (matches) 
		## that contains which keypoints matched in which images.

		## An image will be produced where a montage is shown with
		## the first image followed by the second image beside it.

		## Keypoints are delineated with circles, while lines are connected
		## between matching keypoints.

		## img1,img2 - Grayscale images
		## kp1,kp2 - Detected list of keypoints through any of the OpenCV keypoint 
				  ## detection algorithms
		## matches - A list of matches of corresponding keypoints through any
				  ## OpenCV keypoint matching algorithm

		# Create a new output image that concatenates the two images together
		# (a.k.a) a montage
		rows1 = img1.shape[0]
		cols1 = img1.shape[1]
		rows2 = img2.shape[0]
		cols2 = img2.shape[1]

		out = np.zeros((max([rows1,rows2]),cols1+cols2,3), dtype='uint8')

		# Place the first image to the left
		out[:rows1,:cols1] = np.dstack([img1, img1, img1])

		# Place the next image to the right of it
		out[:rows2,cols1:] = np.dstack([img2, img2, img2])

		# For each pair of points we have between both images
		# draw circles, then connect a line between them
		for mat in matches:

			# Get the matching keypoints for each of the images
			img1_idx = mat.queryIdx
			img2_idx = mat.trainIdx

			# x - columns
			# y - rows
			(x1,y1) = kp1[img1_idx].pt
			(x2,y2) = kp2[img2_idx].pt

			# Draw a small circle at both co-ordinates
			# radius 4
			# colour blue
			# thickness = 1
			cv2.circle(out, (int(x1),int(y1)), 4, (255, 0, 0), 1)   
			cv2.circle(out, (int(x2)+cols1,int(y2)), 4, (255, 0, 0), 1)

			# Draw a line in between the two points
			# thickness = 1
			# colour blue
			cv2.line(out, (int(x1),int(y1)), (int(x2)+cols1,int(y2)), (255, 0, 0), 1)


		# Show the image
		#cv2.imshow('Matched Features', out)
		#cv2.waitKey(0)
		#cv2.destroyWindow('Matched Features')

		# Also return the image if you'd like a copy
		return out
			
    ## def correct_gamma(image, gamma=1.0):
        ## # build a lookup table mapping the pixel values [0, 255] to
        ## # their adjusted gamma values
        ## invGamma = 1.0 / gamma
        ## table = np.array([((i / 255.0) ** invGamma) * 255
                ## for i in np.arange(0, 256)]).astype("uint8")
 
        ## # apply gamma correction using the lookup table
        ## return cv2.LUT(image, table)

    
    
    
	#img = cv2.imread('/home/walou/mpseRosWorkspace/src/autonomous_driving/scripts/ball.jpeg',0)
	train = cv2.imread('/home/walou/mpseRosWorkspace/src/autonomous_driving/scripts/ball_train.jpeg',0)
	#train2 = cv2.imread('/home/walou/mpseRosWorkspace/src/autonomous_driving/scripts/ball_train2.jpeg',0)
	# Initiate ORB detector
	img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
	orb = cv2.ORB(8000,1.2,8,31,0,3,0,31)
	
	#cv2.imshow("ball", img)
	# find the keypoints with ORB
	kp1, des1 = orb.detectAndCompute(img,None)
	kp2, des2 = orb.detectAndCompute(train,None)
	#kp3, des3 = orb.detectAndCompute(train2, None)
	
	bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
	
	matches = bf.match(des1,des2)
	#matches = matches + bf.match(des1,des2)
	
	
	matches = sorted(matches, key= lambda x:x.distance)
	
	res = drawMatches(img, kp1, train, kp2, matches)
	
	#plt.imshow(res),plt.show()
	
	cv2.imshow("bild", res)
	
	
	# compute the descriptors with ORB
	#kp, des = orb.compute(img, kp)
	# draw only keypoints location,not size and orientation
	#img2 = cv2.drawKeypoints(img, kp, None, color=(0,255,0), flags=0)
	#plt.imshow(img2), plt.show()
	
	#cv2.imshow("original", img)
	#cv2.imshow('bild',img)
	#cv2.waitKey(0)
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
