#!/usr/bin/env python
# -*- coding: utf-8 -*-

from lanedetection import LaneModel
from imagepreprocessing import ImagePreparator
from imagepreprocessing import InversePerspectiveMapping
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from std_srvs.srv import Empty, EmptyResponse
from cv_bridge import CvBridge, CvBridgeError
import cv2
import rospy

NODE_NAME = "lane_detection_node"
SUB_TOPIC = "image"
SUB_BASE_THROTTLE_TOPIC = "baseThrottle"
PUB_TOPIC = "debug_image"
PUB_SETPOINT_TOPIC = "setpoint"
PUB_STATE_TOPIC = "state"
PUB_THROTTLE_TOPIC = "lanecontroller/throttle"
RESET_SERVICE = "reset"
QUEUE_SIZE = 1

DEFAULT_LANE_WIDTH = 20
DEFAULT_SEGMENT_START = 6
DEFAULT_SEGMENT_AMOUNT = 1


class LaneDetectionNode:
    def __init__(self, node_name, sub_topic, sub_base_throttle_topic, pub_topic, pub_setpoint_topic, pub_state_topic, pub_throttle_topic, reset_service):
        self.bridge = CvBridge()
        self.img_prep = ImagePreparator()
        self.ipm = InversePerspectiveMapping()

        # Publisher
        self.image_pub = rospy.Publisher(pub_topic, Image, queue_size=QUEUE_SIZE)
        self.setpoint_pub = rospy.Publisher(pub_setpoint_topic, Float64, queue_size=QUEUE_SIZE)
        self.state_pub = rospy.Publisher(pub_state_topic, Float64, queue_size=QUEUE_SIZE)
        self.throttle_pub = rospy.Publisher(pub_throttle_topic, Float64, queue_size=QUEUE_SIZE)

        self.reset_srv = rospy.Service(reset_service, Empty, self.reset_callback)
        self.reset_tracking = False

        rospy.init_node(node_name, anonymous=True)

        self.image_sub = rospy.Subscriber(sub_topic, Image, self.callback)
        self.base_throttle_sub = rospy.Subscriber(sub_base_throttle_topic, Float64, self.callbackBaseThrottle)

        # Base Throttle
        self.base_throttle = rospy.get_param("/autonomous_driving/lane_detection_node/base_throttle", 0.6)

        # Crop Parameters
        self.above_value = rospy.get_param("/autonomous_driving/lane_detection_node/above", 0.58)
        self.below_value = rospy.get_param("/autonomous_driving/lane_detection_node/below", 0.1)
        self.side_value = rospy.get_param("/autonomous_driving/lane_detection_node/side", 0.3)

        # Lane Tracking Parameters
        self.deviation = rospy.get_param("/autonomous_driving/lane_detection_node/deviation", 5)
        self.border = rospy.get_param("/autonomous_driving/lane_detection_node/border", 0)

        # Canny Parameters
        self.threshold_low = rospy.get_param("/autonomous_driving/lane_detection_node/threshold_low", 50)
        self.threshold_high = rospy.get_param("/autonomous_driving/lane_detection_node/threshold_high", 150)
        self.aperture = rospy.get_param("/autonomous_driving/lane_detection_node/aperture", 3)

        # Lane Tracking
        self.init_lanemodel()

        rospy.spin()
    
    def checkCollision(self, rowIndex, colIndex, laneWidth, image):
        for checkIdx in range(colIndex, colIndex - laneWidth, -1):
            if image[rowIndex,checkIdx] == 255:
                return checkIdx
        return -1

    def findNextLanePoint(self, rowIndex, colIndex, leftSpan, rightSpan, image):
        for nextIdxCols in range(colIndex - leftSpan,colIndex + rightSpan, 1):
            h, w = image.shape[:2]
            if nextIdxCols < w and nextIdxCols >= 0:
                if image[rowIndex, nextIdxCols] == 255:
                    return nextIdxCols 
        return colIndex

    def callbackBaseThrottle(self, data):
        self.base_throttle = data.data

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

        # Inverse Perspective Mapping
        self.ipm.initializeTransformationMatrix(cv_image)
        warped = self.ipm.warp(cv_image)

        # crop
        cropped = self.img_prep.crop(warped, self.above_value, self.below_value, self.side_value)

        # grayscale
        gray = self.img_prep.grayscale(cropped)

        # blur
        blurred = self.img_prep.blur(gray, (self.deviation, self.deviation), self.border)

        # canny
        canny = self.img_prep.edge_detection(blurred, self.threshold_low, self.threshold_high, self.aperture)

        # Any Object Detection
        # Preproc 
        cropped2 = self.img_prep.crop(warped, 0.7, 0.1, .388)
        gray2 = self.img_prep.grayscale(cropped2)
        blurred2 = self.img_prep.blur(gray2, (self.deviation, self.deviation), self.border)
        canny2 = self.img_prep.edge_detection(blurred2, self.threshold_low, self.threshold_high, self.aperture)

        # Remove IPM Canny lines
        heigth, width = canny2.shape
        cv2.line(canny2, (0, 0), (45, heigth), (0,0, 0), 3)
        cv2.line(canny2, (width, 0), (width - 44, heigth), (0, 0, 0), 3)
        
        # Dilation
        #kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(2,2))
        #canny2 = cv2.dilate(canny2,kernel,iterations = 1)

        # Finde von Mitte den ersten weißen Punkt rechts
        h, w = canny2.shape[:2]
        idxRows = h - 1 				
        for idxCols in range(w/2,w): 							
            if canny2[idxRows,idxCols] == 255:
                canny2[idxRows,idxCols] = 128
                canny2[idxRows,idxCols - 20] = 128	
                break

        nextIdxCols = idxCols
        laneWidth = 18
        bCollision = False
        for nextIdxRows in range(idxRows - 1,15, -1):
                nextIdxCols = self.findNextLanePoint(nextIdxRows, nextIdxCols, 4, 4, canny2)
                canny2[nextIdxRows,nextIdxCols] = 128
                canny2[nextIdxRows,nextIdxCols - 20] = 128
                if self.checkCollision(nextIdxRows, nextIdxCols, laneWidth, canny2) is not -1:
                    bCollision = True
                    nextIdxCols = self.checkCollision(nextIdxRows, nextIdxCols, laneWidth, canny2)
                    break
        
        canny2 = cv2.cvtColor(canny2, cv2.COLOR_GRAY2BGR)
        cv2.circle(canny2, (w/2, h - 1 ), 1 ,(0,255,0),2)
        if bCollision is True:
            cv2.circle(canny2, (nextIdxCols, nextIdxRows), 1 ,(0,0,255),2)

        # Remove IPM Canny lines
        heigth, width = canny.shape
        if width == 63:  # TODO dirty hack
            cv2.line(canny, (0, 4/2), (18/2, heigth), (0, 0, 0), 2)
            cv2.line(canny, (width, 4/2), (width - 18/2, heigth), (0, 0, 0), 2)
        else:
            cv2.line(canny, (0, 4), (18, heigth), (0, 0, 0), 2)
            cv2.line(canny, (width, 4), (width - 18, heigth), (0, 0, 0), 2)

        # Lane Detection
        canny = cv2.cvtColor(canny, cv2.COLOR_GRAY2BGR)
        self.lane_model.update_segments(canny.copy())
        self.lane_model.draw_segments(canny)
        state_point_x = self.lane_model.state_point_x()

        if self.reset_tracking is True:
            self.init_lanemodel()
            self.reset_tracking = False

        # publish to pid
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(canny2, "bgr8"))
            self.setpoint_pub.publish(0.0)
            if state_point_x:
		heigth, width, _ = canny.shape
                deviation = state_point_x - int(width/2)
                self.state_pub.publish(deviation)
                devThrottle = abs(deviation / 30.0) 
                if devThrottle < 0.1:
                    devThrottle = 0.0
                elif devThrottle > 0.25:
                    devThrottle = 0.25
                self.throttle_pub.publish((1.0 - devThrottle) * self.base_throttle)
        except CvBridgeError as e:
            rospy.logerr(e)

    def init_lanemodel(self):
        lane_width = rospy.get_param("/autonomous_driving/lane_detection_node/lane_width", DEFAULT_LANE_WIDTH)
        segment_start = rospy.get_param("/autonomous_driving/lane_detection_node/segment_start", DEFAULT_SEGMENT_START)
        segment_amount = rospy.get_param("/autonomous_driving/lane_detection_node/segment_amount", DEFAULT_SEGMENT_AMOUNT)
        self.lane_model = LaneModel(lane_width, segment_amount, segment_start)

    def reset_callback(self, req):
        rospy.loginfo("Reset Lanetracking")
        self.reset_tracking = True
        return EmptyResponse()


def main():
    try:
        LaneDetectionNode(NODE_NAME, SUB_TOPIC, SUB_BASE_THROTTLE_TOPIC, PUB_TOPIC, PUB_SETPOINT_TOPIC, PUB_STATE_TOPIC, PUB_THROTTLE_TOPIC, RESET_SERVICE)
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down node %s", NODE_NAME)

if __name__ == '__main__':
    main()
