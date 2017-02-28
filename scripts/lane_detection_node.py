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
PUB_OBJ_TOPIC = "debug_obj_image"
PUB_SETPOINT_TOPIC = "setpoint"
PUB_STATE_TOPIC = "state"
PUB_THROTTLE_TOPIC = "lanecontroller/throttle"
RESET_SERVICE = "reset"
QUEUE_SIZE = 1

DEFAULT_LANE_WIDTH = 20
DEFAULT_SEGMENT_START = 6
DEFAULT_SEGMENT_AMOUNT = 1


class LaneDetectionNode:
    def __init__(self, node_name, sub_topic, sub_base_throttle_topic, pub_topic, pub_obj_topic, pub_setpoint_topic, pub_state_topic, pub_throttle_topic, reset_service):
        self.bridge = CvBridge()
        self.img_prep = ImagePreparator()
        self.ipm = InversePerspectiveMapping()

        # Publisher
        self.image_pub = rospy.Publisher(pub_topic, Image, queue_size=QUEUE_SIZE)
        self.image_obj_pub = rospy.Publisher(pub_obj_topic, Image, queue_size=QUEUE_SIZE)
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
        self.above_value = rospy.get_param("/autonomous_driving/lane_detection_node/above", 0.7)
        self.below_value = rospy.get_param("/autonomous_driving/lane_detection_node/below", 0.1)
        self.side_value = rospy.get_param("/autonomous_driving/lane_detection_node/side", 0.388)

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

        # Bild Vorverarbeiten
        cropped = self.img_prep.crop(warped, self.above_value, self.below_value, self.side_value)
        gray = self.img_prep.grayscale(cropped)
        blurred = self.img_prep.blur(gray, (self.deviation, self.deviation), self.border)
        canny = self.img_prep.edge_detection(blurred, self.threshold_low, self.threshold_high, self.aperture)

        # IPM Canny Linien entfernen (hack)
        heigth, width = canny.shape
        cv2.line(canny, (0, 0), (45, heigth), (0,0, 0), 3)
        cv2.line(canny, (width, 0), (width - 44, heigth), (0, 0, 0), 3)

        # Lane Detection
        cannyBGR = cv2.cvtColor(canny, cv2.COLOR_GRAY2BGR)
        self.lane_model.update_segments(cannyBGR.copy())
        self.lane_model.draw_segments(cannyBGR)
        state_point_x = self.lane_model.state_point_x()

        # Throttle und Steering berechnen
        if state_point_x:
            # Deviation
            deviation = state_point_x - int(width/2)
            # Slow down 
            normedDeviation = abs(deviation / 30.0) 
            devThrottle = normedDeviation
            if normedDeviation < 0.1:
                devThrottle = 0.0
            if normedDeviation > 0.25:
                devThrottle = 0.25
            adjustedThrottle = (1.0 - devThrottle) * self.base_throttle


        # Any-Object-Detection 

        # Finde vom Mittelpunkt den ersten weißen Punkt rechts
        idxRows = heigth - 1 				
        for idxCols in range(width/2,width): 							
            if canny[idxRows,idxCols] == 255:
                canny[idxRows,idxCols] = 128
                canny[idxRows,idxCols - 20] = 128	
                break

        # Finde die weiteren Punkte von rechter Spur
        if normedDeviation < 0.2:
            numberOfRowsToIgnore = 1
        else: 
            numberOfRowsToIgnore = 25
        nextIdxCols = idxCols
        laneWidth = 17
        bCollision = False
        # Bildzeilen durchgehen
        for nextIdxRows in range(idxRows - 1, numberOfRowsToIgnore, -1):
                # Nächsten Punkt finden
                nextIdxCols = self.findNextLanePoint(nextIdxRows, nextIdxCols, 4, 4, canny)
                # Debug prints
                canny[nextIdxRows,nextIdxCols] = 128
                canny[nextIdxRows,nextIdxCols - 20] = 128
                # Prüfe Bildzeile (Straße) auf Objekte  
                collisionCol = self.checkCollision(nextIdxRows, nextIdxCols, laneWidth, canny)
                if collisionCol is not -1:
                    nextIdxCols = collisionCol
                    bCollision = True
                    break
        
        # Debug prints
        cannyObjBGR = cv2.cvtColor(canny, cv2.COLOR_GRAY2BGR)
        cv2.circle(cannyObjBGR, (width/2, heigth - 1 ), 1 ,(0,255,0),2)
        if bCollision is True:
            cannyObjBGR[nextIdxRows,nextIdxCols] = (0,0,255)

        # Objekt erkannt?
        if bCollision is True:
            adjustedThrottle = -1.0

        # Reset?
        if self.reset_tracking is True:
            self.init_lanemodel()
            self.reset_tracking = False
         
        # publish 
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cannyBGR, "bgr8"))
            self.image_obj_pub.publish(self.bridge.cv2_to_imgmsg(cannyObjBGR, "bgr8"))
            self.setpoint_pub.publish(0.0)
           
            if state_point_x:
                # Publish state
                self.state_pub.publish(deviation)
                self.throttle_pub.publish(adjustedThrottle)
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
        LaneDetectionNode(NODE_NAME, SUB_TOPIC, SUB_BASE_THROTTLE_TOPIC, PUB_TOPIC, PUB_OBJ_TOPIC, PUB_SETPOINT_TOPIC, PUB_STATE_TOPIC, PUB_THROTTLE_TOPIC, RESET_SERVICE)
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down node %s", NODE_NAME)

if __name__ == '__main__':
    main()
