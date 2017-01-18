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
PUB_TOPIC = "debug_image"
PUB_SETPOINT_TOPIC = "setpoint"
PUB_STATE_TOPIC = "state"
RESET_SERVICE = "reset"

QUEUE_SIZE = 1

# valide default Werte für eine Bildauflösung von 320x240
DEFAULT_LANE_WIDTH = 20
DEFAULT_SEGMENT_START = 210
DEFAULT_SEGMENT_AMOUNT = 1


class LaneDetectionNode:

    def __init__(self, node_name, sub_topic, pub_topic,  pub_setpoint_topic, pub_state_topic, reset_service):
        self.bridge = CvBridge()
        self.img_prep = ImagePreparator()
        self.ipm = InversePerspectiveMapping()

        # Publisher
        self.image_pub = rospy.Publisher(pub_topic, Image, queue_size=QUEUE_SIZE)
        self.image_pub = rospy.Publisher(pub_topic, Image, queue_size=QUEUE_SIZE)
        self.setpoint_pub = rospy.Publisher(pub_setpoint_topic, Float64, queue_size=QUEUE_SIZE)
        self.state_pub = rospy.Publisher(pub_state_topic, Float64, queue_size=QUEUE_SIZE)

        self.reset_srv = rospy.Service(reset_service, Empty, self.reset_callback)
        self.reset_tracking = False

        rospy.init_node(node_name, anonymous=True)

        self.image_sub = rospy.Subscriber(sub_topic, Image, self.callback)

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
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(canny, "bgr8"))
            self.setpoint_pub.publish(0.0)
            if state_point_x:
                heigth, width, _ = canny.shape
                self.state_pub.publish(state_point_x - int(width/2))
        except CvBridgeError as e:
            rospy.logerr(e)

    def init_lanemodel(self):
        lane_width = rospy.get_param("/autonomous_driving/lane_tracking_node/lane_width", DEFAULT_LANE_WIDTH)
        segment_start = rospy.get_param("/autonomous_driving/lane_tracking_node/segment_start", DEFAULT_SEGMENT_START)
        segment_amount = rospy.get_param("/autonomous_driving/lane_tracking_node/segment_amount", DEFAULT_SEGMENT_AMOUNT)
        self.lane_model = LaneModel(lane_width, segment_amount, segment_start)

    def reset_callback(self, req):
        rospy.loginfo("Reset Lanetracking")
        self.reset_tracking = True
        return EmptyResponse()


def main():
    try:
        LaneDetectionNode(NODE_NAME, SUB_TOPIC, PUB_TOPIC, PUB_SETPOINT_TOPIC, PUB_STATE_TOPIC, RESET_SERVICE)
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down node %s", NODE_NAME)

if __name__ == '__main__':
    main()
