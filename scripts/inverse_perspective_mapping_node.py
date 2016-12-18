#!/usr/bin/env python
# -*- coding: utf-8 -*-

from imagepreprocessing import Camera
from imagepreprocessing import ImagePreparator
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import rospy
import numpy as np

NODE_NAME = "inverse_perspective_mapping_node"
SUB_TOPIC = "image"
PUB_TOPIC = "image_preproc_ipm"
DEFAULT_RESOLUTION = (640, 480)
QUEUE_SIZE = 1


class InversePerspectiveMappingNode:

    def __init__(self, node_name, sub_topic, pub_topic):
        self.camera = Camera(h=20, aperture=140)
        self.img_prep = ImagePreparator()
        self.bridge = CvBridge()
        self.horizon_y = self.camera.get_horizon_y() + 25
        self.image_resolution = DEFAULT_RESOLUTION

        self.image_pub = rospy.Publisher(pub_topic, Image, queue_size=QUEUE_SIZE)

        rospy.init_node(node_name, anonymous=True)

        self.image_sub = rospy.Subscriber(sub_topic, Image, self.callback)

        rospy.spin()

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

        self.image_resolution = len(cv_image[0]), len(cv_image)

        p1_w, p2_w, p3_w, p4_w = self._calculate_world_coordinates()

        rect = np.array([
            [0, self.horizon_y],
            [self.image_resolution[0] - 1, self.horizon_y],
            [self.image_resolution[0] - 1, self.image_resolution[1] - 1],
            [0, self.image_resolution[1] - 1]
        ], dtype="float32")

        p1_new, p2_new, p3_new, p4_new = self._calculate_destination_points(
            p1_w, p2_w, p3_w, p4_w)

        dst = np.array([
            [p1_new[0], p1_new[1]],
            [p2_new[0], p2_new[1]],
            [p3_new[0], p3_new[1]],
            [p4_new[0], p4_new[1]]
        ], dtype="float32")

        warped = self.img_prep.warp_perspective(cv_image, rect, dst, (int(p2_new[0]), self.image_resolution[1] - 1))

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(warped, "bgr8"))
        except CvBridgeError as e:
            rospy.logerr(e)

    def _calculate_world_coordinates(self):
        p1 = self.camera.image_to_world_coordinates(self.horizon_y, 0)
        p2 = self.camera.image_to_world_coordinates(self.horizon_y, self.image_resolution[0] - 1)
        p3 = self.camera.image_to_world_coordinates(self.image_resolution[1] - 1, self.image_resolution[0] - 1)
        p4 = self.camera.image_to_world_coordinates(self.image_resolution[1] - 1, 0)

        return p1, p2, p3, p4

    def _calculate_destination_points(self, p1_w, p2_w, p3_w, p4_w):
        max_y = max(p1_w[0], p2_w[0], p3_w[0], p4_w[0])
        y_factor = self.image_resolution[1] / float(max_y)

        min_x = min(p1_w[1], p2_w[1], p3_w[1], p4_w[1])

        p1_new = ((p1_w[1] - min_x) * y_factor, self.image_resolution[1] - 1 - p1_w[0] * y_factor)
        p2_new = ((p2_w[1] - min_x) * y_factor, self.image_resolution[1] - 1 - p2_w[0] * y_factor)
        p3_new = ((p3_w[1] - min_x) * y_factor, self.image_resolution[1] - 1 - p3_w[0] * y_factor)
        p4_new = ((p4_w[1] - min_x) * y_factor, self.image_resolution[1] - 1 - p4_w[0] * y_factor)

        return p1_new, p2_new, p3_new, p4_new


def main():
    try:
        InversePerspectiveMappingNode(NODE_NAME, SUB_TOPIC, PUB_TOPIC)
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down node %s", NODE_NAME)

if __name__ == '__main__':
    main()
