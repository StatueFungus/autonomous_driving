#!/usr/bin/env python
# -*- coding: utf-8 -*-
from segment_model import SegmentModel
from lane_detector import LaneDetector

SEGMENT_GAB = 5

class LaneModel:

	def __init__(self, lane_width, segment_count, y_offset_first_segment):
		self.lane_width = lane_width
		self.segment_count = segment_count
		self.y_offset_first_segment = y_offset_first_segment
		self.segments = self._instantiate_segments()
		self.lane_detector = LaneDetector()

	def update_segments(self, image):
		for seg in self.segments:
			seg.update_non_zero_points(image)
			seg.left_point, seg.right_point = self.lane_detector.find_lane_points(seg)
			#seg.update_point_distance() # für geraden funktioniert das besser ohne dieses update
			seg.update_point_center()

	def draw_segments(self, image):
		if self.segments:
			for seg in self.segments:
				seg.draw(image)

	def state_point_x(self):
		# TODO : berücksichtigung mehrerer Segmente
		if self.segments:
			return self.segments[0].point_center
		return None

	def _instantiate_segments(self):
		seg_list = []
		for i in range(self.segment_count):
			y_offset_act_segment = self.y_offset_first_segment - (SEGMENT_GAB * i)
			seg_list.append(SegmentModel(y_offset_act_segment, self.lane_width))
		return seg_list