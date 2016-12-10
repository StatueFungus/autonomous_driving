#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
from imagepreprocessing import Visualizer

class SegmentModel:

	def __init__(self, y_offset, point_distance):
		self.y_offset = y_offset
		self.nz_left_points = None
		self.nz_right_points = None
		self.point_center = None
		self.left_point = None
		self.right_point = None
		self.point_distance = point_distance
		self.vis = Visualizer()

	def draw(self, image):
		height,width,_ = image.shape
		self.vis.draw_line(image, (0, self.y_offset), (width, self.y_offset), (255,0,0), 1)
		if self.left_point:
			self.vis.draw_point(image, (self.left_point, self.y_offset), 3, (0,255,0), 2)
		if self.right_point:
			self.vis.draw_point(image, (self.right_point, self.y_offset), 3, (0,255,0), 2)
		if self.point_center:
			self.vis.draw_point(image, (self.point_center, self.y_offset), 3, (0,0,255), 2)

	def update_non_zero_points(self, image):
		self.nz_left_points, self.nz_right_points = self._calc_non_zero(image)

	def update_point_distance(self):
		if self.left_point and self.right_point:
			new_distance = self.right_point - self.left_point
			if abs(new_distance - self.point_distance) < int(self.point_distance * 0.55): # TODO: 0.55 in consts packen
				self.point_distance = new_distance

	def update_point_center(self):
		if self.left_point and self.right_point:
			self.point_center = self.left_point + int((self.right_point - self.left_point) / 2.0)

	def _calc_non_zero(self, image):
		_,width,_ = image.shape
		x_separator = int(width / 2)
		arr = np.array(np.nonzero(image[self.y_offset])[0])
		nz_lp, nz_rp =  [arr[arr<x_separator], arr[~(arr<x_separator)]]
		nz_lp = nz_lp[::-1] # invertiere liste
		return nz_lp, nz_rp