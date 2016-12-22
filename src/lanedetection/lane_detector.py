#!/usr/bin/env python
# -*- coding: utf-8 -*-


class LaneDetector:

    def __init__(self, lane_width, lane_width_tolerance):
        self.lane_width = lane_width
        self.lane_width_tolerance = lane_width_tolerance

    def find_lane_points(self, segment):
        left_points = segment.nz_left_points
        right_points = segment.nz_right_points
        line_distance = segment.point_distance
        segment_center = segment.point_center
        left_point = segment.left_point  # Könnte None sein
        right_point = segment.right_point  # Könnte None sein

        if len(left_points) > 0:
            left_candidate = left_points[0]
        else:
            left_candidate = segment_center + round(line_distance / 2)
        if len(right_points) > 0:
            right_candidate = right_points[0]
        else:
            right_candidate = segment_center - round(line_distance / 2)

        res_left_candidate = self._validate_candidate(left_candidate, segment_center, line_distance)
        res_right_candidate = self._validate_candidate(right_candidate, segment_center, line_distance)

        if res_left_candidate and res_right_candidate:
            return left_candidate, right_candidate
        elif res_left_candidate and not res_right_candidate:
            return left_candidate, left_candidate + line_distance
        elif not res_left_candidate and right_candidate:
            return right_candidate - line_distance, right_candidate
        else:
            return left_point, right_point

    def _validate_candidate(self, new_point, segment_center, line_distance):
        """ Checkt, ob ein Kandidat in einem sinvollen Abstand zur vorherigen Segmentmitte liegt """
        distance_seg_center = abs(new_point - segment_center)
        diff_distance_seg_center = abs(distance_seg_center - round(line_distance / 2))
        if diff_distance_seg_center <= self.lane_width_tolerance:
            return True
        return False   