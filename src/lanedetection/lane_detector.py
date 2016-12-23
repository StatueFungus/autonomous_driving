#!/usr/bin/env python
# -*- coding: utf-8 -*-

WIDTH_TOLERANCE = 0.25  # Tolerierte Abweichung zwischen zwei hintereinander gemessenen Straßenbreiten


class LaneDetector:

    def __init__(self, lane_width):
        self.lane_width = lane_width
        self.lane_width_tolerance = round(self.lane_width * WIDTH_TOLERANCE)

    def find_lane_points(self, segment):
        left_points = segment.nz_left_points
        right_points = segment.nz_right_points
        line_distance = segment.point_distance
        segment_center = segment.point_center
        left_point = segment.left_point  # Könnte None sein
        right_point = segment.right_point  # Könnte None sein

        left_points_score = {}
        for idx, p in enumerate(left_points):
            left_points_score[p] = 0
            # ist der Punkt der nächste zur Segmentmitte?
            if idx == 0:
                left_points_score[p] += 3
            # besitzt der Punkt einen Nachbarpunkt?
            if len(left_points) > idx + 1 and left_points[idx + 1] in range(p-2, p):
                left_points_score[p] += 1
            # liegt der Punkt im Bereich seines Vorgängers?
            if left_point is not None and p in range(left_point - 4, left_point + 4):
                left_points_score[p] += 2

        right_points_score = {}
        for idx, p in enumerate(right_points):
            right_points_score[p] = 0
            # ist der Punkt der nächste zur Segmentmitte?
            if idx == 0:
                right_points_score[p] += 3
            # besitzt der Punkt einen Nachbarpunkt?
            if len(right_points) > idx + 1 and right_points[idx + 1] in range(p, p+2):
                right_points_score[p] += 1
            # liegt der Punkt im Bereich seines Vorgängers?
            if right_point is not None and p in range(right_point - 4, right_point + 4):
                right_points_score[p] += 2

        if left_points_score:
            left_candidate = max(left_points_score, key=left_points_score.get)
        else:
            left_candidate = segment_center + round(line_distance / 2)
        if right_points_score:
            right_candidate = max(right_points_score, key=right_points_score.get)
        else:
            right_candidate = segment_center - round(line_distance / 2)

        res_left_candidate = self._validate_candidate(left_candidate, segment_center, line_distance)
        res_right_candidate = self._validate_candidate(right_candidate, segment_center, line_distance)

        if res_left_candidate and res_right_candidate:
            return int(left_candidate), int(right_candidate)
        elif res_left_candidate and not res_right_candidate:
            return int(left_candidate), int(left_candidate + line_distance)
        elif not res_left_candidate and right_candidate:
            return int(right_candidate - line_distance), int(right_candidate)
        else:
            return left_point, right_point

    def _validate_candidate(self, new_point, segment_center, line_distance):
        """ Checkt, ob ein Kandidat in einem sinvollen Abstand zur vorherigen Segmentmitte liegt """
        distance_seg_center = abs(new_point - segment_center)
        diff_distance_seg_center = abs(distance_seg_center - round(line_distance / 2))
        if diff_distance_seg_center <= self.lane_width_tolerance:
            return True
        return False