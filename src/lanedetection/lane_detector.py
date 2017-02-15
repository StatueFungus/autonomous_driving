#!/usr/bin/env python
# -*- coding: utf-8 -*-

WIDTH_TOLERANCE = 0.35  # Tolerierte Abweichung zwischen zwei hintereinander gemessenen Straßenbreiten
MIN_LANE_DISTANCE = 0.90  # Prozentualer Mindestabstand zwischen zwei Straßenmarkierungen


class LaneDetector:
    '''
        Klasse dient dazu die Straßenmarkierungen auf einer Segment Linie zu berechnen.
    '''

    def __init__(self, lane_width):
        '''
            Konstruktor.

            Parameter
            ---------
            lane_width : Integer
                Straßenbreite
        '''
        self.lane_width = lane_width
        self.lane_width_tolerance = round(self.lane_width * WIDTH_TOLERANCE)
        self.min_lane_distance = round(self.lane_width * MIN_LANE_DISTANCE)

    def find_lane_points(self, segment):
        '''
            Methode berechnet die Straßenmarkierung (linker und rechter Punkt) für eine Segment Linie.

            Parameter
            ---------
            segment : Segment Linie

            Rückgabe
            ---------
            Tupel : Straßenmarkierung >> (left_point, right_point)
        '''
        point_distance = segment.point_distance
        segment_center = segment.point_center

        # kalkuriere Scoring Table
        left_points_score = self._calc_point_score(segment.nz_left_points, segment.left_point)
        right_points_score = self._calc_point_score(segment.nz_right_points, segment.right_point)

        # definiere linken und rechten Kandidaten
        left_candidate = self._define_candidate(left_points_score, segment_center, point_distance, left=True)
        right_candidate = self._define_candidate(right_points_score, segment_center, point_distance, left=False)

        # validiere linken und rechten Kandidaten
        left_candidate_valid = self._validate_candidate(left_candidate, segment_center, point_distance)
        right_candidate_valid = self._validate_candidate(right_candidate, segment_center, point_distance)

        # definiere linken und rechten Punkt
        left_point = segment.left_point
        right_point = segment.right_point

        if left_candidate_valid and right_candidate_valid:
            left_point = left_candidate
            right_point = right_candidate
        elif left_candidate_valid and not right_candidate_valid:
            left_point = left_candidate
            right_point = int(left_candidate + point_distance)
        elif not left_candidate_valid and right_candidate_valid:
            left_point = int(right_candidate - point_distance)
            right_point = right_candidate
        # check, ob Straßenmarkierungen einen validen Mindestabstand zueinander haben
        if abs(right_point - left_point) >= self.min_lane_distance:
            return int(left_point), int(right_point)
        return segment.left_point, segment.right_point

    def _define_candidate(self, point_score, segment_center, point_distance, left):
        if point_score:
            return max(point_score, key=point_score.get)
        return None
        #if left:
        #    return segment_center + round(point_distance / 2)
        #else:
        #    return segment_center - round(point_distance / 2)

    def _calc_point_score(self, points, old_point):
        point_score = {}

        for idx, point in enumerate(points):
            point_score[point] = 0
            # ist der Punkt der nächste zur Segmentmitte?
            if idx == 0:
                point_score[point] += 3
            # liegt der Punkt im Bereich seines Vorgängers?
            if old_point is not None and point in range(old_point - 4, old_point + 4):
                point_score[point] += 2
            # besitzt der Punkt einen Nachbarpunkt?
            if len(points) > idx + 1 and points[idx + 1] in range(point-2, point+2):
                point_score[point] += 1
        return point_score

    def _validate_candidate(self, point, segment_center, line_distance):
        '''
            Methode checkt, ob ein Kandidat in einem sinvollen Abstand zur vorherigen Segmentmitte liegt.

            Parameter
            ---------
            point : Integer
                Kandidat als rechter / linker Punkt.
            segment_center : Integer
                Segmentmitte des vorherigen Frames.
            line_distance : Integer
                Straßenbreite des vorherigen Frames.
        '''
        if point is None:
            return False
        distance_seg_center = abs(point - segment_center)
        diff_distance_seg_center = abs(distance_seg_center - round(line_distance / 2))
        if diff_distance_seg_center <= self.lane_width_tolerance:
            return True
        return False