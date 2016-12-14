#!/usr/bin/env python
# -*- coding: utf-8 -*-


class LaneDetector:

    def find_lane_points(self, segment):
        left_points = segment.nz_left_points
        right_points = segment.nz_right_points
        left_point = segment.left_point
        right_point = segment.right_point
        line_distance = segment.point_distance

        left_points_score = {}
        for idx, p in enumerate(left_points):
            left_points_score[p] = 0
            if len(left_points) > idx + 1 and left_points[idx + 1] in range(p - 3, p):
                print "%s: Moegliche linke Spur" % p
                left_points_score[p] += 1
            if any(rp in right_points for rp in range(p + line_distance - 2, p + line_distance + 3)):
                print "%s: Hat rechte Gegenspur" % p
                left_points_score[p] += 5
            if left_point is not None and p in range(left_point - 4, left_point + 5):
                print "%s: Liegt in der Naehe vom vorherigen Punkt" % p
                left_points_score[p] += 3

        right_points_score = {}
        for idx, p in enumerate(right_points):
            right_points_score[p] = 0
            if len(right_points) > idx + 1 and right_points[idx + 1] in range(p, p + 5):
                print "%s: Moegliche rechte Spur" % p
                right_points_score[p] += 1
            if any(lp in left_points for lp in range(p - line_distance - 3, p - line_distance + 2)):
                print "%s: Hat linke Gegenspur" % p
                right_points_score[p] += 5
            if right_point is not None and p in range(right_point - 4, right_point + 5):
                print "%s: Liegt in der Naehe vom vorherigen Punkt" % p
                right_points_score[p] += 3

        print left_points_score, right_points_score

        if left_points != []:
            left_candidate = max(left_points_score, key=left_points_score.get)
        else:
            left_candidate = left_point
        if right_points != []:
            right_candidate = max(right_points_score, key=right_points_score.get)
        else:
            right_candidate = right_point

        if left_candidate is not None and right_candidate is not None and line_distance is not None:
            if abs(abs(left_candidate - right_candidate) - line_distance) >= int(line_distance * 0.54):
                if left_points_score[left_candidate] == right_points_score[right_candidate]:
                    return left_candidate, right_candidate

                if left_points_score[left_candidate] > right_points_score[right_candidate]:
                    return left_candidate, left_candidate + line_distance
                else:
                    return right_candidate - line_distance, right_candidate

        return left_candidate, right_candidate
