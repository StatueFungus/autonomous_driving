#!/usr/bin/python
# -*- coding: utf-8 -*-
import math
import numpy as np

class LineFilter:
    '''
        Klasse filtert Linien, welche in einem bestimmten Winkel vertikal verlaufen.

    '''

    def __init__(self, image):
        '''
            Konstruktor

        '''
        self.draw_right = False
        self.valid_left_line = False
        self.height, self.width, _ = image.shape

    def _calc_endpoints(self, m, b):
        endpoints = []
        # y = m*x + b --> x = (y - b)/m
        x1 = int((self.image_y_min - b) / m)
        x2 = int((self.image_y_max - b) / m)
        endpoints.append((x1, int(self.image_y_min)))
        endpoints.append((x2, int(self.image_y_max)))
        return endpoints

    def _linear_regression(self, lines):
        lines_x = []
        lines_y = []

        for line in lines:
            for obj in line:
                [x1, y1, x2, y2] = obj
                lines_x.append(x1)
                lines_x.append(x2)
                lines_y.append(y1)
                lines_y.append(y2)

        if len(lines_x) > 0 and len(lines_y) > 0:
            m, b = np.polyfit(lines_x, lines_y, 1) # y = m*x + b
            return (m,b)
        return None

    def filter(self, lines, angle, x_center, y_min, y_max):
        filtered_lines = {'left':[], 'right':[]}
        
        # Falls die mitgegeben Linien leer sind, gebe leeres Dictionary zurück
        if lines is None:
            return filtered_lines 
        if len(lines) == 0:
            return filtered_lines

        self.image_x_center = self.width * x_center
        self.image_y_min = self.height * y_min
        self.image_y_max = self.height * y_max

        right_lines = []
        left_lines = []

        for line in lines:
            right_line_obj=[]
            left_line_obj=[]
            for obj in line:
                [x1, y1, x2, y2] = obj
                # Nur Linien sind von Interesse weĺche in einem bestimmten y-Bereich liegen
                if y1 >= self.image_y_min and y1 <= self.image_y_max and y2 >= self.image_y_min and y2 <= self.image_y_max:
                    dx, dy = x2 - x1, y2 - y1
                    obj_angle = np.arctan2(dy, dx) * 180 / np.pi
                    # Filter Linien mit bestimmten Winkel
                    if math.fabs(obj_angle) > angle:
                        # Filter Linien nach rechter und linker Hälfte
                        if x1 > self.image_x_center and x2 > self.image_x_center:
                            right_line_obj.append(obj)
                        elif x1 < self.image_x_center and x2 < self.image_x_center:
                            left_line_obj.append(obj)

            if len(right_line_obj) != 0:
                right_lines.append(right_line_obj)
            if len(left_line_obj) != 0:
                left_lines.append(left_line_obj)

        # Bilde linke Gerade mithilfe lineare Regression --> y = mx + b
        left_line_valid = False
        left_linear_regression = self._linear_regression(left_lines)
        if left_linear_regression:
            left_line_valid = True
            left_m, left_b = left_linear_regression

        # Bilde rechte Gerade mithilfe lineare Regression --> y = mx + b
        right_line_valid = False
        right_linear_regression = self._linear_regression(right_lines)
        if right_linear_regression:
            right_line_valid = True
            right_m, right_b = right_linear_regression

        # Finde zwei Endpunkte für die linke und rechte Gerade
        endpoints_right_line = []
        endpoints_left_line = []

        if right_line_valid:
            endpoints_right_line = self._calc_endpoints(right_m, right_b)

        if left_line_valid:
            endpoints_left_line = self._calc_endpoints(left_m, left_b)

        filtered_lines['left'] = endpoints_left_line
        filtered_lines['right'] = endpoints_right_line

        return filtered_lines
        
        
                
