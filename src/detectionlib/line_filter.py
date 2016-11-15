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
        self.draw_left = False
        self.height, self.width, _ = image.shape
        self.image_x_center = self.width / 2

    def filter_by_angle(self, lines, filter_angle, bla):
        filtered_lines = {'left':[], 'right':[]}
        
        # Falls die mitgegeben Linien leer sind, gebe leeres Dictionary zurück
        if lines is None:
            return filtered_lines 
        if len(lines) == 0:
            return filtered_lines

        right_lines = []
        left_lines = []

        for line in lines:
            right_line_obj=[]
            left_line_obj=[]
            for obj in line:
                [x1, y1, x2, y2] = obj
                dx, dy = x2 - x1, y2 - y1
                angle = np.arctan2(dy, dx) * 180 / np.pi
                if math.fabs(angle) > filter_angle:
                    # Filter Linien nach rechter und linker Bildhälfte
                    if angle > 0 and x1 > self.image_x_center and x2 > self.image_x_center:
                        right_line_obj.append(obj)
                    elif angle < 0 and x1 < self.image_x_center and x2 < self.image_x_center:
                        left_line_obj.append(obj)

            if len(right_line_obj) != 0:
                right_lines.append(right_line_obj)
            if len(left_line_obj) != 0:
                left_lines.append(left_line_obj)

        ###### LEFT ######
        # linear regression left side
        left_lines_x = []
        left_lines_y = []

        for lline in left_lines:
            for lobj in lline:
                [x1, y1, x2, y2] = obj
                left_lines_x.append(x1)
                left_lines_x.append(x2)
                left_lines_y.append(y1)
                left_lines_y.append(y2)

        if len(left_lines_x) > 0:
            self.draw_left = True
            left_m, left_b = np.polyfit(left_lines_x, left_lines_y, 1) # y = m*x + b
            if left_m > 0:
                self.draw_left = False
        else:
            self.draw_left = False
        #### END LEFT ####

         ###### RIGHT ######
        # linear regression right side
        right_lines_x = []
        right_lines_y = []

        for rline in right_lines:
            for robj in rline:
                [x1, y1, x2, y2] = obj
                right_lines_x.append(x1)
                right_lines_x.append(x2)
                right_lines_y.append(y1)
                right_lines_y.append(y2)

        if len(right_lines_x) > 0:
            self.draw_right = True
            right_m, right_b = np.polyfit(right_lines_x, right_lines_y, 1) # y = m*x + b
            if right_m < 0:
                self.draw_right = False
        else:
            self.draw_right = False
        #### END RIGHT ####


        # Find 2 end points for right and left lines, used for drawing the line
        # y = m*x + b --> x = (y - b)/m
        res_right_line = []
        res_left_line = []

        y1 = self.height
        y2 = int(self.height*bla)

        if self.draw_right:
            right_x1 = int((y1 - right_b) / right_m)
            right_x2 = int((y2 - right_b) / right_m)

        if self.draw_left:
            left_x1 = int((y1 - left_b) / left_m)
            left_x2 = int((y2 - left_b) / left_m)

        if self.draw_right and (math.fabs(right_x1) > 1000000 or math.fabs(right_x2) > 1000000):
            self.draw_right = False
        if self.draw_left and (math.fabs(left_x1) > 1000000 or math.fabs(left_x2) > 1000000):
            self.draw_left = False

        if self.draw_right:
            res_right_line.append((right_x1,y1))
            res_right_line.append((right_x2, y2))

        if self.draw_left:
            res_left_line.append((left_x1,y1))
            res_left_line.append((left_x2, y2))

        filtered_lines['left'] = res_left_line
        filtered_lines['right'] = res_right_line

        return filtered_lines
        
        
                
