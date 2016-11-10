#!/usr/bin/python
# -*- coding: utf-8 -*-
import math
import numpy as np

class LineFilter:
    '''
        Klasse filtert Linien, welche in einem bestimmten Winkel vertikal verlaufen.

    '''

    def __init__(self):
        '''
            Konstruktor

        '''

    def filter(self, lines, filter_angle, y_gab):
        res = []
        for line in lines:
            for obj in line:
                [x1, y1, x2, y2] = obj
                dx, dy = x2 - x1, y2 - y1
                angle = np.arctan2(dy, dx) * 180 / np.pi
                if math.fabs(angle) <= filter_angle:
                    pass
                    #continue
                if y1 > y2 + y_gab or y1 < y2 - y_gab:
                    pass
                    #res.append(obj)    
                res.append(obj)
        return res
                
