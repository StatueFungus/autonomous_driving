#!/usr/bin/python
# -*- coding: utf-8 -*-
import cv2
import numpy as np

class LaneDetector:
    ''' 
        Klasse welche die Straßenmarkierungen auf einem Bild erkennt. 
    
    '''

    def __init__(self):
        '''
            Konstruktor.

        '''

    def houghlines_p(self, image, threshold, min_line_length, max_line_gab):
        '''
            Mittels der Probabilistic Hough Transform werden die Linien aus dem Bild erkannt.

            Parameter
            ---------
            image : Bild mit Straßenmarkierung
            threshold : int
            min_line_length : int
                Minimal Laenge einer Linie. Liniensegemnte die kleiner sind werden ignoriert.
            max_line_gab : int
                Maximale Spalte zwischen zwei Liniensegmenten um diese als einzelne Linie zu erkennen.
            
            Rueckgabe
            ---------
            Array : Bestehend aus Linien >> [x1, y1, x2, y2]. 
                    Falls keine Linien entdeckt wurden ist das Array leer.

        '''
        lines = cv2.HoughLinesP(image, 1, np.pi/180, threshold, min_line_length, max_line_gab)
        if lines is None:
            return []
        return lines

        
