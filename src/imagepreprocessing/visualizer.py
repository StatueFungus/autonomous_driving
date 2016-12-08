#!/usr/bin/python
# -*- coding: utf-8 -*-
import cv2, imutils

class Visualizer:
    ''' Klasse welche Methoden anbietet um Text und Linien auf ein Bild zu rendern und dieses anzuzeigen '''

    def show(self, image, title='image'):
        ''' 
            Zeigt das Bild (self.image) in einem Fenter an. 
            
            Parameter
            ---------
            text : String
                optionale Angabe eines Titels
        
        '''
        cv2.imshow(title, image)
        cv2.waitKey(1)

    def draw_line(self, image, point1, point2, line_color, line_size):
        cv2.line(image, point1, point2, line_color, line_size)

    def draw_text(self, image, text, size, color, position):
        '''
            Zeichnet einen Text auf das Bild (self.image).

            Parameter
            ---------
            text : String
                Anzuzeigender Text
            size : int
                Groesse des Textes
            color : Tupel
                Farbe des Textes >> (255,0,0)
            position : Tupel
                Position des Textes >> (x,y)

        '''
	if imutils.is_cv2():
	    cv2.putText(image, text,position, cv2.FONT_HERSHEY_COMPLEX, size, color, 2, cv2.CV_AA)
	elif imutils.is_cv3():
            cv2.putText(image, text,position, cv2.FONT_HERSHEY_COMPLEX, size, color, 2, cv2.LINE_AA)

    def draw_point(self, image, point, radius, color, thickness):
        cv2.circle(image, point, radius, color, thickness)
