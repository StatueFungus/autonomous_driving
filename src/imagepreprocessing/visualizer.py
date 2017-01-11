#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import imutils


class Visualizer:
    '''
        Klasse welche Hilfsmethoden anbietet um Text und Formen (Linien, Punkte)
        mittels OpenCV auf ein Bild zu rendern und dieses anzuzeigen
    '''

    def show(self, image, title='image'):
        '''
            Zeigt das Bild in einem Fenter an.

            Parameter
            ---------
            image : Das anzuzeigende Bild
            title (optional) : String
                Name des Fensters

        '''
        cv2.imshow(title, image)
        cv2.waitKey(1)

    def draw_line(self, image, point1, point2, line_color, line_size):
        '''
            Zeichnet eine Linie aud ein Bild

            Parameter
            ---------
            image : Das anzuzeigende Bild
            point1 : Tupel
                Startpunkt
            point2 : Tupel
                Endpunkt
            line_color : Tupel
                Farbe der Linie
            line_size : Integer
                Dicke der Linie

        '''
        cv2.line(image, point1, point2, line_color, line_size)

    def draw_text(self, image, text, size, color, position):
        '''
            Zeichnet einen Text auf das Bild.

            Parameter
            ---------
            text : String
                Anzuzeigender Text
            size : Integer
                Groesse des Textes
            color : Tupel
                Farbe des Textes >> (255,0,0)
            position : Tupel
                Position des Textes >> (x,y)

        '''
        if imutils.is_cv2():
            cv2.putText(image, text, position, cv2.FONT_HERSHEY_COMPLEX, size, color, 2, cv2.CV_AA)
        elif imutils.is_cv3():
                cv2.putText(image, text, position, cv2.FONT_HERSHEY_COMPLEX, size, color, 2, cv2.LINE_AA)

    def draw_point(self, image, point, radius, color, thickness):
        '''
            Zeichnet einen Punkt auf das Bild.

            Parameter
            ---------
            image : Das anzuzeigende Bild
            point : Tupel
                Position des Punktes auf dem Bild >> (x,y)
            radius : Integer
                Radius des Punktes
            color : Tupel
                Farbe des Punktes
            thickness : Integer
                Dicke des Punktes

        '''
        cv2.circle(image, point, radius, color, thickness)
