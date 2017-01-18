#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np


class ImagePreparator:
    '''
        Klasse welche Methoden anbietet um das Bild für die Erkennung der Straßenmarkierung
        vorzubereiten.
    '''

    def define_roi(self, image, above=0.0, below=0.0, side=0.0):
        '''
            Bildbereiche welche nicht von Interesse sind werden geschwärzt.

            Parameter
            ---------
            image : das zu maskierende Bild
            above (optional) : Float
                Angabe in Prozent, wie viel vom oberen Bild geschwärzt werden soll.
                Default Wert ist 0.0
                >> 1.0 entspricht dabei 100%
            below (optional) : Float
                Angabe in Prozent, wie viel vom unteren Bild geschwärzt werden soll.
                Default Wert ist 0.0
                >> 1.0 entspricht dabei 100%
            side (optional) : Float
                Angabe in Prozent, wie viel von den Seiten des Bildes geschwärzt werden soll.
                Dabei werden die Seiten nicht senkrecht nach unten maskiert, sondern trapezförmig
                zum oberen maskierten Bildrand (above).
                Default Wert ist 0.0
                >> 1.0 entspricht dabei 100%

            Rückgabe
            ---------
            image : maskiertes Bild

        '''
        height, width, channels = image.shape
        color_black = (0, 0, 0)
        # maskiert untere Bildhäfte
        image[height - int((height*below)):height, :] = color_black
        # definiere Punkte für Polygon und maskiert die obere und seitliche Bildhälfte
        pts = np.array([[0, 0], [0, int(height*(above+0.15))], [int(width*side), int(height*above)], [width-int(width*side), int(height*above)], [width, int(height*(above+0.15))], [width, 0]], np.int32)
        cv2.fillPoly(image, [pts], color_black)
        return image

    def crop(self, image, above=0.0, below=0.0, side=0.0):
        '''
            Bild wird zugeschnitten.

            Parameter
            ---------
            image : das Bild
            above (optional) : Float
                Angabe in Prozent, wie viel vom oberen Bild zugeschnitten werden soll.
                Default Wert ist 0.0
                >> 1.0 entspricht dabei 100%
            below (optional) : Float
                Angabe in Prozent, wie viel vom unteren Bild zugeschnitten werden soll.
                Default Wert ist 0.0
                >> 1.0 entspricht dabei 100%
            side (optional) : Float
                Angabe in Prozent, wie viel von beiden Seiten des Bildes zugeschnitten werden soll.
                Default Wert ist 0.0
                >> 0.5 entspricht dabei 100%

            Rückgabe
            ---------
            image : zugeschnittenes Bild

        '''
        height, width, channels = image.shape
        image = image[int((height*above)):height - int((height*below)), int((width*side)):width - int((width*side))]
        return image

    def grayscale(self, image):
        '''
            Bild wird in Graustufen konvertiert.

            Parameter
            ---------
            image : Bild

            Rückgabe
            ---------
            image : Bild

        '''
        return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    def blur(self, image, kernel_size, border):
        '''
            Methode entfernt mögliches Rauschen vom Bild.

            Parameter
            ---------
            image : Bild
            kernel_size : Tupel
                >> (w,h)
            border : Integer

            Rückgabe
            ---------
            image : Bild

        '''
        return cv2.GaussianBlur(image, kernel_size, border)

    def edge_detection(self, image, threshold1, threshold2, aperture):
        '''
            Methode erkennt Kanten auf einem Bild basierend auf dem Canny-Algorithmus.

            Parameter
            ---------
            image : Bild
            threshold1 : Integer
            threshold2 : Integer
            aperture : Integer

            Rückgabe
            ---------
            image : Bild

        '''
        return cv2.Canny(image, threshold1, threshold2, aperture)

    def warp_perspective(self, image, transformation_matrix, resolution):
        return cv2.warpPerspective(image, transformation_matrix, resolution)

    def filter_color(self, image, lower_color, upper_color):
        '''
            Methode maskiert Bereiche auf einem Bild welche nicht im mitgegebenen
            HSV Farbraum liegen.

            Parameter
            ---------
            image : Bild
            lower_color : Tupel
                >> (h,s,v)
            upper_color : Tupel
                >> (h,s,v)

            Rückgabe
            ---------
            image : Bild

        '''
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_color = np.array(lower_color, np.uint8)
        upper_color = np.array(upper_color, np.uint8)

        color_mask = cv2.inRange(hsv, lower_color, upper_color)
        return cv2.bitwise_and(image, image, mask=color_mask)
