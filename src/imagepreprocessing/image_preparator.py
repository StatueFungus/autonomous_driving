#!/usr/bin/python
# -*- coding: utf-8 -*-
import cv2
import numpy as np

class ImagePreparator:

    def define_roi(self, image, above=0.0, below=0.0, side=0.0):
        ''' 
            Bildbereiche welche nicht von Interesse sind werden geschwaerzt. 

            Parameter
            ---------
            above : double
                Angabe in Prozent, wie viel vom oberen Bild geschwaerzt werden soll.
                >> 1.0 entspricht dabei 100%
            below : double
                Angabe in Prozent, wie viel vom unteren Bild geschwaerzt werden soll.
                >> 1.0 entspricht dabei 100%
            side : double
                Angabe in Prozent, wie viel von den Seiten des Bildes geschwaerzt werden soll.
                Dabei werden die Seiten nicht senkrecht nach unten maskiert, sondern trapezförmig zum oberen Bildrand.
                >> 1.0 entspricht dabei 100%

            Rueckgabe
            ---------
            image : maskiertes Bild
        
        '''
        height, width, channels = image.shape
        color_black = (0,0,0)
        # maskiert untere Bildhäfte
        image[height - int((height*below)):height,:] = color_black
        # definiere Punkte für Polygon und maskiert die obere und seitliche Bildhälfte

        pts = np.array([[0,0],[0,int(height*(above+0.15))],[int(width*side),int(height*above)],[width-int(width*side),int(height*above)],[width,int(height*(above+0.15))],[width,0]], np.int32)
        cv2.fillPoly(image, [pts], color_black)
        return image

    def grayscale(self, image):
        ''' 
            Bild wird in Graustufen konvertiert
        
        '''
        return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    def blur(self, image, deviation, border):
        return cv2.GaussianBlur(image, deviation, border)

    def edge_detection(self, image, threshold1, threshold2, aperture):
        return cv2.Canny(self.image, threshold1, threshold2, aperture)

    def filter_color(self, image, lower_color, upper_color):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_color = np.array(lower_color, np.uint8)
        upper_color = np.array(upper_color, np.uint8)

        color_mask = cv2.inRange(hsv, lower_color, upper_color)
        return cv2.bitwise_and(image, image, mask=color_mask)

