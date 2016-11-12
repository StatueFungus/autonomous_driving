#!/usr/bin/python
# -*- coding: utf-8 -*-
import cv2
import numpy as np

class ImagePreparator:
    
    def __init__(self, image):
        self.image = image        
        self.height, self.width, self.channels = image.shape

    def define_roi(self, above, below, side):
        ''' 
            Bildbereiche welche nicht von Interesse sind werden geschwaerzt. 

            above : double
                Angabe in Prozent, wie viel vom oberen Abschnitt geschwaerzt werden soll.
                >> 1 entspricht dabei 100%
        
        '''
        color_black = (0,0,0)
        # maskiert untere Bildhäfte
        self.image[self.height - int((self.height*below)):self.height,:] = color_black
        # definiere Punkte für Polygon und maskiert die obere Bildhälfte
        pts = np.array([[0,0],[0,int(self.height*(above+0.15))],[int(self.width*side),int(self.height*above)],[self.width-int(self.width*side),int(self.height*above)],[self.width,int(self.height*(above+0.15))],[self.width,0]], np.int32)
        cv2.fillPoly(self.image, [pts], color_black)

    def grayscale(self):
        ''' 
            Bild (self.image) wird in Graustufen konvertiert
        
        '''
        self.image = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)

    def blur(self, deviation, border):
        self.image = cv2.GaussianBlur(self.image, deviation, border)
    
    def global_threshold(self, threshold1, threshold2):
        dst, self.image = cv2.threshold(self.image, threshold1, threshold2, cv2.THRESH_BINARY)

    def canny(self, threshold1, threshold2, aperture):
        self.image = cv2.Canny(self.image, threshold1, threshold2, aperture)

    def filter_white_color(self, lower_white, upper_white):
        white_mask = cv2.inRange(self.image, np.array([lower_white,lower_white,lower_white]), np.array([upper_white,upper_white,upper_white]))
        self.image = cv2.bitwise_and(self.image, self.image, mask=white_mask)

    def morph_open(self, kernel):
        self.image = cv2.morphologyEx(self.image, cv2.MORPH_OPEN, np.ones((kernel, kernel),np.uint8))

