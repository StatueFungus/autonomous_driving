#!/usr/bin/python
# -*- coding: utf-8 -*-
import cv2

class ImagePreparator:
    
    def __init__(self, image=None):
        self.image = image        

    def define_roi(self, above):
        ''' 
            Bildbereiche welche nicht von Interesse sind werden geschwaerzt. 

            above : double
                Angabe in Prozent, wie viel vom oberen Abschnitt geschwaerzt werden soll.
                >> 1 entspricht dabei 100%
        
        '''
        heigth, width, channels = self.image.shape
        black = (0,0,0)
        # oberer Bildabschnitt
        self.image[0:int((heigth*above)),:] = black
        return self.image

    def grayscale(self):
        ''' 
            Bild (self.image) wird in Graustufen konvertiert
        
        '''
        self.image = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
        return self.image

    def adaptive_hist_equalization(self):
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        self.image = clahe.apply(self.image)
        return self.image

    def blur(self, deviation, border):
        self.image = cv2.GaussianBlur(self.image, deviation, border)
        return self.image
    
    def global_threshold(self, threshold1, threshold2):
        dst, thresh = cv2.threshold(self.image, threshold1, threshold2, cv2.THRESH_BINARY)
        return thresh

    def canny(self, threshold1, threshold2, aperture):
        edges = cv2.Canny(self.image, threshold1, threshold2, aperture)
        return edges
