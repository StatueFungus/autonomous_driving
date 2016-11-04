#!/usr/bin/python
# -*- coding: utf-8 -*-
import cv2

class Visualizer:
    ''' Klasse welche Methoden anbietet um Text und Linien auf ein Bild zu rendern und dieses anzuzeigen '''

    def __init__(self, image=None):
        ''' 
            Konstruktor.
           
            Parameter
            ---------    
            image : anzuzeigendes Bild

        '''
        self.image = image

    def show(self, title='image'):
        ''' 
            Zeigt das Bild (self.image) in einem Fenter an. 
            
            Parameter
            ---------
            text : String
                optionale Angabe eines Titels
        
        '''
        cv2.imshow(title, self.image)
        cv2.waitKey(1)

    def draw_lines(self, lines, line_color, line_size):
        '''
            Zeichnet eine Linen auf das Bild (self.image).
            
            Parameter
            ---------
            lines : Array 
                Bestehend aus Linien >> [[x1, y1, x2, y2],...]
            line_color: Tupel
                Farbe der Linie >> (255,0,0)
            line_size: int
                Dicke der Linie in Pixel
        
        '''
        for obj in lines:
            [x1, y1, x2, y2] = obj
            cv2.line(self.image, (x1,y1), (x2,y2), line_color, line_size)

    def draw_text(self, text, size, color, position):
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
        cv2.putText(self.image, text,position, cv2.FONT_HERSHEY_COMPLEX, size, color, 2, cv2.LINE_AA)
