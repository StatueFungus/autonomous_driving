#!/usr/bin/python
# -*- coding: utf-8 -*-
import cv2
import numpy as np
from detectionlib import Visualizer
from detectionlib import LineFilter
from detectionlib import LaneDetector
from detectionlib import ImagePreparator

def main():
    capture = cv2.VideoCapture('data/road.avi')
    ticks = 0
    
    line_filter = LineFilter()
    lane_detector = LaneDetector()

    while capture.isOpened():
        prevTick = ticks
        ticks = cv2.getTickCount()
        t = (ticks - prevTick) / cv2.getTickFrequency()
        fps = int(1 / t)
        
        retval, image = capture.read()
        
        # Aufbereitung des Bildes
        img_prep = ImagePreparator(image.copy())
        img_prep.define_roi(0.5)
        img_prep.grayscale()
        img_prep.adaptive_hist_equalization()
        img_prep.blur((5,5), 3)
        # Canny oder Threshold benutzen
        #img_prep.global_threshold(165, 255)
        img_prep.canny(1, 250, 3)
                    
        # Entdecke Linien
        lines = lane_detector.houghlines_p(img_prep.image, 50, 1, 10)
        
        # Filter korrekte Linien
        flines = line_filter.filter(lines, 10, 50)
        
        # Anzeige des Bildes
        vis = Visualizer(image)
        vis.draw_lines(flines, (0,255,0), 2)
        
        if len(flines) == 0:
            vis.draw_text('NO LINES', 1, (0,0,255), (int(img_prep.width*0.015), int(img_prep.height*0.25)))
        vis.draw_text('FPS: ' + str(fps), 1, (255,0,0), (int(img_prep.width*0.015), int(img_prep.height*0.15)))
        vis.show()


if __name__ == '__main__':
    main()
