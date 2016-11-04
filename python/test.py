#!/usr/bin/python
# -*- coding: utf-8 -*-
import cv2
from detectionlib import Visualizer
from detectionlib import LineFilter
from detectionlib import LaneDetector
from detectionlib import ImagePreparator

def main():
    capture = cv2.VideoCapture('data/rage.avi')
    ticks = 0
    
    line_filter = LineFilter()
    lane_detector = LaneDetector()    
    img_prep = ImagePreparator()
    vis = Visualizer()    

    while capture.isOpened():
        prevTick = ticks
        ticks = cv2.getTickCount()
        t = (ticks - prevTick) / cv2.getTickFrequency()
        fps = int(1 / t)
        
        retval, image = capture.read()
        image_copy = image.copy()
        
        # Aufbereitung des Bildes
        img_prep.image = image_copy
        img_prep.define_roi(0.5)
        img_prep.grayscale()
        img_prep.adaptive_hist_equalization()
        img_prep.blur((5,5), 3)
        thresh = img_prep.global_threshold(165, 255)
        edges = img_prep.canny(1, 165, 3)
        
        # Entdecke Linien
        lines1 = lane_detector.houghlines_p(thresh, 100, 1, 10)
        lines2 = lane_detector.houghlines_p(edges, 100, 1, 10)
        
        # Filter korrekte Linien
        flines1 = line_filter.filter(lines1, 10, 50)
        flines2 = line_filter.filter(lines2, 10, 50)
         
        # Anzeige des Bildes
        vis.image = image
        vis.draw_lines(flines1, (0,255,0), 2)
        vis.draw_lines(flines2, (255,0,0), 2)
        if len(flines1) + len(flines2) == 0:
            vis.draw_text('NO LINES', 1, (0,0,255), (10, 75))
        vis.draw_text('FPS: ' + str(fps), 1, (255,0,0), (10, 30))
        vis.show()
        



if __name__ == '__main__':
    main()
