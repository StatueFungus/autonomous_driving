#!/usr/bin/python
# -*- coding: utf-8 -*- 

import cv2, os
import numpy as np
from imagepreprocessing import Visualizer
from imagepreprocessing import ImagePreparator

def main():
    print os.getcwd()
    capture = cv2.VideoCapture('../data/road.avi')
    ticks = 0

    img_prep = ImagePreparator()
    vis = Visualizer()

    while capture.isOpened():
        prevTick = ticks
        ticks = cv2.getTickCount()
        t = (ticks - prevTick) / cv2.getTickFrequency()
        fps = int(1 / t)
        
        retval, image = capture.read()
        height, width, channels = image.shape
        
        rect= np.array([
        [0, 200],
        [639, 200],
        [639, 359],
        [0, 359]], dtype = "float32")

        dst = np.array([
        [0, 0],
        [639, 0],
        [350, 699],
        [298, 699]], dtype = "float32")

        M = cv2.getPerspectiveTransform(rect, dst)
        warped = cv2.warpPerspective(image, M, (640, 700))

        # Aufbereitung des Bilder
        roi = img_prep.define_roi(warped, 0.6, 0, 0.40)
        gray = img_prep.grayscale(roi)
        blur = img_prep.blur(gray, (5,5), 0)
        canny = img_prep.edge_detection(blur, 50, 150, 3)

        vis.draw_text(canny, 'FPS: ' + str(fps), 1, (255,0,0), (int(width*0.015), int(height*0.15)))
        vis.show(canny)

if __name__ == '__main__':
    main()
