"""
Measure the distance between an arucoMarker and the camera
"""

import cv2
import imutils
import numpy as np
from cv2 import aruco
from smbus2 import SMBus
from imutils import paths

class distanceMeasure():
    
    def dist(img):
        #our set of knowns
        fLength = 333.82
        width = 1.57

        #image manipulation to find edge
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        gray = cv2.Canny(gray, 35, 125)

        #finds the contours of the image
        count = cv2.findContours(gray.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        count = imutils.grab_contours(count)
        contour = max(count, key = cv2.contourArea)

        distance = (width * fLength)/ contour[0]
        
        return distance