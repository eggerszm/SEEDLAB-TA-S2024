import numpy as np
import time
import cv2

camera = cv2.VideoCapture(0)
camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
camera.set(cv2.CAP_PROP_BRIGHTNESS, 100)

time.sleep(2)

while True:
    _, img = camera.read()

    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    cv2.imshow("img", img_gray)

    cv2.waitKey(1)
    
