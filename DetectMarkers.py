"""
Marker Detection
SEED Lab Group 9, Fall 2023

"""

import cv2
import time
import numpy as np
from cv2 import aruco

from Exercise1b import init_lcd

def main():
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    
    camera = cv2.VideoCapture(0)
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    camera.set(cv2.CAP_PROP_BRIGHTNESS, 100)

    time.sleep(0.5)

    lcd = init_lcd()
    lcd.color = [100, 100, 100]

    current_msg = "No markers found"

    while True:
        _, img = camera.read()

        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        corners, ids, rejected = aruco.detectMarkers(img_gray, aruco_dict)

        overlay = aruco.drawDetectedMarkers(img, corners, borderColor=4)

        if ids is not None:
            msg = ""
            for id in ids:
                msg += f"{str(id[0])} "
        else:
            msg = "No markers found"
                
        if msg != current_msg:
            current_msg = msg
            lcd.clear()
            lcd.message = current_msg

        cv2.imshow("overlay", overlay)
        if (cv2.waitKey(1) & 0xFF) == ord("q"):
            cv2.destroyAllWindows()
            lcd.color = [0, 0, 0]
            break


if __name__ == "__main__":
    main()
