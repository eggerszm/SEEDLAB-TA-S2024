"""
Marker Detection
SEED Lab Group 9, Fall 2023

"""

import cv2
import enum
import time
import numpy as np
from cv2 import aruco
from smbus2 import SMBus
from typing import Tuple

import LCDInit

IMG_X_WIDTH = 640
IMG_Y_HEIGHT = 480


def main():
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)

    camera = cv2.VideoCapture(0)
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, IMG_X_WIDTH)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, IMG_Y_HEIGHT)
    camera.set(cv2.CAP_PROP_BRIGHTNESS, 100)

    time.sleep(0.5)

    piLCD = LCDInit.LCD()

    # Load calibration data
    with np.load("calib_data.npz") as data:
        mtx = data["camera_matrix"]
        dist = data["dist_coeff"]

    ctr = (0, 0)
    while True:
        _, img = camera.read()

        ncm = None
        undistort_img = cv2.undistort(img, mtx, dist, None, ncm)

        img_gray = cv2.cvtColor(undistort_img, cv2.COLOR_BGR2GRAY)

        corners, ids, rejected = aruco.detectMarkers(img_gray, aruco_dict)

        overlay = aruco.drawDetectedMarkers(img, corners, borderColor=4)

        if len(corners) == 1:
            ctr = get_center(corners[0])
            overlay = cv2.circle(
                overlay, ctr, radius=2, color=(0, 0, 255), thickness=-1
            )
        elif corners == ():
            pass
        else:
            for c in corners:
                overlay = cv2.circle(
                    overlay, get_center(c), radius=2, color=(0, 0, 255), thickness=-1
                )
        overlay = cv2.line(overlay, (0, int(IMG_Y_HEIGHT/2)), (IMG_X_WIDTH, int(IMG_Y_HEIGHT/2)), (0,0,255), 1)
        overlay = cv2.line(overlay, (int(IMG_X_WIDTH/2), 0), (int(IMG_X_WIDTH/2), IMG_Y_HEIGHT), (0,0,255), 1)
 
        cv2.imshow("original", overlay)

        # Update LCD
        piLCD.soft_update(f"xpos: {ctrx_to_deg(ctr[0]):.2f}")

        if (cv2.waitKey(1) & 0xFF) == ord("q"):
            cv2.destroyAllWindows()
            piLCD.cleanup()
            break


def get_center(corners: np.array) -> Tuple[int, int]:
    """
    Take a corners array and return its center point
    Args:
        corners (np.array): shape (1,4,2), contains the four corners of the marker
    Returns:
        Tuple[int, int]: coordinates of the marker in the frame
    """
    xctr = (
        corners[0, 0, 0] + corners[0, 1, 0] + corners[0, 2, 0] + corners[0, 3, 0]
    ) / 4
    yctr = (
        corners[0, 0, 1] + corners[0, 1, 1] + corners[0, 2, 1] + corners[0, 3, 1]
    ) / 4
    return (int(xctr), int(yctr))

def ctrx_to_deg(xpos: int) -> float:
    return 0.084826 * float(xpos) - 27.218717

if __name__ == "__main__":
    main()
