"""
Marker Detection
SEED Lab Group 9, Fall 2023

"""

import cv2
import enum
import math
import time
import numpy as np
from cv2 import aruco
from smbus2 import SMBus
from typing import Tuple

IMG_X_WIDTH = 640
IMG_Y_HEIGHT = 480

MARKER_LEN = 4 # in


def main():
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)

    camera = cv2.VideoCapture(0)
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, IMG_X_WIDTH)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, IMG_Y_HEIGHT)
    camera.set(cv2.CAP_PROP_BRIGHTNESS, 100)

    time.sleep(0.5)

    # Load calibration data
    with np.load("calib_data.npz") as data:
        mtx = data["camera_matrix"]
        dist = data["dist_coeff"]

    ctr = (0, 0)
    while True:
        _, img = camera.read()

        img = cv2.flip(img, 0)
        img = cv2.flip(img, 1)

        ncm = None
        undistort_img = cv2.undistort(img, mtx, dist, None, ncm)

        img_gray = cv2.cvtColor(undistort_img, cv2.COLOR_BGR2GRAY)

        corners, ids, rejected = aruco.detectMarkers(img_gray, aruco_dict)

        overlay = aruco.drawDetectedMarkers(undistort_img, corners)

        if len(corners) == 1:
            rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, MARKER_LEN, mtx, dist)
            print(f"Dist: {3.25 + tvec[0][0][2]}")
            print()
            overlay = cv2.drawFrameAxes(overlay, mtx, dist, rvec, tvec, 1.0)

        else:
            rvec, tvec = None, None
            pass
 
        cv2.imshow("original", overlay)

        if (cv2.waitKey(1) & 0xFF) == ord("q"):
            cv2.destroyAllWindows()
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
    return -1* (0.084826 * float(xpos) - 27.218717)

if __name__ == "__main__":
    main()
