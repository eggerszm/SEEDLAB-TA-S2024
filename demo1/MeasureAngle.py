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
    ard = SMBus(1)

    while True:
        _, img = camera.read()

        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        corners, ids, rejected = aruco.detectMarkers(img_gray, aruco_dict)

        overlay = aruco.drawDetectedMarkers(img, corners, borderColor=4)

        if len(corners) == 1:
            ctr = get_center(corners[0])
            overlay = cv2.circle(
                overlay, ctr, radius=2, color=(0, 0, 255), thickness=-1
            )
        elif corners == ():
        else:
            for c in corners:
                overlay = cv2.circle(
                    overlay, get_center(c), radius=2, color=(0, 0, 255), thickness=-1
                )

        cv2.imshow("overlay", overlay)

        # Update LCD
        piLCD.write_lcd(f"Angle: ")

        if (cv2.waitKey(1) & 0xFF) == ord("q"):
            cv2.destroyAllWindows()
            piLCD.cleanup()
            ard.close()
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


def get_point_quadrant(point: Tuple[int, int]) -> Quadrant:
    """
    Get the quadrant a point falls into, where (0,0) is the NW corner
    Args:
        point (Tuple[int, int]): (X, Y) coordinate pair to classify
    Returns:
        Quadrant: Quadrant of point, from enum
    """
    if point[0] <= IMG_X_WIDTH / 2 and point[1] <= IMG_Y_HEIGHT / 2:
        return Quadrant.NW
    elif point[0] <= IMG_Y_HEIGHT / 2:
        return Quadrant.SW
    elif point[1] <= IMG_X_WIDTH / 2:
        return Quadrant.NE
    else:
        return Quadrant.SE


if __name__ == "__main__":
    main()
