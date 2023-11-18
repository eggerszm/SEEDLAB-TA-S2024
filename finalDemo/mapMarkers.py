"""Intial proof of concept mapping code for 6 aruco markers"""

import cv2
import numpy as np
from cv2 import aruco

import cameraConfig


def main():
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)


if __name__ == "__main__":
    main()
