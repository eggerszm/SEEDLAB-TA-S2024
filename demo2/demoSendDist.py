"""
Marker Distance and Angle Detection
SEED Lab Group 9, Fall 2023
"""

import cv2
import math
import time
import struct
import numpy as np
from cv2 import aruco
from smbus2 import SMBus


IMG_X_WIDTH = 640
IMG_Y_HEIGHT = 480

MARKER_LEN = 3.95 # in

def main():
    ser = serial.Serial("/dev/ttyACMA0", baudrate=115200, timeout=2)

    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)

    camera = cv2.VideoCapture(0)
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, IMG_X_WIDTH)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, IMG_Y_HEIGHT)
    camera.set(cv2.CAP_PROP_BRIGHTNESS, 100)

    time.sleep(0.5)

    # Load calibration data
    with np.load("calib_data.npz") as data:
        mtx = data["camera_matrix"]
        dist_mtx = data["dist_coeff"]

    ctr = (0, 0)

    while True:
        _, img = camera.read()

        img = cv2.flip(img, 0)
        img = cv2.flip(img, 1)

        ncm = None
        undistort_img = cv2.undistort(img, mtx, dist_mtx, None, ncm)

        img_gray = cv2.cvtColor(undistort_img, cv2.COLOR_BGR2GRAY)

        corners, ids, rejected = aruco.detectMarkers(img_gray, aruco_dict)

        overlay = aruco.drawDetectedMarkers(undistort_img, corners)

        if len(corners) == 1:
            rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, MARKER_LEN, mtx, dist_mtx)

        else:
            rvec, tvec = None, None
            pass

        dist, ang = get_dist_angle(tvec)
        
        try: 
            if dist is not None and ang is not None:
                send_2floats(ang, dist, serBus)
        except IOError:
            print("Failed to transmit")

        cv2.imshow("original", overlay)

        if (cv2.waitKey(1) & 0xFF) == ord("q"):
            cv2.destroyAllWindows()
            break

def get_dist_angle(tvec) -> (float, float):
    if tvec is None:
        return None, None
    dist = (tvec[0][0][2] + 10)
    ang = (-math.atan(tvec[0][0][0] / tvec[0][0][2]))
    print(f"{dist} @ {ang / math.pi *180}")
    return dist, ang

def send_2floats(fl1, fl2, serBus):
    send_bytes = bytearray(struct.pack("f", fl1)) + bytearray(struct.pack("f", fl2))
    serBus.write(send_bytes)
    

if __name__ == "__main__":
    main()

