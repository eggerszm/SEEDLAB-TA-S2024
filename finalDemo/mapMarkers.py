"""Intial proof of concept mapping code for 6 aruco markers"""

import cv2
import numpy as np
from cv2 import aruco

import matplotlib.pyplot as plt

import cameraConfig

MARKER_SIZE = 8.5 # cm-ish, tuning constant


def main():
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)

    camera = cameraConfig.get_camera()

    with np.load("calib_data.npz") as data:
        mtx = data["camera_matrix"]
        dist_mtx = data["dist_coeff"]

    if True:
        _, img = camera.read()


        # Undistort and process the image for detection
        ncm = None
        undist_img = cv2.undistort(img, mtx, dist_mtx, None, ncm)
        img_gray = cv2.cvtColor(undist_img, cv2.COLOR_BGR2GRAY)

        corners, ids, rejected = aruco.detectMarkers(img_gray, aruco_dict)

        overlay = aruco.drawDetectedMarkers(undist_img, corners)

        overlay = cv2.line(overlay, (0, int(480/2)), (640, int(480/2)), (0,0,255), 1)
        overlay = cv2.line(overlay, (int(640/2), 0), (int(640/2), 480), (0,0,255), 1)

        # Estimate pose of each detected marker
        poses = np.zeros((6,2))
        if len(corners) > 0:
            print("One Image:")
            for i in range(0, len(ids)):
                rv, tv, _ = aruco.estimatePoseSingleMarkers(corners[i], MARKER_SIZE, mtx, dist_mtx)
                poses[ids[i], 0] = tv[0, 0, 0]
                poses[ids[i], 1] = tv[0, 0, 2]



        cv2.imshow("Detected Markers", overlay)

        # Plotting for testing
        fig, ax = plt.subplots()
        ax.scatter(poses[:, 0], poses[:, 1])
        plt.show()

        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            plt.close('all')
            #break

    

if __name__ == "__main__":
    main()
