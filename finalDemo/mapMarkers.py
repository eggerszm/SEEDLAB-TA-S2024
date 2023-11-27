"""Intial proof of concept mapping code for 6 aruco markers"""

import math
import cv2
import shapely
import numpy as np
from cv2 import aruco

import matplotlib.pyplot as plt

import cameraConfig
import createPath
import transformations as tf

MARKER_SIZE = 8.5  # cm-ish, tuning constant


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

        corners, ids, _ = aruco.detectMarkers(img_gray, aruco_dict)

        overlay = aruco.drawDetectedMarkers(undist_img, corners)

        overlay = cv2.line(
            overlay, (0, int(480 / 2)), (640, int(480 / 2)), (0, 0, 255), 1
        )
        overlay = cv2.line(
            overlay, (int(640 / 2), 0), (int(640 / 2), 480), (0, 0, 255), 1
        )

        # Estimate pose of each detected marker
        poses = np.zeros((6, 3))
        poses_rot = np.zeros((6, 2))
        if len(corners) > 0:
            print("One Image:")
            for i in range(0, len(ids)):
                rv, tv, _ = aruco.estimatePoseSingleMarkers(
                    corners[i], MARKER_SIZE, mtx, dist_mtx
                )
                poses[ids[i], :] = tv[0, 0, :]


        # TODO: Go to 2D
        poses2d = np.delete(np.copy(poses), 1, 1)

        poly = shapely.Polygon(poses2d)
        ctr = (poly.centroid.x, poly.centroid.y)
        if ctr == (0,0):
            ctr = (1,1)
        print(ctr)

        # Rotate points by 45 deg left
        for i in range(0, 6):
            poses_rot[i, :] = createPath.extend_from_ctr(ctr, poses[i], 20)

        cv2.imshow("Detected Markers", overlay)

        #print(poses, poses_rot)

        # Plotting for testing
        fig = plt.figure()
        ax = fig.add_subplot(projection="3d")
        ax.scatter(0, 0, 0)
        ax.scatter(poses[:, 0], poses[:, 2], poses[:, 1])
        ax.scatter(poses_rot[:, 0], poses_rot[:, 1], 0)
        plt.show()

        if cv2.waitKey(1) & 0xFF == ord("q"):
            cv2.destroyAllWindows()
            plt.close("all")
            # break


if __name__ == "__main__":
    main()
