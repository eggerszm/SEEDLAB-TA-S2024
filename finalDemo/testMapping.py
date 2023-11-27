"""Mock test for mapping"""

import shapely
import numpy as np

import matplotlib.pyplot as plt

import createPath


def main():
    pts = np.array([[70, 1], [88, -15], [94, -22], [102, -3], [96, 12], [87, 15]])
    print(pts)

    poly = shapely.Polygon(pts)
    ctr = (poly.centroid.x, poly.centroid.y)

    offset_pts = np.zeros((6, 2))
    for i in range(0, 6):
        offset_pts[i] = createPath.extend_from_ctr(ctr, pts[i], 10)

    fig, ax = plt.subplots()
    plt.scatter(pts[:, 0], pts[:, 1])
    plt.scatter(offset_pts[:, 0], offset_pts[:, 1])
    plt.show()


if __name__ == "__main__":
    main()
