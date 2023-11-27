"""Creates a target path for the robot to follow"""

import math
import shapely


def extend_from_ctr(ctr, point, offset):
    dx = point[0] - ctr[0]
    dy = point[1] - ctr[1]

    dist = math.sqrt(dx**2 + dy**2)

    c = (offset + dist) / dist

    return (ctr[0] + dx * c, ctr[1] + dy * c)
