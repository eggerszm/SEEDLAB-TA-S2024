"""Coordinate transformations for use with pose estimation"""

import math
import numpy as np

def rotate_y(point, angle):
    rotation_mtx = np.array([[-math.cos(angle), 0.0, -math.sin(angle)], [0.0, 1.0, 0.0], [math.sin(angle), 0.0, math.cos(angle)]])
    
    return np.matmul(rotation_mtx, point)

def main():
    """"Test rotations"""
    print(pt := np.array([1,0,0]))
    print(rotate_y(pt, math.pi/2))


if __name__ == "__main__":
    main()
