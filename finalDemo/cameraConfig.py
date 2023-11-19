"""Camera configuration functions and constants"""

import cv2
import time

IMG_X_WIDTH = 640
IMG_Y_HEIGHT = 480


def get_camera() -> cv2.VideoCapture:
    """Return an configured cv2.VideoCapture object

    Returns:
      cv2.VideoCapture: Camera for aruco capture
    """
    camera = cv2.VideoCapture(0)

    camera.set(cv2.CAP_PROP_FRAME_WIDTH, IMG_X_WIDTH)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, IMG_Y_HEIGHT)
    camera.set(cv2.CAP_PROP_BRIGHTNESS, 100)

    time.sleep(0.5)

    return camera
