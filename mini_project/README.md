# Mini Project
The way the raspberry pi detects the markers is through the use of DetectMarkers.py. The main purpose of this code is to use a camera attached to the raspberry pi and cv2 library to detect an aruco marker to find what wheel position is desired. The raspberry pi shows a live camera feed with two intersecting lines so the person operating the aruco marker can see where the corner detection ends for the camera detection. The marker to wheel position operates in this way:
Top left = 0
Top right = pi/2
Bottom left = pi
Bottom right = 3pi/2
The raspberry pi sends which position to the arduino over i2c using the number associated with that position ('0' for 0 ,'1' for pi/2...). The final function of the raspberry pi is that it takes the set position set by the aruco marker and displays it on the LCD through the use of LCDini.py. This LCD is attached on top of the raspberry pi and is used to verify that the Raspberry pi is setting the right position and that the wheel matches this set position.
