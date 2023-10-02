# Mini Project
The `DetectMarkers.py` script is responsible for detecting markers and sending
the detected marker positions to the Arduino. This script uses openCV's built-in
`aruco.detectMarkers` function on greyscale versions of each frame taken by the
camera. This script also shows a live feed on the pi's desktop that shows the
quadrants and the centers of each detected marker to allow for easy debugging.

The marker positions map to wheel positions as follows:
- Top left (NW) = 0
- Top right (NE) = pi/2
- Bottom left (SW) = pi
- Bottom right (SE) = 3pi/2

The pi sends a byte representing the current target position to the arduino 
whenver the target changes. The value of these bytes correspond to the ASCII
characters 0, 1, 2, and 3. The pi also displays the current target position
on the connected LCD. The `LCDInit.py` module handles updating the LCD when the
message changes an minimizing display flicker when rapidly updating the 
displayed value.
