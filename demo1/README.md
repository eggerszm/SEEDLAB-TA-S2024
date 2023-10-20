# Demo 1

# Operating Software
To operate demo1, the user should run `MeasureAngle.py` to measure the angle between the camera and the aruco marker. `LCDInit.py` and `calib_data.npz`
need to be in the same folder as `MeasureAngle.py` to run `MeasureAngle.py`. DO NOT RUN `LCDInit.py`and `calib_data.npz`. unless you want to generate 
your own camera calibration data, DO NOT RUN `CalibrateCamera.py`.

To operate the movement of the robot in demo1, the user should run `AngularWIP`.

Files that should not be run:
-Anything in `testCode(NOT MAIN CODE)`
-`LCDInit.py`
-`CalibrateCamera.py`
-`calib_data.npz`
# code
The `MeasureAngle.py` should be ran when measuring the angle on the Raspberry Pi. This program uses `calib_data.npz` to calibrate the camera
and then measure the x,y cordinates of the aruco marker. If you do not want to use our data for calibration or are using a sepperate file, 
you can use `CalibrateCamera.py` to generate your own `calib_data.npz`. The program then uses these coordnitates to measure the angle
using the equation (0.084826 * float(xpos) - 27.218717). This equation was derived by taking data points of position of the aruco marker
and creating a line of best fit on excel. Once the degrees are calculated the program then uses `LCDInit.py` to then display the degrees
on the lcd screen.

The `LCDInit.py` module handles updating the LCD when the
message changes an minimizing display flicker when rapidly updating the 
displayed value.

The `CalibrateCamera.py` uses the camera to make calibrate data and save it under `calib_data.npz`. This data is then used in `MeasureAngle.py`
to calibrate the camera.
