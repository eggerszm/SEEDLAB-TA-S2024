# Demo 2

# Operating Software
To operate demo 2 the user should run `demoSendDist.py` on the raspberry pi and `Demo2Rework` for the arduino. Inside the `Demo2Rework`
there are several other files that do not need to be run but need to be present to run this code. 



Files that should not be run:
-Anything in `testCode(NOT MAIN CODE)`
-`calib_data.npz`
-`I2CFuncions`
-`Motors`
-`PID_Control`
# code



The `Demo1Rework` file facilitates the operation of the robot to locate the Aruco marker, execute a 90-degree turn, and perform a circular rotation if prompted by the user. This software employs multiple switch statements that modify the robot's functionality. 
The FIND_MARKER case initiates the robot to rotate 2*pi until it detects the Aruco marker. 
Once detected, the TURN_TO_MARKER case locks the robot onto the Aruco marker. 
The DRIVE_TO_MARKER case enables the robot to proceed towards the Aruco marker until it reaches its desired distance ( < 1 ft). 
The TURN_RIGHT case allows the robot to rotate pi/2 radians to the right of the Aruco marker. 
The DO_A_CIRCLE case prompts the robot to perform a full 2 pi rotation around the Armco. 
The STOP case terminates the robot's operation.

The `demoSendDist.py` file contains the operation of the raspberry pi. The main purpose of the raspberry pi is to use the camera and opencv to detect the aruco marker. It then places a contour around the marker and gathers information as well as get the distance from the camera to the marker. The second purpose is that it takes the distance and the width from the contour to find the angle from the camera to the robot. The final purpose of this code is to use i2c to communicate to the arduino with the main purpose of sending the angle and the distance from the marker. The information sent is then used by the arduino to align the robot with the marker and to travel the specified distance and from that point the arduino cna do the rest.


