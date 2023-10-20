# SEED Lab Team 9 Repository
The purpose of this repository is to allow the entire team to share code and documentation. For the Computer Vision and Communication specialization, this allowed one person to focus on communication between the Pi and the Arduion while the other person worked on the Aruco marker detector. For the Localization and Control specialization, this allowed for both team members to work collaboratively on the code for the controller. Most importantly, it allowed both specialization teams to share documentation with each other, making the final integration much simpler. This repository is broken into separate directories for each of the projects. 

## Mini Project
All code and documentation of the mini project is located in the `mini_project` directory
- `DetectMarkers.py`: Contains code for the Raspberry pi that detects the aruco marker and sends data to the arduino
- `LCDinit.py`: Abstractions for easy interfacing with the LCD display on the raspberry pi
- `README.md`: Project-specific overview and documentation
- `rdwr_test`: Final arduino code which takes data from the pi and sets wheel position
- `step_test`: Testing script for system characterization, records data from the step response of the motor

# Demo 1
All code and documentation of demo 1 is located in the `demo1` directory
- `MeasureAngle.py`: Contains code for the Raspberry pi that measures the angle between the aruco marker and the camera. (run this to run demo1)
- `LCDinit.py`: Abstractions for easy interfacing with the LCD display on the raspberry pi
- `README.md`: Project-specific overview and documentation
- `Angular`: Final arduino code which goes a specified distance and stops. It also rotates the robot a specified amount (run this to run demo1)
- `testCode(NOT MAIN CODE)`: folder that contains all of our prototyping for the project as well as a readme to explain what we were prototyping.
- `CalibrateCamera.py`: Abstraction that calibrates the camera.
- `calib_data.npz`: data used to calibrate the camera