# SEED Lab Team 9 Repository
The purpose of this repository  was to allow for several people work on the code to build the final product of the robot. ON the python end this allowed us to build the communication between the Pi and the Arduion while the other person worked on the Aruco detector under the same document. ON the arduino end it helped with setting up the correct controller and taking the set wheel position which is then used to place that wheel in said position. Most importantly, it allowed computer vision and localization to work on eachothers documents as well as made it easier to glue together all of our code for the final product. The organization of the repository is as follows:
## Mini Project
Code and documentation of the Raspberry pi for the mini project is located in the mini_project folder
    -DetectMarkers.py: Contains code for the Raspberry.py that detects the aruco marker and sets position
    -LCDinit.py: Contains code for the Raspberry.py that displays set position on the lcd
    -README
Code and documentation of the arduino is located in the rdwr_test folder
    -rdwr_test: COntains code for the Arduino that control the wheel
    -ReadME
Note that the folder for step_test contains code that was used for testing the robot and feedback system. This folder does NOT contain the final code.
