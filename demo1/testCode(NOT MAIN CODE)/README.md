# Test Code
Note: This is NOT the main code for running the project. This is all code that was used to create the final product 

The `distanceDeg1.0.py` Is a possible soltution to measuring the angle from the camera to the aruco marker. 
The idea is that we calculate the  focul length of the camera and measure the width of the precieved aruco marker
we could measure the distance from the camera to the aruco marker using the equation ((focul length * width)/precieved width).
With the distance from the middle we can use inverse tangent of (xdistnace/distance) to get the angle. This code will be further
developed due to the ability to measure distance being useful

The `take20pictures` takes 20 pictures and was used to calibrate the camera

`images` contains the 20 images taken using the `take20picture.py`

`rotationalcontrol` contains `rotationalcontrol.INO` which is what we used to 