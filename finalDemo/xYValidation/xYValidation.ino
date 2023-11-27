/*
Final Demo Starter
SEED Lab Group 9
Zander Eggers, Gideon Kukoyi, James Clark, Elijah Price
November 7th 2023

The purpose of this code is to complete demo 2 by doing the following items:
  Find aruco marker between 3 and 8 feet away,
  Drive up to within 1 foot of it,
  Do a circle around it when requested

This is done through PID control, utilizing a similar control structure to demo 1 in which linear voltage and
angular voltage are controlled seperately and then added or subtracted to drive each motor independently

This code will not be updated further, though know it is certainly not "robust"

To Use:
  Set robot parameters
  Determine if you want to do a circle
    Set IS_CIRCLE_TIME
*/

#include <Encoder.h>
#include <Wire.h>

// Define Constants
#define WHEEL_DIAMETER_IN_CM 15.0
#define ROBOT_DIAMETER_IN_CM 36.947125 // Determined empirically
#define DESIRED_TS_MS 10.0 

#define DESIRED_RADIUS 6.0

#define COUNTS_PER_FOOT 2070.0
#define COUNTS_PER_INCH 172.5

#define SPIN_RATE PI / 20.0 // Ensure this value is not too fast to allow camera to see marker
#define CIRCLE_LINEAR_SPEED 10.0 // cm/s

#define TURN_TO_MARKER_ERROR_BAND 0.01
#define ANGLE_ERROR_BAND 0.01

#define IS_CIRCLE_TIME true

// Define States
enum State {
  FIND_MARKER, //0
  TURN_TO_MARKER, //1
  ANGLE_CHECK, //2
  DRIVE_TO_MARKER, //3
  TURN_RIGHT, //4
  DO_A_CIRCLE, //5
  START_TEST, //6
  TURN_TO_XY, //7
  DRIVE_TO_XY, //8
  STOP //9
};

// Testing Array -- Values given in counts to make a regular hexagon of "radius" 1 ft
double xyMatrix[7][2] = { {COUNTS_PER_FOOT, COUNTS_PER_FOOT},
                          {COUNTS_PER_FOOT / 2.0, 0.866025 * COUNTS_PER_FOOT},
                          {-COUNTS_PER_FOOT / 2.0, 0.866025 * COUNTS_PER_FOOT},
                          {-COUNTS_PER_FOOT, 0.0},
                          {-COUNTS_PER_FOOT / 2.0, -0.866025 * COUNTS_PER_FOOT},
                          {COUNTS_PER_FOOT / 2.0, -0.866025 * COUNTS_PER_FOOT},
                          {COUNTS_PER_FOOT, 0.0} };

// Define Globals
double lastTimeMs, startTimeMs;
State currState = TURN_TO_XY;
double previousThetaLeft = 0.0, previousThetaRight = 0.0;
double currX = 0.0, currY = 0.0;
double prevPos = 0.0;
unsigned int xYIndex = 0;

double integralValX = 0.0, integralValAngle = 0.0, integralValRho = 0.0, integralValOmega = 0.0;
double prevErrorX = 0.0, prevErrorAngle = 0.0, prevErrorRho = 0.0, prevErrorOmega = 0.0;

double lastPiMeasuredAngle = NAN, lastPiMeasuredDistance = NAN;
int settleCountTurnRight = 0;
int settleCountTurnMark = 0;
int angleCheckCount = 0;

double desiredPos = 0.0, desiredAngle = 0.0;

// Initialize Encoders
Encoder EncLeft(3, 6); // Encoder on left wheel is pins 3 and 6 -- Could use #defines for this later
Encoder EncRight(2, 5); // Encoder on right wheel is pins 2 and 5


void setup() {
  Serial.begin(115200);
  Serial.println("Ready!"); // For ReadfromArduino.mlx

  SetupMotors();
  SetupI2C();

  // Timing
  lastTimeMs - millis();
  startTimeMs = lastTimeMs;

  desiredAngle = DesiredAngleXY(currX, currY, 4 * COUNTS_PER_FOOT, 5* COUNTS_PER_FOOT);
}



void loop() {
  double currentTime = (lastTimeMs - startTimeMs) / 1000.0; // Current Time in seconds

  long currentCountLeft = -EncLeft.read(); // Negative because this motor is facing the other direction
  long currentCountRight = EncRight.read();

  double thetaRight = (2.0 * PI * (double)currentCountRight) / 3200.0; // Right wheel position in radians
  double thetaLeft = (2.0 * PI * (double)currentCountLeft) / 3200.0; // Left wheel position in radians

  double thetaDotRight = (thetaRight - previousThetaRight) / DESIRED_TS_MS; // Right wheel angular velocity in rad/ms
  double thetaDotLeft = (thetaLeft - previousThetaLeft) / DESIRED_TS_MS; // Left wheel angular velocity in rad/ms

  double currentPos = (currentCountLeft + currentCountRight) / 2.0; // Distance traveled in counts
  double currentRho = WHEEL_DIAMETER_IN_CM / 2.0 * (thetaDotRight + thetaDotLeft) / 2.0; // Average speed of the wheels is linear velocity measured in cm / ms

  double currentAngle = (WHEEL_DIAMETER_IN_CM / 2.0) * (thetaRight - thetaLeft) / ROBOT_DIAMETER_IN_CM; // Current heading of the robot related to where it started
  double currentOmega = (WHEEL_DIAMETER_IN_CM / 2.0) * (thetaDotRight - thetaDotLeft) / ROBOT_DIAMETER_IN_CM; // Current change of heading in radians per second

  double kPOmega = 19.0;
  double kIOmega = 0.1;
  double kDOmega = 0.05;

  double kPRho = 0.1;
  double kIRho = 0.05;
  double kDRho = 0.0;

  double kPX = 0.01;
  double kIX = 0.00003;
  double kDX = 0.0;
  
  double kPAngle = 19.0;
  double kIAngle = 0.0;
  double kDAngle = 0.0;

  double desiredRho, desiredOmega;

  currX = xValue(currentAngle, currentPos - prevPos, currX);
  currY = yValue(currentAngle, currentPos - prevPos, currY);

  bool moveStateFlag = false;

  //Go To (4 ft, 5 ft)

  switch(currState) {
    case TURN_TO_XY:

        desiredPos = 0.0;

        kPRho = 0.8;
        kIRho = 0.6;

        if ( abs(currentAngle - desiredAngle ) < TURN_TO_MARKER_ERROR_BAND ) { // Counts if within errorband to allow for settling time
          settleCountTurnMark++;
        }
        if  ( settleCountTurnMark >= 50 ) {
          moveStateFlag = true;
          desiredPos = DesiredDistanceXY(currX, currY, 4 * COUNTS_PER_FOOT, 5 * COUNTS_PER_FOOT) + currentPos; // Set desired position to the distance from the aruco marker minus 1 foot
          currState = DRIVE_TO_XY;
        }
        break;
    case DRIVE_TO_XY:
        // desiredAngle = 0.0;

        if((desiredPos - currentPos) < 0 ) {
          moveStateFlag = true;
          xYIndex++;
          currState = START_TEST;
          if(xYIndex == 8) currState = STOP;
        }
        break;
  }

  if( moveStateFlag ) { // When moving states, reset any counters, integral values, and encoders
    integralValAngle = 0;
    integralValOmega = 0;
    integralValRho = 0;
    integralValX = 0;
    angleCheckCount = 0;
    settleCountTurnMark = 0;
  }

  switch(currState) { // Fall thru to run controllers and find desiredRho, desiredOmega
    case FIND_MARKER: //omega, x, rho
      desiredRho = PID(desiredPos, currentPos, DESIRED_TS_MS, kPX, kIX, kDX, &integralValX, &prevErrorX);
      desiredOmega = SPIN_RATE;
      break;

    case TURN_TO_MARKER: //theta, omega, x, rho
    case DRIVE_TO_MARKER:
    case TURN_RIGHT:
    case STOP:
    case START_TEST:
    case TURN_TO_XY:
    case DRIVE_TO_XY:
      desiredRho = PID(desiredPos, currentPos, DESIRED_TS_MS, kPX, kIX, kDX, &integralValX, &prevErrorX);
      desiredOmega = PID(desiredAngle, currentAngle, DESIRED_TS_MS, kPAngle, kIAngle, kDAngle, &integralValAngle, &prevErrorAngle);
      break;

    case DO_A_CIRCLE: // omega, rho
    case ANGLE_CHECK:
      desiredRho = CIRCLE_LINEAR_SPEED;
      desiredOmega = CIRCLE_LINEAR_SPEED / DESIRED_RADIUS;
      break;
  }

  double voltDelta = PID(desiredOmega, currentOmega, DESIRED_TS_MS, kPOmega, kIOmega, kDOmega, &integralValOmega, &prevErrorOmega);
  double voltSum = PID(desiredRho, currentRho, DESIRED_TS_MS, kPRho, kIRho, kDRho, &integralValRho, &prevErrorRho);

  RunMotors(voltSum, voltDelta);

  previousThetaLeft = thetaLeft;
  previousThetaRight = thetaRight;
  prevPos = currentPos;

  //Debug Statements
  Serial.print(currentTime);Serial.print("\t");
  Serial.print(currState);Serial.print("\t");

  Serial.print(currentAngle, 4);Serial.print("\t");
  Serial.print(desiredAngle, 4);Serial.print("\t");

  Serial.println();
  


  while(millis() < lastTimeMs + DESIRED_TS_MS); // Wait til end of desired sample time
  lastTimeMs = millis();
}