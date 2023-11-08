#include <Encoder.h>
#include <Wire.h>

// Define Constants
#define WHEEL_DIAMETER_IN_CM 15.0
#define ROBOT_DIAMETER_IN_CM 36.947125 // Determined empirically
#define DESIRED_TS_MS 10.0 

#define DESIRED_RADIUS 38.1

#define COUNTS_PER_FOOT 2070.0
#define COUNTS_PER_INCH 172.5

#define SPIN_RATE PI / 20.0
#define CIRCLE_LINEAR_SPEED 10.0 // cm/s

#define TURN_TO_MARKER_ERROR_BAND 0.01
#define ANGLE_ERROR_BAND 0.01

#define IS_CIRCLE_TIME true

// Define States
enum State {
  FIND_MARKER,
  TURN_TO_MARKER,
  DRIVE_TO_MARKER,
  TURN_RIGHT,
  DO_A_CIRCLE,
  STOP
};

// Define Globals
double lastTimeMs, startTimeMs;
State currState = FIND_MARKER;

double previousThetaLeft = 0.0, previousThetaRight = 0.0;

double integralValX = 0.0, integralValAngle = 0.0, integralValRho = 0.0, integralValOmega = 0.0;
double prevErrorX = 0.0, prevErrorAngle = 0.0, prevErrorRho = 0.0, prevErrorOmega = 0.0;

double lastPiMeasuredAngle, lastPiMeasuredDistance;
int settleCountTurnRight = 0;

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
}

void loop() {
  double currentTime = (lastTimeMs - startTimeMs) / 1000.0; // Current Time in seconds

  long currentCountLeft = -EncLeft.read(); // Negative because this motor is facing the other direction
  long currentCountRight = EncRight.read();

  double thetaRight = (2.0 * PI * (double)currentCountRight) / 3200.0; // Right wheel position in radians
  double thetaLeft = (2.0 * PI * (double)currentCountLeft) / 3200.0; // Left wheel position in radians

  double thetaDotRight = (thetaRight - previousThetaRight) / DESIRED_TS_MS; // Right wheel angular velocity
  double thetaDotLeft = (thetaLeft - previousThetaLeft) / DESIRED_TS_MS; // Left wheel angular velocity

  double currentPos = (currentCountLeft + currentCountRight) / 2.0;
  double currentRho = WHEEL_DIAMETER_IN_CM / 2.0 * (thetaDotRight + thetaDotLeft) / 2.0; // Average speed of the wheels is linear velocity measured in cm

  double currentAngle = (WHEEL_DIAMETER_IN_CM / 2.0) * (thetaRight - thetaLeft) / ROBOT_DIAMETER_IN_CM; // Current heading of the robot related to where it started
  double currentOmega = (WHEEL_DIAMETER_IN_CM / 2.0) * (thetaDotRight - thetaDotLeft) / ROBOT_DIAMETER_IN_CM; // Current change of heading in radians per second

  double kPOmega = 19.0;
  double kIOmega = 0.1;
  double kDOmega = 0.05;

  double kPRho = 0.8;
  double kIRho = 0.6;
  double kDRho = 0.0;

  double kPX = 0.01;
  double kIX = 0.00003;
  double kDX = 0.0;
  
  double kPAngle = 19.0;
  double kIAngle = 0.0;
  double kDAngle = 0.0;

  double desiredRho, desiredOmega;


  bool moveStateFlag = false;

  switch(currState) { // Tuning, transitions, targets (for desiredPos and desiredAngle)
    case FIND_MARKER:

      desiredPos = 0;

      if( !isnan(lastPiMeasuredAngle) ) {
        moveStateFlag = true;
        desiredAngle = lastPiMeasuredAngle;
        currState = TURN_TO_MARKER;
      }
      break;

    case TURN_TO_MARKER:

      if ( abs(currentAngle - desiredAngle) < TURN_TO_MARKER_ERROR_BAND ) {
        moveStateFlag = true;
        desiredPos = (lastPiMeasuredDistance * COUNTS_PER_INCH) - COUNTS_PER_FOOT;
        currState = DRIVE_TO_MARKER;
      }
      break;

    case DRIVE_TO_MARKER:

      desiredAngle = 0.0;

      if ( desiredPos - currentPos < 0 ) {
        moveStateFlag = true;
        if (IS_CIRCLE_TIME) {
          currState = TURN_RIGHT;
        } else {
          currState = STOP;
        }
      }
      break;

    case TURN_RIGHT:

      desiredAngle = -PI / 2.0;
      desiredPos = 0.0;

      if ( abs(currentAngle + (PI/2.0) ) < ANGLE_ERROR_BAND ) {
        settleCountTurnRight++;
      }

      if  ( settleCountTurnRight >= 50 ) {
        moveStateFlag = true;
        currState = DO_A_CIRCLE;
      }
      break;

    case DO_A_CIRCLE:

      if (currentAngle > 2.0 * PI) {
        moveStateFlag = true;
        currState = STOP;
      }
      break;

    case STOP:

      desiredPos = 0.0;
      desiredAngle = 0.0;
      break;
  }

  if( moveStateFlag ) {
    EncLeft.write(0);
    EncRight.write(0);
    integralValAngle = 0;
    integralValOmega = 0;
    integralValRho = 0;
    integralValX = 0;
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
      desiredRho = PID(desiredPos, currentPos, DESIRED_TS_MS, kPX, kIX, kDX, &integralValX, &prevErrorX);
      desiredOmega = PID(desiredAngle, currentAngle, DESIRED_TS_MS, kPAngle, kIAngle, kDAngle, &integralValAngle, &prevErrorAngle);
      break;

    case DO_A_CIRCLE: // omega, rho
      desiredRho = CIRCLE_LINEAR_SPEED;
      desiredOmega = CIRCLE_LINEAR_SPEED / DESIRED_RADIUS;
      break;
  }

  double voltSum = PID(desiredOmega, currentOmega, DESIRED_TS_MS, kPOmega, kIOmega, kDOmega, &integralValOmega, &prevErrorOmega);
  double voltDelta = PID(desiredRho, currentRho, DESIRED_TS_MS, kPRho, kIRho, kDRho, &integralValRho, &prevErrorRho);

  RunMotors(voltSum, voltDelta);

  previousThetaLeft = thetaLeft;
  previousThetaRight = thetaRight;

  Serial.print(voltSum);
  Serial.print("\t");
  Serial.println(voltDelta);


  while(millis() < lastTimeMs + DESIRED_TS_MS);
  lastTimeMs = millis();
}