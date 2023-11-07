/*
October 13, 2023
SEED LAB Team 9
Utilize readme for function
*/
#include <Encoder.h>
#include <Wire.h>

#define IS_CIRCLE_TIME false

// Physical Constants of the robot
#define BATTERY_VOLTAGE 8.2
#define ROBOT_DIAMETER_IN_CM 36.5 * 1.0125 // 1.0125 is determined empirically
#define WHEEL_DIAMETER_IN_CM 15.0

// I2C Bus config
#define MY_ADDR 8

// Tune to minimize slip
#define MAX_PWM 255

#define TARGET_ANGLE_IN_RADIANS PI
#define TARGET_DISTANCE_IN_FEET 1.0

#define ERROR_BAND_ANGLE 0.1 // Determined during testing
#define WAIT_CYCLES_ANGLE 100 // Number of cycles to wait while angle settles. Each cycle is desiredTsMs ms long.

// Rough Estimates - still require some fine tuning, but pretty decent steady state
#define KpANGLE 19.0 // NEEDS TUNING - Defines how fast it reaches the angle


#define KiLINEAR 0.6 // .4
#define KpLINEAR 0.8 // 3

#define VSUM_SAT_BAND 1.8
#define VDEL_SAT_BAND 1.6

#define KpPOS 0.010
#define KiPOS 0.00003

#define desiredTsMs 10 // Desired sampling time in ms -- previously was 10, may want to change back

#define SPIN_SPEED PI / 20.0
#define CIRCLE_LINEAR_SPEED 10.0 //Feet per second

// States!
enum States{
  SPIN,
  ANGLE_TO_MARK,
  CAM_ANGLE_TO_MARK,
  DRIVE_TO_MARK,
  TURN_90,
  DO_A_CIRCLE,
  STOP
};

double KdANGULAR_VELOCITY = 0.05; // NEEDS TUNING
double KpANGULAR_VELOCITY = 19.0; // NEEDS TUNING- Defines Agrressiveness at accelerating to desired velocity
double KiANGULAR_VELOCITY = 0.1; // NEEDS TUNING

double previousThetaRight = 0.0;
double previousThetaLeft = 0.0;

double previousAngularVelocityError = 0.0;

long targetPos = 0;
int angleErrorCount = 0;

float currentTime; // Current time in seconds
unsigned long lastTimeMs = 0; // Time at which last loop finished running in ms
unsigned long startTimeMs; // Time when the code starts running in ms

double velocityIntegral;
double posIntegral;

double angularVelocityError = 0;
double angularVelocityIntegral = 0;

// Continously updated from i2c when the pi sends updates
volatile float lastPiMeasuredDistance = -1.0;
volatile float lastPiMeasuredAngle = -1.0;

enum States state = DRIVE_TO_MARK; // Start in SPIN state

// Encoder setup
Encoder EncLeft(3, 6); // Encoder on left wheel is pins 3 and 6
Encoder EncRight(2, 5); // Encoder on right wheel is pins 2 and 5

double AngularVelocity_P(double targetAngle, double currentAngle) {
  double errorAngle = targetAngle - currentAngle;
  return KpANGLE * errorAngle; // COULD ADD I AND/OR D control
}

double VoltDelta_PID(double targetAngularVelocity, double currentAngularVelocity, double previousAngularVelocityError) {
  angularVelocityError = targetAngularVelocity - currentAngularVelocity;
  double angularVelocityDerivative = (angularVelocityError - previousAngularVelocityError) / desiredTsMs;
  angularVelocityIntegral = angularVelocityIntegral + ( double(desiredTsMs) / 1000.0 ) * angularVelocityError;

  return (KpANGULAR_VELOCITY * angularVelocityError) + (KiANGULAR_VELOCITY * angularVelocityIntegral) + (KdANGULAR_VELOCITY * angularVelocityDerivative);
}

double LinearVelocity_PI(double targetPos, double currentPos) {
  long errorPos = targetPos - currentPos;
  posIntegral = Sat_d(posIntegral + ( double(desiredTsMs) / 1000.0 ) * errorPos, -10000.0, 10000.0);

  return KpPOS * errorPos + KiPOS * posIntegral;  
}

double VoltSum_PI(double targetVelocity, double currentVelocity) {
  double velocityError = targetVelocity - currentVelocity;
  velocityIntegral = Sat_d(velocityIntegral +  ( double(desiredTsMs) / 1000.0 ) * velocityError, -12.0, 12.0);

  return KpLINEAR * velocityError + KiLINEAR * velocityIntegral;
}

double Sat_d(double x, double min, double max) {
  return (x < min) ? min : ((x > max) ? max : x);
}

double desiredPos;

void setup() {
  // Serial Setup - mostly used for debugging
  Serial.begin(115200);
  Serial.println("Ready!"); // For ReadfromArduino.mlx

  // Set up i2c communication from pi -> arduino
  Wire.begin(MY_ADDR);
  Wire.onReceive(receive);

  // Motor Pins
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);

  pinMode(7, OUTPUT); // Right H-Bridge
  pinMode(8, OUTPUT); // Left H-Bridge
  pinMode(9, OUTPUT); // Right power
  pinMode(10, OUTPUT); // Left power

  // Test Pins
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);

  // Timing
  lastTimeMs - millis();
  startTimeMs = lastTimeMs;

  while(millis() < 1000);
  desiredPos = (lastPiMeasuredDistance * 2070.0) / 12.0;
}

bool newDataFlag = false;

void loop() {
  currentTime = (float)(lastTimeMs - startTimeMs)/1000;

  digitalWrite(12, HIGH);
  long currentCountLeft = -EncLeft.read(); //Negative because this motor is facing "backwards"
  long currentCountRight = EncRight.read();
  digitalWrite(12, LOW);

  double thetaRight = (2.0 * PI * (double)currentCountRight) / 3200.0; // Right wheel position in radians
  double thetaLeft = (2.0 * PI * (double)currentCountLeft) / 3200.0; // Left wheel position in radians

  double thetaDotRight = (thetaRight - previousThetaRight) / desiredTsMs; // Right wheel angular velocity
  double thetaDotLeft = (thetaLeft - previousThetaLeft) / desiredTsMs; // Left wheel angular velocity

  double currentPos = (currentCountLeft + currentCountRight) / 2.0;
  double currentVelocity = WHEEL_DIAMETER_IN_CM / 2.0 * (thetaDotRight + thetaDotLeft) / 2.0; // Average speed of the wheels is linear velocity measured in cm

  double currentAngle = (WHEEL_DIAMETER_IN_CM / 2.0) * (thetaRight - thetaLeft) / ROBOT_DIAMETER_IN_CM; // Current heading of the robot related to where it started
  double currentAngularVelocity = (WHEEL_DIAMETER_IN_CM / 2.0) * (thetaDotRight - thetaDotLeft) / ROBOT_DIAMETER_IN_CM; // Current change of heading in radians per second

  double voltDelta, voltSum;

  double targetAngle;

  double desiredVelocity, desiredAngularVelocity, desiredRadius; // Later define desiredPos in here


  // State Diagram
  switch(state) {
    case SPIN: // Spin until we find the marker

      KdANGULAR_VELOCITY = 0.05;
      KpANGULAR_VELOCITY = 19.0;
      KiANGULAR_VELOCITY = 0.1;

      voltDelta = Sat_d(VoltDelta_PID(SPIN_SPEED, currentAngularVelocity, previousAngularVelocityError), -VDEL_SAT_BAND * BATTERY_VOLTAGE, VDEL_SAT_BAND * BATTERY_VOLTAGE );
      desiredVelocity = LinearVelocity_PI(0, currentPos);
      voltSum = Sat_d(VoltSum_PI(desiredVelocity, currentVelocity), -VSUM_SAT_BAND*BATTERY_VOLTAGE, VSUM_SAT_BAND*BATTERY_VOLTAGE);

      // go to next state?
      if (lastPiMeasuredAngle != -1) {
        state = ANGLE_TO_MARK;
        EncLeft.write(0);
        EncRight.write(0);
        currentPos = 0;
        currentAngle = 0;
        targetAngle = lastPiMeasuredAngle;
      }
      break;

    case ANGLE_TO_MARK:

      KdANGULAR_VELOCITY = 0.05;
      KpANGULAR_VELOCITY = 19.0;
      KiANGULAR_VELOCITY = 0.1;

      desiredAngularVelocity = AngularVelocity_P(targetAngle, currentAngle);
      voltDelta = Sat_d(VoltDelta_PID(desiredAngularVelocity, currentAngularVelocity, previousAngularVelocityError), -VDEL_SAT_BAND * BATTERY_VOLTAGE, VDEL_SAT_BAND * BATTERY_VOLTAGE );
      desiredVelocity = LinearVelocity_PI(0, currentPos);
      voltSum = Sat_d(VoltSum_PI(desiredVelocity, currentVelocity), -VSUM_SAT_BAND*BATTERY_VOLTAGE, VSUM_SAT_BAND*BATTERY_VOLTAGE);
      
      //go to next state?
      if (abs(currentAngle - targetAngle) < ERROR_BAND_ANGLE) {
        EncLeft.write(0);
        EncRight.write(0);
        // if (abs(lastPiMeasuredAngle) < ERROR_BAND_ANGLE) {
          state = DRIVE_TO_MARK;
          desiredPos = (lastPiMeasuredDistance * 2070.0) / 12.0;
          angularVelocityIntegral = 0;
          targetAngle = 0;
          currentAngle = 0;
        // } else {
          // targetAngle = lastPiMeasuredAngle;
        // }
      }
      
      break;

    // case CAM_ANGLE_TO_MARK:
    //   break;

    case DRIVE_TO_MARK:

      // desiredPos = 4 * 2070; // Testing Statement

      // lastPiMeasuredAngle = 48.0;

      // if (lastPiMeasuredDistance != -1 && ((lastPiMeasuredDistance * 2070.0) / 12.0) != desiredPos) {
      //   desiredPos = (lastPiMeasuredDistance * 2070.0) / 12.0;
      //   EncLeft.write(0);
      //   EncRight.write(0);
      // }

      KdANGULAR_VELOCITY = 0.05;
      KpANGULAR_VELOCITY = 19.0;
      KiANGULAR_VELOCITY = 0.1;


      desiredAngularVelocity = AngularVelocity_P(0.0, currentAngle);
      desiredVelocity = LinearVelocity_PI(desiredPos, currentPos);

      Serial.print(currentPos);
      Serial.print("\t");
      Serial.println(desiredPos);

      voltDelta = Sat_d(VoltDelta_PID(desiredAngularVelocity, currentAngularVelocity, previousAngularVelocityError), -VDEL_SAT_BAND * BATTERY_VOLTAGE, VDEL_SAT_BAND * BATTERY_VOLTAGE );
      voltSum = Sat_d(VoltSum_PI(desiredVelocity, currentVelocity), -VSUM_SAT_BAND*BATTERY_VOLTAGE, VSUM_SAT_BAND*BATTERY_VOLTAGE);
      
      // go to next state?
      if (abs(currentPos - desiredPos) < 2070) { //Are we within a foot?
        if (IS_CIRCLE_TIME) {
          state = DO_A_CIRCLE;
          EncLeft.write(0);
          EncRight.write(0);
          currentPos = 0;
          currentAngle = 0;
          desiredRadius = lastPiMeasuredDistance;
        } else {
          // state = STOP;
          EncLeft.write(0);
          EncRight.write(0);
          currentPos = 0;
          currentAngle = 0;
        }
      }
      break;

    case TURN_90:

      KdANGULAR_VELOCITY = 0.05;
      KpANGULAR_VELOCITY = 19.0;
      KiANGULAR_VELOCITY = 0.1;

      desiredAngularVelocity = AngularVelocity_P(-PI / 2.0, currentAngle);
      desiredVelocity = LinearVelocity_PI(0.0, currentPos);

      voltDelta = Sat_d(VoltDelta_PID(desiredAngularVelocity, currentAngularVelocity, previousAngularVelocityError), -VDEL_SAT_BAND * BATTERY_VOLTAGE, VDEL_SAT_BAND * BATTERY_VOLTAGE );
      voltSum = Sat_d(VoltSum_PI(desiredVelocity, currentVelocity), -VSUM_SAT_BAND*BATTERY_VOLTAGE, VSUM_SAT_BAND*BATTERY_VOLTAGE);

      // go to next state?
      if (abs(currentAngle + (PI/2.0) ) < ERROR_BAND_ANGLE) {
        state = DO_A_CIRCLE;
        angularVelocityIntegral = 0;
        EncLeft.write(0);
        EncRight.write(0);
        currentPos = 0;
        currentAngle = 0;
      }
      break;

    case DO_A_CIRCLE:

      // desiredRadius = 30.0; // Testing statments

      KdANGULAR_VELOCITY = 0.0;
      KpANGULAR_VELOCITY = 19.0;
      KiANGULAR_VELOCITY = 0.0;

      desiredVelocity = CIRCLE_LINEAR_SPEED;
      desiredAngularVelocity = desiredVelocity / desiredRadius;

      voltDelta = Sat_d(VoltDelta_PID(desiredAngularVelocity, currentAngularVelocity, 0), -VDEL_SAT_BAND * BATTERY_VOLTAGE, VDEL_SAT_BAND * BATTERY_VOLTAGE );
      voltSum = Sat_d(VoltSum_PI(desiredVelocity, currentVelocity), -VSUM_SAT_BAND*BATTERY_VOLTAGE, VSUM_SAT_BAND*BATTERY_VOLTAGE);

      if(abs(currentAngle) > 2.0 * PI) {
        state = STOP;
        EncLeft.write(0);
        EncRight.write(0);
        currentPos = 0;
        currentAngle = 0;
      }
      break;

    case STOP:

      KdANGULAR_VELOCITY = 0.05;
      KpANGULAR_VELOCITY = 19.0;
      KiANGULAR_VELOCITY = 0.1;

      desiredAngularVelocity = AngularVelocity_P(0, currentAngle);
      desiredVelocity = LinearVelocity_PI(0.0, currentPos);

      voltDelta = Sat_d(VoltDelta_PID(desiredAngularVelocity, currentAngularVelocity, previousAngularVelocityError), -VDEL_SAT_BAND * BATTERY_VOLTAGE, VDEL_SAT_BAND * BATTERY_VOLTAGE );
      voltSum = Sat_d(VoltSum_PI(desiredVelocity, currentVelocity), -VSUM_SAT_BAND*BATTERY_VOLTAGE, VSUM_SAT_BAND*BATTERY_VOLTAGE);
      break;
    
    }
  // Run Motors

  // Set voltages
  double voltageRight = (voltSum + voltDelta) / 2.0;
  double voltageLeft = (voltSum - voltDelta) / 2.0;

  // H-Bridge Direction
  if(voltageLeft > 0) {
    digitalWrite(8, HIGH);
  } else {
    digitalWrite(8, LOW);
  }

  if(voltageRight > 0) {
    digitalWrite(7, HIGH);
  } else {
    digitalWrite(7, LOW);
  }

  // Powering the motors
  int PWMLeft = min(MAX_PWM * abs(voltageLeft) / BATTERY_VOLTAGE, MAX_PWM);
  analogWrite(10, PWMLeft);

  int PWMRight = min(MAX_PWM * abs(voltageRight) / BATTERY_VOLTAGE, MAX_PWM);
  analogWrite(9, PWMRight);


  // Set all previous values
  previousThetaLeft = thetaLeft;
  previousThetaRight = thetaRight;
  previousAngularVelocityError = angularVelocityError;
  
  // Debugging Statements
  if (newDataFlag) {
    Serial.print(currentTime,3);
    Serial.print("\t");
    Serial.println(state);
    // Serial.print("\t");
    // Serial.print(currentPos);
    // Serial.print("\t");
    // Serial.print(currentCountLeft);
    // Serial.print("\t");
    // Serial.print(currentCountRight);
    // Serial.print("\t");
    // Serial.print(thetaLeft, 4);
    // Serial.print("\t");
    // Serial.print(thetaRight, 4);
    // Serial.print("\t");  
    // Serial.print(thetaDotLeft, 4);
    // Serial.print("\t");
    // Serial.print(thetaDotRight, 4);
    // Serial.print("\t");
    // Serial.print(currentAngularVelocity, 4);
    // Serial.print("\t");
    // Serial.println(currentAngle);
    // newDataFlag = false;
  }


  while(millis() < lastTimeMs + desiredTsMs);
  lastTimeMs = millis();
}



void receive(int nbytes) {
  // noInterrupts();

  digitalWrite(11, HIGH);
  char offset = Wire.read();

  // Read in 
  char in_data[4];
  double out_data;
  for(int i=0; i < nbytes-1; i++) {
    in_data[i] = Wire.read();
  }
  memcpy(&out_data, in_data, 4);

  // Offset dictates where the data goes
  switch (offset) {
    case 0:
      lastPiMeasuredDistance = out_data;
      break;
    case 1:
      lastPiMeasuredAngle = out_data;
      break;
  }
  digitalWrite(11, LOW);
  // interrupts();
  newDataFlag = true;
  
}
