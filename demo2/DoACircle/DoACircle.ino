/*
October 13, 2023
SEED LAB Team 9
Utilize readme for function
*/
#include <Encoder.h>

// Physical Constants of the robot
#define BATTERY_VOLTAGE 8.2
#define ROBOT_DIAMETER_IN_CM 36.5 * 1.0125 // 1.0125 is determined empirically
#define WHEEL_DIAMETER_IN_CM 15.0

// Parameters of the circle we want to do (in feet)
#define CIRCLE_RADIUS 100.0

#define LINEAR_SPEED 10.0 // Feet per second

// Tune to minimize slip or go fast
#define MAX_PWM 255

// Rough Estimates - still require some fine tuning, but pretty decent steady state
#define KpANGLE 19.0 // NEEDS TUNING - Defines how fast it reaches the angle
#define KdANGULAR_VELOCITY 0 // NEEDS TUNING
#define KpANGULAR_VELOCITY 19.0 // NEEDS TUNING- Defines Agrressiveness at accelerating to desired velocity
#define KiANGULAR_VELOCITY 0 // NEEDS TUNING - 0.1

#define KiLINEAR 0.0 // 0.6
#define KpLINEAR 3.0 // 3

#define VSUM_SAT_BAND 1.8
#define VDEL_SAT_BAND 1.6

#define desiredTsMs 5 // Desired sampling time in ms -- previously was 10, may want to change back

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

// Encoder setup
Encoder EncLeft(3, 6); // Encoder on left wheel is pins 3 and 6
Encoder EncRight(2, 5); // Encoder on right wheel is pins 2 and 5

double VoltDelta_PID(double targetAngularVelocity, double currentAngularVelocity, double previousAngularVelocityError) {
  angularVelocityError = targetAngularVelocity - currentAngularVelocity;
  double angularVelocityDerivative = (angularVelocityError - previousAngularVelocityError) / desiredTsMs;
  angularVelocityIntegral = angularVelocityIntegral + ( double(desiredTsMs) / 1000.0 ) * angularVelocityError;

  return (KpANGULAR_VELOCITY * angularVelocityError) + (KiANGULAR_VELOCITY * angularVelocityIntegral) + (KdANGULAR_VELOCITY * angularVelocityDerivative);
}

double VoltSum_PI(double targetVelocity, double currentVelocity) {
  double velocityError = targetVelocity - currentVelocity;
  velocityIntegral = Sat_d(velocityIntegral +  ( double(desiredTsMs) / 1000.0 ) * velocityError, -12.0, 12.0);

  return KpLINEAR * velocityError + KiLINEAR * velocityIntegral;
}

double Sat_d(double x, double min, double max) {
  return (x < min) ? min : ((x > max) ? max : x);
}

void setup() {
  // Serial Setup - mostly used for debugging
  Serial.begin(115200);
  Serial.println("Ready!"); // For ReadfromArduino.mlx

  // Motor Pins
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);

  pinMode(7, OUTPUT); // Right H-Bridge
  pinMode(8, OUTPUT); // Left H-Bridge
  pinMode(9, OUTPUT); // Right power
  pinMode(10, OUTPUT); // Left power

  // Timing
  lastTimeMs - millis();
  startTimeMs = lastTimeMs;

}

void loop() {
  currentTime = (float)(lastTimeMs - startTimeMs)/1000;

  long currentCountLeft = -EncLeft.read(); //Negative because this motor is facing "backwards"
  long currentCountRight = EncRight.read();

  double thetaRight = 2.0 * PI * (double)currentCountRight / 3200.0; // Right wheel position in radians
  double thetaLeft = 2.0 * PI* (double)currentCountLeft / 3200.0; // Left wheel position in radians

  double thetaDotRight = (thetaRight - previousThetaRight) / desiredTsMs; // Right wheel angular velocity
  double thetaDotLeft = (thetaLeft - previousThetaLeft) / desiredTsMs; // Left wheel angular velocity

  long currentPos = (currentCountLeft + currentCountRight) / 2; 
  // If we haven't completed the circle, set target velocities

  double currentAngle = WHEEL_DIAMETER_IN_CM / 2.0 * (thetaRight - thetaLeft) / ROBOT_DIAMETER_IN_CM;

  double desiredAngularVelocity;
  double desiredVelocity;
  if(abs(currentAngle) < 2.0 * PI) {
    // This math has been triple checked. It is correct. (Hopefully)
    desiredVelocity = LINEAR_SPEED;
    desiredAngularVelocity = desiredVelocity / CIRCLE_RADIUS;
  } else {
    desiredVelocity = 0;
    desiredAngularVelocity = 0;
  }

  // Angular Control

  // PD control on robot angle to find voltage
  double currentAngularVelocity = WHEEL_DIAMETER_IN_CM / 2.0 * (thetaDotRight - thetaDotLeft) / ROBOT_DIAMETER_IN_CM;
  double voltDelta = Sat_d(VoltDelta_PID(desiredAngularVelocity, currentAngularVelocity, previousAngularVelocityError), -VDEL_SAT_BAND * BATTERY_VOLTAGE, VDEL_SAT_BAND * BATTERY_VOLTAGE );

  // Linear Control
  double voltSum = 0;
 

  // PI control for linear velocity to find voltage
  double currentVelocity = WHEEL_DIAMETER_IN_CM / 2.0 * (thetaDotRight + thetaDotLeft) / 2.0; // Average speed of the wheels is linear velocity measured in cm

  voltSum = VoltSum_PI(desiredVelocity, currentVelocity);
  
  // voltSum shouldn't exceed 1.9x battery voltage to allow room for turning control
  voltSum = Sat_d(voltSum, -VSUM_SAT_BAND*BATTERY_VOLTAGE, VSUM_SAT_BAND*BATTERY_VOLTAGE);

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

  if (currentTime < 20) {
  }


  while(millis() < lastTimeMs + desiredTsMs);
  lastTimeMs = millis();
}

