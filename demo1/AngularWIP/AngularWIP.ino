/*
October 13, 2023
SEED LAB Team 9
Utilize readme for function
*/
#include <Encoder.h>

// Physical Constants of the robot
#define BATTERY_VOLTAGE 8.0
#define ROBOT_DIAMETER_IN_CM 36.5
#define WHEEL_DIAMETER_IN_CM 15.0

// Tune to minimize slip
#define MAX_PWM 100

#define TARGET_ANGLE_IN_RADIANS 0
#define TARGET_DISTANCE_IN_FEET 1.0

#define ERROR_BAND_COUNTS 10

// Rough Estimates - still require some fine tuning, but pretty decent steady state
#define KpANGLE 5 // NEEDS TUNING - Defines how fast it reaches the angle
#define KdANGULAR_VELOCITY 0.1 // NEEDS TUNING
#define KpANGULAR_VELOCITY 10 // NEEDS TUNING- Defines Agrressiveness at accelerating to desired velocity

#define KiLINEAR 0.05
#define KpLINEAR 3.4

#define KpPOS 0.009
#define KiPOS 0.00003



double previousThetaRight = 0.0;
double previousThetaLeft = 0.0;

double previousAngularVelocityError = 0.0;

float currentTime; // Current time in seconds
unsigned long lastTimeMs = 0; // Time at which last loop finished running in ms
unsigned long startTimeMs; // Time when the code starts running in ms
unsigned long desiredTsMs = 5; // Desired sampling time in ms -- previously was 10, may want to change back

double velocityIntegral;
double posIntegral;

// Encoder setup
Encoder EncLeft(3, 6); // Encoder on left wheel is pins 3 and 6
Encoder EncRight(2, 5); // Encoder on right wheel is pins 2 and 5

void setup() {
  // Serial Setup - mostly used for debugging
  Serial.begin(115200);
  Serial.println("Ready!"); // For ReadfromArduino.mlx

  // Motor Pins
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);

  pinMode(7, OUTPUT); // Left H-Bridge
  pinMode(8, OUTPUT); // Right H-Bridge
  pinMode(9, OUTPUT); // Left power
  pinMode(10, OUTPUT); // Right power

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

  // Angular Control

  double currentAngle = WHEEL_DIAMETER_IN_CM / 2.0 * (thetaRight - thetaLeft) / ROBOT_DIAMETER_IN_CM; // Current heading of the robot related to where it started

  // P control on robot angle to find desired angular velocity
  double targetAngle = TARGET_ANGLE_IN_RADIANS;
  double errorAngle = targetAngle - currentAngle;

  double desiredAngularVelocity = KpANGLE * errorAngle; // COULD ADD I AND/OR D control

  // PD control on robot angle to find voltage
  double currentAngularVelocity = WHEEL_DIAMETER_IN_CM / 2.0 * (thetaDotRight - thetaDotLeft) / ROBOT_DIAMETER_IN_CM;
  double angularVelocityError = desiredAngularVelocity - currentAngularVelocity;

  double angularVelocityDerivative = (angularVelocityError - previousAngularVelocityError) / desiredTsMs;
  previousAngularVelocityError = angularVelocityError;

  double voltDelta = KpANGULAR_VELOCITY * angularVelocityError + KdANGULAR_VELOCITY * angularVelocityDerivative;


  // Linear Control

  // Set desiredVelocity using PI control
  long targetPos = 2070 * TARGET_DISTANCE_IN_FEET;
  long currentPos = (currentCountLeft + currentCountRight) / 2;
  long errorPos = targetPos - currentPos;

  posIntegral = posIntegral + ( double(desiredTsMs) / 1000.0 ) * errorPos;
  long desiredVelocity = KpPOS * errorPos + KiPOS * posIntegral;
  

  // PI control for linear velocity to find voltage
  double currentVelocity = WHEEL_DIAMETER_IN_CM / 2.0 * (thetaDotRight + thetaDotLeft) / 2.0; // Average speed of the wheels is linear velocity measured in cm
  double velocityError = desiredVelocity - currentVelocity;

  velocityIntegral = velocityIntegral +  ( double(desiredTsMs) / 1000.0 ) * velocityError;

  double voltSum = KpLINEAR * velocityError + KiLINEAR * velocityIntegral;


  // Run Motors

  // Set voltages
  double voltageRight = (voltSum + voltDelta) / 2.0;
  double voltageLeft = (voltSum - voltDelta) / 2.0;

  // H-Bridge Direction
  if(voltageLeft > 0) {
    digitalWrite(7, HIGH);
  } else {
    digitalWrite(7, LOW);
  }

  if(voltageRight > 0) {
    digitalWrite(8, HIGH);
  } else {
    digitalWrite(8, LOW);
  }

  // Powering the motors
  int PWMLeft = 255 * abs(voltageLeft) / BATTERY_VOLTAGE;
  analogWrite(9, min(PWMLeft, MAX_PWM));

  int PWMRight = 255 * abs(voltageRight) / BATTERY_VOLTAGE;
  analogWrite(10, min(PWMRight, MAX_PWM));


  // Set all previous values
  previousThetaLeft = thetaLeft;
  previousThetaRight = thetaRight;

  
  // Debugging Statements
  if (currentTime < 5) {
    Serial.print(currentTime, 3);
    Serial.print("\t");
    Serial.print(voltSum, 3);
    Serial.print("\t");
    Serial.print(voltDelta, 3);
    Serial.print("\t");
    Serial.print(desiredVelocity);
    Serial.print("\t");
    Serial.print(voltageLeft);
    Serial.print("\t");
    Serial.print(voltageRight);
    Serial.print("\t");
    Serial.print(currentCountLeft);
    Serial.print("\t");
    Serial.print(currentCountRight);
    Serial.print("\t");
    Serial.print(currentPos);
    Serial.print("\t");
    Serial.println(errorPos);
  }

  while(millis() < lastTimeMs + desiredTsMs);
  lastTimeMs = millis();
}

