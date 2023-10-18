/*
October 13, 2023
SEED LAB Team 9
Utilize readme for function
*/
#include <Encoder.h>

// Physical Constants of the robot
#define BATTERY_VOLTAGE 8.2
#define ROBOT_DIAMETER_IN_CM 36.5 * 1.01 // 1.01 is determined empirically
#define WHEEL_DIAMETER_IN_CM 15.0

// Tune to minimize slip
#define MAX_PWM 50

#define TARGET_ANGLE_IN_RADIANS PI
#define TARGET_DISTANCE_IN_FEET 3.0

#define ERROR_BAND_ANGLE 0.001
#define WAIT_CYCLES_ANGLE 100 // Number of cycles to wait while angle settles. Each cycle is desiredTsMs ms long.

// Rough Estimates - still require some fine tuning, but pretty decent steady state
#define KpANGLE 19.0 // NEEDS TUNING - Defines how fast it reaches the angle
#define KdANGULAR_VELOCITY 0.2 // NEEDS TUNING
#define KpANGULAR_VELOCITY 19.0 // NEEDS TUNING- Defines Agrressiveness at accelerating to desired velocity
#define KiANGULAR_VELOCITY 0.0 // NEEDS TUNING

#define KiLINEAR 0.4 // .4
#define KpLINEAR 0.8 // 3

#define VSUM_SAT_BAND 1.8

#define KpPOS 0.005
#define KiPOS 0.00004

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
  posIntegral = Sat_d(posIntegral + ( double(desiredTsMs) / 1000.0 ) * errorPos, -5000.0, 5000.0);

  return KpPOS * errorPos + KiPOS * posIntegral;  
}

double VoltSum_PI(double targetVelocity, double currentVelocity) {
  double velocityError = targetVelocity - currentVelocity;
  velocityIntegral = Sat_d(velocityIntegral +  ( double(desiredTsMs) / 1000.0 ) * velocityError, -6.0, 6.0);

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
  double targetAngle = TARGET_ANGLE_IN_RADIANS;

  // P control on robot angle to find desired angular velocity
  double desiredAngularVelocity = AngularVelocity_P(targetAngle, currentAngle); // COULD ADD I AND/OR D control

  // PD control on robot angle to find voltage
  double currentAngularVelocity = WHEEL_DIAMETER_IN_CM / 2.0 * (thetaDotRight - thetaDotLeft) / ROBOT_DIAMETER_IN_CM;
  double voltDelta = VoltDelta_PID(desiredAngularVelocity, currentAngularVelocity, previousAngularVelocityError);

  // Linear Control
  double voltSum = 0;

  // Set desiredVelocity using PI control after turning is complete
  if (abs(targetAngle - currentAngle) < ERROR_BAND_ANGLE || targetAngle == 0) { // Check if turning is within error band. For demo, set error band very low
    angleErrorCount++;
  }
  
  if (angleErrorCount == WAIT_CYCLES_ANGLE) {
    targetPos = 2070 * TARGET_DISTANCE_IN_FEET;
    // Serial.println("CHARGE!");
  }

  long currentPos = (currentCountLeft + currentCountRight) / 2;

  double desiredVelocity = LinearVelocity_PI(targetPos, currentPos);
  

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
  previousAngularVelocityError = angularVelocityError;
  
  // Debugging Statements

  if (currentTime < 10) {
    // Serial.print(currentTime, 3);
    // Serial.print("\t");
    // Serial.print(voltageLeft);
    // Serial.print("\t");
    // Serial.print(voltageRight);
    // Serial.print("\t");
    // Serial.print(angularVelocityIntegral);
    // Serial.print("\t");
    // Serial.print(voltDelta, 4);
    // Serial.print("\t");
    // Serial.print(voltSum, 3);
    // Serial.print("\t");
    // Serial.print(voltDelta, 3);
    // Serial.print("\t");
    // Serial.print(voltageLeft);
    // Serial.print("\t");
    // Serial.print(voltageRight);
    // Serial.print("\t");
    // Serial.print(currentCountLeft);
    // Serial.print("\t");
    // Serial.print(currentCountRight);
    // Serial.print("\t");
    // Serial.print(currentPos);
    // Serial.print("\t");
    // Serial.print(desiredAngularVelocity);
    // Serial.print("\t");
    // Serial.print(targetAngle, 5);
    // Serial.print("\t");
    // Serial.print(currentAngle, 5);
    // Serial.print("\t");
    // Serial.println(targetAngle - currentAngle, 5);
  }

  while(millis() < lastTimeMs + desiredTsMs);
  lastTimeMs = millis();
}

