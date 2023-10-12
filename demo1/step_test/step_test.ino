/*
This code is deeply flawed. Will fix at a later date
*/
#include <Wire.h>
#include <Encoder.h>
// #include <PID_v1.h>
#define MY_ADDR 8

#define KpLEFT 0.012
#define KiLEFT 0.00019
#define KpRIGHT 0.012
#define KiRIGHT 0.00019

#define BATTERY_VOLTAGE 8

#define TARGET_FEET 10.0

#define MAX_PWM 100


/* This needs to be replaced by a feedback loop for angle

Currently, simple fudge factor to keep it relatively "straight"
However, this is eyeballed and needs to be fixed.
It should change over time as the angle gets out of hand
Current idea to fix this:
angleError = rightCount - leftCount
This idea may be flawed:
Instead ensure that ve
Implement feedback loop with desired = 0
Can later implement transfer function to find actual angle
    This will require math I don't want to do right now
    Has something to do with radius of the robot and pi
This would allow us to add desired angles as required for the demo.

Will work on this tomorrow (maybe with Gideon)
*/
#define LEFT_TOO_FAST 2


volatile uint8_t offset = 0;
int req_message = 1;

long int targetPos = 0;
long int currentPos = 0;

int errorLeft, errorRight;
double integralLeft, integralRight;
double voltageLeft, voltageRight;
unsigned long Ts = 0, Tc = 0;
unsigned long lastTimeMs = 0, desiredTsMs = 10, startTimeMs;

float currentTime;
bool printFlag = 0;

Encoder EncLeft(3,6); //Encoder is on pins 3 and 6
Encoder EncRight(2,5); //Encoder B is on pins 2 and 5

void setup() {
  // Wire.begin(MY_ADDR);
  // //recieves the information from the wire
  // Wire.onReceive(receive);
  // Wire.onRequest(request);
  //sets the serial so we can recieve characters
  Serial.begin(115200);
  Serial.println("Ready!"); // For ReadfromArduino.mlx

  //Pins for motor
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

  long currentPosLeft = -EncLeft.read();
  long currentPosRight = EncRight.read();

  long targetPos = 2070.0 * TARGET_FEET; // 3200.0 / (15.0 * PI) * 30.48

  errorLeft = targetPos - currentPosLeft;
  errorRight = targetPos - currentPosRight;

  // if (abs(errorLeft) < DEADBAND_SIZE) {
  //   integralLeft = 0;
  // } else {
    integralLeft = integralLeft + ( double(desiredTsMs) / 1000.0 ) * errorLeft;
  //}

  // if (abs(errorRight) < DEADBAND_SIZE) {
  //  integralRight = 0;
  // } else {
    integralRight = integralRight + ( double(desiredTsMs) / 1000.0 ) * errorRight;
  // }

  voltageLeft = KpLEFT * errorLeft + KiLEFT * integralLeft;
  voltageRight = KpRIGHT * errorRight + KiRIGHT * integralRight;

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

  int PWMLeft = 255 * abs(voltageLeft)/BATTERY_VOLTAGE;
  analogWrite(9, max(min(PWMLeft, MAX_PWM) - LEFT_TOO_FAST, 0) );

  int PWMRight = 255 * abs(voltageRight)/BATTERY_VOLTAGE;
  analogWrite(10, min(PWMRight, MAX_PWM));

  // Print Statements

  if (currentTime < 10) { // Print for 10 seconds
    Serial.print(currentTime, 3);
    Serial.print("\t");
    Serial.print(targetPos);
    Serial.print("\t");
    Serial.print(voltageRight, 3);
    Serial.print("\t");
    Serial.print(voltageLeft, 3);
    Serial.print("\t");
    Serial.print(currentPosRight);
    Serial.print("\t");
    Serial.println(currentPosLeft);

  } else if (!printFlag) {
    Serial.println("Finished");
    printFlag = 1;
  }

  while(millis() < lastTimeMs + desiredTsMs);
  lastTimeMs = millis();
}
