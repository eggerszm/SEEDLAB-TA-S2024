#include <Wire.h>
#include <Encoder.h>
// #include <PID_v1.h>
#define MY_ADDR 8

#define KpLEFT 0.012
#define KiLEFT 0.00019
#define KpRIGHT 0.012
#define KiRIGHT 0.00019

#define BATTERY_VOLTAGE 8

#define TARGET_FEET 1.0

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

  long currentPosLeft = EncLeft.read();
  long currentPosB = EncRight.read();

  long targetPos = 2700.0 * TARGET_FEET; // 3200.0 / (15 * PI) * 30.48

  errorLeft = targetPos - currentPosLeft;
  errorRight = targetPos - currentPosRight;

  integralLeft = integralLeft + ( double(desiredTsMs) / 1000.0 ) * errorLeft;
  integralRight = integralRight + ( double(desiredTsMs) / 1000.0 ) * errorRight;

  voltageLeft = KpLEFT * errorA + KiLEFT * integralLeft;
  voltageRight = KpRIGHT * errorB + KiRIGHT * integralRight;

  // H-Bridge Direction
  if(voltageLeft > 0) {
    digitalWrite(7, HIGH);
  } else {
    digitalWrite(7, LOW);
  }

  if(voltageRight > 0) {
    digitalWrite(8, HIGH);
  } else {
    digitalWrite(8, Low);
  }

  int PWMLeft = 255 * abs(voltageLeft)/BATTERY_VOLTAGE;
  analogWrite(9, min(PWMLeft, 255));

  int PWMRight = 255 * abs(voltageRight)/BATTERY_VOLTAGE;
  analogWrite(10, min(PWMRight, 255));

  // double posRad = 2.0 * PI * (double)((double)currentPos / 3200.0);

  // Print Statements
  // if (currentTime < 10) { // Print for 10 seconds
  //   Serial.print(currentTime, 3);
  //   Serial.print("\t");
  //   Serial.print(min(voltage, BATTERY_VOLTAGE), 3);
  //   Serial.print("\t");
  //   // Serial.print(error);
  //   // Serial.print("\t");
  //   // Serial.print(integral, 5);
  //   // Serial.print("\t");
  //   // Serial.print(currentPos);
  //   // Serial.print("\t");
  //   Serial.println(posRad, 4);

  // } else if (!printFlag) {
  //   Serial.println("Finished");
  //   printFlag = 1;
  // }

  while(millis() < lastTimeMs + desiredTsMs);
  lastTimeMs = millis();
}

// void request(){
//   Wire.write(currentPos);
// }

// void receive(int arg){
//   while (Wire.available()) {
//     targetPos = (int)Wire.read() * 800; // Number of counts for target position
//   }
//   if ((targetPos - currentPos) > 1600) {
//     targetPos -= 3200;
//   } else if ((targetPos - currentPos) < -1600) {
//     targetPos += 3200;
//   }
// }