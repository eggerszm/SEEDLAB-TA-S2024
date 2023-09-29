#include <Wire.h>
#include <Encoder.h>
// #include <PID_v1.h>
#define MY_ADDR 8

#define Kp 1.5
#define Ki 0.1
#define BATTERY_VOLTAGE 8.2

volatile uint8_t offset = 0;
int req_message = 1;

long int targetPos = 0;
long int currentPos = 0;

int error, voltage, integral;
unsigned long Ts = 0, Tc = 0;
unsigned long lastTimeMs = 0, desiredTsMs = 10;

Encoder EncA(3,6); //Encoder is on pins 3 and 6

void setup() {
  Wire.begin(MY_ADDR);
  //recieves the information from the wire
  Wire.onReceive(receive);
  Wire.onRequest(request);
  //sets the serial so we can recieve characters
  Serial.begin(9600);

  //Pins for motor
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);

  pinMode(7, OUTPUT); // Left H-Bridge
  pinMode(8, OUTPUT); // Right H-Bridge
  pinMode(9, OUTPUT); // Left power
  pinMode(10, OUTPUT); // Right power

}

void loop() {
  long currentPos = EncA.read();

  error = targetPos - currentPos;
  integral = integral + desiredTsMs * error;

  voltage = Kp * error + Ki * integral;

  // H-Bridge Direction
  if(voltage > 0) {
    digitalWrite(8, HIGH);
  } else {
    digitalWrite(8, LOW)
  }

  int PWM = 255 * abs(voltage)/BATTERY_VOLTAGE;
  analogWrite(10, min(PWM, 255));

  while(millis() < lastTimeMs + desiredTsMs);
  lastTimeMs = millis();
}

void request(){
  Wire.write(currentPos);
}

void receive(int arg){
  while (Wire.available()) {
    targetPos = (int)Wire.read() * 800; // Number of counts for target position
  }
  if ((targetPos - currentPos) > 1600) {
    targetPos -= 3200;
  } else if ((targetPos - currentPos) < -1600) {
    targetPos += 3200;
  }
}