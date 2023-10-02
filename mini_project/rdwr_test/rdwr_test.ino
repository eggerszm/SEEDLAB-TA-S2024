#include <Wire.h>
#include <Encoder.h>
// #include <PID_v1.h>
#define MY_ADDR 8

#define Kp 0.016
#define Ki 0.0002
#define BATTERY_VOLTAGE 8

volatile uint8_t offset = 0;
int req_message = 1;

long int targetPos = 0;
volatile long int currentPos = 0;

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
  Serial.begin(115200);

  //Pins for motor
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);

  pinMode(7, OUTPUT); // Left H-Bridge. Determines direction
  pinMode(8, OUTPUT); // Right H-Bridge. Determines direction
  pinMode(9, OUTPUT); // Left power
  pinMode(10, OUTPUT); // Right power

}

void loop() {
  currentPos = EncA.read();

  error = targetPos - currentPos;
  integral = integral + desiredTsMs * error;

  voltage = Kp * error + Ki * integral;

  // H-Bridge Direction
  if(voltage > 0) {
    digitalWrite(8, HIGH);
  } else {
    digitalWrite(8, LOW);
  }

  int PWM = 255 * abs(voltage)/BATTERY_VOLTAGE;
  analogWrite(10, min(PWM, 255));

  while(millis() < lastTimeMs + desiredTsMs);
  lastTimeMs = millis();
  Serial.println(currentPos);
}

void request(){
  
  // Serial.println(currentPos);
  Wire.write(((byte *)currentPos)[0]); // I2C opperates in byte chunks which is why we convert currentPos to byte
  Wire.write(((byte *)currentPos)[1]); // Maybe add &currentPos instead of currentPos - Dawson Gullickson
  Wire.write(((byte *)currentPos)[2]);
  Wire.write(((byte *)currentPos)[3]);
}

void receive(int arg){
  Wire.read(); //reads in offset (random number). Read it first to "skip" past offset
  while (Wire.available()) {
    int fromPi = (int)Wire.read(); //reads in 0,1,2, or 3
    // Serial.println(fromPi);
    targetPos = (fromPi - 48) * 800; // Number of counts for target position. // -48 because of ASCII, 800 counts per (pi/2) rotation
  }
  if ((targetPos - currentPos) > 1600) {
    targetPos -= 3200;
  } else if ((targetPos - currentPos) < -1600) {
    targetPos += 3200;
  }
}