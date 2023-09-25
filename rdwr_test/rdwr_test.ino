#include <Wire.h>
#define MY_ADDR 8


volatile uint8_t offset = 0;
int req_message = 1;

void setup() {
  Wire.begin(MY_ADDR);
  //recieves the information from the wire
  Wire.onReceive(receive);
  Wire.onRequest(request);
  //sets the serial so we can recieve characters
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
}

void request(){
  Wire.write(++req_message);
}

void receive(){
  //Inititializing the string that stores the message
  String msg = "";
  //the first infor is always the offset so we use it to allow is to get to the other information
  offset = Wire.read();

  while (Wire.available()){
    msg = msg + (char)Wire.read();
  }
  Serial.print(msg);
}