#include <Wire.h>

#define MY_ADDR 8


void setup() {
  Wire.begin(MY_ADDR);
  // put your setup code here, to run once:
  Wire.onReceive(receive);
  Wire.onRequest(request);
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
}

void request() {
  byte toSend[4] = {2, 1, 2, 3};
  byte b = 4;
  Wire.write(toSend, 4);
  // Wire.write(toSend, 4);
}

void receive(int nbytes) {
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

      break;
    case 1:

      break;
  }



}