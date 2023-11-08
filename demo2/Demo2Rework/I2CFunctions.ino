#define MY_ADDR 8

void SetupI2C() {
  Wire.begin(MY_ADDR);
  Wire.onReceive(receive);
}

void receive(int nbytes) {
  digitalWrite(11, HIGH);
  char offset = Wire.read();

  // Read in 
  char in_data[8];
  double out_data;
  for(int i=0; i < nbytes-1; i++) {
    in_data[i] = Wire.read();
  }

  memcpy(&lastPiMeasuredAngle, &(in_data[0]), 4);
  memcpy(&lastPiMeasuredDistance, &(in_data[4]), 4);

  digitalWrite(11, LOW);
}