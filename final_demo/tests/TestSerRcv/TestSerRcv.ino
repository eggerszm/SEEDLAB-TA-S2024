void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

int inByte;

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available() > 0) {
    inByte = Serial.read();

    Serial.write(inByte);
  }
}
