#include <Wire.h>
#define MY_ADDR 8


volatile uint8_t offset = 0;

void setup() {
  // sets the address
  Wire.begin(MY_ADDR);
  //recieves the information from the wire
  Wire.onReceive(receive);
  //sets the serial so we can recieve characters
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
}


void receive(){
  //Inititializing the string that stores the message
  String msg = "";
  //the first infor is always the offset so we use it to allow is to get to the other information
  offset = Wire.read();

  Serial.print("String: ");

  //This while loop takes all the the characters and adds it the the total message
  while (Wire.available()){
    char let = Wire.read();
    //Serial.print(let);
    msg= msg + let;
    //Serial.print(let,DEC);
    
  }

  //prints out the whole message
  Serial.print(msg);

  Serial.print(" ASCII code:");

  /*this for loop converts each character in the msg into ascii and prints it out.
  It also adds a space to make the ascii more readable.*/
  for(int l = 0; l < msg.length(); l++){
    Serial.print(msg[l],DEC);
    Serial.print(" ");
  }

}