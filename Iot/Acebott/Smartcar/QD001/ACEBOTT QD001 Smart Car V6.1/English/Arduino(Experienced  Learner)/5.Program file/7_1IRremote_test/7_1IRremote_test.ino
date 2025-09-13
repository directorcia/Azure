#include<IRremote.h>

#define IRpin 4//Declare the pin of the infrared receiver 

IRrecv myIRrecv(IRpin);//Creating an infrared receiver object

void setup() {
  Serial.begin(9600);
  myIRrecv.enableIRIn();//Start receiving infrared
}

void loop() {
  if(myIRrecv.decode()){//Whether infrared is received
    //Print infrared decoding values
    Serial.println(myIRrecv.decodedIRData.decodedRawData,HEX);
    myIRrecv.resume();//Wait for the next infrared reading
  }
  delay(100);
}

