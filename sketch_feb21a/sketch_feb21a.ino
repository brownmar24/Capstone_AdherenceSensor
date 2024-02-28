#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

void setup() {
  // put your setup code here, to run once:
  SerialBT.begin("ESP32test");
  delay(1000);
}

void loop() {
  // put your main code here, to run repeatedly:
  String inputFromOtherSide;serial 
  if (SerialBT.available()) {
    inputFromOtherSide = SerialBT.readString();
    SerialBT.println("You had entered: ");
    SerialBT.println(inputFromOtherSide);
  }
}
