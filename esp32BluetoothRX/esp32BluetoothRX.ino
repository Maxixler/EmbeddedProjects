#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

void setup() {
  Serial.begin(9600);
  bluetoothSerial.begin(9600);
  SerialBT.begin("ESP32test", true);
  delay(100);
}

void loop() {
  if (Serial.available()) {
    Serial.print("Received Serial: ");
  // char data = Serial.read();
  // Serial.write(data);    
    SerialBT.write(Serial.read());
  }
  // Check if the Bluetooth serial port is available
  if (SerialBT.available()) {
    Serial.print("Received BT Signal: ");
  // char data = SerialBT.read();
  // SerialBT.write(data);  
    Serial.write(SerialBT.read());
  }
  delay(20);
}
