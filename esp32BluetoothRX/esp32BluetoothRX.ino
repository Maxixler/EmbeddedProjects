#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

const int adcPin = 34; // ADC pin
const int dacPin = 25; // DAC pin

void setup() {
  Serial.begin(9600);
  bluetoothSerial.begin(9600);
  SerialBT.begin("ESP32test", true);
  delay(100);

  // Configure ADC and DAC pins
 pinMode(adcPin, INPUT);
 pinMode(dacPin, OUTPUT);
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
  // Read the analog signal from the ADC pin and write it to the DAC pin
  int adcValue = analogRead(adcPin);
  analogWrite(dacPin, adcValue);

  delay(20);
}
