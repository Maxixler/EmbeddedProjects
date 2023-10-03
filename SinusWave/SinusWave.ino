#include <SoftwareSerial.h>
const int sampleRate = 9600; // Sampling rate in Hz
const int numSamples = 360; // Number of samples to generate
const float frequency = 1000; // Frequency of the sinusoid in Hz
#define RX 0
#define TX 1

SoftwareSerial bluetooth(RX, TX);

void setup() {
  // Set the sampling rate
  Serial.begin(sampleRate);
  bluetooth.begin(9600);
  Serial.println("Starting Bluetooth...");
  // Generate the sinusoid
 
}
void loop() {
  // Check if the Bluetooth serial port is available 
  if(bluetooth.available()){   
    for (int i = 0; i < numSamples; i++) {
      float sample = sin(2 * PI * frequency * i / sampleRate);
      (sample >= 0) ? bluetooth.write(1) : bluetooth.write(-1);
    }
  }
    delay(20);
    bluetooth.flush();
}