#include <Arduino.h>

HardwareSerial Serial32(1);

void setup() {
  Serial.begin(115200);
  Serial32.begin(921600);
  Serial.println("HelloWorld!");
}

void loop() {
  // Check if data is available from STM32
  if (Serial32.available()) {
    // Read the incoming byte
    char incomingByte = Serial32.read();
    
    // Send the byte to PC Serial
    Serial.write(incomingByte);
  }
}
