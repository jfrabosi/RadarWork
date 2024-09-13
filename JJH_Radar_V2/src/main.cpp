#include <Arduino.h>

HardwareSerial Serial32(2);

void setup() {
  Serial.begin(115200);
  Serial32.begin(921600, SERIAL_8N1, 16, 17);
  Serial.println("HelloWorld!");
}

void loop() {
  // Check if data is available from STM32
  if (Serial32.available()) {
    String line = Serial32.readStringUntil('\n');
    Serial.println(line);
  }
}