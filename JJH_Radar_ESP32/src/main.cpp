#include <Arduino.h>
#include "BluetoothSerial.h"

String deviceName = "ESP32-HUZZAH32";

// Check if Bluetooth is available
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

// Check Serial Port Profile
#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Port Profile for Bluetooth is not available or not enabled. It is only available for the ESP32 chip.
#endif

BluetoothSerial SerialBT;
HardwareSerial Serial32(2);

unsigned long startTime = millis();
unsigned long nowTime = millis();

bool checkForMCode(String input);

void setup() {
  Serial.begin(115200);
  Serial32.begin(921600, SERIAL_8N1, 16, 17);
  Serial.println("ESP32 Configuration Selector");
  SerialBT.begin(deviceName);
  Serial.printf("The device with name \"%s\" is started.\nNow you can pair it with Bluetooth!\n", deviceName.c_str());

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  // Check if data is available from STM32
  nowTime = millis();
  if (Serial32.available()){
    String line = Serial32.readStringUntil('\n');
    if (!checkForMCode(line)){
      SerialBT.println("STM32: " + line);
      digitalWrite(LED_BUILTIN, HIGH);
      startTime = millis();
    }
  }
  else if (nowTime - startTime > 200){
    digitalWrite(LED_BUILTIN, LOW);
  }

  if (SerialBT.available()){
    char input = SerialBT.read();
    SerialBT.print(input);
    Serial32.print(input);  // Send the character to STM32
  }
}

bool checkForMCode(String input){
  input.trim();  // Remove any leading/trailing whitespace
  
  if (input == "M807") {
    Serial.println("Received M807, sending configuration string");
    Serial32.print("00.40,01.20,05.0,02,5,35.0,1,0.50");
    // Serial32.print("MneTwoThreeFourFiveSixSevenEightN");
    // Serial.println("MneTwoThreeFourFiveSixSevenEightN");
    return true;
  } 
  else if (input == "M806") {
    Serial.println("Received M806, sending okay");
    return true;
  }
  else if (input == "M917") {
    SerialBT.print("Input: ");
    return true;
  }
  else if (input == "M918") {
    SerialBT.println("");
    return true;
  }

  return false;  // Default case: not an M-code we're looking for
}
