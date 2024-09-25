#include <Arduino.h>
#include "BluetoothSerial.h"
#include <driver/uart.h>

#define MAX_BUFFER_SIZE 35
#define TX_PIN 17

// Check if Bluetooth is available
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

// Check Serial Port Profile
#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Port Profile for Bluetooth is not available or not enabled. It is only available for the ESP32 chip.
#endif

typedef struct
{
  float start_m;
  float end_m;
  float update_rate;
  int max_step_length;
  int max_profile;
  float signal_quality;
  int reflector_shape;
  float threshold_sensitivity;
  int testing_update_rate;
  float true_update_rate;
} config_settings_t;

String deviceName = "ESP32-HUZZAH32";
config_settings_t current_config;

BluetoothSerial SerialBT;
HardwareSerial Serial32(2);

unsigned long startTime = millis();
unsigned long nowTime = millis();

bool checkForMCode(String input, config_settings_t *config);
bool get_uart_BT(char *result, uint16_t buf_size, bool wait_for_enter_key);
void config_menu(config_settings_t *config);
void print_config_menu(config_settings_t *config);
float parse_float(const char *string);
int parse_int(const char *string);
int printfln(const char *format, ...);
void print_input(void);
void print_n(void);
bool stop_radar(int delay_time);
bool save_config_adalogger(config_settings_t *config);
bool send_config_stm32(config_settings_t *config);
bool read_config_adalogger(config_settings_t *config);
bool read_config_stm32(config_settings_t *config);
bool check_for_blocking_m_codes(void);

void setup() {
  Serial.begin(115200);
  SerialBT.begin(deviceName);
  Serial.printf("The device with name \"%s\" is started.\nNow you can pair it with Bluetooth!\n", deviceName.c_str());
  Serial32.begin(921600, SERIAL_8N1, 16, 17);
  Serial.println("ESP32 Configuration Selector");
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(200);
  digitalWrite(LED_BUILTIN, LOW);
  read_config_adalogger(&current_config);
}

void loop() {
  // Check if data is available from STM32
  nowTime = millis();
  if (Serial32.available()){
    String line = Serial32.readStringUntil('\n');
    if (!checkForMCode(line, &current_config)){
      SerialBT.println("STM32: " + line);
    }
  }

  // if character is sent via bluetooth, enter config menu
  if (SerialBT.available()){
    char input = SerialBT.read();
    int time_delay = (int) (3000.0f / current_config.update_rate) + 1000;
    if (stop_radar(time_delay)){
      config_menu(&current_config);
    }
  }
}

bool checkForMCode(String input, config_settings_t *config){
  input.trim();  // Remove any leading/trailing whitespace
  Serial.print(input);
  
  // measured actual update rate
  if (input == "M807") {
    bool received = false;
    Serial32.print("M808");

    unsigned long start_time = millis();
    while (!received && (millis() - start_time) < 3000){
      if (Serial32.available()){
        String line = Serial32.readStringUntil('\n');
        line.trim();
        float actual_update_rate = line.toFloat();
        if (!isnan(actual_update_rate) && actual_update_rate >= 0.0f && actual_update_rate <= 150.0f){
          config->true_update_rate = actual_update_rate;
          received = true;
        }
        else{
          SerialBT.println("STM32: Update rate measurement failed.");
        }
      }
    }

    config->testing_update_rate = 0;
    config_menu(&current_config);
    return true;
  } 

  // send configuration file
  else if (input == "M918") {
    SerialBT.println("STM32: Configuration file requested.");
    return send_config_stm32(&current_config);
  }

  // STM32 reset
  else if (input == "M001") {
    SerialBT.println("STM32: Reinitialized. Restart program.");
    print_n();
    Serial32.print("M002");
    return true;
  }

  // ESP32 reset
  else if (input == "M805") {
    SerialBT.println("STM32: Radar power-cycled. Restart program.");
    print_n();
    Serial32.print("M806");
    return true;
  }

  return false;  // Default case: not an M-code we're looking for
}

bool get_uart_BT(char *result, uint16_t buf_size, bool wait_for_enter_key){
  const uint32_t TIMEOUT = 1000; // 1000ms timeout
  uint16_t received = 0;
  uint32_t startTime = millis();

  while (received < buf_size) {
    if (SerialBT.available()) {
      char c = SerialBT.read();
      result[received] = c;
      SerialBT.print(c);
      received++;
      startTime = millis(); // Reset timeout

      if (c == '\r' || c == '\n') {
        result[received - 1] = '\0'; // Replace newline with null terminator
        break; // End of message
      }
    }

    // Check for timeout
    if ((millis() - startTime > TIMEOUT) && !wait_for_enter_key) {
      if (received == 0) {
        return false; // No data received within timeout
      }
      break; // Partial message received
    }
  }

  result[received] = '\0'; // Ensure null-termination
  SerialBT.println("");
  return true;
}

void config_menu(config_settings_t *config){
  bool good_config = false;
  bool good_value = false;

  char _received_uart_data[MAX_BUFFER_SIZE];

  printfln("");
  printfln("       :+###%%#%%#%%#%%#%%###*:              ");
  printfln("     -#%%*==------------=*%%#:            ");
  printfln("    =%%*-:::::::::::::::::=%%+            ");
  printfln("   :%%#-::::::::::::::::::*%%=            ");
  printfln("   +%%=::::::::**-:::::::-##:.......     ");
  printfln("  .%%#-:::::::=%%%%%%%%#%%####%%%%%%%%%%%%%%%%%%%%%%%%%%%%: ");
  printfln("  *%%=::::::::*%%%%%%#=:::::::::::::::::-*%%=");
  printfln(" :%%*-:::::::-#%%%%%%%%+::::::::::::::::::=##");
  printfln(" *%%+::::::::*%%%%%%%%#+==========::::::::+%%+");
  printfln(":##-:::::::-#*=------=+%%%%%%%%%%=:::::::-##:");
  printfln("+%%+::::::::-=:::::::::*%%%%%%%%*::::::::+%%* ");
  printfln("##=::::::::::::::::::+%%%%%%%%#=:::::::-#%%: ");
  printfln("+%%*-:::::::::::::::=*%%%%%%%%%%*::::::::+%%+  ");
  printfln(" =%%%%%%%%#########***********-:::::::-##:  ");
  printfln("     .......:%%#-::::::::::::::::::*%%=   ");
  printfln("            *%%=:::::::::::::::::-*%%*    ");
  printfln("           -%%*::::::::=*******#%%%%#:     ");
  printfln("          .#%%=::::::::*%%*====-:.        ");
  printfln("          *%%*::::::::=%%%%                ");
  printfln("        :%%#+-:::::::=#%%:                ");
  printfln("       -%%%%%%%%%%%%%%%%%%%%%%%%%%%%=                 ");
  printfln("");
  delay(500);

  print_config_menu(config);

  while (!good_config){
    memset(&_received_uart_data, 0, MAX_BUFFER_SIZE);
    while (!get_uart_BT(_received_uart_data, 1, false));

    if (check_for_blocking_m_codes()) good_config = true;

    char config_char = tolower(_received_uart_data[0]);
    good_value = false;

    switch (config_char){

      case 's':
        print_n();
        printfln("Set the start of the measurement range. Current value: %.2f m", config->start_m);
        printfln("Format: XX.XX");
        printfln("  Input must match format exactly. Enter leading and trailing zeros as appropriate.");
        printfln("  Value must be between 00.10 and 20.00, inclusive.");
        printfln("  Units are in meters.");
        printfln("Notes:");
        printfln("  Generally advised to leave a margin of 0.1m above maximum wave height.");
        printfln("  Keep as high as possible. Greatly affects power consumption and maximum possible update rate.");
        printfln("Example:");
        printfln("  Radar is placed 10m above average wave height, and waves height fluctuates within +/- 1m.");
        printfln("  Start of measurement range should be set to 0.8m.");
        printfln("  User would enter \"00.80\" into terminal with no quotes.");
        print_n();
        print_input();

        while (!good_value){
          while (!get_uart_BT(_received_uart_data, 5, true));
          print_n();

          float new_start_m = parse_float(_received_uart_data);
          if (!isnan(new_start_m) && new_start_m >= 0.1f && new_start_m <= 20.0f) {
            printfln("Start of measurement range set to %.2f m", new_start_m);
            printfln("Is this the desired value? Type Y for yes, any other character for no.");
            print_n();
            print_input();
            while (!get_uart_BT(_received_uart_data, 1, true));
            print_n();

            config_char = tolower(_received_uart_data[0]);
            if (config_char == 'y'){
              config->start_m = new_start_m;
              good_value = true;
              printfln("Value saved.");
              print_n();
              delay(1000);
              print_config_menu(config);
            }
            else{
              printfln("Please enter a new number between 00.10 and 20.00.");
              print_n();
              print_input();
            }
          }
          else {
            printfln("Invalid input. Please enter a number between 00.10 and 20.00.");
            print_n();
            print_input();
          }
        }
        break;

      case 'e':
        print_n();
        printfln("Set the end of the measurement range. Current value: %.2f m", config->end_m);
        printfln("Format: XX.XX");
        printfln("  Input must match format exactly. Enter leading and trailing zeros as appropriate.");
        printfln("  Value must be between 00.10 and 20.00, inclusive.");
        printfln("  Units are in meters.");
        printfln("Notes:");
        printfln("  Generally advised to leave a margin of 0.1m below maximum wave height.");
        printfln("  Keep as low as possible. Greatly affects power consumption and maximum possible update rate.");
        printfln("Example:");
        printfln("  Radar is placed 10m above average wave height, and waves height fluctuates within +/- 1m.");
        printfln("  End of measurement range should be set to 1.2m.");
        printfln("  User would enter \"01.20\" into terminal with no quotes.");
        print_n();
        print_input();

        while (!good_value){
          while (!get_uart_BT(_received_uart_data, 5, true));
          print_n();

          float new_end_m = parse_float(_received_uart_data);
          if (!isnan(new_end_m) && new_end_m >= 0.1f && new_end_m <= 20.0f) {
            printfln("End of measurement range set to %.2f m", new_end_m);
            printfln("Is this the desired value? Type Y for yes, any other character for no.");
            print_n();
            print_input();
            while (!get_uart_BT(_received_uart_data, 1, true));
            print_n();

            config_char = tolower(_received_uart_data[0]);
            if (config_char == 'y'){
              config->end_m = new_end_m;
              good_value = true;
              printfln("Value saved.");
              print_n();
              delay(1000);
              print_config_menu(config);
            }
            else{
              printfln("Please enter a new number between 00.10 and 20.00.");
              print_n();
              print_input();
            }
          }
          else {
            printfln("Invalid input. Please enter a number between 00.10 and 20.00.");
            print_n();
            print_input();
          }
        }
        break;

      case 'u':
        print_n();
        printfln("Set the update rate. Current value: %.1f Hz", config->update_rate);
        printfln("Format: XX.X");
        printfln("  Input must match format exactly. Enter leading and trailing zeros as appropriate.");
        printfln("  Value must be between 00.1 and 60.0, inclusive.");
        printfln("  Units are in Hertz.");
        printfln("Notes:");
        printfln("  Keep as low as is required. Greatly affects power consumption.");
        printfln("Example:");
        printfln("  A measurement is required every 0.2 seconds.");
        printfln("  Update rate should be set to 5 Hz.");
        printfln("  User would enter \"05.0\" into terminal with no quotes.");
        print_n();
        print_input();

        while (!good_value){
          while (!get_uart_BT(_received_uart_data, 4, true));
          print_n();

          float new_update_rate = parse_float(_received_uart_data);
          if (!isnan(new_update_rate) && new_update_rate >= 0.1f && new_update_rate <= 60.0f) {

            if (new_update_rate >= 20.0f) {
              printfln("WARNING: Actual update rate may be lower than desired.");
              printfln("Test true update rate in configuration menu.");
              print_n();
            }

            printfln("Update rate set to %.1f Hz", new_update_rate);
            printfln("Is this the desired value? Type Y for yes, any other character for no.");
            print_n();
            print_input();
            while (!get_uart_BT(_received_uart_data, 1, true));
            print_n();

            config_char = tolower(_received_uart_data[0]);
            if (config_char == 'y'){
              config->update_rate = new_update_rate;
              good_value = true;
              printfln("Value saved.");
              print_n();
              delay(1000);
              print_config_menu(config);
            }
            else{
              printfln("Please enter a new number between 00.1 and 60.0, inclusive.");
              print_n();
              print_input();
            }
          }
          else {
            printfln("Invalid input. Please enter a number between 00.1 and 60.0, inclusive.");
            print_n();
            print_input();
          }
        }
        break;

      case 'l':
        print_n();
        printfln("Set the maximum step length. Current value: %i (%.1f mm)", config->max_step_length, (float) config->max_step_length*2.5f);
        printfln("Format: XX");
        printfln("  Input must match format exactly. Enter a single integer between 01 and 99, inclusive.");
        printfln("Notes:");
        printfln("  The maximum step length is set in multiples of 2.5mm. Multiply the inputed setting by 2.5mm");
        printfln("  to get the maximum step length in mm.");
        printfln("  Set the maximum step length to the desired resolution for measuring wave heights.");
        printfln("  This is an upper bound - the distance algorithm may return finer resolution measurements.");
        printfln("  Keep maximum step length as high as possible. Greatly affects power consumption and maximum possible update rate.");
        printfln("Example:");
        printfln("  The user wants to determine wave heights accurate to within 5mm.");
        printfln("  The user should set the maximum step length to 2, since 2 * 2.5mm = 5mm.");
        printfln("  User would enter \"2\" into terminal with no quotes.");
        print_n();
        print_input();

        while (!good_value){
          while (!get_uart_BT(_received_uart_data, 2, true));
          print_n();

          int new_max_step = parse_int(_received_uart_data);
          if (new_max_step >= 1 && new_max_step <= 99) {

            printfln("Maximum step length set to %i (%.1f mm)", new_max_step, (float) new_max_step*2.5f);
            printfln("Is this the desired value? Type Y for yes, any other character for no.");
            print_n();
            print_input();
            while (!get_uart_BT(_received_uart_data, 1, true));
            print_n();

            config_char = tolower(_received_uart_data[0]);
            if (config_char == 'y'){
              config->max_step_length = new_max_step;
              good_value = true;
              printfln("Value saved.");
              print_n();
              delay(1000);
              print_config_menu(config);
            }
            else{
              printfln("Please enter an integer between 1 and 99, inclusive.");
              print_n();
              print_input();
            }
          }
          else {
            printfln("Invalid input. Please enter an integer between 1 and 99, inclusive.");
            print_n();
            print_input();
          }
        }
        break;

      case 'p':
        print_n();
        printfln("Set the maximum measurement profile. Current value: %i", config->max_profile);
        printfln("Format: X");
        printfln("  Input must match format exactly. Enter a single integer between 1 and 5, inclusive.");
        printfln("  Units are arbitrary.");
        printfln("Notes:");
        printfln("  Measurement profile is used to set the length and shape of the emitted radio pulse.");
        printfln("  Higher profiles transmit more energy, increasing signal-to-noise ratio and range.");
        printfln("  Keep profile as low as possible. Greatly affects power consumption and maximum possible update rate.");
        printfln("Example:");
        printfln("  The user starts the radar with maximum profile setting of 2, but can only detect waves half the time.");
        printfln("  The user should increase the profile setting by 1, repeating until the waves are always detected.");
        printfln("  User would enter \"3\" into terminal with no quotes.");
        print_n();
        print_input();

        while (!good_value){
          while (!get_uart_BT(_received_uart_data, 1, true));
          print_n();

          int new_max_profile = parse_int(_received_uart_data);
          if (new_max_profile >= 1 && new_max_profile <= 5) {

            printfln("Maximum profile set to %i", new_max_profile);
            printfln("Is this the desired value? Type Y for yes, any other character for no.");
            print_n();
            print_input();
            while (!get_uart_BT(_received_uart_data, 1, true));
            print_n();

            config_char = tolower(_received_uart_data[0]);
            if (config_char == 'y'){
              config->max_profile = new_max_profile;
              good_value = true;
              printfln("Value saved.");
              print_n();
              delay(1000);
              print_config_menu(config);
            }
            else{
              printfln("Please enter an integer between 1 and 5, inclusive.");
              print_n();
              print_input();
            }
          }
          else {
            printfln("Invalid input. Please enter an integer between 1 and 5, inclusive.");
            print_n();
            print_input();
          }
        }
        break;

      case 'q':
        print_n();
        printfln("Set the signal quality. Current value: %.1f", config->signal_quality);
        printfln("Format: XX.X");
        printfln("  Input must match format exactly. Enter leading and trailing zeros as appropriate.");
        printfln("  Value must be between 00.0 and 35.0, inclusive.");
        printfln("  Units are arbitrary.");
        printfln("Notes:");
        printfln("  Higher signal quality results in a higher signal-to-noise ratio target, and increases the number of averages");
        printfln("  taken for a single datapoint. This generally increases the accuracy of measurements.");
        printfln("  Keep as low as possible. Affects power consumption and maximum possible update rate.");
        printfln("Example:");
        printfln("  The user starts the radar with signal quality setting of 15.0, but measurements appear inaccurate.");
        printfln("  The user should increase the profile setting by 1.0, repeating until the measurements are accurate enough.");
        printfln("  User would enter \"16.0\" into terminal with no quotes.");
        print_n();
        print_input();
        while (!good_value){
          while (!get_uart_BT(_received_uart_data, 4, true));
          print_n();

          float new_quality = parse_float(_received_uart_data);
          if (!isnan(new_quality) && new_quality >= 0.0f && new_quality <= 35.0f) {

            printfln("Signal quality set to %.1f", new_quality);
            printfln("Is this the desired value? Type Y for yes, any other character for no.");
            print_n();
            print_input();
            while (!get_uart_BT(_received_uart_data, 1, true));
            print_n();

            config_char = tolower(_received_uart_data[0]);
            if (config_char == 'y'){
              config->signal_quality = new_quality;
              good_value = true;
              printfln("Value saved.");
              print_n();
              delay(1000);
              print_config_menu(config);
            }
            else{
              printfln("Please enter a new number between 00.0 and 35.0.");
            print_n();
              print_input();
            }
          }
          else {
            printfln("Invalid input. Please enter a number between 00.0 and 35.0.");
            print_n();
            print_input();
          }
        }
        break;

      case 'r':
        print_n();
        printfln("Sets the reflector shape. Current value: %i", config->reflector_shape);
        printfln("Format: X");
        printfln("  Input must match format exactly.");
        printfln("  Enter \"1\" to use planar reflector shape.");
        printfln("  Enter \"0\" to use generic reflector shape.");
        printfln("Notes:");
        printfln("  Planar reflector shape should be used for determining the distance to flat objects like water surfaces,");
        printfln("  sheets of metal, walls, and more.");
        printfln("  Generic reflector shape should be used for determining the distance to any other objects.");
        printfln("Example:");
        printfln("  The radar is installed on the edge of a pier above a body of water.");
        printfln("  Planar reflector shape should be used, and the user would enter \"1\" with no quotes.");
        print_n();
        print_input();

        while (!good_value){
          while (!get_uart_BT(_received_uart_data, 1, true));
          print_n();

          int new_shape = parse_int(_received_uart_data);
          if (new_shape == 1 || new_shape == 0) {

            printfln("Reflector shape set to %i", new_shape);
            printfln("Is this the desired value? Type Y for yes, N for no.");
            print_n();
            print_input();
            while (!get_uart_BT(_received_uart_data, 1, true));
            print_n();

            config_char = tolower(_received_uart_data[0]);
            if (config_char == 'y'){
              config->reflector_shape = new_shape;
              good_value = true;
              printfln("Value saved.");
              print_n();
              delay(1000);
              print_config_menu(config);
            }
            else{
              printfln("Please enter either a 0 or a 1.");
              print_n();
              print_input();
            }
          }
          else {
            printfln("Invalid input. Please enter either a 0 or a 1.");
            print_n();
            print_input();
          }
        }
        break;

      case 't':
        print_n();
        printfln("Set the threshold sensitivity. Current value: %.2f", config->threshold_sensitivity);
        printfln("Format: X.XX");
        printfln("  Input must match format exactly. Enter leading and trailing zeros as appropriate.");
        printfln("  Value must be between 0.00 and 1.00, inclusive.");
        printfln("  Units are arbitrary.");
        printfln("Notes:");
        printfln("  Controls the false positive rate. A higher threshold means that more energy must be reflected from an object");
        printfln("  to record a distance measurement.");
        printfln("  Tune in conjunction with the signal quality and measurement profile to eliminate background objects.");
        printfln("Example:");
        printfln("  The user starts the radar with threshold sensitivity setting of 0.5, measuring water level over the side of a pier.");
        printfln("  The radar keeps detecting the distance to the metal beams supporting the pier, which messes up the measurements.");
        printfln("  The user should decrease the sensitivity by 0.1, adjusting until the beams are no longer detected by the radar.");
        printfln("  User would enter \"0.4\" into terminal with no quotes.");
        print_n();
        print_input();

        while (!good_value){
          while (!get_uart_BT(_received_uart_data, 4, true));
          print_n();

          float new_threshold = parse_float(_received_uart_data);
          if (!isnan(new_threshold) && new_threshold >= 0.0f && new_threshold <= 1.0f) {

            printfln("Signal quality set to %.2f", new_threshold);
            printfln("Is this the desired value? Type Y for yes, any other character for no.");
            print_n();
            print_input();
            while (!get_uart_BT(_received_uart_data, 1, true));
            print_n();

            config_char = tolower(_received_uart_data[0]);
            if (config_char == 'y'){
              config->threshold_sensitivity = new_threshold;
              good_value = true;
              printfln("Value saved.");
              print_n();
              delay(1000);
              print_config_menu(config);
            }
            else{
              printfln("Please enter a new number between 0.00 and 1.00.");
              print_n();
              print_input();
            }
          }
          else {
            printfln("Invalid input. Please enter a number between 0.00 and 1.00.");
            print_n();
            print_input();
          }
        }
        break;

      case 'a':
        if (config->start_m >= config->end_m){
          print_n();
          printfln("WARNING: Start of range is greater than end of range! This must be fixed before continuing.");
          print_n();
          printfln("Please send one of the characters from the menu above.");
          print_n();
          print_input();
        }
        else{
          print_n();
          good_config = true;
          config->testing_update_rate = 1;
          if (!send_config_stm32(config)){
            printfln("Failed to send configuration to STM32. Please try again.");
            print_n();
            break;
          }
          printfln("Testing actual update rate...");
        }
        break;
      
      case 'x':
        print_n();
        printfln("Send any character to re-open configuration menu.");
        print_n();
        delay(3000);

        if (config->start_m >= config->end_m){
          printfln("WARNING: Start of range is greater than end of range! This must be fixed before continuing.");
          print_n();
          printfln("Please send one of the characters from the menu above.");
          print_n();
          print_input();
        }
        else{
          good_config = true;
          printfln("Configuration saved to SD card. Returning to normal operations.");
          delay(3000);
          print_n();
          if (!send_config_stm32(config)){
            printfln("Failed to send configuration to STM32. Please try again.");
            print_n();
            break;
          }
          printfln("DATA_START");
        }
        break;

      default:
        print_n();
        printfln("Input is not accepted. Please send one of the characters above.");
        print_n();
        print_input();
        break;
    }
  }
}

int printfln(const char *format, ...) {
  va_list args;
  char buf[256]; // Adjust size as needed
  int result;

  va_start(args, format);
  result = vsnprintf(buf, sizeof(buf), format, args);
  va_end(args);

  SerialBT.print("ESP32: ");
  SerialBT.print(buf);
  SerialBT.println();

  delay(100);
  return result;
}

void print_input(void){
  SerialBT.print("Input: ");
}

void print_n(void){
  printfln("");
}

void print_config_menu(config_settings_t *config){
  printfln("--- CONFIGURATION MENU ---");
  printfln("Send one of the below characters to change that configuration setting.");
  printfln("Changes will be saved automatically.");
  printfln("S: Change the start of the measurement range. Affects power consumption and update rate.");
  printfln("   Current value: %.2f m", config->start_m);
  printfln("E: Change the end of the measurement range. Affects power consumption and update rate.");
  printfln("   Current value: %.2f m", config->end_m);
  printfln("U: Change the update rate. Affects power consumption.");
  printfln("   Current value: %.1f Hz", config->update_rate);
  printfln("L: Change the maximum step length of the measurements. Affects power consumption, update rate, and resolution.");
  printfln("   Current value: %i (%.1f mm)", config->max_step_length, (float) config->max_step_length*2.5f);
  printfln("P: Change the maximum measurement profile. Affects power consumption, update rate, and accuracy.");
  printfln("   Current value: %i", config->max_profile);
  printfln("Q: Change the signal quality. Affects power consumption, update rate, and accuracy.");
  printfln("   Current value: %.1f", config->signal_quality);
  printfln("R: Change the reflector shape.");
  printfln("   Current value: %i (0: generic, 1: planar)", config->reflector_shape);
  printfln("T: Change the threshold sensitivity. Affects accuracy.");
  printfln("   Current value: %.2f", config->threshold_sensitivity);
  printfln("A: Measure the actual update rate.");
  printfln("   True update rate: %.1f Hz", config->true_update_rate);
  printfln("X: Exit the menu.");
  print_n();
  print_input();
}

float parse_float(const char *string) {
  char *end;
  float result = strtof(string, &end);
  if (end == string || *end != '\0') {
      // Parsing failed
      return NAN;
  }
  return result;
}

int parse_int(const char *string) {
    char *end;
    long result = strtol(string, &end, 10);
    if (end == string || *end != '\0') {
        // Parsing failed
        return -1;
    }
    return (int)result;
}

bool stop_radar(int delay_time){
  float time_s = (float) delay_time / 1000.0f;
  SerialBT.println("");
  SerialBT.println("STM32: Stopping radar...");
  SerialBT.printf("STM32: Process will take approximately %.1f seconds.", time_s);
  SerialBT.println("");
  SerialBT.println("STM32: ");

  // Disable UART
  uart_driver_delete(UART_NUM_2);
  
  // Configure TX_PIN as output and set it LOW
  gpio_set_direction(GPIO_NUM_17, GPIO_MODE_OUTPUT);
  gpio_set_level(GPIO_NUM_17, 0);
  
  unsigned long start_time = millis();
  while ((millis() - start_time) < delay_time);

  Serial32.begin(921600, SERIAL_8N1, 16, 17);

  start_time = millis();
  while ((millis() - start_time) < 3000){
    if (Serial32.available()){
      String line = Serial32.readStringUntil('\n');
      line.trim();
      if (line == "M805") {
        Serial32.print("M806");
        SerialBT.println("STM32: Radar stopped.");
        return true;
      }
    }
  }
  
  SerialBT.println("STM32: Failed to stop radar.");
  return false;
}

bool save_config_adalogger(config_settings_t *config){
  return true;
}

bool send_config_stm32(config_settings_t *config){
  char config_string[64];
  snprintf(config_string, sizeof(config_string), 
           "%05.2f,%05.2f,%04.1f,%02d,%d,%04.1f,%d,%04.2f,%d,%04.1f\0",
           config->start_m,
           config->end_m,
           config->update_rate,
           config->max_step_length,
           config->max_profile,
           config->signal_quality,
           config->reflector_shape,
           config->threshold_sensitivity,
           config->testing_update_rate,
           config->true_update_rate);
  
  Serial32.print(config_string);
  Serial.println(config_string);

  unsigned long start_time = millis();
  while ((millis() - start_time) < 2000){
    if (Serial32.available()){
      String line = Serial32.readStringUntil('\n');
      line.trim();
      if (line == "M919") {
        SerialBT.println("STM32: Configuration received and loaded.");
        SerialBT.println("STM32: ");
        return true;
      }
    }
  }
  SerialBT.println("STM32: Failed to load configuration.");
  return false;
}

bool read_config_adalogger(config_settings_t *config){
  config->start_m = 0.50f;
  config->end_m = 0.90f;
  config->update_rate = 1.0f;
  config->max_step_length = 4;
  config->max_profile = 3;
  config->signal_quality = 20.0f;
  config->reflector_shape = 1;
  config->threshold_sensitivity = 0.50f;
  config->testing_update_rate = 0;
  config->true_update_rate = 10.1f;
  return true;
}

bool read_config_stm32(config_settings_t *config){
  return true;
}

bool check_for_blocking_m_codes(void){
  if (Serial32.available()){
    String line = Serial32.readStringUntil('\n');
    line.trim();
    if (line == "M001"){
      return true;
    }
    else if (line == "M805"){
      return true;
    }
  }
  return false;
}