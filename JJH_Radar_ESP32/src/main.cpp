// --- LIBRARIES ---
#include <stdint.h>
#include <stdbool.h>
#include <Arduino.h>
#include <BluetoothSerial.h>
#include <Wire.h>
#include <SPI.h>
#include <FS.h>
#include <SD.h>
#include "RTClib.h"
#include <driver/uart.h>

// check if Bluetooth is available
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
  #error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

// check Serial Port Profile
#if !defined(CONFIG_BT_SPP_ENABLED)
  #error Serial Port Profile for Bluetooth is not available or not enabled. It is only available for the ESP32 chip.
#endif

// --- DEFINITIONS ---
#define MAX_BUFFER_SIZE 128
#define TX_PIN 17
#define BT_TIMEOUT_MINS 10U
#define DEVICE_NAME "ESP32_HUZZAH32"
#define PRINT_DEBUG_TO_BT false

// #define REASSIGN_PINS
// int sck = 5;
// int miso = 19;
// int mosi = 18;
// int cs = 33;



// --- CUSTOM TYPEDEFS AND STRUCTS ---
typedef float float32_t;

typedef enum
{
  STATE_LOAD_CONFIG_SD,
  STATE_LISTENING,
  STATE_CONFIG_MENU
} StateNames;

typedef struct
{
  float32_t start_m;
  float32_t end_m;
  float32_t update_rate;
  uint8_t max_step_length;
  uint8_t max_profile;
  float32_t signal_quality;
  uint8_t reflector_shape;
  float32_t threshold_sensitivity;
  uint8_t testing_update_rate;
  float32_t true_update_rate;
} ConfigSettings;

// --- GLOBAL VARIABLES ---

// serial connections
String deviceName = DEVICE_NAME;
BluetoothSerial SerialBT;
HardwareSerial Serial32(2);
bool BT_on = true;

// config settings struct
ConfigSettings current_config;

// data-logging
RTC_PCF8523 rtc;
const int chipSelect = 33;

// state-tracking
uint8_t state = STATE_LOAD_CONFIG_SD;
bool sending_config = false;

// time-keeping
uint32_t start_time_BT = millis();
uint32_t current_time = millis();

// --- FUNCTION PROTOTYPES --- 
bool get_uart_BT(char *result, uint16_t buf_size, uint32_t timeout);
void config_menu(ConfigSettings *config);
void print_config_menu(ConfigSettings *config);
float32_t parse_float(const char *string);
uint8_t parse_int(const char *string);
void printf_BT_slow(const char *format, ...);
void print_input(void);
void print_n(void);
bool stop_radar(uint32_t delay_time, uint32_t timeout);
bool save_config_adalogger(ConfigSettings *config);
bool send_config_stm32(ConfigSettings *config);
bool read_config_adalogger(ConfigSettings *config);
bool read_config_stm32(ConfigSettings *config);
bool read_until_newline_stm32(char* buffer, uint8_t buffer_size);
uint16_t check_for_m_code(const char* buffer);
void print_CP(void);
bool save_new_update_rate(ConfigSettings *config);
void printf_debug(const char *format, ...);
void printf_debug_stm32(const char *format, ...);
void printf_stm32(const char *format, ...);
void setup_rtc(void);
void setup_sd_card(void);
void list_dir(fs::FS &fs, const char *dirname, uint8_t levels);
void read_file(fs::FS &fs, const char *path);
void write_file(fs::FS &fs, const char *path, const char *message);
void append_file(fs::FS &fs, const char *path, const char *message);
void rename_file(fs::FS &fs, const char *path1, const char *path2);
void delete_file(fs::FS &fs, const char *path);

// --- STATE 0: INIT_ESP ---
void setup()
{
  // start wired USB serial connection
  Serial.begin(115200);

  // start Bluetooth serial connection
  SerialBT.begin(deviceName);
  printf_debug("The device with name \"%s\" is started.\nNow you can pair it with Bluetooth!\n", deviceName.c_str());
  printf_debug("Bluetooth automatically turns off after %i minutes! Turn back on by hitting ESP32 reset button.\n", BT_TIMEOUT_MINS);

  // start STM32 serial connection
  Serial32.begin(921600, SERIAL_8N1, 16, 17);

  // set up RTC
  setup_rtc();

  // set up SD card
  setup_sd_card();
  
  // set up LED pin, flash for visual indication
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  
  // turn LED on until Bluetooth turns off
  delay(3000);
  digitalWrite(LED_BUILTIN, HIGH);
  start_time_BT = millis();
}

// --- MAIN LOOP ---
void loop()
{
  // check if Bluetooth timeout is exceeded; if so, shut off Bluetooth to save power
  if ((millis() - start_time_BT) > ((uint32_t) BT_TIMEOUT_MINS * 60000UL))
  {
    SerialBT.end();
    BT_on = false;
    digitalWrite(LED_BUILTIN, LOW);
  }

  // --- STATE 1: LOAD_CONFIG_SD ---
  if (state == STATE_LOAD_CONFIG_SD)
  {
    if (read_config_adalogger(&current_config))
    {
      printf_BT_slow("Configuration loaded from SD card.");
    }
    else
    {
      printf_BT_slow("ERROR: Failed to load config from SD card. Using default config.");
    }
    state = STATE_LISTENING;
  }

  
  // --- STATE 2: LISTENING ---
  else if (state == STATE_LISTENING)
  {
    char buffer_S2[MAX_BUFFER_SIZE];

    // check if there's data from STM32 UART
    if (read_until_newline_stm32(buffer_S2, MAX_BUFFER_SIZE))
    {
      uint16_t code_S2 = check_for_m_code(buffer_S2);
      DateTime d_now;

      switch (code_S2)
      {
        // No M-code: just print message to BT
        case 0:
          printf_debug_stm32("%s", buffer_S2);
          break;
        
        // M114: Denotes start of data
        case 114:
          printf_debug_stm32("M114");
          printf_stm32("DATA_START");
          break;

        // M115: Precedes new distance data
        case 115:
          while (strcmp(buffer_S2, "M115") == 0)
          {
            read_until_newline_stm32(buffer_S2, MAX_BUFFER_SIZE);
          }
          printf_debug_stm32("M115");
          d_now = rtc.now();
          printf_stm32("[%02d/%02d/%02d %02d:%02d:%02d] %s",
                        d_now.year() % 100, d_now.month(), d_now.day(), 
                        d_now.hour(), d_now.minute(), d_now.second(), 
                        buffer_S2);
          break;
        
        // M116: Denotes end of data
        case 116:
          printf_debug_stm32("M116");
          printf_stm32("DATA_END");
          break;

        // M807: New config data requested
        // we're not in the config menu, so just send current config
        case 807:
          printf_debug_stm32("M807");
          if (!send_config_stm32(&current_config))
          {
            printf_debug("ERROR: Couldn't send config.");
          }
          break;

        // M815: Update rate test completed, waiting for acknowledgement
        case 815: 
          printf_debug_stm32("M815");
          if (!save_new_update_rate(&current_config))
          {
            printf_debug("ERROR: Failed to save actual update rate.");
          }

          // either way, go to config menu
          state = STATE_CONFIG_MENU;
          print_CP();
          print_config_menu(&current_config);
          break;

        // M918: TX pin on STM pulled LOW during active radar; requesting acknowledge to stop radar
        // we shouldn't see this here...
        case 918:
          Serial32.print("M919");
          printf_debug_stm32("M918");
          printf_debug("ERROR: Radar stopped, M-code received outside of stop_radar().");
          break;

        // M808: Config successfully loaded by STM32
        // we shouldn't see this here...
        case 808:
          printf_debug_stm32("M808");
          printf_debug("ERROR: Config loaded by STM32, M-code received outside of send_config_stm32().");
          break;
        
        // M806: Config not loaded successfully by STM32
        // we shouldn't see this here...
        case 806:
          printf_debug_stm32("M806");
          printf_debug("ERROR: Config NOT loaded by STM32, M-code received outside of send_config_stm32().");
          break;

        default:
          printf_debug("ERROR: M-code not recognized. Code sent: %i", code_S2);
          break;
      }
    }

    // check if there's any data from BT; if so, stop the radar and enter the config menu
    if (SerialBT.available()){

      // clear the input, we don't need it
      (void) SerialBT.read();

      // stop the radar
      uint32_t radar_time_delay = (uint32_t) (3000.0f / current_config.update_rate) + 1000;
      if (stop_radar(radar_time_delay, 3000UL))
      {
        state = STATE_CONFIG_MENU;
        print_CP();
        print_config_menu(&current_config);
      }
    }
  }

  // --- STATE 3: CONFIG_MENU ---
  else if (state == STATE_CONFIG_MENU)
  {
    char buffer_S3_BT[MAX_BUFFER_SIZE];
    char buffer_S3_STM[MAX_BUFFER_SIZE];

    // check for any blocking M-codes
    if (read_until_newline_stm32(buffer_S3_STM, MAX_BUFFER_SIZE))
    {
      uint16_t code_S3 = check_for_m_code(buffer_S3_STM);
      switch (code_S3)
      {
        // M815: Update rate test completed, waiting for acknowledgement
        case 815: 
          printf_debug_stm32("");
          printf_debug_stm32("M815");
          if (!save_new_update_rate(&current_config))
          {
            printf_debug("ERROR: Failed to save actual update rate.");
          }

          printf_debug("ERROR: Update test completed, but M-code caught inside of config menu.");
          print_CP();
          print_config_menu(&current_config);
          break;

        // M918: TX pin on STM pulled LOW during active radar; requesting acknowledge to stop radar
        case 918:
          Serial32.print("M919");
          printf_debug_stm32("");
          printf_debug_stm32("M918");
          printf_debug("ERROR: Radar stopped, M-code received outside of stop_radar().");
          break;

        default:
          break;
      }
    }

    // register any BT inputs
    if (get_uart_BT(buffer_S3_BT, 1U, 10UL))
    {
      char config_setting = tolower(buffer_S3_BT[0]);
      bool good_setting = false;
      
      switch(config_setting)
      {
        case 's':
          print_n();
          printf_BT_slow("Set the start of the measurement range. Current value: %05.2f m", current_config.start_m);
          printf_BT_slow("Format: XX.XX");
          printf_BT_slow("  Input must match format exactly. Enter leading and trailing zeros as appropriate.");
          printf_BT_slow("  Value must be between 00.10 and 20.00, inclusive.");
          printf_BT_slow("  Units are in meters.");
          printf_BT_slow("Notes:");
          printf_BT_slow("  Generally advised to leave a margin of 0.1m above maximum wave height.");
          printf_BT_slow("  Keep as high as possible. Greatly affects power consumption and maximum possible update rate.");
          printf_BT_slow("Example:");
          printf_BT_slow("  Radar is placed 10m above average wave height, and waves height fluctuates within +/- 1m.");
          printf_BT_slow("  Start of measurement range should be set to 0.8m.");
          printf_BT_slow("  User would enter \"00.80\" into terminal with no quotes.");
          print_n();
          print_input();

          // loop until a good setting is saved
          while (!good_setting)
          {
            // wait for five chars to be sent over BT
            (void) get_uart_BT(buffer_S3_BT, 5, 0);
            print_n();
            float32_t new_start_m = parse_float(buffer_S3_BT);

            // check if float is within range
            if (!isnan(new_start_m) && (new_start_m >= 0.1f) && (new_start_m <= 20.0f))
            {
              printf_BT_slow("Start of measurement range set to %05.2f m", new_start_m);
              printf_BT_slow("Is this the desired value? Type Y for yes, any other character for no.");
              print_n();
              print_input();

              // get acknowledgement
              (void) get_uart_BT(buffer_S3_BT, 1, 0);
              print_n();
              char ack_char = tolower(buffer_S3_BT[0]);
              
              if (ack_char == 'y')
              {
                // save to config and return to menu
                current_config.start_m = new_start_m;
                good_setting = true;
                printf_BT_slow("Value saved.");
                print_n();
                delay(1000);
                print_config_menu(&current_config);
              }

              else
              {
                printf_BT_slow("Please enter a new number between 00.10 and 20.00.");
                print_n();
                print_input();
              }
            }
            
            // bad input or out of range
            else
            {
              printf_BT_slow("Invalid input. Please enter a number between 00.10 and 20.00.");
              print_n();
              print_input();
            }
          }

          // new setting has been saved
          break;

        case 'e':
          print_n();
          printf_BT_slow("Set the end of the measurement range. Current value: %05.2f m", current_config.end_m);
          printf_BT_slow("Format: XX.XX");
          printf_BT_slow("  Input must match format exactly. Enter leading and trailing zeros as appropriate.");
          printf_BT_slow("  Value must be between 00.10 and 20.00, inclusive.");
          printf_BT_slow("  Units are in meters.");
          printf_BT_slow("Notes:");
          printf_BT_slow("  Generally advised to leave a margin of 0.1m below maximum wave height.");
          printf_BT_slow("  Keep as low as possible. Greatly affects power consumption and maximum possible update rate.");
          printf_BT_slow("Example:");
          printf_BT_slow("  Radar is placed 10m above average wave height, and waves height fluctuates within +/- 1m.");
          printf_BT_slow("  End of measurement range should be set to 1.2m.");
          printf_BT_slow("  User would enter \"01.20\" into terminal with no quotes.");
          print_n();
          print_input();

          while (!good_setting)
          {
            (void) get_uart_BT(buffer_S3_BT, 5, 0);
            print_n();
            float32_t new_end_m = parse_float(buffer_S3_BT);

            if (!isnan(new_end_m) && (new_end_m >= 0.1f) && (new_end_m <= 20.0f)) 
            {
              printf_BT_slow("End of measurement range set to %05.2f m", new_end_m);
              printf_BT_slow("Is this the desired value? Type Y for yes, any other character for no.");
              print_n();
              print_input();

              (void) get_uart_BT(buffer_S3_BT, 1, 0);
              print_n();
              char ack_char = tolower(buffer_S3_BT[0]);
              
              if (ack_char == 'y')
              {
                current_config.end_m = new_end_m;
                good_setting = true;
                printf_BT_slow("Value saved.");
                print_n();
                delay(1000);
                print_config_menu(&current_config);
              }
              else
              {
                printf_BT_slow("Please enter a new number between 00.10 and 20.00.");
                print_n();
                print_input();
              }
            }
            else
            {
              printf_BT_slow("Invalid input. Please enter a number between 00.10 and 20.00.");
              print_n();
              print_input();
            }
          }
          
          break;

        case 'u':
          print_n();
          printf_BT_slow("Set the update rate. Current value: %04.1f Hz", current_config.update_rate);
          printf_BT_slow("Format: XX.X");
          printf_BT_slow("  Input must match format exactly. Enter leading and trailing zeros as appropriate.");
          printf_BT_slow("  Value must be between 00.1 and 60.0, inclusive.");
          printf_BT_slow("  Units are in Hertz.");
          printf_BT_slow("Notes:");
          printf_BT_slow("  Keep as low as is required. Greatly affects power consumption.");
          printf_BT_slow("Example:");
          printf_BT_slow("  A measurement is required every 0.2 seconds.");
          printf_BT_slow("  Update rate should be set to 5 Hz.");
          printf_BT_slow("  User would enter \"05.0\" into terminal with no quotes.");
          print_n();
          print_input();

          while (!good_setting)
          {
            (void) get_uart_BT(buffer_S3_BT, 4, 0);
            print_n();
            float32_t new_update_rate = parse_float(buffer_S3_BT);

            if (!isnan(new_update_rate) && (new_update_rate >= 0.1f) && (new_update_rate <= 60.0f)) 
            {
              if (new_update_rate >= 20.0f) 
              {
                printf_BT_slow("WARNING: Actual update rate may be lower than desired.");
                printf_BT_slow("Test true update rate in configuration menu.");
                print_n();
              }

              printf_BT_slow("Update rate set to %04.1f Hz", new_update_rate);
              printf_BT_slow("Is this the desired value? Type Y for yes, any other character for no.");
              print_n();
              print_input();

              (void) get_uart_BT(buffer_S3_BT, 1, 0);
              print_n();
              char ack_char = tolower(buffer_S3_BT[0]);
              
              if (ack_char == 'y')
              {
                current_config.update_rate = new_update_rate;
                good_setting = true;
                printf_BT_slow("Value saved.");
                print_n();
                delay(1000);
                print_config_menu(&current_config);
              }
              else
              {
                printf_BT_slow("Please enter a new number between 00.1 and 60.0, inclusive.");
                print_n();
                print_input();
              }
            }
            else
            {
              printf_BT_slow("Invalid input. Please enter a number between 00.1 and 60.0, inclusive.");
              print_n();
              print_input();
            }
          }

          break;
        
        case 'l':
          print_n();
          printf_BT_slow("Set the maximum step length. Current value: %02i (%.1f mm)", current_config.max_step_length, (float) current_config.max_step_length*2.5f);
          printf_BT_slow("Format: XX");
          printf_BT_slow("  Input must match format exactly. Enter a single integer between 01 and 99, inclusive.");
          printf_BT_slow("Notes:");
          printf_BT_slow("  The maximum step length is set in multiples of 2.5mm. Multiply the inputed setting by 2.5mm");
          printf_BT_slow("  to get the maximum step length in mm.");
          printf_BT_slow("  Set the maximum step length to the desired resolution for measuring wave heights.");
          printf_BT_slow("  This is an upper bound - the distance algorithm may return finer resolution measurements.");
          printf_BT_slow("  Keep maximum step length as high as possible. Greatly affects power consumption and maximum possible update rate.");
          printf_BT_slow("Example:");
          printf_BT_slow("  The user wants to determine wave heights accurate to within 5mm.");
          printf_BT_slow("  The user should set the maximum step length to 2, since 2 * 2.5mm = 5mm.");
          printf_BT_slow("  User would enter \"2\" into terminal with no quotes.");
          print_n();
          print_input();

          while (!good_setting)
          {
            (void) get_uart_BT(buffer_S3_BT, 2, 0);
            print_n();
            uint8_t new_max_step = parse_int(buffer_S3_BT);

            if (new_max_step >= 1 && new_max_step <= 99) 
            {
              printf_BT_slow("Maximum step length set to %02i (%.1f mm)", new_max_step, (float) new_max_step*2.5f);
              printf_BT_slow("Is this the desired value? Type Y for yes, any other character for no.");
              print_n();
              print_input();

              (void) get_uart_BT(buffer_S3_BT, 1, 0);
              print_n();
              char ack_char = tolower(buffer_S3_BT[0]);
              
              if (ack_char == 'y')
              {
                current_config.max_step_length = new_max_step;
                good_setting = true;
                printf_BT_slow("Value saved.");
                print_n();
                delay(1000);
                print_config_menu(&current_config);
              }
              else
              {
                printf_BT_slow("Please enter an integer between 1 and 99, inclusive.");
                print_n();
                print_input();
              }
            }
            else
            {
              printf_BT_slow("Invalid input. Please enter an integer between 1 and 99, inclusive.");
              print_n();
              print_input();
            }
          }

          break;

        case 'p':
          print_n();
          printf_BT_slow("Set the maximum measurement profile. Current value: %i", current_config.max_profile);
          printf_BT_slow("Format: X");
          printf_BT_slow("  Input must match format exactly. Enter a single integer between 1 and 5, inclusive.");
          printf_BT_slow("  Units are arbitrary.");
          printf_BT_slow("Notes:");
          printf_BT_slow("  Measurement profile is used to set the length and shape of the emitted radio pulse.");
          printf_BT_slow("  Higher profiles transmit more energy, increasing signal-to-noise ratio and range.");
          printf_BT_slow("  Keep profile as low as possible. Greatly affects power consumption and maximum possible update rate.");
          printf_BT_slow("Example:");
          printf_BT_slow("  The user starts the radar with maximum profile setting of 2, but can only detect waves half the time.");
          printf_BT_slow("  The user should increase the profile setting by 1, repeating until the waves are always detected.");
          printf_BT_slow("  User would enter \"3\" into terminal with no quotes.");
          print_n();
          print_input();

          while (!good_setting)
          {
            (void) get_uart_BT(buffer_S3_BT, 1, 0);
            print_n();
            int new_max_profile = parse_int(buffer_S3_BT);

            if (new_max_profile >= 1 && new_max_profile <= 5) 
            {
              printf_BT_slow("Maximum profile set to %i", new_max_profile);
              printf_BT_slow("Is this the desired value? Type Y for yes, any other character for no.");
              print_n();
              print_input();

              (void) get_uart_BT(buffer_S3_BT, 1, 0);
              print_n();
              char ack_char = tolower(buffer_S3_BT[0]);
              
              if (ack_char == 'y')
              {
                current_config.max_profile = new_max_profile;
                good_setting = true;
                printf_BT_slow("Value saved.");
                print_n();
                delay(1000);
                print_config_menu(&current_config);
              }
              else
              {
                printf_BT_slow("Please enter an integer between 1 and 5, inclusive.");
                print_n();
                print_input();
              }
            }
            else
            {
              printf_BT_slow("Invalid input. Please enter an integer between 1 and 5, inclusive.");
              print_n();
              print_input();
            }
          }
          break;

        case 'q':
          print_n();
          printf_BT_slow("Set the signal quality. Current value: %04.1f", current_config.signal_quality);
          printf_BT_slow("Format: XX.X");
          printf_BT_slow("  Input must match format exactly. Enter leading and trailing zeros as appropriate.");
          printf_BT_slow("  Value must be between 00.0 and 35.0, inclusive.");
          printf_BT_slow("  Units are arbitrary.");
          printf_BT_slow("Notes:");
          printf_BT_slow("  Higher signal quality results in a higher signal-to-noise ratio target, and increases the number of averages");
          printf_BT_slow("  taken for a single datapoint. This generally increases the accuracy of measurements.");
          printf_BT_slow("  Keep as low as possible. Affects power consumption and maximum possible update rate.");
          printf_BT_slow("Example:");
          printf_BT_slow("  The user starts the radar with signal quality setting of 15.0, but measurements appear inaccurate.");
          printf_BT_slow("  The user should increase the profile setting by 1.0, repeating until the measurements are accurate enough.");
          printf_BT_slow("  User would enter \"16.0\" into terminal with no quotes.");
          print_n();
          print_input();

          while (!good_setting)
          {
            (void) get_uart_BT(buffer_S3_BT, 4, 0);
            print_n();
            float32_t new_quality = parse_float(buffer_S3_BT);

            if (!isnan(new_quality) && new_quality >= 0.0f && new_quality <= 35.0f) 
            {
              printf_BT_slow("Signal quality set to %04.1f", new_quality);
              printf_BT_slow("Is this the desired value? Type Y for yes, any other character for no.");
              print_n();
              print_input();

              (void) get_uart_BT(buffer_S3_BT, 1, 0);
              print_n();
              char ack_char = tolower(buffer_S3_BT[0]);
              
              if (ack_char == 'y')
              {
                current_config.signal_quality = new_quality;
                good_setting = true;
                printf_BT_slow("Value saved.");
                print_n();
                delay(1000);
                print_config_menu(&current_config);
              }
              else
              {
                printf_BT_slow("Please enter a new number between 00.0 and 35.0.");
                print_n();
                print_input();
              }
            }
            else
            {
              printf_BT_slow("Invalid input. Please enter a number between 00.0 and 35.0.");
              print_n();
              print_input();
            }
          }
          break;

        
        case 'r':
          print_n();
          printf_BT_slow("Sets the reflector shape. Current value: %i", current_config.reflector_shape);
          printf_BT_slow("Format: X");
          printf_BT_slow("  Input must match format exactly.");
          printf_BT_slow("  Enter \"1\" to use planar reflector shape.");
          printf_BT_slow("  Enter \"0\" to use generic reflector shape.");
          printf_BT_slow("Notes:");
          printf_BT_slow("  Planar reflector shape should be used for determining the distance to flat objects like water surfaces,");
          printf_BT_slow("  sheets of metal, walls, and more.");
          printf_BT_slow("  Generic reflector shape should be used for determining the distance to any other objects.");
          printf_BT_slow("Example:");
          printf_BT_slow("  The radar is installed on the edge of a pier above a body of water.");
          printf_BT_slow("  Planar reflector shape should be used, and the user would enter \"1\" with no quotes.");
          print_n();
          print_input();

          while (!good_setting)
          {
            (void) get_uart_BT(buffer_S3_BT, 1, 0);
            print_n();
            int new_shape = parse_int(buffer_S3_BT);

            if (new_shape == 1 || new_shape == 0) 
            {
              printf_BT_slow("Reflector shape set to %i", new_shape);
              printf_BT_slow("Is this the desired value? Type Y for yes, N for no.");
              print_n();
              print_input();

              (void) get_uart_BT(buffer_S3_BT, 1, 0);
              print_n();
              char ack_char = tolower(buffer_S3_BT[0]);
              
              if (ack_char == 'y')
              {
                current_config.reflector_shape = new_shape;
                good_setting = true;
                printf_BT_slow("Value saved.");
                print_n();
                delay(1000);
                print_config_menu(&current_config);
              }
              else
              {
                printf_BT_slow("Please enter either a 0 or a 1.");
                print_n();
                print_input();
              }
            }
            else
            {
              printf_BT_slow("Invalid input. Please enter either a 0 or a 1.");
              print_n();
              print_input();
            }
          }
          break;

        case 't':
          print_n();
          printf_BT_slow("Set the threshold sensitivity. Current value: %04.2f", current_config.threshold_sensitivity);
          printf_BT_slow("Format: X.XX");
          printf_BT_slow("  Input must match format exactly. Enter leading and trailing zeros as appropriate.");
          printf_BT_slow("  Value must be between 0.00 and 1.00, inclusive.");
          printf_BT_slow("  Units are arbitrary.");
          printf_BT_slow("Notes:");
          printf_BT_slow("  Controls the false positive rate. A higher threshold means that more energy must be reflected from an object");
          printf_BT_slow("  to record a distance measurement.");
          printf_BT_slow("  Tune in conjunction with the signal quality and measurement profile to eliminate background objects.");
          printf_BT_slow("Example:");
          printf_BT_slow("  The user starts the radar with threshold sensitivity setting of 0.5, measuring water level over the side of a pier.");
          printf_BT_slow("  The radar keeps detecting the distance to the metal beams supporting the pier, which messes up the measurements.");
          printf_BT_slow("  The user should decrease the sensitivity by 0.1, adjusting until the beams are no longer detected by the radar.");
          printf_BT_slow("  User would enter \"0.4\" into terminal with no quotes.");
          print_n();
          print_input();

          while (!good_setting)
          {
            (void) get_uart_BT(buffer_S3_BT, 4, 0);
            print_n();
            float32_t new_threshold = parse_float(buffer_S3_BT);

            if (!isnan(new_threshold) && new_threshold >= 0.0f && new_threshold <= 1.0f) 
            {
              printf_BT_slow("Signal quality set to %04.2f", new_threshold);
              printf_BT_slow("Is this the desired value? Type Y for yes, any other character for no.");
              print_n();
              print_input();

              (void) get_uart_BT(buffer_S3_BT, 1, 0);
              print_n();
              char ack_char = tolower(buffer_S3_BT[0]);
              
              if (ack_char == 'y')
              {
                current_config.threshold_sensitivity = new_threshold;
                good_setting = true;
                printf_BT_slow("Value saved.");
                print_n();
                delay(1000);
                print_config_menu(&current_config);
              }
              else
              {
                printf_BT_slow("Please enter a new number between 0.00 and 1.00.");
                print_n();
                print_input();
              }
            }
            else
            {
              printf_BT_slow("Invalid input. Please enter a number between 0.00 and 1.00.");
              print_n();
              print_input();
            }
          }
          break;

        case 'a':
          if (current_config.start_m >= current_config.end_m)
          {
            print_n();
            printf_BT_slow("WARNING: Start of range is greater than end of range! This must be fixed before continuing.");
            print_n();
            printf_BT_slow("Please send one of the characters from the menu above.");
            print_n();
            print_input();
          }
          else
          {
            current_config.testing_update_rate = 1;
            if (!send_config_stm32(&current_config))
            {
              break;
            }
            printf_BT_slow("Testing actual update rate...");
            state = STATE_LISTENING;
          }
          break;

        case 'x':
          print_n();
          printf_BT_slow("Send any character to re-open configuration menu.");
          print_n();
          delay(3000);

          if (current_config.start_m >= current_config.end_m)
          {
            printf_BT_slow("WARNING: Start of range is greater than end of range! This must be fixed before continuing.");
            print_n();
            printf_BT_slow("Please send one of the characters from the menu above.");
            print_n();
            print_input();
          }
          else
          {
            if (!save_config_adalogger(&current_config)){
              printf_BT_slow("Failed to save configuration to SD card. Please try again.");
              print_n();
              break;
            }
            printf_BT_slow("Configuration saved to SD card. Returning to normal operations.");
            delay(3000);
            print_n();
            if (!send_config_stm32(&current_config))
            {
              break;
            }
            state = STATE_LISTENING;
          }
          break;

        default:
          print_n();
          printf_BT_slow("Input is not accepted. Please send one of the characters above.");
          print_n();
          print_input();
          break;
      }
    }
  }

  // --- STATE inf: SOMETHING_WENT_WRONG ---
  else
  {
    state = STATE_LISTENING;
  }
}


bool get_uart_BT(char *result, uint16_t buf_size, uint32_t timeout)
{
  if (BT_on)
  {
    uint16_t received = 0;
    uint32_t startTime = millis();

    while (received < buf_size) {
      if (SerialBT.available()) {
        char c = SerialBT.read();
        result[received] = c;
        SerialBT.print(c);
        Serial.print(c);
        received++;
        startTime = millis(); // Reset timeout

        if (c == '\r' || c == '\n') {
          result[received - 1] = '\0'; // Replace newline with null terminator
          break; // End of message
        }
      }

      // Check for timeout (if timeout == 0, then it never times out)
      if ((millis() - startTime > timeout) && (timeout != 0)) {
        if (received == 0) {
          return false; // No data received within timeout
        }
        break; // Partial message received
      }
    }

    result[received] = '\0'; // Ensure null-termination
    SerialBT.println("");
    Serial.println("");
    return true;
  }
  return false;
}


bool read_until_newline_stm32(char* buffer, uint8_t buffer_size)
{
  uint8_t index = 0;
  char c;
  
  while (Serial32.available() && (index < buffer_size - 1)) 
  {
    c = Serial32.read();
    
    if (c == '\n') {
      break;  // End of line reached
    }
    
    if (c != '\r') {
      buffer[index++] = c;
    }
  }

  if (index == 0) return false;

  buffer[index] = '\0';  // Null-terminate the string
  return true;
}


uint16_t check_for_m_code(const char* buffer)
{
  for (uint8_t i = 0; buffer[i] != '\0'; i++) {

    // Check if current character is 'M' and there are at least 3 more characters
    if (buffer[i] == 'M' && isdigit(buffer[i+1]) && isdigit(buffer[i+2]) && isdigit(buffer[i+3])) {

      // Extract the numeric part of the M-code
      uint16_t code = (buffer[i+1] - '0') * 100 + (buffer[i+2] - '0') * 10 + (buffer[i+3] - '0');
      
      return code;
    }
  }

  return 0;
}


float32_t parse_float(const char *string)
{
  char *end;
  float result = strtof(string, &end);
  if (end == string || *end != '\0') {
    // Parsing failed
    return NAN;
  }
  return result;
}


uint8_t parse_int(const char *string)
{
  char *end;
  long result = strtol(string, &end, 10);
  if (end == string || *end != '\0') {
    // Parsing failed
    return 255;
  }
  return (uint8_t)result;
}


bool stop_radar(uint32_t delay_time, uint32_t timeout)
{
  float32_t time_s = (float32_t) delay_time / 1000.0f;
  printf_stm32("");
  printf_stm32("Stopping radar...");
  printf_stm32("Process will take approximately %.1f seconds.", time_s);

  // disable UART
  uart_driver_delete(UART_NUM_2);
  
  // configure TX_PIN as output and set it LOW
  // STM32 constantly checks if its RX_PIN is LOW during active radar state
  gpio_set_direction(GPIO_NUM_17, GPIO_MODE_OUTPUT);
  gpio_set_level(GPIO_NUM_17, 0);
  
  // wait for delay time
  uint32_t start_time = millis();
  while ((millis() - start_time) < delay_time);

  // restart UART
  Serial32.begin(921600UL, SERIAL_8N1, 16, 17);

  // check for M918 code until timeout
  start_time = millis();
  char buffer_stop_radar[MAX_BUFFER_SIZE]; 
  while ((millis() - start_time) < timeout)
  {
    if (read_until_newline_stm32(buffer_stop_radar, MAX_BUFFER_SIZE))
    {
      Serial.printf("%s\n", buffer_stop_radar);
      uint16_t code_stop_radar = check_for_m_code(buffer_stop_radar);
      if (code_stop_radar == 918)
      {
        Serial32.print("M919");
        printf_stm32("");
        printf_stm32("Radar stopped.");
        printf_stm32("");
        return true;
      }
    }
  }

  printf_stm32("");
  printf_stm32("Failed to stop radar.");
  printf_stm32("");
  
  return false;
}


bool read_config_adalogger(ConfigSettings *config)
{
  config->start_m = 0.20f;
  config->end_m = 0.50f;
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


bool save_config_adalogger(ConfigSettings *config)
{
  return true;
}


bool send_config_stm32(ConfigSettings *config)
{
  sending_config = true;
  char config_string[64];
  printf_debug("Sending config to STM...");
  snprintf(config_string, sizeof(config_string), 
           "%05.2f,%05.2f,%04.1f,%02d,%d,%04.1f,%d,%04.2f,%d,%04.1f\r",
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
  printf_debug("Sent from ESP32:     %s", config_string);

  char buffer_config_send[MAX_BUFFER_SIZE];
  unsigned long start_time = millis();
  while ((millis() - start_time) < 2500){
    if (Serial32.available()){
      if (read_until_newline_stm32(buffer_config_send, MAX_BUFFER_SIZE))
      {
        printf_debug("Received from STM32: %s", buffer_config_send);
        uint16_t code_config_send = check_for_m_code(buffer_config_send);
        if (code_config_send == 808)
        {
          printf_stm32("");
          printf_stm32("Configuration received and loaded onto radar sensor.");
          printf_stm32("");
          sending_config = false;
          return true;
        }
        else if (code_config_send == 806) {
          printf_stm32("");
          printf_stm32("Configuration sent to STM32 contains errors!");
          printf_stm32("");
          sending_config = false;
          return false;
        }
      }
    }
  }
  printf_stm32("Failed to receive configuration message.");
  sending_config = false;
  return false;
}


bool save_new_update_rate(ConfigSettings *config)
{
  Serial32.print("M816");

  char buffer_815[MAX_BUFFER_SIZE];
  uint32_t start_time_815 = millis();
  config->testing_update_rate = false;

  while ((millis() - start_time_815) < 5000)
  {
    if (read_until_newline_stm32(buffer_815, MAX_BUFFER_SIZE))
    {
      float32_t new_update_rate = parse_float(buffer_815);
      printf_BT_slow("Actual update rate measured as %02.1f Hz.", new_update_rate);
      config->true_update_rate = new_update_rate;
      config->testing_update_rate = false;
      return true;
    }
  }

  printf_BT_slow("Failed to measure actual update rate, please try again.");
  return false;
}


void printf_BT_slow(const char *format, ...)
{
  va_list args;
  char buf[256]; // Adjust size as needed

  va_start(args, format);
  (void) vsnprintf(buf, sizeof(buf), format, args);
  va_end(args);

  if (BT_on)
  {
    SerialBT.print("ESP32: ");
    SerialBT.print(buf);
    SerialBT.println();
    delay(100);
  }

  Serial.print("ESP32: ");
  Serial.print(buf);
  Serial.println();
}


void print_input(void)
{
  SerialBT.print("Input: ");
}


void print_n(void)
{
  printf_BT_slow("");
}


void printf_debug(const char *format, ...)
{
  va_list args;
  char buf[256]; // Adjust size as needed

  va_start(args, format);
  (void) vsnprintf(buf, sizeof(buf), format, args);
  va_end(args);

  if (BT_on && PRINT_DEBUG_TO_BT)
  {
    SerialBT.print("ESP32: ");
    SerialBT.print(buf);
    SerialBT.println();
  }

  Serial.print("ESP32: ");
  Serial.print(buf);
  Serial.println();
}

void printf_debug_stm32(const char *format, ...)
{
  va_list args;
  char buf[256]; // Adjust size as needed

  va_start(args, format);
  (void) vsnprintf(buf, sizeof(buf), format, args);
  va_end(args);

  if (BT_on && PRINT_DEBUG_TO_BT)
  {
    SerialBT.print("STM32: ");
    SerialBT.print(buf);
    SerialBT.println();
  }

  Serial.print("STM32: ");
  Serial.print(buf);
  Serial.println();
}

void printf_stm32(const char *format, ...)
{
  va_list args;
  char buf[256]; // Adjust size as needed

  va_start(args, format);
  (void) vsnprintf(buf, sizeof(buf), format, args);
  va_end(args);

  if (BT_on)
  {
    SerialBT.print("STM32: ");
    SerialBT.print(buf);
    SerialBT.println();
  }

  Serial.print("STM32: ");
  Serial.print(buf);
  Serial.println();
}

void print_config_menu(ConfigSettings *config)
{
  printf_BT_slow("--- CONFIGURATION MENU ---");
  printf_BT_slow("Send one of the below characters to change that configuration setting.");
  printf_BT_slow("Changes will be saved automatically.");
  printf_BT_slow("S: Change the start of the measurement range. Affects power consumption and update rate.");
  printf_BT_slow("   Current value: %05.2f m", config->start_m);
  printf_BT_slow("E: Change the end of the measurement range. Affects power consumption and update rate.");
  printf_BT_slow("   Current value: %05.2f m", config->end_m);
  printf_BT_slow("U: Change the update rate. Affects power consumption.");
  printf_BT_slow("   Current value: %04.1f Hz", config->update_rate);
  printf_BT_slow("L: Change the maximum step length of the measurements. Affects power consumption, update rate, and resolution.");
  printf_BT_slow("   Current value: %02i (%.1f mm)", config->max_step_length, (float) config->max_step_length*2.5f);
  printf_BT_slow("P: Change the maximum measurement profile. Affects power consumption, update rate, and accuracy.");
  printf_BT_slow("   Current value: %i", config->max_profile);
  printf_BT_slow("Q: Change the signal quality. Affects power consumption, update rate, and accuracy.");
  printf_BT_slow("   Current value: %04.1f", config->signal_quality);
  printf_BT_slow("R: Change the reflector shape.");
  printf_BT_slow("   Current value: %i (0: generic, 1: planar)", config->reflector_shape);
  printf_BT_slow("T: Change the threshold sensitivity. Affects accuracy.");
  printf_BT_slow("   Current value: %04.2f", config->threshold_sensitivity);
  printf_BT_slow("A: Measure the actual update rate.");
  printf_BT_slow("   True update rate: %04.1f Hz", config->true_update_rate);
  printf_BT_slow("X: Exit the menu.");
  print_n();
  print_input();
}


void print_CP(void)
{
  printf_BT_slow("");
  printf_BT_slow("       :+###%%#%%#%%#%%#%%###*:              ");
  printf_BT_slow("     -#%%*==------------=*%%#:            ");
  printf_BT_slow("    =%%*-:::::::::::::::::=%%+            ");
  printf_BT_slow("   :%%#-::::::::::::::::::*%%=            ");
  printf_BT_slow("   +%%=::::::::**-:::::::-##:.......     ");
  printf_BT_slow("  .%%#-:::::::=%%%%%%%%#%%####%%%%%%%%%%%%%%%%%%%%%%%%%%%%: ");
  printf_BT_slow("  *%%=::::::::*%%%%%%#=:::::::::::::::::-*%%=");
  printf_BT_slow(" :%%*-:::::::-#%%%%%%%%+::::::::::::::::::=##");
  printf_BT_slow(" *%%+::::::::*%%%%%%%%#+==========::::::::+%%+");
  printf_BT_slow(":##-:::::::-#*=------=+%%%%%%%%%%=:::::::-##:");
  printf_BT_slow("+%%+::::::::-=:::::::::*%%%%%%%%*::::::::+%%* ");
  printf_BT_slow("##=::::::::::::::::::+%%%%%%%%#=:::::::-#%%: ");
  printf_BT_slow("+%%*-:::::::::::::::=*%%%%%%%%%%*::::::::+%%+  ");
  printf_BT_slow(" =%%%%%%%%#########***********-:::::::-##:  ");
  printf_BT_slow("     .......:%%#-::::::::::::::::::*%%=   ");
  printf_BT_slow("            *%%=:::::::::::::::::-*%%*    ");
  printf_BT_slow("           -%%*::::::::=*******#%%%%#:     ");
  printf_BT_slow("          .#%%=::::::::*%%*====-:.        ");
  printf_BT_slow("          *%%*::::::::=%%%%                ");
  printf_BT_slow("        :%%#+-:::::::=#%%:                ");
  printf_BT_slow("       -%%%%%%%%%%%%%%%%%%%%%%%%%%%%=                 ");
  printf_BT_slow("");
  delay(500);
}


void setup_rtc()
{
  if (!rtc.begin())
  {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    while (1) delay(10);
  }

  if (!rtc.initialized() || rtc.lostPower()) 
  {
    Serial.println("RTC is NOT initialized, let's set the time!");
    delay(3000);
    // When time needs to be set on a new device, or after a power loss, the
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
    //
    // Note: allow 2 seconds after inserting battery or applying external power
    // without battery before calling adjust(). This gives the PCF8523's
    // crystal oscillator time to stabilize. If you call adjust() very quickly
    // after the RTC is powered, lostPower() may still return true.
  }

  rtc.start();

  DateTime now = rtc.now();
  Serial.print(now.year(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.day(), DEC);
  Serial.print(" ");
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.print(now.second(), DEC);
  Serial.println();
}


void list_dir(fs::FS &fs, const char *dirname, uint8_t levels) {
  Serial.printf("Listing directory: %s\n", dirname);

  File root = fs.open(dirname);
  if (!root) {
    Serial.println("Failed to open directory");
    return;
  }
  if (!root.isDirectory()) {
    Serial.println("Not a directory");
    return;
  }

  File file = root.openNextFile();
  while (file) {
    if (file.isDirectory()) {
      Serial.print("  DIR : ");
      Serial.println(file.name());
      if (levels) {
        list_dir(fs, file.name(), levels - 1);
      }
    } else {
      Serial.print("  FILE: ");
      Serial.print(file.name());
      Serial.print("  SIZE: ");
      Serial.println(file.size());
    }
    file = root.openNextFile();
  }
}


void read_file(fs::FS &fs, const char *path) {
  Serial.printf("Reading file: %s\n", path);

  File file = fs.open(path);
  if (!file) {
    Serial.println("Failed to open file for reading");
    return;
  }

  Serial.print("Read from file: ");
  while (file.available()) {
    Serial.write(file.read());
  }
  file.close();
}


void write_file(fs::FS &fs, const char *path, const char *message) {
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  if (file.print(message)) {
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  file.close();
}


void append_file(fs::FS &fs, const char *path, const char *message) {
  Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  if (file.print(message)) {
    Serial.println("Message appended");
  } else {
    Serial.println("Append failed");
  }
  file.close();
}


void rename_file(fs::FS &fs, const char *path1, const char *path2) {
  Serial.printf("Renaming file %s to %s\n", path1, path2);
  if (fs.rename(path1, path2)) {
    Serial.println("File renamed");
  } else {
    Serial.println("Rename failed");
  }
}


void deleteFile(fs::FS &fs, const char *path) {
  Serial.printf("Deleting file: %s\n", path);
  if (fs.remove(path)) {
    Serial.println("File deleted");
  } else {
    Serial.println("Delete failed");
  }
}


void setup_sd_card(void)
{
  Serial.println("\n\n\nInitializing SPI...");
  // SPI.begin(5, 19, 18, 33);
  delay(3000);

  Serial.print("Initializing SD card...");

  if (!SD.begin()) {
    Serial.println("initialization failed. Things to check:");
    Serial.println("1. is a card inserted?");
    Serial.println("2. is your wiring correct?");
    Serial.println("3. did you change the chipSelect pin to match your shield or module?");
    Serial.println("Note: press reset button on the board and reopen this Serial Monitor after fixing your issue!");
    while (true);
  }

  Serial.println("initialization done.\n");

  uint8_t cardType = SD.cardType();

  if (cardType == CARD_NONE) {
    Serial.println("No SD card attached");
    return;
  }

  Serial.print("SD Card Type: ");
  if (cardType == CARD_MMC) {
    Serial.println("MMC");
  } else if (cardType == CARD_SD) {
    Serial.println("SDSC");
  } else if (cardType == CARD_SDHC) {
    Serial.println("SDHC");
  } else {
    Serial.println("UNKNOWN");
  }

  list_dir(SD, "/", 0);
  Serial.printf("Total space: %lluMB\n", SD.totalBytes() / (1024 * 1024));
  Serial.printf("Used space: %lluMB\n", SD.usedBytes() / (1024 * 1024));
}
