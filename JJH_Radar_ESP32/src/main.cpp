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
#include <inttypes.h>

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
#define DEBUG_BUFFER_SIZE 4096
#define BT_TIMEOUT_MINS 10U
#define GPS_DELAY_MINS 60U
#define MAX_LINES_PER_DATA_FILE 1000000UL
#define DEVICE_NAME "ESP32_HUZZAH32"
#define PRINT_DEBUG_TO_BT false
#define GPS_RX 32
#define GPS_TX 14
#define FLAG_LED_PIN 27


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
  uint8_t text_width;
  char latitude[32];
  char longitude[32];
  char elevation[16];
} ConfigSettings;

typedef struct
{
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
  uint16_t millisecond;
} DateTimeMS;

struct GPSData {
  int hour;
  int minute;
  int second;
  char latitude[32];
  char longitude[32];
  char elevation[16];
};


// --- GLOBAL VARIABLES ---

// serial connections
String deviceName = DEVICE_NAME;
BluetoothSerial SerialBT;
HardwareSerial Serial32(2);
HardwareSerial gpsSerial(1);

// printing stuff
bool BT_on = true;
bool gibberish_blocker = false;
bool printing_config = false;
uint8_t text_width = 40;
bool serial_plotter = false;
uint32_t serial_plotter_init;

// config settings struct
ConfigSettings current_config;

// data-logging
RTC_PCF8523 rtc;
const uint8_t chip_select = 33;
bool SD_on = false;
char* debug_file_path = nullptr;
char* data_file_path = nullptr;
char debug_buffer[DEBUG_BUFFER_SIZE];
size_t debug_buffer_pos = 0;
uint32_t last_flush_time = 0;
const uint32_t FLUSH_INTERVAL = 5000U; // Flush every 5 seconds
uint32_t lines_saved = 0;

// state-tracking
uint8_t state = STATE_LOAD_CONFIG_SD;
bool sending_config = false;
bool first_data_since_powercycle = true;

// time-keeping and GPS
uint32_t start_time_BT = millis();
DateTimeMS init_time;
uint32_t init_millis;
uint32_t init_814;
bool gps_enable = true;
uint32_t start_time_GPS = millis();
GPSData gps_data;

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
void printf_datalog(const char *format, ...);
void setup_rtc(void);
void setup_sd_card(void);
void list_dir(fs::FS &fs, const char *dirname, uint8_t levels);
void create_dir(fs::FS &fs, const char *path);
void read_file(fs::FS &fs, const char *path);
void append_file(fs::FS &fs, const char *path, const char *message);
void delete_file(fs::FS &fs, const char *path);
bool start_new_debug_file(char** debug_file_path);
bool start_new_data_file(char** data_file_path);
void flush_debug_buffer(void);
void check_sizes(void);
DateTimeMS timekeeper(DateTimeMS initial_time, uint32_t initial_millis);
uint8_t days_in_month(uint8_t month, uint16_t year);
bool is_leap_year(uint16_t year);
void set_init_time(DateTimeMS *initial_time, uint32_t *initial_millis);
void printf_serial_plotter(const char *format, ...);
void convertDMtoDMS(float decimal_degrees, char *result, size_t size, bool isLat, char direction);
bool parseGPGGA(const char* sentence, GPSData* data);
void processGPSData(GPSData &gps_data);
void gpsSetPowerOff(void); 
void gpsSetPowerOn(void); 
int8_t getUTCOffset(void);
bool isDST(uint16_t year, uint8_t month, uint8_t day, uint8_t hour);
void syncRTCWithGPS(GPSData* gps_data);
void cpslo(unsigned long time);


// --- STATE 0: INIT_ESP ---
void setup()
{
  // start wired USB serial connection
  Serial.begin(115200);

  // start Bluetooth serial connection
  SerialBT.begin(deviceName);
  printf_debug("\nThe device with name \"%s\" is started. Now you can pair it with Bluetooth!\n", deviceName.c_str());
  printf_debug("Bluetooth automatically turns off after %i minutes! Turn back on by hitting ESP32 reset button.\n", BT_TIMEOUT_MINS);

  // start STM32 serial connection
  Serial32.begin(921600UL, SERIAL_8E1, 16, 17);

  // check sizes
  check_sizes();

  // set up RTC
  setup_rtc();
  
  // set up GPS
  gpsSerial.begin(9600UL, SERIAL_8N1, GPS_RX, GPS_TX);
  gpsSetPowerOn();

  // set initial time for datalogging
  set_init_time(&init_time, &init_millis);

  // set up SD card
  setup_sd_card();

  // start a new debug file
  if (start_new_debug_file(&debug_file_path))
  {
    flush_debug_buffer();
  }
  
  // set up LED pin, flash for visual indication
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(FLAG_LED_PIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  digitalWrite(FLAG_LED_PIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(FLAG_LED_PIN, LOW);
  
  // turn LED on until Bluetooth turns off
  delay(3000);
  digitalWrite(LED_BUILTIN, HIGH);
  start_time_BT = millis();
}

// --- MAIN LOOP ---
void loop()
{
  // check if Bluetooth timeout is exceeded; if so, shut off Bluetooth to save power
  if (BT_on && (millis() - start_time_BT) > ((uint32_t) BT_TIMEOUT_MINS * 60000UL))
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
      if (strcmp(current_config.latitude, "Not Set") != 0 && strcmp(current_config.longitude, "Not Set") != 0) {
        strncpy(gps_data.latitude, current_config.latitude, sizeof(gps_data.latitude) - 1);
        strncpy(gps_data.longitude, current_config.longitude, sizeof(gps_data.longitude) - 1);
        strncpy(gps_data.elevation, current_config.elevation, sizeof(gps_data.elevation) - 1);
        gps_data.latitude[sizeof(gps_data.latitude) - 1] = '\0';
        gps_data.longitude[sizeof(gps_data.longitude) - 1] = '\0';
        gps_data.elevation[sizeof(gps_data.elevation) - 1] = '\0';
        printf_debug("Loaded previous GPS coordinates: %s %s", gps_data.latitude, gps_data.longitude);
      } else {
        printf_debug("No previous GPS coordinates found in config");
      }
    }
    else
    {
      printf_BT_slow("ERROR: Failed to load config from SD card. Using default config.");
    }
    flush_debug_buffer();

    // empty the UART buffer
    char buffer_S1[MAX_BUFFER_SIZE];
    read_until_newline_stm32(buffer_S1, MAX_BUFFER_SIZE);

    state = STATE_LISTENING;
  }

  
  // --- STATE 2: LISTENING ---
  else if (state == STATE_LISTENING)
  {
    const unsigned long PERIOD = 16000;  // Length of complete CP sequence
    unsigned long t = millis() % PERIOD;  // Current position in sequence
    
    cpslo(t);

    char buffer_S2[MAX_BUFFER_SIZE];

    // check if there's data from STM32 UART
    if (read_until_newline_stm32(buffer_S2, MAX_BUFFER_SIZE))
    {
      uint16_t code_S2 = check_for_m_code(buffer_S2);
      DateTimeMS d_now;
      uint32_t end_814;
      char* m_code_pos = nullptr;

      switch (code_S2)
      {
        // No M-code: just print message to BT
        case 0:
          if (!gibberish_blocker)
          {
            printf_debug_stm32("%s", buffer_S2);
          }
          break;
        
        // M114: Denotes start of data
        case 114:
          printf_debug_stm32("M114");
          gibberish_blocker = false;
          printf_stm32("DATA_START");
          (void) start_new_data_file(&data_file_path);
          d_now = timekeeper(init_time, init_millis);

          // Save configuration to data file 
          if (first_data_since_powercycle){
            printf_datalog("First Data File Since Powering On: True");
          }
          else{
            printf_datalog("First Data File Since Powering On: False");
          }
          printf_datalog("Data File: %s", data_file_path);
          printf_datalog("Start Time: [%02d/%02d/%02d %02d:%02d:%02d.%03u]",
                        d_now.year % 100, d_now.month, d_now.day, 
                        d_now.hour, d_now.minute, d_now.second, d_now.millisecond);
          printf_datalog("Location: %s %s", gps_data.latitude, gps_data.longitude);  
          printf_datalog("Elevation: %s", gps_data.elevation);
          printf_datalog("Start of range: %.2f m", current_config.start_m);
          printf_datalog("End of range: %.2f m", current_config.end_m);
          printf_datalog("Update rate: %.1f Hz", current_config.update_rate);
          printf_datalog("Maximum step length: %d (%.1f mm)", current_config.max_step_length, 
                        (float)current_config.max_step_length * 2.5f);
          printf_datalog("Maximum profile: %d", current_config.max_profile);
          printf_datalog("Signal quality: %.1f", current_config.signal_quality);
          printf_datalog("Reflector shape: %d (0: generic, 1: planar)", current_config.reflector_shape);
          printf_datalog("Threshold sensitivity: %.2f", current_config.threshold_sensitivity);
          printf_datalog("True update rate: %.1f Hz", current_config.true_update_rate);
          printf_datalog("Text width: %d characters", current_config.text_width);
          printf_datalog("---"); // Add a separator line

          // Print to debug file as well
          printf_debug("---------- ---------- ---------- ---------- -------- ---------- ---------- ---------- ----------");
          printf_debug("---------- ---------- ---------- ---------- NEW DATA ---------- ---------- ---------- ----------");
          printf_debug("---------- ---------- ---------- ---------- -------- ---------- ---------- ---------- ----------");
          if (first_data_since_powercycle){
            printf_debug("First Data File Since Powering On: True");
          }
          else{
            printf_debug("First Data File Since Powering On: False");
          }
          printf_debug("Data File: %s", data_file_path);
          printf_debug("Start Time: [%02d/%02d/%02d %02d:%02d:%02d.%03u]",
                        d_now.year % 100, d_now.month, d_now.day, 
                        d_now.hour, d_now.minute, d_now.second, d_now.millisecond);
          printf_debug("Location: %s %s", gps_data.latitude, gps_data.longitude);
          printf_debug("Elevation: %s", gps_data.elevation);
          printf_debug("Start of range: %.2f m", current_config.start_m);
          printf_debug("End of range: %.2f m", current_config.end_m);
          printf_debug("Update rate: %.1f Hz", current_config.update_rate);
          printf_debug("Maximum step length: %d (%.1f mm)", current_config.max_step_length, 
                        (float)current_config.max_step_length * 2.5f);
          printf_debug("Maximum profile: %d", current_config.max_profile);
          printf_debug("Signal quality: %.1f", current_config.signal_quality);
          printf_debug("Reflector shape: %d (0: generic, 1: planar)", current_config.reflector_shape);
          printf_debug("Threshold sensitivity: %.2f", current_config.threshold_sensitivity);
          printf_debug("True update rate: %.1f Hz", current_config.true_update_rate);
          printf_debug("Text width: %d characters", current_config.text_width);
          printf_debug("---"); // Add a separator line

          first_data_since_powercycle = false;
          flush_debug_buffer();

        // M115: Precedes new distance data
        case 115:
          gibberish_blocker = false;
          m_code_pos = strstr(buffer_S2, "M115 ");

          if (m_code_pos != nullptr) 
          {
            // Move the pointer to the character after "M115 "
            char* data_start = m_code_pos + 5;
            
            if (serial_plotter)
            {
              // Find the position of the next space
              char* space_pos = strchr(data_start, ' ');
              
              if (space_pos != nullptr)
              {
                // Null-terminate the string at the space position
                *space_pos = '\0';
              }

              printf_serial_plotter("%s", data_start);
            }
            else
            {

              d_now = timekeeper(init_time, init_millis);
              if (!current_config.testing_update_rate)
              {
                printf_datalog("[%02d/%02d/%02d %02d:%02d:%02d.%03u] %s",
                              d_now.year % 100, d_now.month, d_now.day, 
                              d_now.hour, d_now.minute, d_now.second, d_now.millisecond,
                              data_start);
              }
              else
              {
                printf_debug_stm32("[%02d/%02d/%02d %02d:%02d:%02d.%03u] %s",
                                    d_now.year % 100, d_now.month, d_now.day, 
                                    d_now.hour, d_now.minute, d_now.second, d_now.millisecond,
                                    data_start);
              }
              lines_saved++;
              if (lines_saved >= MAX_LINES_PER_DATA_FILE)
              {
                (void) start_new_data_file(&data_file_path);

                // Save configuration to data file 
                if (first_data_since_powercycle){
                  printf_datalog("First Data File Since Powering On: True");
                }
                else{
                  printf_datalog("First Data File Since Powering On: False");
                }
                printf_datalog("Data File: %s", data_file_path);
                printf_datalog("Start Time: [%02d/%02d/%02d %02d:%02d:%02d.%03u]",
                              d_now.year % 100, d_now.month, d_now.day, 
                              d_now.hour, d_now.minute, d_now.second, d_now.millisecond);
                printf_datalog("Location: %s %s", gps_data.latitude, gps_data.longitude);
                printf_datalog("Elevation: %s", gps_data.elevation);
                printf_datalog("Start of range: %.2f m", current_config.start_m);
                printf_datalog("End of range: %.2f m", current_config.end_m);
                printf_datalog("Update rate: %.1f Hz", current_config.update_rate);
                printf_datalog("Maximum step length: %d (%.1f mm)", current_config.max_step_length, 
                              (float)current_config.max_step_length * 2.5f);
                printf_datalog("Maximum profile: %d", current_config.max_profile);
                printf_datalog("Signal quality: %.1f", current_config.signal_quality);
                printf_datalog("Reflector shape: %d (0: generic, 1: planar)", current_config.reflector_shape);
                printf_datalog("Threshold sensitivity: %.2f", current_config.threshold_sensitivity);
                printf_datalog("True update rate: %.1f Hz", current_config.true_update_rate);
                printf_datalog("Text width: %d characters", current_config.text_width);
                printf_datalog("---"); // Add a separator line

                // Print to debug file as well
                printf_debug("---------- ---------- ---------- ---------- -------- ---------- ---------- ---------- ----------");
                printf_debug("---------- ---------- ---------- ---------- NEW DATA ---------- ---------- ---------- ----------");
                printf_debug("---------- ---------- ---------- ---------- -------- ---------- ---------- ---------- ----------");
                if (first_data_since_powercycle){
                  printf_debug("First Data File Since Powering On: True");
                }
                else{
                  printf_debug("First Data File Since Powering On: False");
                }
                printf_debug("Data File: %s", data_file_path);
                printf_debug("Start Time: [%02d/%02d/%02d %02d:%02d:%02d.%03u]",
                              d_now.year % 100, d_now.month, d_now.day, 
                              d_now.hour, d_now.minute, d_now.second, d_now.millisecond);
                printf_debug("Location: %s %s", gps_data.latitude, gps_data.longitude);
                printf_debug("Elevation: %s", gps_data.elevation);
                printf_debug("Start of range: %.2f m", current_config.start_m);
                printf_debug("End of range: %.2f m", current_config.end_m);
                printf_debug("Update rate: %.1f Hz", current_config.update_rate);
                printf_debug("Maximum step length: %d (%.1f mm)", current_config.max_step_length, 
                              (float)current_config.max_step_length * 2.5f);
                printf_debug("Maximum profile: %d", current_config.max_profile);
                printf_debug("Signal quality: %.1f", current_config.signal_quality);
                printf_debug("Reflector shape: %d (0: generic, 1: planar)", current_config.reflector_shape);
                printf_debug("Threshold sensitivity: %.2f", current_config.threshold_sensitivity);
                printf_debug("True update rate: %.1f Hz", current_config.true_update_rate);
                printf_debug("Text width: %d characters", current_config.text_width);
                printf_debug("---"); // Add a separator line
                lines_saved = 0;
                flush_debug_buffer();
              }
            }
          }
          break;
        
        // M116: Denotes end of data
        case 116:
          printf_debug_stm32("M116");
          gibberish_blocker = false;
          printf_stm32("DATA_END");
          break;

        // M317: No distance data returned for measurement
        case 317:
          Serial.println("STM32: no_dists");
          if (BT_on && !current_config.testing_update_rate) SerialBT.println("STM32: no_dists");
          gibberish_blocker = false;
          break;

        // M807: New config data requested
        // we're not in the config menu, so just send current config
        case 807:
          printf_debug_stm32("M807");
          gibberish_blocker = false;
          if (!send_config_stm32(&current_config))
          {
            printf_debug("ERROR: Couldn't send config.");
          }
          break;

        // M814: Starting update rate test
        case 814:
          init_814 = millis();
          printf_debug_stm32("M814");
          gibberish_blocker = false;
          break;

        // M815: Update rate test completed, waiting for acknowledgement
        case 815: 
          end_814 = millis();
          printf_debug_stm32("M815");
          gibberish_blocker = false;
          current_config.testing_update_rate = false;

          m_code_pos = strstr(buffer_S2, "M815 ");

          if (m_code_pos != nullptr) 
          {
            // Move the pointer to the character after "M115 "
            char* data_start = m_code_pos + 5;
            
            uint8_t update_count = parse_int(data_start);
            printf_debug_stm32("%s", data_start);
            if (update_count >= 0 && update_count <= 101) 
            {
              float32_t new_update_rate = (float32_t) update_count * 1000.0f / (float32_t) (end_814-init_814);
              printf_BT_slow("Actual update rate measured as %02.1f Hz.", new_update_rate);
              current_config.true_update_rate = new_update_rate;
            }
            else
            {
              printf_BT_slow("Bad value returned, try update rate test again.");
            }
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
          gibberish_blocker = false;
          printf_debug("ERROR: Radar stopped, M-code received outside of stop_radar().");
          break;

        // M808: Config successfully loaded by STM32
        // we shouldn't see this here...
        case 808:
          printf_debug_stm32("M808");
          gibberish_blocker = false;
          printf_debug("ERROR: Config loaded by STM32, M-code received outside of send_config_stm32().");
          break;
        
        // M806: Config not loaded successfully by STM32
        // we shouldn't see this here...
        case 806:
          printf_debug_stm32("M806");
          gibberish_blocker = false;
          printf_debug("ERROR: Config NOT loaded by STM32, M-code received outside of send_config_stm32().");
          break;

        // M197: About to power down, don't print anything until next M-code
        case 197:
          // printf_debug_stm32("M197");
          gibberish_blocker = true;
          break;

        // M197: Gibberish-blocker release
        case 198:
          // printf_debug_stm32("M198");
          gibberish_blocker = false;
          break;

        default:
          printf_debug("ERROR: M-code not recognized. Code sent: %i", code_S2);
          gibberish_blocker = false;
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

    if ((millis() - start_time_GPS) > ((uint32_t) GPS_DELAY_MINS * 60000UL))
    {
      gps_enable = true;
      start_time_GPS = millis();
      gpsSetPowerOn();
      printf_BT_slow("GPS POWERON");
    }

    if (gps_enable)
    {
      processGPSData(gps_data);
    }

    // check for serial plotter timeout
    if (serial_plotter && (millis() - serial_plotter_init) > 60000)
    {
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
    if (!BT_on)
    {
      // send config to STM or start radar
    }

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
          gibberish_blocker = false;
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
          gibberish_blocker = false;
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
      start_time_BT = millis();
      serial_plotter = false;

      switch(config_setting)
      {
        case 's':
          print_n();
          printf_BT_slow("Set the start of the measurement range. Current value: %05.2f m", current_config.start_m);
          printing_config = true;
          printf_BT_slow("Format: XX.XX");
          printf_BT_slow(" Input must match format exactly. Enter leading and trailing zeros as appropriate.");
          printf_BT_slow(" Value must be between 00.10 and 20.00, inclusive.");
          printf_BT_slow(" Units are in meters.");
          printf_BT_slow("Notes:");
          printf_BT_slow(" Generally advised to leave a margin of 0.1m above maximum wave height.");
          printf_BT_slow(" Keep as high as possible. Greatly affects power consumption and maximum possible update rate.");
          printf_BT_slow("Example:");
          printf_BT_slow(" Radar is placed 10m above average wave height, and waves height fluctuates within +/- 1m.");
          printf_BT_slow(" Start of measurement range should be set to 0.8m.");
          printf_BT_slow(" User would enter \"00.80\" into terminal with no quotes.");
          printing_config = false;
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
          printing_config = true;
          printf_BT_slow("Format: XX.XX");
          printf_BT_slow(" Input must match format exactly. Enter leading and trailing zeros as appropriate.");
          printf_BT_slow(" Value must be between 00.10 and 20.00, inclusive.");
          printf_BT_slow(" Units are in meters.");
          printf_BT_slow("Notes:");
          printf_BT_slow(" Generally advised to leave a margin of 0.1m below maximum wave height.");
          printf_BT_slow(" Keep as low as possible. Greatly affects power consumption and maximum possible update rate.");
          printf_BT_slow("Example:");
          printf_BT_slow(" Radar is placed 10m above average wave height, and waves height fluctuates within +/- 1m.");
          printf_BT_slow(" End of measurement range should be set to 1.2m.");
          printf_BT_slow(" User would enter \"01.20\" into terminal with no quotes.");
          printing_config = false;
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
          printing_config = true;
          printf_BT_slow("Format: XX.X");
          printf_BT_slow(" Input must match format exactly. Enter leading and trailing zeros as appropriate.");
          printf_BT_slow(" Value must be between 00.1 and 10.5, inclusive.");
          printf_BT_slow(" Units are in Hertz.");
          printf_BT_slow("Notes:");
          printf_BT_slow(" Keep as low as is required. Greatly affects power consumption.");
          printf_BT_slow("Example:");
          printf_BT_slow(" A measurement is required every 0.2 seconds.");
          printf_BT_slow(" Update rate should be set to 5 Hz.");
          printf_BT_slow(" User would enter \"05.0\" into terminal with no quotes.");
          printing_config = false;
          print_n();
          print_input();

          while (!good_setting)
          {
            (void) get_uart_BT(buffer_S3_BT, 4, 0);
            print_n();
            float32_t new_update_rate = parse_float(buffer_S3_BT);

            if (!isnan(new_update_rate) && (new_update_rate >= 0.1f) && (new_update_rate <= 10.5f)) 
            {
              printf_BT_slow("WARNING: Actual update rate may be lower than desired.");
              printf_BT_slow("Test true update rate in configuration menu.");
              print_n();

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
                printf_BT_slow("Please enter a new number between 00.1 and 10.5, inclusive.");
                print_n();
                print_input();
              }
            }
            else
            {
              printf_BT_slow("Invalid input. Please enter a number between 00.1 and 10.5, inclusive.");
              print_n();
              print_input();
            }
          }

          break;
        
        case 'l':
          print_n();
          printf_BT_slow("Set the maximum step length. Current value: %02i (%.1f mm)", current_config.max_step_length, (float) current_config.max_step_length*2.5f);
          printing_config = true;
          printf_BT_slow("Format: XX");
          printf_BT_slow(" Input must match format exactly. Enter a single integer between 01 and 99, inclusive.");
          printf_BT_slow("Notes:");
          printf_BT_slow(" The maximum step length is set in multiples of 2.5mm. Multiply the inputed setting by 2.5mm to get the maximum step length in mm.");
          printf_BT_slow(" Set the maximum step length to the desired resolution for measuring wave heights.");
          printf_BT_slow(" This is an upper bound - the distance algorithm may return finer resolution measurements.");
          printf_BT_slow(" Keep maximum step length as high as possible. Greatly affects power consumption and maximum possible update rate.");
          printf_BT_slow("Example:");
          printf_BT_slow(" The user wants to determine wave heights accurate to within 5mm.");
          printf_BT_slow(" The user should set the maximum step length to 2, since 2 * 2.5mm = 5mm.");
          printf_BT_slow(" User would enter \"2\" into terminal with no quotes.");
          printing_config = false;
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
                printf_BT_slow("Please enter an integer between 01 and 99, inclusive.");
                print_n();
                print_input();
              }
            }
            else
            {
              printf_BT_slow("Invalid input. Please enter an integer between 01 and 99, inclusive.");
              print_n();
              print_input();
            }
          }

          break;

        case 'p':
          print_n();
          printf_BT_slow("Set the maximum measurement profile. Current value: %i", current_config.max_profile);
          printing_config = true;
          printf_BT_slow("Format: X");
          printf_BT_slow(" Input must match format exactly. Enter a single integer between 1 and 5, inclusive.");
          printf_BT_slow(" Units are arbitrary.");
          printf_BT_slow("Notes:");
          printf_BT_slow(" Measurement profile is used to set the length and shape of the emitted radio pulse.");
          printf_BT_slow(" Higher profiles transmit more energy, increasing signal-to-noise ratio and range.");
          printf_BT_slow(" Keep profile as low as possible. Greatly affects power consumption and maximum possible update rate.");
          printf_BT_slow("Example:");
          printf_BT_slow(" The user starts the radar with maximum profile setting of 2, but can only detect waves half the time.");
          printf_BT_slow(" The user should increase the profile setting by 1, repeating until the waves are always detected.");
          printf_BT_slow(" User would enter \"3\" into terminal with no quotes.");
          printing_config = false;
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
          printing_config = true;
          printf_BT_slow("Format: XX.X");
          printf_BT_slow(" Input must match format exactly. Enter leading and trailing zeros as appropriate.");
          printf_BT_slow(" Value must be between 00.0 and 35.0, inclusive.");
          printf_BT_slow(" Units are arbitrary.");
          printf_BT_slow("Notes:");
          printf_BT_slow(" Higher signal quality results in a higher signal-to-noise ratio target, and increases the number of averages");
          printf_BT_slow(" taken for a single datapoint. This generally increases the accuracy of measurements.");
          printf_BT_slow(" Keep as low as possible. Affects power consumption and maximum possible update rate.");
          printf_BT_slow("Example:");
          printf_BT_slow(" The user starts the radar with signal quality setting of 15.0, but measurements appear inaccurate.");
          printf_BT_slow(" The user should increase the profile setting by 1.0, repeating until the measurements are accurate enough.");
          printf_BT_slow(" User would enter \"16.0\" into terminal with no quotes.");
          printing_config = false;
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
          printing_config = true;
          printf_BT_slow("Format: X");
          printf_BT_slow(" Input must match format exactly.");
          printf_BT_slow(" Enter \"1\" to use planar reflector shape.");
          printf_BT_slow(" Enter \"0\" to use generic reflector shape.");
          printf_BT_slow("Notes:");
          printf_BT_slow(" Planar reflector shape should be used for determining the distance to flat objects like water surfaces,");
          printf_BT_slow(" sheets of metal, walls, and more.");
          printf_BT_slow(" Generic reflector shape should be used for determining the distance to any other objects.");
          printf_BT_slow("Example:");
          printf_BT_slow(" The radar is installed on the edge of a pier above a body of water.");
          printf_BT_slow(" Planar reflector shape should be used, and the user would enter \"1\" with no quotes.");
          printing_config = false;
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
          printing_config = true;
          printf_BT_slow(" Input must match format exactly. Enter leading and trailing zeros as appropriate.");
          printf_BT_slow(" Value must be between 0.00 and 1.00, inclusive.");
          printf_BT_slow(" Units are arbitrary.");
          printf_BT_slow("Notes:");
          printf_BT_slow(" Controls the false positive rate. A higher threshold means that more energy must be reflected from an object");
          printf_BT_slow(" to record a distance measurement.");
          printf_BT_slow(" Tune in conjunction with the signal quality and measurement profile to eliminate background objects.");
          printf_BT_slow("Example:");
          printf_BT_slow(" The user starts the radar with threshold sensitivity setting of 0.5, measuring water level over the side of a pier.");
          printf_BT_slow(" The radar keeps detecting the distance to the metal beams supporting the pier, which messes up the measurements.");
          printf_BT_slow(" The user should decrease the sensitivity by 0.10, adjusting until the beams are no longer detected by the radar.");
          printf_BT_slow(" User would enter \"0.40\" into terminal with no quotes.");
          printing_config = false;
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

        case 'w':
          print_n();
          printf_BT_slow("Set the text width for the Bluetooth UART output. Current value: %03i characters", current_config.text_width);
          printing_config = true;
          printf_BT_slow("Format: XXX");
          printf_BT_slow(" Input must match format exactly. Enter a single integer between 010 and 140, inclusive.");
          printf_BT_slow(" Alternatively, enter \"000\" to disable text wrapping.");
          printf_BT_slow("Notes:");
          printf_BT_slow(" The text in the Bluetooth terminal can be adjusted for the width of your device.");
          printf_BT_slow("Example:");
          printf_BT_slow(" The user is interfacing with the ESP32 using a mobile phone Bluetooth serial terminal.");
          printf_BT_slow(" The user notices that the text does not wrap around the phone screen properly, with five characters hanging at the end of each line.");
          printf_BT_slow(" User would subtract 5 from the current text width and enter the result into the terminal.");
          printing_config = false;
          print_n();
          print_input();

          while (!good_setting)
          {
            (void) get_uart_BT(buffer_S3_BT, 3, 0);
            print_n();
            int new_text_width = parse_int(buffer_S3_BT);

            if (new_text_width >= 10 && new_text_width <= 140) 
            {
              printf_BT_slow("Text width set to %03i characters", new_text_width);
              printf_BT_slow("Is this the desired value? Type Y for yes, any other character for no.");
              print_n();
              print_input();

              (void) get_uart_BT(buffer_S3_BT, 1, 0);
              print_n();
              char ack_char = tolower(buffer_S3_BT[0]);
              
              if (ack_char == 'y')
              {
                current_config.text_width = new_text_width;
                text_width = current_config.text_width;
                good_setting = true;
                printf_BT_slow("Value saved.");
                print_n();
                delay(1000);
                print_config_menu(&current_config);
              }
              else
              {
                printf_BT_slow("Please enter an integer between 010 and 140, inclusive, or enter 000 to disable text wrapping.");
                print_n();
                print_input();
              }
            }
            else
            {
              printf_BT_slow("Invalid input. Please enter an integer between 010 and 140, inclusive, or enter 000 to disable text wrapping.");
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
            print_n();
            if (!send_config_stm32(&current_config))
            {
              printf_debug("ERROR: Couldn't send config.");
              print_n();
              print_config_menu(&current_config);
              break;
            }
            printf_BT_slow("Testing actual update rate...");
            state = STATE_LISTENING;
          }
          break;

        case 'b':
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
            print_n();
            if (!send_config_stm32(&current_config))
            {
              printf_debug("ERROR: Couldn't send config.");
              print_n();
              print_config_menu(&current_config);
              break;
            }
            serial_plotter = true;
            serial_plotter_init = millis();
            printf_BT_slow("Print to Bluetooth in serial plotter mode...");
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
              printf_debug("ERROR: Couldn't send config.");
              print_n();
              print_config_menu(&current_config);
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
    flush_debug_buffer();

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
    append_file(SD, debug_file_path, result);
    append_file(SD, debug_file_path, "\n");
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
  memset(buffer, 0, buffer_size);
  
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
  Serial32.begin(921600UL, SERIAL_8E1, 16, 17);

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
  File file = SD.open("/radar_config.txt", FILE_READ);
  if (!file) {
    // File doesn't exist, create it with default values
    file.close();
    ConfigSettings default_config = {
      0.10f,          // start_m
      0.50f,          // end_m
      0.8f,           // update_rate
      1,              // max_step_length
      5,              // max_profile
      20.0f,          // signal_quality
      1,              // reflector_shape
      0.50f,          // threshold_sensitivity
      0,              // testing_update_rate
      10.1f,          // true_update_rate
      40,             // text_width
      "Not Set",      // latitude
      "Not Set",      // longitude
      "Not Set"       // elevation
    };
    *config = default_config;
    return save_config_adalogger(config);
  }

  // File exists, read and parse it
  char buf[192];  // Increased buffer size to accommodate GPS coordinates
  printf_debug("Loaded config file.");
  size_t len = file.readBytesUntil('\n', buf, sizeof(buf) - 1);
  file.close();
  
  if (len == 0) {
    printf_debug("Error: Empty config file");
    delete_file(SD, "/radar_config.txt");
    return false;
  }
  
  buf[len] = '\0';

  char lat_buf[32], lon_buf[32], elev_buf[16];
  int parsed = sscanf(buf, "%f,%f,%f,%hhu,%hhu,%f,%hhu,%f,%hhu,%f,%hhu,%[^,],%[^,],%s",
                      &config->start_m,
                      &config->end_m,
                      &config->update_rate,
                      &config->max_step_length,
                      &config->max_profile,
                      &config->signal_quality,
                      &config->reflector_shape,
                      &config->threshold_sensitivity,
                      &config->testing_update_rate,
                      &config->true_update_rate,
                      &config->text_width,
                      lat_buf,
                      lon_buf,
                      elev_buf);

  if (parsed != 14) {  // Updated to include elevation
    printf_debug("Error: Failed to parse config file");
    delete_file(SD, "/radar_config.txt");
    return false;
  }

  strncpy(config->latitude, lat_buf, sizeof(config->latitude) - 1);
  strncpy(config->longitude, lon_buf, sizeof(config->longitude) - 1);
  strncpy(config->elevation, elev_buf, sizeof(config->elevation) - 1);
  config->latitude[sizeof(config->latitude) - 1] = '\0';
  config->longitude[sizeof(config->longitude) - 1] = '\0';
  config->elevation[sizeof(config->elevation) - 1] = '\0';

  text_width = config->text_width;
  return true;
}


bool save_config_adalogger(ConfigSettings *config)
{
  // Delete the existing file if it exists
  if (SD.exists("/radar_config.txt")) {
    delete_file(SD, "/radar_config.txt");
  }

  // Prepare the configuration string
  char config_string[192];  // Might need to increase buffer size
  snprintf(config_string, sizeof(config_string), 
           "%05.2f,%05.2f,%04.1f,%02d,%d,%04.1f,%d,%04.2f,%d,%04.1f,%d,%s,%s,%s\n",
           config->start_m,
           config->end_m,
           config->update_rate,
           config->max_step_length,
           config->max_profile,
           config->signal_quality,
           config->reflector_shape,
           config->threshold_sensitivity,
           config->testing_update_rate,
           config->true_update_rate,
           config->text_width,
           config->latitude,
           config->longitude,
           config->elevation);

  // Append the new configuration to the file
  append_file(SD, "/radar_config.txt", config_string);

  // Verify that the file was written correctly
  File file = SD.open("/radar_config.txt", FILE_READ);
  if (!file) {
    printf_debug("Error: Unable to open config file for verification");
    return false;
  }

  char verify_buf[192];  // Increased buffer size
  size_t bytesRead = file.readBytesUntil('\n', verify_buf, sizeof(verify_buf) - 1);
  file.close();

  if (bytesRead != strlen(config_string) - 1) {  // -1 because readBytesUntil doesn't include '\n'
    printf_debug("Error: Config file verification failed");
    return false;
  }

  printf_debug("New config file saved");
  text_width = config->text_width;
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
          printf_stm32("Configuration received and loaded onto radar sensor.");
          printf_stm32("");
          sending_config = false;
          return true;
        }
        else if (code_config_send == 806) {
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
      printf_debug_stm32("%s", buffer_815);
      if (!isnan(new_update_rate))
      {
        printf_BT_slow("Actual update rate measured as %02.1f Hz.", new_update_rate);
        config->true_update_rate = new_update_rate;
        config->testing_update_rate = false;
        return true;
      }
      else
      {
        printf_BT_slow("NAN returned, try again.");
      }
    }
  }

  printf_BT_slow("Failed to measure actual update rate, please try again.");
  return false;
}


void print_input(void)
{
  flush_debug_buffer();
  if (BT_on)
  {
    SerialBT.print("Input: ");
    delay(100);
  }

  if (SD_on)
  {
    append_file(SD, debug_file_path, "Input: ");
  }

  Serial.print("Input: ");
}


void print_n(void)
{
  printf_BT_slow("");
}


void printf_BT_slow(const char *format, ...)
{
  uint32_t start_time_BTslow = millis();
  va_list args;
  char buf[256]; // Adjust size as needed

  va_start(args, format);
  (void) vsnprintf(buf, sizeof(buf), format, args);
  va_end(args);

  if (BT_on)
  {
    SerialBT.print("ESP32: ");
    if (text_width > 0)
    {
      size_t len = strlen(buf);
      size_t start = 0;
      while (start < len)
      {
        size_t end = start + text_width;
        if (start > 0)
        {
          if (printing_config)
          {
            SerialBT.print("ESP32:    ");
            end -= 3;
          }
          else
          {
            SerialBT.print("ESP32: ");
          }
        }
        if (end > len)
        {
          end = len;
        }
        SerialBT.write(reinterpret_cast<const uint8_t*>(buf + start), end - start);
        SerialBT.println();
        start = end;
        while (abs(millis() - start_time_BTslow) < 100);
        start_time_BTslow = millis();
      }
      if (start == 0) SerialBT.println();
    }
    else
    {
      SerialBT.print(buf);
      SerialBT.println();
      while (abs(millis() - start_time_BTslow) < 100);
    }
  }

  if (SD_on) {
    size_t remaining = DEBUG_BUFFER_SIZE - debug_buffer_pos - 1; // Leave space for null terminator
    int written = snprintf(debug_buffer + debug_buffer_pos, remaining, "ESP32: %s\n", buf);
    
    if (written > 0) {
      debug_buffer_pos += written;
    }

    // Check if buffer is nearly full or it's time to flush
    if (debug_buffer_pos > DEBUG_BUFFER_SIZE - 256 || millis() - last_flush_time > FLUSH_INTERVAL) {
      flush_debug_buffer();
    }
  }

  Serial.print("ESP32: ");
  Serial.print(buf);
  Serial.println();
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
    if (text_width > 0)
    {
      size_t len = strlen(buf);
      size_t start = 0;
      while (start < len)
      {
        if (start > 0)
        {
          SerialBT.print("ESP32: ");
        }
        size_t end = start + text_width;
        if (end > len)
        {
          end = len;
        }
        SerialBT.write(reinterpret_cast<const uint8_t*>(buf + start), end - start);
        SerialBT.println();
        start = end;
      }
      if (start == 0) SerialBT.println();
    }
    else
    {
      SerialBT.print(buf);
      SerialBT.println();
    }
  }

  if (SD_on) {
    size_t remaining = DEBUG_BUFFER_SIZE - debug_buffer_pos - 1; // Leave space for null terminator
    int written = snprintf(debug_buffer + debug_buffer_pos, remaining, "ESP32: %s\n", buf);
    
    if (written > 0) {
      debug_buffer_pos += written;
    }

    // Check if buffer is nearly full or it's time to flush
    if (debug_buffer_pos > DEBUG_BUFFER_SIZE - 256 || millis() - last_flush_time > FLUSH_INTERVAL) {
      flush_debug_buffer();
    }
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
    if (text_width > 0)
    {
      size_t len = strlen(buf);
      size_t start = 0;
      while (start < len)
      {
        if (start > 0)
        {
          SerialBT.print("STM32: ");
        }
        size_t end = start + text_width;
        if (end > len)
        {
          end = len;
        }
        SerialBT.write(reinterpret_cast<const uint8_t*>(buf + start), end - start);
        SerialBT.println();
        start = end;
      }
      if (start == 0) SerialBT.println();
    }
    else
    {
      SerialBT.print(buf);
      SerialBT.println();
    }
  }

  if (SD_on) {
    size_t remaining = DEBUG_BUFFER_SIZE - debug_buffer_pos - 1; // Leave space for null terminator
    int written = snprintf(debug_buffer + debug_buffer_pos, remaining, "STM32: %s\n", buf);
    
    if (written > 0) {
      debug_buffer_pos += written;
    }

    // Check if buffer is nearly full or it's time to flush
    if (debug_buffer_pos > DEBUG_BUFFER_SIZE - 256 || millis() - last_flush_time > FLUSH_INTERVAL) {
      flush_debug_buffer();
    }
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
    if (text_width > 0)
    {
      size_t len = strlen(buf);
      size_t start = 0;
      while (start < len)
      {
        if (start > 0)
        {
          SerialBT.print("STM32: ");
        }
        size_t end = start + text_width;
        if (end > len)
        {
          end = len;
        }
        SerialBT.write(reinterpret_cast<const uint8_t*>(buf + start), end - start);
        SerialBT.println();
        start = end;
      }
      if (start == 0) SerialBT.println();
    }
    else
    {
      SerialBT.print(buf);
      SerialBT.println();
    }
  }

  if (SD_on) {
    size_t remaining = DEBUG_BUFFER_SIZE - debug_buffer_pos - 1; // Leave space for null terminator
    int written = snprintf(debug_buffer + debug_buffer_pos, remaining, "STM32: %s\n", buf);
    
    if (written > 0) {
      debug_buffer_pos += written;
    }

    // Check if buffer is nearly full or it's time to flush
    if (debug_buffer_pos > DEBUG_BUFFER_SIZE - 256 || millis() - last_flush_time > FLUSH_INTERVAL) {
      flush_debug_buffer();
    }
  }

  Serial.print("STM32: ");
  Serial.print(buf);
  Serial.println();
}


void printf_datalog(const char *format, ...)
{
  va_list args;
  char buf[256]; // Adjust size as needed

  va_start(args, format);
  (void) vsnprintf(buf, sizeof(buf), format, args);
  va_end(args);

  if (BT_on)
  {
    SerialBT.print("STM32: ");
    if (text_width > 0)
    {
      size_t len = strlen(buf);
      size_t start = 0;
      while (start < len)
      {
        if (start > 0)
        {
          SerialBT.print("STM32: ");
        }
        size_t end = start + text_width;
        if (end > len)
        {
          end = len;
        }
        SerialBT.write(reinterpret_cast<const uint8_t*>(buf + start), end - start);
        SerialBT.println();
        start = end;
      }
      if (start == 0) SerialBT.println();
    }
    else
    {
      SerialBT.print(buf);
      SerialBT.println();
    }
  }

  if (SD_on) {
    // size_t remaining = DEBUG_BUFFER_SIZE - debug_buffer_pos - 1; // Leave space for null terminator
    // int written = snprintf(debug_buffer + debug_buffer_pos, remaining, "STM32: %s\n", buf);
    
    // if (written > 0) {
    //   debug_buffer_pos += written;
    // }

    // // Check if buffer is nearly full or it's time to flush
    // if (debug_buffer_pos > DEBUG_BUFFER_SIZE - 256 || millis() - last_flush_time > FLUSH_INTERVAL) {
    //   flush_debug_buffer();
    // }
    // Delete the existing file if it exists
    if (data_file_path == nullptr)
    {
      (void) start_new_data_file(&data_file_path);
    }
    append_file(SD, data_file_path, buf);
    append_file(SD, data_file_path, "\n");
  }

  Serial.print("STM32: ");
  Serial.print(buf);
  Serial.println();
}


void printf_serial_plotter(const char *format, ...)
{
  va_list args;
  char buf[256]; // Adjust size as needed

  va_start(args, format);
  (void) vsnprintf(buf, sizeof(buf), format, args);
  va_end(args);

  if (BT_on)
  {
    SerialBT.print(buf);
    SerialBT.println();
  }

  Serial.print(buf);
  Serial.println();
}


void print_config_menu(ConfigSettings *config)
{
  printf_BT_slow("--- CONFIGURATION MENU ---");
  printf_BT_slow("Send one of the below characters to change that configuration setting.");
  printf_BT_slow("Changes will be saved upon exiting menu.");
  printing_config = true;
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
  printf_BT_slow("W: Set the text width for the Bluetooth UART output.");
  printf_BT_slow("   Current value: %03i characters", current_config.text_width);
  printf_BT_slow("A: Measure the actual update rate.");
  printf_BT_slow("   True update rate: %04.1f Hz", config->true_update_rate);
  printf_BT_slow("B: Print to Bluetooth in serial plotter mode for two minutes.");
  printf_BT_slow("   Useful for tuning settings with visual indicators.");
  printf_BT_slow("   Recommended to use Arduino IDE serial plotter.");
  printf_BT_slow("X: Exit the menu.");
  printing_config = false;
  print_n();
  print_input();
}


void print_CP(void)
{
  SD_on = false;
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
  SD_on = true;
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


void list_dir(fs::FS &fs, const char *dirname, uint8_t levels)
{
  Serial.printf("Listing directory: %s\n", dirname);

  File root = fs.open(dirname);
  if (!root) 
  {
    Serial.println("Failed to open directory");
    return;
  }
  if (!root.isDirectory()) 
  {
    Serial.println("Not a directory");
    return;
  }

  File file = root.openNextFile();
  while (file) {
    if (file.isDirectory()) 
    {
      Serial.print("  DIR : ");
      Serial.println(file.name());
      if (levels) 
      {
        list_dir(fs, file.name(), levels - 1);
      }
    } 
    else 
    {
      Serial.print("  FILE: ");
      Serial.print(file.name());
      Serial.print("  SIZE: ");
      Serial.println(file.size());
    }
    file = root.openNextFile();
  }
}


void create_dir(fs::FS &fs, const char *path)
{
  Serial.printf("Creating Dir: %s\n", path);
  if (fs.mkdir(path)) 
  {
    Serial.println("Dir created");
  } 
  else 
  {
    Serial.println("mkdir failed");
  }
}


void append_file(fs::FS &fs, const char *path, const char *message)
{
  File file = fs.open(path, FILE_APPEND);
  if (!file) 
  {
    Serial.println("Failed to open file for appending");
    return;
  }
  if (!file.print(message))
  { 
    Serial.println("Append failed");
  }
  file.close();
}


void delete_file(fs::FS &fs, const char *path)
{
  Serial.printf("Deleting file: %s\n", path);
  if (fs.remove(path)) 
  {
    Serial.println("File deleted");
  } 
  else 
  {
    Serial.println("Delete failed");
  }
}


void setup_sd_card(void)
{
  Serial.print("Initializing SD card...");

  if (!SD.begin(chip_select)) {
    Serial.println("initialization failed. Things to check:");
    Serial.println("1. is a card inserted?");
    Serial.println("2. is your wiring correct?");
    Serial.println("3. did you change the chip_select pin to match your shield or module?");
    Serial.println("Note: press reset button on the board and reopen this Serial Monitor after fixing your issue!");
    while (true);
  }

  SD_on = true;

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

  // Check for required directories
  if (!SD.exists("/DEBUG_LOGS") || !SD.exists("/DATA"))
  {
    Serial.println("Required directories not found. Creating them...");
    if (!SD.exists("/DEBUG_LOGS"))
    {
      create_dir(SD, "/DEBUG_LOGS");
    }
    if (!SD.exists("/DATA"))
    {
      create_dir(SD, "/DATA");
    }
  } 
  else 
  {
    Serial.println("Required directories found.");
  }

  list_dir(SD, "/", 0);
  Serial.printf("Total space: %lluMB\n", SD.totalBytes() / (1024 * 1024));
  Serial.printf("Used space: %lluMB\n", SD.usedBytes() / (1024 * 1024));
}


bool start_new_debug_file(char** debug_file_path)
{
  static char filename[64];  // Static to ensure the memory persists after function returns

  // Create a new filename
  DateTime now = rtc.now();
  snprintf(filename, sizeof(filename), "/DEBUG_LOGS/%02d-%02d-%02d_%02d-%02d-%02d_debug.txt",
           now.year() % 100, now.month(), now.day(),
           now.hour(), now.minute(), now.second());

  if (SD.exists(filename)) 
  {
    Serial.printf("Debug file already exists: %s\n", filename);
    return false;
  }
  else 
  {
    File file = SD.open(filename, FILE_WRITE);
    if (!file) 
    {
      Serial.println("Failed to create new debug file");
      return false;
    } 
    else 
    {
      file.close();
      Serial.printf("Created new debug file: %s\n", filename);
    }
  }
  
  if (*debug_file_path != nullptr)
  {
    free(*debug_file_path);  // Free the old memory if it was allocated
  }
  *debug_file_path = strdup(filename);  // Allocate new memory and copy the filename
  
  return true;
}


bool start_new_data_file(char** data_file_path)
{
  static char filename[64];  // Static to ensure the memory persists after function returns
  bool new_file_created = false;

  // Check if data_file_path is not null and points to a valid file path
  if (*data_file_path != nullptr && SD.exists(*data_file_path))
  {
    File existingFile = SD.open(*data_file_path, FILE_READ);
    if (existingFile)
    {
      // Check if the file is empty
      if (existingFile.size() == 0)
      {
        existingFile.close();
        // Delete the empty file
        if (SD.remove(*data_file_path))
        {
          Serial.print("Deleted empty data file: ");
          Serial.println(*data_file_path);
        }
        else
        {
          Serial.print("Failed to delete empty data file: ");
          Serial.println(*data_file_path);
        }
      }
      else
      {
        existingFile.close();
      }
    }
  }

  // Create a new filename
  DateTime now = rtc.now();
  snprintf(filename, sizeof(filename), "/DATA/%02d-%02d-%02d_%02d-%02d-%02d_data.txt",
           now.year() % 100, now.month(), now.day(),
           now.hour(), now.minute(), now.second());

  if (SD.exists(filename)) 
  {
    Serial.print("Data file already exists: ");
    Serial.println(filename);
    return false;
  } 
  else 
  {
    File file = SD.open(filename, FILE_WRITE);
    if (!file) 
    {
      Serial.println("Failed to create new data file");
      return false;
    } 
    else 
    {
      file.close();
      Serial.print("Created new data file: ");
      Serial.println(filename);
      new_file_created = true;
    }
  }
  
  // Update the data_file_path
  if (new_file_created) 
  {
    if (*data_file_path != nullptr) 
    {
      free(*data_file_path);  // Free the old memory if it was allocated
    }
    *data_file_path = strdup(filename);  // Allocate new memory and copy the filename
  }
  
  return true;
}


void flush_debug_buffer(void) 
{
  if (SD_on && debug_buffer_pos > 0) 
  {
    debug_buffer[debug_buffer_pos] = '\0'; // Null-terminate the string
    append_file(SD, debug_file_path, debug_buffer);
    debug_buffer_pos = 0;
    last_flush_time = millis();
  }
}


void check_sizes(void)
{
  if (sizeof(unsigned long) != 4)
  {
    Serial.println("\nERROR: Size of unsigned long is not 4 bytes; re-evaluate use of millis().");
    while (true);
  }
  if (sizeof(unsigned int) != 4)
  {
    Serial.println("\nERROR: Size of unsigned int is not 4 bytes; re-evaluate use of millis().");
    while (true);
  }
  if (sizeof(float) != 4)
  {
    Serial.println("\nERROR: Size of unsigned long is not 4 bytes; rename float32_t.");
    while (true);
  }
}


DateTimeMS timekeeper(DateTimeMS initial_time, uint32_t initial_millis)
{
  uint32_t current_millis = millis();
  uint64_t elapsed;
  static uint32_t overflow_count = 0;
  uint64_t overflow = (uint64_t) (0xFFFFFFFF * overflow_count);

  if (current_millis >= initial_millis) {
    elapsed = (uint64_t) (current_millis - initial_millis) + overflow;
  } else {
    // Overflow occurred
    overflow_count++;
    elapsed = (uint64_t) ((0xFFFFFFFF - initial_millis) + current_millis + 1UL) + overflow;
  }

  DateTimeMS current = initial_time;

  // Add elapsed time to initial time
  current.millisecond += elapsed % 1000;
  elapsed /= 1000;
  
  if (current.millisecond >= 1000) 
  {
      current.millisecond -= 1000;
      elapsed++;
  }
  
  current.second += elapsed % 60;
  elapsed /= 60;
  
  if (current.second >= 60) 
  {
      current.second -= 60;
      elapsed++;
  }
  
  current.minute += elapsed % 60;
  elapsed /= 60;
  
  if (current.minute >= 60) 
  {
      current.minute -= 60;
      elapsed++;
  }
  
  current.hour += elapsed % 24;
  elapsed /= 24;
  
  if (current.hour >= 24) 
  {
      current.hour -= 24;
      elapsed++;
  }
  
  // Add days, handling month and year transitions
  while (elapsed > 0) 
  {
    uint8_t days_this_month = days_in_month(current.month, current.year);
    if (current.day + elapsed > days_this_month) 
    {
      elapsed -= (days_this_month - current.day + 1);
      current.day = 1;
      if (++current.month > 12) 
      {
        current.month = 1;
        current.year++;
      }
    }
    else 
    {
      current.day += elapsed;
      break;
    }
  }
  
  return current;
}


uint8_t days_in_month(uint8_t month, uint16_t year) 
{
  static const uint8_t days_in_month[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  if (month == 2 && is_leap_year(year)) 
  {
    return 29;
  }
  return days_in_month[month - 1];
}


bool is_leap_year(uint16_t year) 
{
  return (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0);
}


void set_init_time(DateTimeMS *initial_time, uint32_t *initial_millis)
{
  DateTime i_now = rtc.now();
  initial_time->year = i_now.year();
  initial_time->month = i_now.month();
  initial_time->day = i_now.day();
  initial_time->hour = i_now.hour();
  initial_time->minute = i_now.minute();
  initial_time->second = i_now.second();
  initial_time->millisecond = millis() % 1000;
  *initial_millis = millis();

  Serial.println("Initial time:");
  Serial.print(initial_time->year, DEC);
  Serial.print('/');
  Serial.print(initial_time->month, DEC);
  Serial.print('/');
  Serial.print(initial_time->day, DEC);
  Serial.print(" ");
  Serial.print(initial_time->hour, DEC);
  Serial.print(':');
  Serial.print(initial_time->minute, DEC);
  Serial.print(':');
  Serial.print(initial_time->second, DEC);
  Serial.print('.');
  Serial.print(initial_time->millisecond, DEC);
  Serial.println();
  Serial.println("Initial millis:");
  Serial.printf("%u\n", *initial_millis);
}


void convertDMtoDMS(float decimal_degrees, char *result, size_t size, bool isLat, char direction) {
    // Take absolute value for calculations
    float abs_degrees = fabs(decimal_degrees);
    
    // Split into degrees and minutes
    int degrees = (int)abs_degrees;
    float minutes = (abs_degrees - degrees) * 60;
    
    // Split minutes into whole minutes and seconds
    int whole_minutes = (int)minutes;
    float seconds = (minutes - whole_minutes) * 60;
    
    // Use the passed direction indicator directly
    // No need to check the sign of decimal_degrees anymore
    
    // Format result with leading zeros for minutes and consistent decimal places for seconds
    snprintf(result, size, "%d°%02d'%05.2f\"%c", 
             degrees, whole_minutes, seconds, direction);
}


bool parseGPGGA(const char* sentence, GPSData* data) {
  static uint8_t validCount = 0;
  
  if (strncmp(sentence, "$GPGGA", 6) != 0) {
    return false;
  }

  // Find start of time field (after first comma)
  const char* ptr = strchr(sentence, ',');
  if (!ptr) return false;
  ptr++;

  // Parse time (format: HHMMSS.SS)
  int time = atoi(ptr);
  int hour = time / 10000;
  int minute = (time / 100) % 100;
  int second = time % 100;

  // Skip to latitude field
  ptr = strchr(ptr, ',');
  if (!ptr) return false;
  ptr++;
  
  // Parse latitude - GPGGA format is DDMM.MMMM
  float rawLat = atof(ptr);
  if (rawLat == 0.0f) return false; // Invalid latitude
  
  float latDegrees = (int)(rawLat / 100);                // Extract degrees
  float latMinutes = fmod(rawLat, 100.0);                // Extract minutes
  float latitude = latDegrees + (latMinutes / 60.0);     // Convert to decimal degrees
  
  // Skip to N/S indicator
  ptr = strchr(ptr, ',');
  if (!ptr) return false;
  ptr++;
  
  // Validate and apply hemisphere
  if (*ptr != 'N' && *ptr != 'S') return false;
  char latDirection = *ptr;  // Store direction for later
  
  // Skip to longitude field
  ptr = strchr(ptr, ',');
  if (!ptr) return false;
  ptr++;
  
  // Parse longitude - GPGGA format is DDDMM.MMMM
  float rawLon = atof(ptr);
  if (rawLon == 0.0f) return false; // Invalid longitude
  
  float lonDegrees = (int)(rawLon / 100);                // Extract degrees
  float lonMinutes = fmod(rawLon, 100.0);                // Extract minutes
  float longitude = lonDegrees + (lonMinutes / 60.0);    // Convert to decimal degrees
  
  // Skip to E/W indicator
  ptr = strchr(ptr, ',');
  if (!ptr) return false;
  ptr++;
  
  // Validate and store hemisphere
  if (*ptr != 'E' && *ptr != 'W') return false;
  char lonDirection = *ptr;  // Store direction for later

  // Skip to fix quality
  ptr = strchr(ptr, ',');
  if (!ptr) return false;
  ptr++;
  int fixQuality = atoi(ptr);
  if (fixQuality == 0) return false; // No fix

  // Skip to number of satellites
  ptr = strchr(ptr, ',');
  if (!ptr) return false;
  ptr++;
  int satellites = atoi(ptr);
  if (satellites < 4) return false; // Not enough satellites for reliable position

  // Skip to HDOP
  ptr = strchr(ptr, ',');
  if (!ptr) return false;
  ptr++;
  float hdop = atof(ptr);
  if (hdop > 5.0) return false; // Poor accuracy

  // After parsing HDOP, find elevation field
  ptr = strchr(ptr, ',');
  if (!ptr) return false;
  ptr++;
  float elevation = atof(ptr);
  
  // Skip to elevation unit (should be M for meters)
  ptr = strchr(ptr, ',');
  if (!ptr) return false;
  ptr++;
  if (*ptr != 'M') return false;

  // Increment valid count if we got this far
  validCount++;
  
  // Only proceed if we've had enough valid readings
  if (validCount <= 10) return false;

  // If we get here, we have a valid reading. Save the data.
  int8_t tzOffset = getUTCOffset();
  data->hour = hour + tzOffset;  // Apply correct timezone offset
  if (data->hour < 0) {
      data->hour += 24;
  }
  data->minute = minute;
  data->second = second;

  // Save elevation
  snprintf(data->elevation, sizeof(data->elevation), "%.1f m", elevation);

  // Convert coordinates to DMS format, passing the NMEA direction indicators
  convertDMtoDMS(latitude, data->latitude, sizeof(data->latitude), true, latDirection);
  convertDMtoDMS(longitude, data->longitude, sizeof(data->longitude), false, lonDirection);
  
  validCount = 0;

  return true;
}


void processGPSData(GPSData &gps_data) {
  static char buffer[256];
  static uint8_t idx = 0;
  
  while (gpsSerial.available() > 0) {
    char c = gpsSerial.read();
    
    // Start of new NMEA sentence
    if (c == '$') {
      idx = 0;
      buffer[idx++] = c;
    }
    // Continue buffering
    else if (idx < sizeof(buffer)-1) {
      buffer[idx++] = c;
      
      // Complete sentence received (CR or LF)
      if (c == '\r' || c == '\n') {
        buffer[idx-1] = '\0'; // Replace CR/LF with null terminator
        
        // Only process if we have a complete sentence starting with $
        if (buffer[0] == '$' && idx > 7) {
          // For debugging
          // printf_BT_slow("GPS: %s", buffer);
          
          if (parseGPGGA(buffer, &gps_data)) {
            printf_datalog("Time: %02d:%02d:%02d PST", 
                          gps_data.hour, gps_data.minute, gps_data.second);
            printf_datalog("Location: %s %s", 
                          gps_data.latitude, gps_data.longitude);
            printf_datalog("Elevation: %s", gps_data.elevation);

            printf_debug("Time: %02d:%02d:%02d PST", 
                          gps_data.hour, gps_data.minute, gps_data.second);
            printf_debug("Location: %s %s", 
                          gps_data.latitude, gps_data.longitude);
            printf_debug("Elevation: %s", gps_data.elevation);
            
            // Update config with new GPS coordinates and elevation
            strncpy(current_config.latitude, gps_data.latitude, sizeof(current_config.latitude) - 1);
            strncpy(current_config.longitude, gps_data.longitude, sizeof(current_config.longitude) - 1);
            strncpy(current_config.elevation, gps_data.elevation, sizeof(current_config.elevation) - 1);
            current_config.latitude[sizeof(current_config.latitude) - 1] = '\0';
            current_config.longitude[sizeof(current_config.longitude) - 1] = '\0';
            current_config.elevation[sizeof(current_config.elevation) - 1] = '\0';
            
            // Save the updated config to SD card
            save_config_adalogger(&current_config);
            
            gps_enable = false;
            gpsSetPowerOff();
            printf_BT_slow("GPS POWEROFF");
            syncRTCWithGPS(&gps_data);
          }
        }
        idx = 0; // Reset for next sentence
      }
    }
    // Buffer overflow protection
    else {
      idx = 0;
    }
  }
}


void gpsSetPowerOff(void) {
    // RXM-PMREQ command verified to work on NEO-6M
    uint8_t powerOffCmd[] = {
        0xB5, 0x62,           // UBX sync chars
        0x02, 0x41,           // RXM-PMREQ message class/ID
        0x08, 0x00,           // Length (8 bytes)
        0x00, 0x00, 0x00, 0x00,   // duration (set to 0 for infinite)
        0x02, 0x00, 0x00, 0x00,   // flags (backup mode)
        0x4D, 0x3B            // Checksum as verified in forum
    };
    
    // Clear any existing data
    while(gpsSerial.available()) {
        gpsSerial.read();
    }
    
    // Send command to GPS module
    for(int i = 0; i < sizeof(powerOffCmd); i++) {
        gpsSerial.write(powerOffCmd[i]);
    }

    delay(100);
}


void gpsSetPowerOn(void) {
  // Clear any stale data
  while(gpsSerial.available()) {
      gpsSerial.read();
  }
  
  // Send 0xFF to wake up the module from backup mode
  gpsSerial.write(0xFF);
  delay(100);  // Wait for GPS to wake up

  // UBX-CFG-RXM command to set maximum performance mode
  uint8_t powerOnCmd[] = {
    0xB5, 0x62,           // UBX sync chars  
    0x06, 0x11,           // CFG-RXM message class/ID
    0x02, 0x00,           // Length (2 bytes)
    0x08, 0x00,           // reserved1=8, lpMode=0 (Max Performance Mode)
    0x21, 0x91            // Checksum
  };
  
  // Send command to GPS module
  for (int i = 0; i < sizeof(powerOnCmd); i++) {
    gpsSerial.write(powerOnCmd[i]);
  }
  delay(100); // Give GPS time to process command

  // Optional: Wait for GPS to start transmitting
  uint32_t startTime = millis();
  while (!gpsSerial.available() && (millis() - startTime < 2000)) {
      delay(10);
  }
}


bool isDST(uint16_t year, uint8_t month, uint8_t day, uint8_t hour) {
    // DST starts on second Sunday in March at 2am
    // DST ends on first Sunday in November at 2am
    
    if (month < 3 || month > 11) {
        return false;  // Jan, Feb, Dec are always PST
    }
    
    if (month > 3 && month < 11) {
        return true;   // Apr to Oct are always PDT
    }

    // Calculate the target Sunday for start/end
    int targetSunday;
    if (month == 3) {
        // Second Sunday in March
        targetSunday = 14 - ((year + year/4 + 2) % 7);
        // If day is before target Sunday, or it's the day but before 2am
        if (day < targetSunday || (day == targetSunday && hour < 2)) {
            return false;
        }
        return true;
    } else {  // month == 11
        // First Sunday in November
        targetSunday = 7 - ((year + year/4 + 2) % 7);
        // If day is after target Sunday, or it's the day and after 2am
        if (day > targetSunday || (day == targetSunday && hour >= 2)) {
            return false;
        }
        return true;
    }
}


int8_t getUTCOffset(void) {
    DateTime now = rtc.now();
    bool inDST = isDST(now.year(), now.month(), now.day(), now.hour());
    return inDST ? -7 : -8;
}


void syncRTCWithGPS(GPSData* gps_data) {
    DateTime now = rtc.now();
    
    // Convert current RTC and GPS times to total seconds for comparison
    int32_t rtc_seconds = ((int32_t)now.hour() * 60 + now.minute()) * 60 + now.second();
    int32_t gps_seconds = ((int32_t)gps_data->hour * 60 + gps_data->minute) * 60 + gps_data->second;
    
    // Calculate the difference (can be negative)
    int32_t second_diff = gps_seconds - rtc_seconds;
    
    // Handle day boundary cases
    // If difference is more than 12 hours, we're likely crossing midnight
    if (second_diff > 12 * 3600) {
        // GPS time is ahead, but we're actually crossing backwards
        // e.g., RTC: 00:01:00, GPS: 23:10:00 (previous day)
        second_diff -= 24 * 3600;
    } else if (second_diff < -12 * 3600) {
        // GPS time is behind, but we're actually crossing forwards
        // e.g., RTC: 23:55:00, GPS: 00:05:00 (next day)
        second_diff += 24 * 3600;
    }
    
    // Update if there's any difference
    if (second_diff != 0) {
        DateTime adjustment = now;
        
        // Calculate the new date/time
        if (second_diff >= 0) {
            // Moving forward in time
            adjustment = now + TimeSpan(0, 0, 0, second_diff);
        } else {
            // Moving backward in time
            adjustment = now - TimeSpan(0, 0, 0, -second_diff);
        }
        
        // Update the RTC
        rtc.adjust(DateTime(adjustment.year(),
                          adjustment.month(),
                          adjustment.day(),
                          gps_data->hour,
                          gps_data->minute,
                          gps_data->second));
        
        printf_debug("RTC synchronized with GPS time:");
        printf_debug("Old RTC time: %02d/%02d/%02d %02d:%02d:%02d",
                    now.year() % 100, now.month(), now.day(),
                    now.hour(), now.minute(), now.second());
        printf_debug("New RTC time: %02d/%02d/%02d %02d:%02d:%02d",
                    adjustment.year() % 100, adjustment.month(), adjustment.day(),
                    gps_data->hour, gps_data->minute, gps_data->second);
    }
}


void cpslo(unsigned long t){
  const unsigned long DOT = 250;      // Duration of a dot
  // C: -.-. (dash-dot-dash-dot)
  if (t < DOT*3) {                      // First dash
      digitalWrite(FLAG_LED_PIN, HIGH);
  }
  else if (t < DOT*4) {                 // Gap
      digitalWrite(FLAG_LED_PIN, LOW);
  }
  else if (t < DOT*5) {                 // Dot
      digitalWrite(FLAG_LED_PIN, HIGH);
  }
  else if (t < DOT*6) {                 // Gap
      digitalWrite(FLAG_LED_PIN, LOW);
  }
  else if (t < DOT*9) {                 // Second dash
      digitalWrite(FLAG_LED_PIN, HIGH);
  }
  else if (t < DOT*10) {                // Gap
      digitalWrite(FLAG_LED_PIN, LOW);
  }
  else if (t < DOT*11) {                // Second dot
      digitalWrite(FLAG_LED_PIN, HIGH);
  }
  else if (t < DOT*14) {                // Gap between letters
      digitalWrite(FLAG_LED_PIN, LOW);
  }
  // P: .--. (dot-dash-dash-dot)
  else if (t < DOT*15) {                // First dot
      digitalWrite(FLAG_LED_PIN, HIGH);
  }
  else if (t < DOT*16) {                // Gap
      digitalWrite(FLAG_LED_PIN, LOW);
  }
  else if (t < DOT*19) {                // First dash
      digitalWrite(FLAG_LED_PIN, HIGH);
  }
  else if (t < DOT*20) {                // Gap
      digitalWrite(FLAG_LED_PIN, LOW);
  }
  else if (t < DOT*23) {                // Second dash
      digitalWrite(FLAG_LED_PIN, HIGH);
  }
  else if (t < DOT*24) {                // Gap
      digitalWrite(FLAG_LED_PIN, LOW);
  }
  else if (t < DOT*25) {                // Final dot
      digitalWrite(FLAG_LED_PIN, HIGH);
  }
  else if (t < DOT*28) {                // Gap between letters
      digitalWrite(FLAG_LED_PIN, LOW);
  }
  // S: ... (three dots)
  else if (t < DOT*29) {                // First dot
      digitalWrite(FLAG_LED_PIN, HIGH);
  }
  else if (t < DOT*30) {                // Gap
      digitalWrite(FLAG_LED_PIN, LOW);
  }
  else if (t < DOT*31) {                // Second dot
      digitalWrite(FLAG_LED_PIN, HIGH);
  }
  else if (t < DOT*32) {                // Gap
      digitalWrite(FLAG_LED_PIN, LOW);
  }
  else if (t < DOT*33) {                // Third dot
      digitalWrite(FLAG_LED_PIN, HIGH);
  }
  else if (t < DOT*36) {                // Gap between letters
      digitalWrite(FLAG_LED_PIN, LOW);
  }
  // L: .-.. (dot-dash-dot-dot)
  else if (t < DOT*37) {                // First dot
      digitalWrite(FLAG_LED_PIN, HIGH);
  }
  else if (t < DOT*38) {                // Gap
      digitalWrite(FLAG_LED_PIN, LOW);
  }
  else if (t < DOT*41) {                // Dash
      digitalWrite(FLAG_LED_PIN, HIGH);
  }
  else if (t < DOT*42) {                // Gap
      digitalWrite(FLAG_LED_PIN, LOW);
  }
  else if (t < DOT*43) {                // Second dot
      digitalWrite(FLAG_LED_PIN, HIGH);
  }
  else if (t < DOT*44) {                // Gap
      digitalWrite(FLAG_LED_PIN, LOW);
  }
  else if (t < DOT*45) {                // Third dot
      digitalWrite(FLAG_LED_PIN, HIGH);
  }
  else if (t < DOT*48) {                // Gap between letters
      digitalWrite(FLAG_LED_PIN, LOW);
  }
  // O: --- (three dashes)
  else if (t < DOT*51) {                // First dash
      digitalWrite(FLAG_LED_PIN, HIGH);
  }
  else if (t < DOT*52) {                // Gap
      digitalWrite(FLAG_LED_PIN, LOW);
  }
  else if (t < DOT*55) {                // Second dash
      digitalWrite(FLAG_LED_PIN, HIGH);
  }
  else if (t < DOT*56) {                // Gap
      digitalWrite(FLAG_LED_PIN, LOW);
  }
  else if (t < DOT*59) {                // Third dash
      digitalWrite(FLAG_LED_PIN, HIGH);
  }
  else {                                // Final gap before repeat
      digitalWrite(FLAG_LED_PIN, LOW);
  }
}

