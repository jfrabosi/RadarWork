// Copyright (c) Acconeer AB, 2022-2024
// All rights reserved
// This file is subject to the terms and conditions defined in the file
// 'LICENSES/license_acconeer.txt', (BSD 3-Clause License) which is part
// of this source code package.

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>
#include <stdarg.h>

#include "acc_definitions_a121.h"
#include "acc_definitions_common.h"
#include "acc_detector_distance.h"
#include "acc_hal_definitions_a121.h"
#include "acc_hal_integration_a121.h"
#include "acc_integration.h"
#include "acc_integration_log.h"
#include "acc_rss_a121.h"
#include "acc_sensor.h"
#include "acc_version.h"

#include "stm32l4xx_hal.h"
#include "main.h"

extern UART_HandleTypeDef DEBUG_UART_HANDLE;

/** \example example_detector_distance.c
 * @brief This is an example on how the Detector Distance API can be used
 * @n
 * This example executes as follows:
 *   - Retrieve HAL integration
 *   - Initialize distance detector resources:
 *     + Create distance detector configuration
 *     + Update configuration settings
 *     + Create Distance detector handle
 *     + Create buffer for detector calibration data
 *     + Create buffer for sensor data
 *   - Create and calibrate the sensor
 *   - Calibrate the detector
 *   - Measure distances with the detector (loop):
 *     + Prepare sensor with the detector
 *     + Measure and wait until a read can be done
 *     + Process measurement and print the result
 *     + Handle "calibration_needed" indication
 *   - Cleanup:
 *     + Destroy detector configuration
 *     + Destroy detector handle
 *     + Destroy sensor data buffer
 *     + Destroy detector calibration data buffer
 */


typedef enum
{
  DISTANCE_PRESET_CONFIG_BALANCED,
  DISTANCE_PRESET_CONFIG_HIGH_ACCURACY,
} distance_preset_config_t;

typedef enum
{
  CONFIG_START_M,
  CONFIG_END_M,
  CONFIG_UPDATE_RATE,
  CONFIG_MAX_STEP_LENGTH,
  CONFIG_MAX_PROFILE,
  CONFIG_SIGNAL_QUALITY,
  CONFIG_REFLECTOR_SHAPE,
  CONFIG_THRESHOLD_SENSITIVITY,
  CONFIG_EXIT
} config_settings_names_t;

#define SENSOR_ID (1U)
#define SENSOR_TIMEOUT_MS (2000U)
#define DEFAULT_UPDATE_RATE (15.0f)

#define MAX_BUFFER_SIZE 35

typedef struct
{
  acc_sensor_t                      *sensor;
  acc_detector_distance_config_t    *config;
  acc_detector_distance_handle_t    *handle;
  void                              *buffer;
  uint32_t                          buffer_size;
  uint8_t                           *detector_cal_result_static;
  uint32_t                          detector_cal_result_static_size;
  acc_detector_cal_result_dynamic_t detector_cal_result_dynamic;
} distance_detector_resources_t;

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
  bool testing_update_rate;
  float true_update_rate;
  int low_power_mode;
} config_settings_t;

static void cleanup(distance_detector_resources_t *resources);


static void set_config(acc_detector_distance_config_t *detector_config, distance_preset_config_t preset);


static bool initialize_detector_resources(distance_detector_resources_t *resources);


static bool do_sensor_calibration(acc_sensor_t     *sensor,
                                  acc_cal_result_t *sensor_cal_result,
                                  void             *buffer,
                                  uint32_t         buffer_size);


static bool do_full_detector_calibration(distance_detector_resources_t *resources,
                                         const acc_cal_result_t        *sensor_cal_result);


static bool do_detector_calibration_update(distance_detector_resources_t *resources,
                                           const acc_cal_result_t        *sensor_cal_result);


static bool do_detector_get_next(distance_detector_resources_t  *resources,
                                 const acc_cal_result_t         *sensor_cal_result,
                                 acc_detector_distance_result_t *result);


static void print_distance_result(const acc_detector_distance_result_t *result);

static bool get_esp32_serial(char *result, uint16_t buf_size, bool wait_for_enter_key);

static bool load_config(config_settings_t *config);

static void config_menu(config_settings_t *config, acc_detector_distance_config_t *detector_config);

static void print_config_menu(config_settings_t *config);

static float parse_float(const char *string);

static int parse_int(const char *string);

static void set_custom_config(config_settings_t *config, acc_detector_distance_config_t *detector_config);

static int Hprintf(const char *format, ...);

int acconeer_main_JJH(int argc, char *argv[]);

int acconeer_main_JJH(int argc, char *argv[])
{
  (void)argc;
  (void)argv;
  distance_detector_resources_t resources = { 0 };

  char received_uart_data[MAX_BUFFER_SIZE];
  bool change_config = false;
  config_settings_t current_config;
  uint32_t startTime = HAL_GetTick();
  int16_t update_counter = 0;
  uint32_t testTime = 1000;
  uint8_t debug_buf = 0;

  printf("--- INITIALIZATION ---\n");

  printf("Acconeer software version %s\n", acc_version_get());

  const acc_hal_a121_t *hal = acc_hal_rss_integration_get_implementation();

  if (!acc_rss_hal_register(hal))
  {
    return EXIT_FAILURE;
  }

  resources.config = acc_detector_distance_config_create();
  if (resources.config == NULL)
  {
    printf("acc_detector_distance_config_create() failed\n");
    cleanup(&resources);
    return EXIT_FAILURE;
  }

  set_config(resources.config, DISTANCE_PRESET_CONFIG_BALANCED);

  uint32_t sleep_time_ms = (uint32_t)(1000.0f / DEFAULT_UPDATE_RATE);

  acc_integration_set_periodic_wakeup(sleep_time_ms);

  current_config.low_power_mode = 1;

  load_config(&current_config);
  config_menu(&current_config, resources.config);
  set_custom_config(&current_config, resources.config);

  while(true){

    if (change_config){
      cleanup(&resources);

      resources.config = acc_detector_distance_config_create();
      if (resources.config == NULL)
      {
        printf("acc_detector_distance_config_create() failed\n");
        cleanup(&resources);
        return EXIT_FAILURE;
      }

      config_menu(&current_config, resources.config);
      set_custom_config(&current_config, resources.config);
      change_config = false;
    }

    if (!initialize_detector_resources(&resources))
    {
      printf("Initializing detector resources failed\n");
      cleanup(&resources);
      return EXIT_FAILURE;
    }

    // Print the configuration
//    acc_detector_distance_config_log(resources.handle, resources.config);

    /* Turn the sensor on */
    acc_hal_integration_sensor_supply_on(SENSOR_ID);
    acc_hal_integration_sensor_enable(SENSOR_ID);

    resources.sensor = acc_sensor_create(SENSOR_ID);
    if (resources.sensor == NULL)
    {
      printf("acc_sensor_create() failed\n");
      cleanup(&resources);
      return EXIT_FAILURE;
    }

    acc_cal_result_t sensor_cal_result;

    if (!do_sensor_calibration(resources.sensor, &sensor_cal_result, resources.buffer, resources.buffer_size))
    {
      printf("Sensor calibration failed\n");
      cleanup(&resources);
      return EXIT_FAILURE;
    }

    if (!do_full_detector_calibration(&resources, &sensor_cal_result))
    {
      printf("Detector calibration failed\n");
      cleanup(&resources);
      return EXIT_FAILURE;
    }

    if (current_config.testing_update_rate){
      update_counter = -3;
      startTime = HAL_GetTick();
      uint32_t sixTime = sleep_time_ms * 3;
      testTime = (sixTime < 3000) ? 3000 : sixTime; // max(3000, sixTime)
    }

    HAL_UART_Receive_DMA(&DEBUG_UART_HANDLE, &debug_buf, 1);

    while (!change_config)
    {
      acc_detector_distance_result_t result = { 0 };

      if (!do_detector_get_next(&resources, &sensor_cal_result, &result))
      {
        printf("Could not get next result\n");
        cleanup(&resources);
        return EXIT_FAILURE;
      }

      /* If "calibration needed" is indicated, the sensor needs to be recalibrated and the detector calibration updated */
      if (result.calibration_needed)
      {
        printf("Sensor recalibration and detector calibration update needed ... \n");

        if (!do_sensor_calibration(resources.sensor, &sensor_cal_result, resources.buffer, resources.buffer_size))
        {
          printf("Sensor calibration failed\n");
          cleanup(&resources);
          return EXIT_FAILURE;
        }

        /* Once the sensor is recalibrated, the detector calibration should be updated and measuring can continue. */
        if (!do_detector_calibration_update(&resources, &sensor_cal_result))
        {
          printf("Detector calibration update failed\n");
          cleanup(&resources);
          return EXIT_FAILURE;
        }

        printf("Sensor recalibration and detector calibration update done!\n");
      }

      else
      {
        if (current_config.low_power_mode || current_config.testing_update_rate){
          acc_hal_integration_sensor_disable(SENSOR_ID);
          print_distance_result(&result);
          acc_integration_sleep_until_periodic_wakeup();
          acc_hal_integration_sensor_enable(SENSOR_ID);
        }
        else{
          print_distance_result(&result);
        }
      }

      if (debug_buf != 0){
        change_config = true;
        debug_buf = 0;
        printf("\nDATA_END\n");
      }

      if (current_config.testing_update_rate){
        update_counter++;
        if (update_counter == 0){
          startTime = HAL_GetTick();
        }
        if (update_counter >= 100 || HAL_GetTick() - startTime >= testTime){
          current_config.true_update_rate = (float) update_counter * 1000 / ((float)(HAL_GetTick() - startTime));
          current_config.testing_update_rate = false;
          change_config = true;
        }
      }
    }
  }

  cleanup(&resources);

  printf("Done!\n");

  return EXIT_SUCCESS;
}

static bool get_esp32_serial(char *result, uint16_t buf_size, bool wait_for_enter_key)
{
  uint8_t buffer[MAX_BUFFER_SIZE];
  HAL_StatusTypeDef status;
  uint16_t received = 0;
  uint32_t startTime = HAL_GetTick();

  while (received < buf_size)
  {
    if (__HAL_UART_GET_FLAG(&DEBUG_UART_HANDLE, UART_FLAG_RXNE) == SET)
    {
      status = HAL_UART_Receive(&DEBUG_UART_HANDLE, &buffer[received], 1, 100);

      if (status == HAL_OK)
      {
        startTime = HAL_GetTick() - 10;
        received++;
        if (buffer[received - 1] == '\r')
        {
          buffer[received - 1] = '\0';
          break;  // End of message
        }
      }
    }

    // Check for timeout (e.g., 1000ms)
    if ((HAL_GetTick() - startTime > 1000) && !wait_for_enter_key)
    {
      if (received == 0)
      {
        return false;  // No data received within timeout
      }
      break;  // Partial message received
    }
  }

  buffer[received] = '\0';  // Null-terminate the string
  strcpy(result, (char*)buffer);
  return true;
}

static bool load_config(config_settings_t *config){
  char _received_uart_data[MAX_BUFFER_SIZE];

  printf("M807\n");

  while (!get_esp32_serial(_received_uart_data, 33, false));

  // Check if received data fits format
  // format: "00.40,01.20,05.0,02,5,35.0,1,0.50\0"
  //   start_m, end_m, update_rate, max_step_length, max_profile, signal_quality, reflector_shape, threshold_sensitivity
  //   float,   float, float,       int,             int,         float,          int,             float
  if (strchr(_received_uart_data, ',') != NULL) {
    char *token;
    int field_count = 0;

    token = strtok(_received_uart_data, ",");
    while (token != NULL && field_count < 8) {
      switch (field_count) {
        case 0: config->start_m = atof(token); break;
        case 1: config->end_m = atof(token); break;
        case 2: config->update_rate = atof(token); break;
        case 3: config->max_step_length = atoi(token); break;
        case 4: config->max_profile = atoi(token); break;
        case 5: config->signal_quality = atof(token); break;
        case 6: config->reflector_shape = atoi(token); break;
        case 7: config->threshold_sensitivity = atof(token); break;
      }
      token = strtok(NULL, ",");
      field_count++;
    }

    if (field_count == 8) {
      printf("Successfully loaded configuration.\n");
      return true;
    }
  }

  printf("Invalid config format. Default settings applied.\n");
  return false;
}

static void config_menu(config_settings_t *config, acc_detector_distance_config_t *detector_config){
  bool good_config = false;
  bool good_value = false;

  char _received_uart_data[MAX_BUFFER_SIZE];

  Hprintf("\n");
  Hprintf("       :+###%%#%%#%%#%%#%%###*:              \n");
  Hprintf("     -#%%*==------------=*%%#:            \n");
  Hprintf("    =%%*-:::::::::::::::::=%%+            \n");
  Hprintf("   :%%#-::::::::::::::::::*%%=            \n");
  Hprintf("   +%%=::::::::**-:::::::-##:.......     \n");
  Hprintf("  .%%#-:::::::=%%%%%%%%#%%####%%%%%%%%%%%%%%%%%%%%%%%%%%%%: \n");
  Hprintf("  *%%=::::::::*%%%%%%#=:::::::::::::::::-*%%=\n");
  Hprintf(" :%%*-:::::::-#%%%%%%%%+::::::::::::::::::=##\n");
  Hprintf(" *%%+::::::::*%%%%%%%%#+==========::::::::+%%+\n");
  Hprintf(":##-:::::::-#*=------=+%%%%%%%%%%=:::::::-##:\n");
  Hprintf("+%%+::::::::-=:::::::::*%%%%%%%%*::::::::+%%* \n");
  Hprintf("##=::::::::::::::::::+%%%%%%%%#=:::::::-#%%: \n");
  Hprintf("+%%*-:::::::::::::::=*%%%%%%%%%%*::::::::+%%+  \n");
  Hprintf(" =%%%%%%%%#########***********-:::::::-##:  \n");
  Hprintf("     .......:%%#-::::::::::::::::::*%%=   \n");
  Hprintf("            *%%=:::::::::::::::::-*%%*    \n");
  Hprintf("           -%%*::::::::=*******#%%%%#:     \n");
  Hprintf("          .#%%=::::::::*%%*====-:.        \n");
  Hprintf("          *%%*::::::::=%%%%                \n");
  Hprintf("        :%%#+-:::::::=#%%:                \n");
  Hprintf("       -%%%%%%%%%%%%%%%%%%%%%%%%%%%%=                 \n");
  Hprintf("\n");
  HAL_Delay(500);

  print_config_menu(config);

  while (!good_config){
    memset(&_received_uart_data, 0, MAX_BUFFER_SIZE);
    while (!get_esp32_serial(_received_uart_data, 1, false));
    printf("M918\n");
    HAL_Delay(50);

    char config_char = tolower(_received_uart_data[0]);
    good_value = false;

    switch (config_char){

      case 's':
        Hprintf("\nSet the start of the measurement range. Current value: %.2f m\n", config->start_m);
        Hprintf("Format: XX.XX\n");
        Hprintf("  Input must match format exactly. Enter leading and trailing zeros as appropriate.\n");
        Hprintf("  Units are in meters.\n");
        Hprintf("Notes:\n");
        Hprintf("  Generally advised to leave a margin of 0.1m above maximum wave height.\n");
        Hprintf("Example:\n");
        Hprintf("  Radar is placed 10m above average wave height, and waves height fluctuates within +/- 1m.\n");
        Hprintf("  Start of measurement range should be set to 0.8m.\n");
        Hprintf("  User would enter \"00.80\" into terminal with no quotes.\n\n");
        Hprintf("M917\n");

        while (!good_value){
          while (!get_esp32_serial(_received_uart_data, 5, true));
          Hprintf("M918\n");

          float new_start_m = parse_float(_received_uart_data);
          if (!isnan(new_start_m) && new_start_m >= 0.1f && new_start_m <= 20.0f) {
            acc_detector_distance_config_start_set(detector_config, new_start_m);
            Hprintf("\nStart of measurement range set to %.2f m\n", new_start_m);

            Hprintf("Is this the desired value? Type Y for yes, N for no.\n\n");
            Hprintf("M917\n");
            while (!get_esp32_serial(_received_uart_data, 1, true));
            Hprintf("M918\n");

            config_char = tolower(_received_uart_data[0]);
            if (config_char == 'y'){
              config->start_m = new_start_m;
              good_value = true;
              Hprintf("\nValue saved.\n\n");
              HAL_Delay(1000);
              print_config_menu(config);
            }
            else{
              Hprintf("\nPlease enter a new number between 00.10 and 20.00.\n\n");
              Hprintf("M917\n");
            }
          }
          else {
            Hprintf("\nInvalid input. Please enter a number between 00.10 and 20.00.\n\n");
            Hprintf("M917\n");
          }
        }
        break;

      case 'e':

        Hprintf("\nSet the end of the measurement range. Current value: %.2f m\n", config->end_m);
        Hprintf("Format: XX.XX\n");
        Hprintf("  Input must match format exactly. Enter leading and trailing zeros as appropriate.\n");
        Hprintf("  Units are in meters.\n");
        Hprintf("Notes:\n");
        Hprintf("  Generally advised to leave a margin of 0.1m below maximum wave height.\n");
        Hprintf("Example:\n");
        Hprintf("  Radar is placed 10m above average wave height, and waves height fluctuates within +/- 1m.\n");
        Hprintf("  End of measurement range should be set to 1.2m.\n");
        Hprintf("  User would enter \"01.20\" into terminal with no quotes.\n\n");
        Hprintf("M917\n");

        while (!good_value){
          while (!get_esp32_serial(_received_uart_data, 5, true));
          Hprintf("M918\n");

          float new_end_m = parse_float(_received_uart_data);
          if (!isnan(new_end_m) && new_end_m >= 0.1f && new_end_m <= 20.0f) {
            acc_detector_distance_config_start_set(detector_config, new_end_m);
            Hprintf("\End of measurement range set to %.2f m\n", new_end_m);

            Hprintf("Is this the desired value? Type Y for yes, N for no.\n\n");
            Hprintf("M917\n");
            while (!get_esp32_serial(_received_uart_data, 1, true));
            Hprintf("M918\n");

            config_char = tolower(_received_uart_data[0]);
            if (config_char == 'y'){
              config->end_m = new_end_m;
              good_value = true;
              Hprintf("\nValue saved.\n\n");
              HAL_Delay(1000);
              print_config_menu(config);
            }
            else{
              Hprintf("\nPlease enter a new number between 00.10 and 20.00.\n\n");
              Hprintf("M917\n");
            }
          }
          else {
            Hprintf("\nInvalid input. Please enter a number between 00.10 and 20.00.\n\n");
            Hprintf("M917\n");
          }
        }
        break;

      case 'u':
        Hprintf("\nSet the update rate. Current value: %.2f Hz\n", config->update_rate);
        Hprintf("Format: XX.X\n");
        Hprintf("  Input must match format exactly. Enter leading and trailing zeros as appropriate.\n");
        Hprintf("  Units are in Hertz.\n");
        Hprintf("Notes:\n");
        Hprintf("  Keep as low as required. Greatly affects power consumption.\n");
        Hprintf("Example:\n");
        Hprintf("  A measurement is required every 0.2 seconds.\n");
        Hprintf("  Update rate should be set to 5 Hz.\n");
        Hprintf("  User would enter \"05.0\" into terminal with no quotes.\n\n");
        Hprintf("M917\n");

        while (!good_value){
          while (!get_esp32_serial(_received_uart_data, 4, true));
          Hprintf("M918\n");

          float new_update_rate = parse_float(_received_uart_data);
          if (!isnan(new_update_rate) && new_update_rate >= 0.1f && new_update_rate <= 99.9f) {
            if (new_update_rate >= 20.0f) {
              Hprintf("\nWARNING: Actual update rate may be lower than desired.\n");
              Hprintf("Test true update rate in configuration menu.\n");
            }
            Hprintf("\nUpdate rate set to %.1f Hz\n", new_update_rate);

            Hprintf("Is this the desired value? Type Y for yes, N for no.\n\n");
            Hprintf("M917\n");
            while (!get_esp32_serial(_received_uart_data, 1, true));
            Hprintf("M918\n");

            config_char = tolower(_received_uart_data[0]);
            if (config_char == 'y'){
              config->update_rate = new_update_rate;
              good_value = true;
              Hprintf("\nValue saved.\n\n");
              HAL_Delay(1000);
              print_config_menu(config);
            }
            else{
              Hprintf("\nPlease enter a new number between 00.1 and 99.9.\n\n");
              Hprintf("M917\n");
            }
          }
          else {
            Hprintf("\nInvalid input. Please enter a number between 00.1 and 99.9.\n\n");
            Hprintf("M917\n");
          }
        }
      break;
      case 'l':


        break;
      case 'p':


        break;
      case 'q':

        break;
      case 'r':

        break;
      case 't':

        break;
      case 'a':
        if (config->start_m >= config->end_m){
          Hprintf("\nWARNING: Start of range is greater than end of range! This must be fixed before continuing.\n\n");
          Hprintf("\nInput is not accepted. Please send one of the characters above.\n");
          Hprintf("M917\n");
        }
        else{
          good_config = true;
          config->testing_update_rate = true;
          Hprintf("\nTesting actual update rate...\n");
        }
        break;

      case 'm':
        Hprintf("\nEnables or disables low power mode. Current value: %i\n", config->low_power_mode);
        Hprintf("Format: X\n");
        Hprintf("  Input must match format exactly.\n");
        Hprintf("  Enter \"1\" to enable low power mode.\n");
        Hprintf("  Enter \"0\" to disable low power mode.\n");
        Hprintf("Notes:\n");
        Hprintf("  In low power mode, the STM32 powers down between measurements. In this mode, the configuration\n");
        Hprintf("  menu can only be called by power cycling the entire unit.\n");
        Hprintf("  In low power mode, the actual update rate will NOT be controlled by the update set rate.\n");
        Hprintf("  The testing of the actual update rate in the configuration menu is not affected by low power mode.\n");
        Hprintf("Example:\n");
        Hprintf("  The user wants to test different configuration settings to tune the radar.\n");
        Hprintf("  Low power mode should be disabled, and the user would enter \"0\" with no quotes.\n");
        Hprintf("  Once the user settles on a configuration (and the settings will not be changed), low power mode\n");
        Hprintf("  should be enabled, and the user would enter \"1\" with no quotes.\n\n");
        Hprintf("M917\n");

        while (!good_value){
          while (!get_esp32_serial(_received_uart_data, 1, true));
          Hprintf("M918\n");

          int new_power_mode = parse_int(_received_uart_data);
          if (new_power_mode == 1 || new_power_mode == 0) {

            Hprintf("\nLow power mode set to %i\n", new_power_mode);
            Hprintf("Is this the desired value? Type Y for yes, N for no.\n\n");
            Hprintf("M917\n");
            while (!get_esp32_serial(_received_uart_data, 1, true));
            Hprintf("M918\n");

            config_char = tolower(_received_uart_data[0]);
            if (config_char == 'y'){
              config->low_power_mode = new_power_mode;
              good_value = true;
              Hprintf("\nValue saved.\n\n");
              HAL_Delay(1000);
              print_config_menu(config);
            }
            else{
              Hprintf("\nPlease enter either a 0 or a 1.\n\n");
              Hprintf("M917\n");
            }
          }
          else {
            Hprintf("\nInvalid input. Please enter either a 0 or a 1.\n\n");
            Hprintf("M917\n");
          }
        }
      break;

      case 'x':
        if (config->low_power_mode){
          Hprintf("\nWARNING: Low power mode is enabled. In this mode, the configuration menu will not re-open unless the STM32 is\n");
          Hprintf("power-cycled. This mode is only recommended for final deployment. Do you want to keep low power mode enabled (Y/N)?\n\n");
          Hprintf("M917\n");
          while (!get_esp32_serial(_received_uart_data, 1, true));
          Hprintf("M918\n");

          while (!good_value){
            config_char = tolower(_received_uart_data[0]);
            if (config_char == 'n'){
              config->low_power_mode = 0;
              good_value = true;
              Hprintf("\nLow power mode disabled.\n\n");
              Hprintf("WARNING: Low power mode is disabled. The actual update rate will be different from the set update rate.\n");
              Hprintf("The actual update rate will return to the set update rate when low power mode is enabled.\n\n");
              Hprintf("Send any character to re-open configuration menu.\n\n");
              HAL_Delay(3000);
            }
            else if (config_char == 'y'){
              good_value = true;
              Hprintf("\nLow power mode enabled.\n\n");
              HAL_Delay(1000);
            }
            else{
              Hprintf("\nPlease enter either Y or N.\n\n");
              Hprintf("M917\n");
            }
          }
        }
        else{
          Hprintf("\nWARNING: Low power mode is disabled. The actual update rate will be different from the set update rate.\n");
          Hprintf("The actual update rate will return to the set update rate when low power mode is enabled.\n\n");
          Hprintf("Send any character to re-open configuration menu.\n\n");
          HAL_Delay(3000);
        }

        if (config->start_m >= config->end_m){
          Hprintf("WARNING: Start of range is greater than end of range! This must be fixed before continuing.\n");
          Hprintf("\nPlease send one of the characters from the menu above.\n\n");
          Hprintf("M917\n");
        }
        else{
          good_config = true;
          Hprintf("Configuration saved to SD card. Returning to normal operations.\n\n");
          HAL_Delay(3000);
          Hprintf("DATA_START\n");
        }
        break;

      default:
        Hprintf("\nInput is not accepted. Please send one of the characters above.\n\n");
        Hprintf("M917\n");
        break;
    }
  }
}

static int Hprintf(const char *format, ...) {
  va_list args;
  int result;

  va_start(args, format);
  result = vprintf(format, args);
  va_end(args);

  HAL_Delay(100);
  return result;
}

static void print_config_menu(config_settings_t *config){
  Hprintf("--- CONFIGURATION MENU ---\n");
  Hprintf("Send one of the below characters to change that configuration setting.\n");
  Hprintf("Changes will be saved automatically.\n\n");
  Hprintf("S: Change the start of the measurement range.\n");
  Hprintf("   Current value: %.2f m\n", config->start_m);
  Hprintf("E: Change the end of the measurement range.\n");
  Hprintf("   Current value: %.2f m\n", config->end_m);
  Hprintf("U: Change the update rate. Affects power consumption.\n");
  Hprintf("   Current value: %.1f Hz\n", config->update_rate);
  Hprintf("L: Change the maximum step length of the measurements. Affects power consumption and resolution.\n");
  Hprintf("   Current value: %i\n", config->max_step_length);
  Hprintf("P: Change the measurement profile. Affects power consumption and accuracy.\n");
  Hprintf("   Current value: %i\n", config->max_profile);
  Hprintf("Q: Change the signal quality. Affects power consumption and accuracy.\n");
  Hprintf("   Current value: %.1f\n", config->signal_quality);
  Hprintf("R: Change the reflector shape.\n");
  Hprintf("   Current value: %i (0: generic, 1: planar)\n", config->reflector_shape);
  Hprintf("T: Change the threshold sensitivity.\n");
  Hprintf("   Current value: %.2f\n", config->threshold_sensitivity);
  Hprintf("M: Enables or disables low power mode.\n");
  Hprintf("   Current value: %i (0: disabled, 1: enabled)\n", config->low_power_mode);
  Hprintf("A: Measure the actual update rate.\n");
  Hprintf("   True update rate: %.1f Hz\n", config->true_update_rate);
  Hprintf("X: Exit the menu.\n\n");
  Hprintf("M917\n");
}

static float parse_float(const char *string) {
  char *end;
  float result = strtof(string, &end);
  if (end == string || *end != '\0') {
      // Parsing failed
      return NAN;
  }
  return result;
}

static int parse_int(const char *string) {
    char *end;
    long result = strtol(string, &end, 10);
    if (end == string || *end != '\0') {
        // Parsing failed
        return -1;
    }
    return (int)result;
}

static void set_custom_config(config_settings_t *config, acc_detector_distance_config_t *detector_config){
  acc_detector_distance_config_start_set(detector_config, config->start_m);
  acc_detector_distance_config_end_set(detector_config, config->end_m);
  acc_detector_distance_config_max_step_length_set(detector_config, config->max_step_length);
  acc_detector_distance_config_max_profile_set(detector_config, config->max_profile);
  acc_detector_distance_config_reflector_shape_set(detector_config, config->reflector_shape);
  acc_detector_distance_config_threshold_sensitivity_set(detector_config, config->threshold_sensitivity);
  acc_detector_distance_config_signal_quality_set(detector_config, config->signal_quality);

  uint32_t sleep_time_ms = (uint32_t)(1000.0f / config->update_rate);
  acc_integration_set_periodic_wakeup(sleep_time_ms);

  acc_detector_distance_config_peak_sorting_set(detector_config, ACC_DETECTOR_DISTANCE_PEAK_SORTING_STRONGEST);
  acc_detector_distance_config_threshold_method_set(detector_config, ACC_DETECTOR_DISTANCE_THRESHOLD_METHOD_CFAR);
  acc_detector_distance_config_close_range_leakage_cancellation_set(detector_config, false);
}

static void cleanup(distance_detector_resources_t *resources)
{
  acc_hal_integration_sensor_disable(SENSOR_ID);
  acc_hal_integration_sensor_supply_off(SENSOR_ID);

  acc_detector_distance_config_destroy(resources->config);
  acc_detector_distance_destroy(resources->handle);

  acc_integration_mem_free(resources->buffer);
  acc_integration_mem_free(resources->detector_cal_result_static);

  if (resources->sensor != NULL)
  {
    acc_sensor_destroy(resources->sensor);
  }
}


static void set_config(acc_detector_distance_config_t *detector_config, distance_preset_config_t preset)
{
  // Add configuration of the detector here
  switch (preset)
  {
    case DISTANCE_PRESET_CONFIG_BALANCED:
      acc_detector_distance_config_start_set(detector_config, 0.25f);
      acc_detector_distance_config_end_set(detector_config, 3.0f);
      acc_detector_distance_config_max_step_length_set(detector_config, 0U);
      acc_detector_distance_config_max_profile_set(detector_config, ACC_CONFIG_PROFILE_5);
      acc_detector_distance_config_reflector_shape_set(detector_config, ACC_DETECTOR_DISTANCE_REFLECTOR_SHAPE_GENERIC);
      acc_detector_distance_config_peak_sorting_set(detector_config, ACC_DETECTOR_DISTANCE_PEAK_SORTING_STRONGEST);
      acc_detector_distance_config_threshold_method_set(detector_config, ACC_DETECTOR_DISTANCE_THRESHOLD_METHOD_CFAR);
      acc_detector_distance_config_threshold_sensitivity_set(detector_config, 0.5f);
      acc_detector_distance_config_signal_quality_set(detector_config, 15.0f);
      acc_detector_distance_config_close_range_leakage_cancellation_set(detector_config, false);
      break;

    case DISTANCE_PRESET_CONFIG_HIGH_ACCURACY:
      acc_detector_distance_config_start_set(detector_config, 0.25f);
      acc_detector_distance_config_end_set(detector_config, 3.0f);
      acc_detector_distance_config_max_step_length_set(detector_config, 2U);
      acc_detector_distance_config_max_profile_set(detector_config, ACC_CONFIG_PROFILE_3);
      acc_detector_distance_config_reflector_shape_set(detector_config, ACC_DETECTOR_DISTANCE_REFLECTOR_SHAPE_GENERIC);
      acc_detector_distance_config_peak_sorting_set(detector_config, ACC_DETECTOR_DISTANCE_PEAK_SORTING_STRONGEST);
      acc_detector_distance_config_threshold_method_set(detector_config, ACC_DETECTOR_DISTANCE_THRESHOLD_METHOD_CFAR);
      acc_detector_distance_config_threshold_sensitivity_set(detector_config, 0.5f);
      acc_detector_distance_config_signal_quality_set(detector_config, 20.0f);
      acc_detector_distance_config_close_range_leakage_cancellation_set(detector_config, false);
      break;
  }
}


static bool initialize_detector_resources(distance_detector_resources_t *resources)
{
  resources->handle = acc_detector_distance_create(resources->config);
  if (resources->handle == NULL)
  {
    printf("acc_detector_distance_create() failed\n");
    return false;
  }

  if (!acc_detector_distance_get_sizes(resources->handle, &(resources->buffer_size), &(resources->detector_cal_result_static_size)))
  {
    printf("acc_detector_distance_get_sizes() failed\n");
    return false;
  }

  resources->buffer = acc_integration_mem_alloc(resources->buffer_size);
  if (resources->buffer == NULL)
  {
    printf("sensor buffer allocation failed\n");
    return false;
  }

  resources->detector_cal_result_static = acc_integration_mem_alloc(resources->detector_cal_result_static_size);
  if (resources->detector_cal_result_static == NULL)
  {
    printf("calibration buffer allocation failed\n");
    return false;
  }

  return true;
}


static bool do_sensor_calibration(acc_sensor_t     *sensor,
                                  acc_cal_result_t *sensor_cal_result,
                                  void             *buffer,
                                  uint32_t         buffer_size)
{
  bool           status              = false;
  bool           cal_complete        = false;
  const uint16_t calibration_retries = 1U;

  // Random disturbances may cause the calibration to fail. At failure, retry at least once.
  for (uint16_t i = 0; !status && (i <= calibration_retries); i++)
  {
    // Reset sensor before calibration by disabling/enabling it
    acc_hal_integration_sensor_disable(SENSOR_ID);
    acc_hal_integration_sensor_enable(SENSOR_ID);

    do
    {
      status = acc_sensor_calibrate(sensor, &cal_complete, sensor_cal_result, buffer, buffer_size);

      if (status && !cal_complete)
      {
        status = acc_hal_integration_wait_for_sensor_interrupt(SENSOR_ID, SENSOR_TIMEOUT_MS);
      }
    } while (status && !cal_complete);
  }

  if (status)
  {
    /* Reset sensor after calibration by disabling/enabling it */
    acc_hal_integration_sensor_disable(SENSOR_ID);
    acc_hal_integration_sensor_enable(SENSOR_ID);
  }

  return status;
}


static bool do_full_detector_calibration(distance_detector_resources_t *resources,
                                         const acc_cal_result_t        *sensor_cal_result)
{
  bool done = false;
  bool status;

  do
  {
    status = acc_detector_distance_calibrate(resources->sensor,
                                             resources->handle,
                                             sensor_cal_result,
                                             resources->buffer,
                                             resources->buffer_size,
                                             resources->detector_cal_result_static,
                                             resources->detector_cal_result_static_size,
                                             &resources->detector_cal_result_dynamic,
                                             &done);

    if (status && !done)
    {
      status = acc_hal_integration_wait_for_sensor_interrupt(SENSOR_ID, SENSOR_TIMEOUT_MS);
    }
  } while (status && !done);

  return status;
}


static bool do_detector_calibration_update(distance_detector_resources_t *resources,
                                           const acc_cal_result_t        *sensor_cal_result)
{
  bool done = false;
  bool status;

  do
  {
    status = acc_detector_distance_update_calibration(resources->sensor,
                                                      resources->handle,
                                                      sensor_cal_result,
                                                      resources->buffer,
                                                      resources->buffer_size,
                                                      &resources->detector_cal_result_dynamic,
                                                      &done);

    if (status && !done)
    {
      status = acc_hal_integration_wait_for_sensor_interrupt(SENSOR_ID, SENSOR_TIMEOUT_MS);
    }
  } while (status && !done);

  return status;
}


static bool do_detector_get_next(distance_detector_resources_t  *resources,
                                 const acc_cal_result_t         *sensor_cal_result,
                                 acc_detector_distance_result_t *result)
{
  bool result_available = false;

  do
  {
    if (!acc_detector_distance_prepare(resources->handle, resources->config, resources->sensor, sensor_cal_result, resources->buffer,
                                       resources->buffer_size))
    {
      printf("acc_detector_distance_prepare() failed\n");
      return false;
    }

    if (!acc_sensor_measure(resources->sensor))
    {
      printf("acc_sensor_measure() failed\n");
      return false;
    }

    if (!acc_hal_integration_wait_for_sensor_interrupt(SENSOR_ID, SENSOR_TIMEOUT_MS))
    {
      printf("Sensor interrupt timeout\n");
      return false;
    }

    if (!acc_sensor_read(resources->sensor, resources->buffer, resources->buffer_size))
    {
      printf("acc_sensor_read() failed\n");
      return false;
    }

    if (!acc_detector_distance_process(resources->handle, resources->buffer, resources->detector_cal_result_static,
                                       &resources->detector_cal_result_dynamic,
                                       &result_available, result))
    {
      printf("acc_detector_distance_process() failed\n");
      return false;
    }
  } while (!result_available);

  return true;
}


static void print_distance_result(const acc_detector_distance_result_t *result)
{
  printf("%d detected distances", result->num_distances);
  if (result->num_distances > 0)
  {
    printf(": ");

    for (uint8_t i = 0; i < result->num_distances; i++)
    {
      printf("%" PRIfloat " m ", ACC_LOG_FLOAT_TO_INTEGER(result->distances[i]));
    }
  }

  printf("\n");
}
