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
//#include <ctype.h>
//#include <math.h>
//#include <stdarg.h>

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
#define DEFAULT_UPDATE_RATE (3.0f)
#define MAX_BUFFER_SIZE 64
#define HAL_GETTICK_SCALAR 1.00f //1.56f

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

static bool change_config = true;
uint32_t sleep_time_ms;

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


static void empty_uart_input_buffer(void);


static bool load_config(config_settings_t *config);


static bool wait_for_m_code(const char* send, const char* receive, int timeout_s);


static void set_custom_config(config_settings_t *config, acc_detector_distance_config_t *detector_config);


int acconeer_main_JJH(int argc, char *argv[]);


int acconeer_main_JJH(int argc, char *argv[])
{
  (void)argc;
  (void)argv;
  distance_detector_resources_t resources = { 0 };

  config_settings_t current_config;
  uint32_t startTime = HAL_GetTick();
  int16_t update_counter = 0;
  uint32_t testTime = 1000;

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

  sleep_time_ms = (uint32_t)(1000.0f / DEFAULT_UPDATE_RATE);

  acc_integration_set_periodic_wakeup(sleep_time_ms);

  current_config.update_rate = DEFAULT_UPDATE_RATE;

  if (load_config(&current_config))
  {
    set_custom_config(&current_config, resources.config);
    change_config = false;
  }

  while(true){

    while (change_config){
      cleanup(&resources);

      resources.config = acc_detector_distance_config_create();
      if (resources.config == NULL)
      {
        printf("acc_detector_distance_config_create() failed\n");
        cleanup(&resources);
        return EXIT_FAILURE;
      }

      if (load_config(&current_config))
      {
        set_custom_config(&current_config, resources.config);
        change_config = false;
      }
    }

    if (!initialize_detector_resources(&resources))
    {
      printf("Initializing detector resources failed\n");
      cleanup(&resources);
      return EXIT_FAILURE;
    }

    HAL_Delay(500);

    // Print the configuration
    acc_detector_distance_config_log(resources.handle, resources.config);

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

    HAL_Delay(3000);

    if (current_config.testing_update_rate){
      update_counter = -3;
      startTime = HAL_GetTick();
      uint32_t sixTime = sleep_time_ms * 6;         // time period for six measurements to occur
      testTime = (sixTime < 5000) ? 5000 : sixTime; // max(5000, sixTime)
    }
    else{
      printf("M114\n");
    }

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
        acc_hal_integration_sensor_disable(SENSOR_ID);
        print_distance_result(&result);
        printf("M197\n");
        acc_integration_sleep_until_periodic_wakeup();
        printf("M198\n");
        acc_hal_integration_sensor_enable(SENSOR_ID);
      }


      if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == GPIO_PIN_RESET){
        printf("M116\n");
        int timeout = (int) (3.0f / current_config.update_rate);
        if (wait_for_m_code("M918", "M919", 4 + timeout)){
          change_config = true;
        }
        else{
          printf("\nError: TX pulled low without reply.\n");
        }
      }

      if (current_config.testing_update_rate){
        update_counter++;
        if (update_counter == 0){
          printf("M814\n");
        }
        if (update_counter >= 100 || (HAL_GetTick() - startTime) >= testTime){
          current_config.testing_update_rate = false;
          change_config = true;
          printf("M815 %i\n", update_counter);
          HAL_Delay(100);
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
    status = HAL_UART_Receive(&DEBUG_UART_HANDLE, &buffer[received], 1, 100);

    if (status == HAL_OK)
    {
      startTime = HAL_GetTick();
      received++;
      if (buffer[received - 1] == '\r')
      {
        buffer[received - 1] = '\0';
        break;  // End of message
      }
    }

    // Check for timeout (e.g., 1000ms)
    if ((HAL_GetTick() - startTime > 100) && !wait_for_enter_key)
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


static void empty_uart_input_buffer(void)
{
  uint8_t t[1];
  while (HAL_UART_Receive(&DEBUG_UART_HANDLE, t, 1, 1) == HAL_OK);
}


static bool load_config(config_settings_t *config){
  char _received_uart_data[MAX_BUFFER_SIZE];
  uint32_t startTime = HAL_GetTick();

  empty_uart_input_buffer();
  printf("M807\n");

  while (!get_esp32_serial(_received_uart_data, 40, false)){
    if ((HAL_GetTick() - startTime) > 1000){
      startTime = HAL_GetTick();
      printf("M807\n");
      empty_uart_input_buffer();
    }
  }

  printf(_received_uart_data);
  printf("\n");

  // Check if received data fits format
  // format: "00.40,01.20,05.0,02,5,35.0,1,0.50,0,05.1\0"
  //   start_m, end_m, update_rate, max_step_length, max_profile, signal_quality, reflector_shape, threshold_sensitivity, testing_update_rate, true_update_rate
  //   float,   float, float,       int,             int,         float,          int,             float,                 int,                 float
  if (strchr(_received_uart_data, ',') != NULL) {
    char *token;
    int field_count = 0;

    token = strtok(_received_uart_data, ",");
    while (token != NULL && field_count < 10) {
      switch (field_count) {
        case 0: config->start_m = atof(token); break;
        case 1: config->end_m = atof(token); break;
        case 2: config->update_rate = atof(token); break;
        case 3: config->max_step_length = atoi(token); break;
        case 4: config->max_profile = atoi(token); break;
        case 5: config->signal_quality = atof(token); break;
        case 6: config->reflector_shape = atoi(token); break;
        case 7: config->threshold_sensitivity = atof(token); break;
        case 8: config->testing_update_rate = atof(token); break;
        case 9: config->true_update_rate = atof(token); break;
      }
      token = strtok(NULL, ",");
      field_count++;
    }

    if (field_count == 10) {
      printf("M808\n");
      return true;
    }
  }

  printf("M806\n");
  printf("Invalid config format.\n");
  return false;
}


static bool wait_for_m_code(const char* send, const char* receive, int timeout_s){
  char _m_code_uart[4];
  uint32_t _start_time = HAL_GetTick() + 1001;
  int count = 0;

  while (count < timeout_s)
  {
    while (!get_esp32_serial(_m_code_uart, 4, false))
    {
      HAL_Delay(50);
      if ((HAL_GetTick() - _start_time) > 1000)
      {
        _start_time = HAL_GetTick();
        count++;
        if (send != NULL && *send != '\0') {
          printf("%s\n", send);
        }
      }
    }

    if (strcmp((char*)_m_code_uart, receive) == 0)
    {
      return true;
    }
  }
  return false;
}


static void set_custom_config(config_settings_t *config, acc_detector_distance_config_t *detector_config){
  acc_detector_distance_config_start_set(detector_config, config->start_m);
  acc_detector_distance_config_end_set(detector_config, config->end_m);
  acc_detector_distance_config_max_step_length_set(detector_config, config->max_step_length);
  acc_detector_distance_config_max_profile_set(detector_config, config->max_profile);
  acc_detector_distance_config_reflector_shape_set(detector_config, config->reflector_shape);
  acc_detector_distance_config_threshold_sensitivity_set(detector_config, config->threshold_sensitivity);
  acc_detector_distance_config_signal_quality_set(detector_config, config->signal_quality);

  sleep_time_ms = (uint32_t)(1000.0f * HAL_GETTICK_SCALAR / config->update_rate);
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
  if (result->num_distances > 0)
  {
    printf("M115 %.3f m, %.2f\n", result->distances[0], result->strengths[0]);
  }
  else
  {
    printf("M317\n");
  }

//  printf("%d detected distances", result->num_distances);
//  if (result->num_distances > 0)
//  {
//    printf(": ");
//
//    for (uint8_t i = 0; i < result->num_distances; i++)
//    {
//      printf("%" PRIfloat " m ", ACC_LOG_FLOAT_TO_INTEGER(result->distances[i]));
//    }
//  }
//
//  printf("\n");
}
