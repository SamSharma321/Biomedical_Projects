/* Copyright (c) 2016 Musa Mahmood
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

/** @file
 *
 * @defgroup ble_sdk_srv_icm IMU Measurement Service
 * @{
 * @ingroup ble_sdk_srv
 * @brief IMU (Inertial Measurement Unit) Measurement Service module.
 *
 * @details This module implements the IMU Measurement Service with a combined sample characteristic.
 *          During initialization it adds the IMU Measurement Service and sample characteristic
 *          to the BLE stack database.
 *
 *          If specified, the module supports notification of combined samples
 *          through the ble_icm_update_sample() function.
 *          If an event handler is supplied by the application, the IMU Service will
 *          generate IMU Service events to the application.
 *
 * @note The application must propagate BLE stack events to the IMU Service module by calling
 *       ble_icm_on_ble_evt() from the @ref ble_stack_handler callback.
 */

#ifndef BLE_ICM_H__
#define BLE_ICM_H__

#include "ble.h"
#include "ble_srv_common.h"
#include "sdk_config.h"
#include "icm20948.h"
#include <stdint.h>

/* ============================================================================
 * BLE UUID DEFINITIONS
 * ============================================================================ */

// Base UUID for custom ICM service
#define IMU_UUID_BASE \
  { 0x57, 0x80, 0xD2, 0x94, 0xA3, 0xB2, 0xFE, 0x39, 0x5F, 0x87, 0xFD, 0x35, 0x00, 0x00, 0x8C, 0x22 }

// Service UUID
#define BLE_UUID_IMU_MEASUREMENT_SERVICE 0x1CF0

// Characteristic UUIDs for IMU data
#define BLE_UUID_ICM_CONFIG_CHAR 0x1CFF
#define BLE_UUID_ICM_SAMPLE_CHAR 0x1CF4

/* ============================================================================
 * PACKET CONFIGURATION
 * ============================================================================
 * 
 * ICM20948 sample @ 200 Hz (5 ms):
 *   - Accel: 6 bytes (X, Y, Z as int16_t)
 *   - Gyro:  6 bytes (X, Y, Z as int16_t)
 *   - Temp:  2 bytes (int16_t)
 *   - Total: 14 bytes per sample
 * 
 * For MTU 247 (effective data ~244 bytes):
 *   - Max samples per packet: 244 / 14 = 17 samples
 *   - Packet length: 17 * 14 = 238 bytes (safe margin)
 * 
 *   - Full sample: 238 bytes = 17 samples
 */

#define ICM_NUM_OF_SAMPLES 20
#define ICM_BYTES_PER_SAMPLE 12
#define ICM_SAMPLE_PACKET_LENGTH (ICM_NUM_OF_SAMPLES*ICM_BYTES_PER_SAMPLE)  /* 20 full samples (12 bytes each - ignoring temperature sensor data) */
/* Forward declaration of ble_icm_t type */
typedef struct ble_icm_s ble_icm_t;

/* Setup handler for writes (for future configuration via BLE) */
typedef void (*ble_icm_write_config_handler_t)(uint16_t conn_handle, ble_icm_t *p_icm, uint8_t *data);

/**@brief IMU Measurement Service structure. This contains all options and data needed for
 *        initialization of the service. */
struct ble_icm_s {
  uint8_t dev_idx;                                      /**< Device index of the ICM20948 */
  uint8_t uuid_type;                                    /**< UUID type for custom base */ // TODO can be removed from here?
  uint16_t conn_handle;                                 /**< Connection handle */
  uint16_t service_handle;                              /**< Handle of BLE Service (as provided by the BLE stack). */
  
  /* Configuration characteristic handles */
  ble_gatts_char_handles_t icm_config_char_handles;     /**< Handles for config characteristic (read/write/notify) */
  
  /* Data characteristic handles */
  ble_gatts_char_handles_t icm_sample_char_handles;     /**< Handles for full sample characteristic (accel+gyro) */
  
  /* Event handler callback */
  ble_icm_write_config_handler_t icm_config_handler;
  
  /* Configuration storage */ // TODO, how to update this by the user?
  uint8_t icm_current_configuration[8];                 /**< Current ICM configuration (can store register values) */
  
  /* Data buffers for BLE notifications */
  uint8_t icm_sample_buffer[ICM_SAMPLE_PACKET_LENGTH];     /**< Packed raw sample bytes for BLE transport. */
  icm20948_sample_t icm_sample_parsed[ICM_NUM_OF_SAMPLES]; /**< Parsed 16-bit samples aligned with packed buffer. */
  
  /* Sample counter for packet assembly */
  uint16_t icm_sample_count;                            /**< Number of full samples in current buffer */
};

/* External Variables*/
extern ble_icm_t m_icm[NUM_OF_ICM_DEVICES];

/* ============================================================================
 * BLE ICM SERVICE API
 * ============================================================================ */

/**@brief Function for initializing the IMU Measurement Service.
 *
 * @param[in]   p_icm       IMU Service structure.
 */
void ble_icm_service_init(ble_icm_t *p_icm);

/**@brief IMU Measurement Service BLE stack event handler.
 *
 * @details Handles all events from the BLE stack of interest to the IMU Service.
 *
 * @param[in]   p_icm      IMU Service structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */
void ble_icm_on_ble_evt(ble_icm_t *p_icm, ble_evt_t *p_ble_evt);

/**@brief Function for updating IMU configuration characteristic and optionally notify.
 *
 * @param[in]   p_icm       IMU Service structure.
 * @param[in]   notify      Whether to send BLE notification.
 */
void ble_icm_update_configuration(ble_icm_t *p_icm, bool notify);

/**@brief Function for notifying full IMU sample data (accel + gyro).
 *
 * @details Sends sample_count samples from the sample_buffer via BLE notification.
 *          Automatically resets the counter after notification.
 *
 * @param[in]   p_icm       IMU Service structure.
 */
void ble_icm_update_sample(ble_icm_t *p_icm);

#endif // BLE_ICM_H__
