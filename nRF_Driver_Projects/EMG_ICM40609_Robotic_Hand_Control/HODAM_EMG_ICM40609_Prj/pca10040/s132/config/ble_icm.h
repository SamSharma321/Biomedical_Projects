/* Copyright (c) 2016 Musa Mahmood
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

/** @file
 * @defgroup ble_sdk_srv_icm IMU Measurement Service
 * @{
 * @ingroup ble_sdk_srv
 * @brief IMU (Inertial Measurement Unit) Measurement Service module.
 * @details This module implements the IMU Measurement Service with Accelerometer and Gyroscope characteristics.
 *          During initialization it adds the IMU Measurement Service and Accel/Gyro characteristics
 *          to the BLE stack database. Optionally it can also add a Report Reference descriptor
 *          to the characteristics (used when including the IMU Service in the HID service).
 *          If specified, the module will support notification of the Accel and Gyro characteristics
 *          through the ble_icm_update_accel(), ble_icm_update_gyro(), and ble_icm_update_sample() functions.
 *          If an event handler is supplied by the application, the IMU Service will
 *          generate IMU Service events to the application.
 * @note The application must propagate BLE stack events to the IMU Service module by calling
 *       ble_icm_on_ble_evt() from the @ref ble_stack_handler callback.
 */

#ifndef BLE_ICM_H__
#define BLE_ICM_H__                      
#include "ble.h"
#include "ble_srv_common.h"
#include "sdk_config.h"
#include <stdint.h>
#include "usr_config.h"

/* ============================================================================
 * BLE UUID DEFINITIONS
 * ============================================================================ */

// Base UUID for custom ICM service
#define IMU_UUID_BASE                    { 0x57, 0x80, 0xD2, 0x94, 0xA3, 0xB2, 0xFE, 0x39, 0x5F, 0x87, 0xFD, 0x35, 0x00, 0x00, 0x8C, 0x22 }

// Service UUID
#define BLE_UUID_IMU_MEASUREMENT_SERVICE 0x1CF0
// Characteristic UUIDs for IMU data
#define BLE_UUID_ICM_CONFIG_CHAR         0x1CFF
#define BLE_UUID_ICM_ACCEL_CHAR          0x1CF1
#define BLE_UUID_ICM_GYRO_CHAR           0x1CF2
#define BLE_UUID_ICM_TEMP_CHAR           0x1CF3
#define BLE_UUID_ICM_SAMPLE_CHAR         0x1CF4
/* ============================================================================
 * PACKET CONFIGURATION
 * ============================================================================
 * 
 * ICM20649 sample @ 200 Hz (5 ms):
 *   - Accel: 6 bytes (X, Y, Z as int16_t)
 *   - Gyro:  6 bytes (X, Y, Z as int16_t)
 *   - Temp:  2 bytes (int16_t)
 *   - Total: 14 bytes per sample
 * 
 * For MTU 247 (effective data ~244 bytes):
 *   - Max samples per packet: 244 / 14 = 17 samples
 *   - Packet length: 17 * 14 = 238 bytes (safe margin)
 * 
 * For separate characteristics:
 *   - Accel only: 240 bytes = 40 samples
 *   - Gyro only: 240 bytes = 40 samples
 *   - Full sample: 238 bytes = 17 samples
 */
#define ICM_ACCEL_PACKET_LENGTH          100  /* 17 accel samples (6 bytes each) */
#define ICM_GYRO_PACKET_LENGTH           100  /* 17 gyro samples (6 bytes each) */
#define ICM_SAMPLE_PACKET_LENGTH         238  /* 17 full samples (14 bytes each) */
#define ICM_TEMP_PACKET_LENGTH           40   /* 20 temp samples (2 bytes each) */
/* Forward declaration of ble_icm_t type */
typedef struct ble_icm_s ble_icm_t;

/* Setup handler for writes (for future configuration via BLE) */
typedef void (*ble_icm_write_config_handler_t)(uint16_t conn_handle, ble_icm_t *p_icm, uint8_t *data);

/* Setup handler structure */
typedef struct {
  ble_icm_write_config_handler_t icm_config_handler;
} ble_icm_init_t;

/*! @brief IMU Measurement Service structure. This contains all options and data needed for
 *        initialization of the service. */
struct ble_icm_s {
  uint8_t dev_idx;                                      /**< Device index of the ICM20649 */
  uint8_t uuid_type;                                    /**< UUID type for custom base */
  uint16_t conn_handle;                                 /**< Connection handle */
  uint16_t service_handle;                              /**< Handle of BLE Service (as provided by the BLE stack). */
  
  /* Configuration characteristic handles */
  ble_gatts_char_handles_t icm_config_char_handles;     /**< Handles for config characteristic (read/write/notify) */
  
  /* Data characteristic handles */
  ble_gatts_char_handles_t icm_accel_char_handles;      /**< Handles for accelerometer characteristic */
  ble_gatts_char_handles_t icm_gyro_char_handles;       /**< Handles for gyroscope characteristic */
  ble_gatts_char_handles_t icm_temp_char_handles;       /**< Handles for temperature characteristic */
  ble_gatts_char_handles_t icm_sample_char_handles;     /**< Handles for full sample characteristic (accel+gyro+temp) */
  
  /* Event handler callback */
  ble_icm_write_config_handler_t icm_config_handler;
  
  /* Configuration storage */
  uint8_t icm_current_configuration[8];                 /**< Current ICM configuration (can store register values) */
  
  /* Data buffers for BLE notifications */
  uint8_t icm_accel_buffer[ICM_ACCEL_PACKET_LENGTH];   /**< Buffer for accelerometer data packets */
  uint8_t icm_gyro_buffer[ICM_GYRO_PACKET_LENGTH];     /**< Buffer for gyroscope data packets */
  uint8_t icm_temp_buffer[ICM_TEMP_PACKET_LENGTH];     /**< Buffer for temperature data packets */
  uint8_t icm_sample_buffer[ICM_SAMPLE_PACKET_LENGTH]; /**< Buffer for full sample data packets */
  
  /* Sample counters for packet assembly */
  uint16_t icm_accel_count;                             /**< Number of accel samples in current buffer */
  uint16_t icm_gyro_count;                              /**< Number of gyro samples in current buffer */
  uint16_t icm_temp_count;                              /**< Number of temp samples in current buffer */
  uint16_t icm_sample_count;                            /**< Number of full samples in current buffer */
};

/* External Variables*/
extern ble_icm_t m_icm[NUM_OF_ICM_DEVICES];

/* ============================================================================
 * BLE ICM SERVICE API
 * ============================================================================ */

/*! @brief Function for initializing the IMU Measurement Service.
 * @param[in]   p_icm       IMU Service structure.
 * @param[in]   p_init      Initialization parameters for the service.
 */
void ble_icm_service_init(ble_icm_t *p_icm, const ble_icm_init_t *p_init);

/*! @brief IMU Measurement Service BLE stack event handler.
 * @details Handles all events from the BLE stack of interest to the IMU Service.
 * @param[in]   p_icm      IMU Service structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */
void ble_icm_on_ble_evt(ble_icm_t *p_icm, ble_evt_t *p_ble_evt);

/*! @brief Function for updating IMU configuration characteristic and optionally notify.
 * @param[in]   p_icm       IMU Service structure.
 * @param[in]   notify      Whether to send BLE notification.
 */
void ble_icm_update_configuration(ble_icm_t *p_icm, bool notify);

/*! @brief Function for notifying accelerometer data.
 * @details Sends accel_count samples from the accel_buffer via BLE notification.
 *          Automatically resets the counter after notification.
 * @param[in]   p_icm       IMU Service structure.
 */
void ble_icm_update_accel(ble_icm_t *p_icm);

/*! @brief Function for notifying gyroscope data.
 * @details Sends gyro_count samples from the gyro_buffer via BLE notification.
 *          Automatically resets the counter after notification.
 * @param[in]   p_icm       IMU Service structure.
 */
void ble_icm_update_gyro(ble_icm_t *p_icm);

/*! @brief Function to update internal register configurations for the ICM20639 devices. */
void icm_config_handler(uint16_t conn_handle, ble_icm_t *p_icm, uint8_t *data);

/*! @brief Function for notifying temperature data.
 * @details Sends temp_count samples from the temp_buffer via BLE notification.
 *          Automatically resets the counter after notification.
 * @param[in]   p_icm       IMU Service structure.
 */
void ble_icm_update_temp(ble_icm_t *p_icm);

/*! @brief Function for notifying full IMU sample data (accel + gyro + temp).
 * @details Sends sample_count samples from the sample_buffer via BLE notification.
 *          Automatically resets the counter after notification.
 * @param[in]   p_icm       IMU Service structure.
 */
void ble_icm_update_sample(ble_icm_t *p_icm);

/*! @brief Function for notifying all available IMU data.
 * @details Sends all buffered data (accel, gyro, temp, and/or combined samples).
 *          Useful for batch updates.
 * @param[in]   p_icm       IMU Service structure.
 */
void ble_icm_update_all(ble_icm_t *p_icm);

#endif // BLE_ICM_H__
