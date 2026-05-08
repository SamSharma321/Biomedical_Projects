/* Edited By: Sameera Sharma for Dysphagia Project with BITN */
/** @file
 *
 * @defgroup ble_sdk_srv_eeg Biopotential Measurement Service
 * @{
 * @ingroup ble_sdk_srv
 * @brief Biopotential Measurement Service module.
 *
 * @details This module implements the Biopotential Measurement Service with the Voltage characteristic.
 *          During initialization it adds the Biopotential Measurement Service and Voltage characteristic
 *          to the BLE stack dataBISe. Optionally it can also add a Report Reference descriptor
 *          to the Battery Level characteristic (used when including the Biopotential Measurement Service in
 *          the HID service).
 *
 *          If specified, the module will support notification of the Battery Level characteristic
 *          through the ble_eeg_battery_level_update() function.
 *          If an event handler is supplied by the application, the Biopotential Measurement Service will
 *          generate Biopotential Measurement Service events to the application.
 *
 * @note The application must propagate BLE stack events to the Biopotential Measurement Service module by calling
 *       ble_eeg_on_ble_evt() from the from the @ref ble_stack_handler callback.
 */

#ifndef BLE_EEG_H__
#define BLE_EEG_H__

#include "ble.h"
#include <stdint.h>

// Base UUID
#define BMS_UUID_BASE {0x57, 0x80, 0xD2, 0x94, 0xA3, 0xB2, 0xFE, 0x39, 0x5F, 0x87, 0xFD, 0x35, 0x00, 0x00, 0x8B, 0x22}

// Service UUID
#define BLE_UUID_BIOPOTENTIAL_EEG_MEASUREMENT_SERVICE 0xEEF0

// Characteristic UUIDs
#define BLE_UUID_EEG_CH1_CHAR 0xEEF1
#define BLE_UUID_EEG_CH2_CHAR 0xEEF2

#define EEG_PACKET_LENGTH 120

/**@brief Biopotential Measurement Service init structure. This contains all options and data needed for
 *        initialization of the service. */
typedef struct
{
  uint8_t uuid_type;
  uint16_t conn_handle;    /**< Event handler to be called for handling events in the Biopotential Measurement Service. */
  uint16_t service_handle; /**< Handle of ble Service (as provided by the BLE stack). */
  ble_gatts_char_handles_t eeg_ch1_handles; /**< Handles related to the our body V measure characteristic. */
  ble_gatts_char_handles_t eeg_ch2_handles; /**< Handles related to the our body V measure characteristic. */
  uint8_t eeg_ch1_buffer[EEG_PACKET_LENGTH]; //246 or 4* = 
  uint8_t eeg_ch2_buffer[EEG_PACKET_LENGTH]; //246 or 4* = 
  uint16_t eeg_ch1_count;
} ble_eeg_t;

void ble_eeg_service_init(ble_eeg_t *p_eeg);

/**@brief Biopotential Measurement Service BLE stack event handler.
 *
 * @details Handles all events from the BLE stack of interest to the Biopotential Measurement Service.
 *
 * @param[in]   p_eeg      Biopotential Measurement Service structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */
void ble_eeg_on_ble_evt(ble_eeg_t *p_eeg, ble_evt_t *p_ble_evt);

/**@brief function for updating/notifying BLE of new value.
*
*/

void ble_eeg_update_1ch_v2(ble_eeg_t *p_eeg);
void ble_eeg_update_2ch(ble_eeg_t *p_eeg);




#endif // BLE_EEG_H__