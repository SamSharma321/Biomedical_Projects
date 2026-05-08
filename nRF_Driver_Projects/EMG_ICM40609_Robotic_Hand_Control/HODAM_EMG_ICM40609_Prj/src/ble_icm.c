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

#include "ble_icm.h"
#include "icm40609.h"
#include "app_error.h"
#include "app_util.h"
#include "ble_srv_common.h"
#include "nordic_common.h"
#include "nrf_log.h"
#include <string.h>

/* This file handles BLE communication for ICM40609 IMU sensor data.
 * The flow is as follows: ICM20649 → TWI → buffer → BLE notifications → Central.
 * It allows for optional configuration of ICM40609 registers via BLE.
 * Data format in buffers:
 *   - Accel/Gyro: Raw 16-bit signed integers (2 bytes each), big-endian
 *   - Temp: Raw 16-bit signed integer (2 bytes), big-endian
 *   - Full sample: Accel (6) + Gyro (6) + Temp (2) = 14 bytes per sample
 */

// Max payload size for a single ATT notification when using MTU 247 (effective data ~244 bytes)
#define MAX_LEN_BLE_PACKET_BYTES 246
/* ============================================================================
 * WRITE HANDLER
 * ============================================================================ */

/*! @brief Handle writes to the ICM configuration characteristic.
 * @param[in]   p_icm       ICM Service structure.
 * @param[in]   p_ble_evt   BLE event containing write data.
 * @return None.
 */
static void on_write(ble_icm_t *p_icm, ble_evt_t *p_ble_evt)
{
    ble_gatts_evt_write_t *p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    if ((p_evt_write->handle == p_icm->icm_config_char_handles.value_handle) &&
        (p_evt_write->len >= 1) && (p_icm->icm_config_handler != NULL))
    {
        // Pass the raw payload to the application-provided config handler.
        p_icm->icm_config_handler(p_ble_evt->evt.gap_evt.conn_handle, p_icm, &p_evt_write->data[0]);
    }
}

/* ============================================================================
 * BLE EVENT DISPATCHER
 * ============================================================================ */

/*! @brief BLE event dispatcher for this service (connect/disconnect and writes).
 * @param[in]   p_icm       ICM Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 * @return None.
 */
void ble_icm_on_ble_evt(ble_icm_t *p_icm, ble_evt_t *p_ble_evt)
{
    if ((p_icm == NULL) || (p_ble_evt == NULL))
    {
        return;
    }

    switch (p_ble_evt->header.evt_id)
    {
    case BLE_GAP_EVT_CONNECTED:
        p_icm->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
        NRF_LOG_INFO("ICM Service: BLE connected.");
        break;

    case BLE_GAP_EVT_DISCONNECTED:
        p_icm->conn_handle = BLE_CONN_HANDLE_INVALID;
        NRF_LOG_INFO("ICM Service: BLE disconnected.");
        break;

    case BLE_GATTS_EVT_WRITE:
        on_write(p_icm, p_ble_evt);
        break;

    default:
        break;
    }
}

/*! @brief Handle configuration writes from BLE central.
 * This callback is invoked when the central device writes to the ICM
 * configuration characteristic. Can be used to dynamically change sensor
 * settings.
 * @param[in]     conn_handle  Connection handle from BLE event.
 * @param[in,out] p_icm        ICM service instance.
 * @param[in]     data         Incoming configuration payload.
 * @return None.
 */
void icm_config_handler(uint16_t conn_handle, ble_icm_t *p_icm, uint8_t *data)
{
    if (p_icm == NULL || data == NULL)
    {
        return;
    }

    NRF_LOG_INFO("ICM Config command received: 0x%02X", data[0]);

    // Store configuration byte
    p_icm->icm_current_configuration[0] = data[0];

    // Example: Handle different commands
    switch (data[0])
    {
    case 0x01:
        NRF_LOG_INFO("ICM: Start streaming");
        break;
    case 0x02:
        NRF_LOG_INFO("ICM: Stop streaming");
        break;
    case 0x03:
        NRF_LOG_INFO("ICM: Reset device");
        break;
    default:
        NRF_LOG_INFO("ICM: Unknown command 0x%02X", data[0]);
        break;
    }
    // Notify central that config was received
    ble_icm_update_configuration(p_icm, true);
}

/* ============================================================================
 * CHARACTERISTIC ADDITION HELPERS
 * ============================================================================ */

/*! @brief Adds the ICM configuration characteristic (read/write/notify).
 * @param[in]   p_icm       ICM Service structure.
 * @param[in]   p_init      Initialization parameters.
 * @return      NRF_SUCCESS or error code.
 */
static uint32_t icm_config_char_add(ble_icm_t *p_icm, const ble_icm_init_t *p_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t attr_char_value;
    ble_uuid_t ble_uuid;
    ble_gatts_attr_md_t attr_md;
    ble_gatts_attr_md_t cccd_md;

    memset(&char_md, 0, sizeof(char_md));
    memset(&cccd_md, 0, sizeof(cccd_md));

    // Set up CCCD (Client Characteristic Configuration Descriptor) permissions
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc = BLE_GATTS_VLOC_STACK;

    // Set up characteristic properties
    char_md.char_props.read = 1;
    char_md.char_props.write = 1;
    char_md.char_props.notify = 1;
    char_md.p_cccd_md = &cccd_md;

    // Set up UUID
    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_ICM_CONFIG_CHAR);

    // Set up attribute metadata
    memset(&attr_md, 0, sizeof(attr_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    attr_md.vloc = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen = 1;

    // Set up attribute value
    memset(&attr_char_value, 0, sizeof(attr_char_value));
    attr_char_value.p_uuid = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len = sizeof(p_icm->icm_current_configuration);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len = sizeof(p_icm->icm_current_configuration);
    attr_char_value.p_value = p_icm->icm_current_configuration;

    return sd_ble_gatts_characteristic_add(p_icm->service_handle,
        &char_md,
        &attr_char_value,
        &p_icm->icm_config_char_handles);
}

/*! @brief Adds the accelerometer data characteristic (read/notify).
 * @param[in]   p_icm   ICM Service structure.
 * @return      NRF_SUCCESS or error code.
 */
static uint32_t icm_accel_char_add(ble_icm_t *p_icm)
{
    uint32_t err_code = 0;
    ble_uuid_t char_uuid;
    uint8_t encoded_initial_data[ICM_ACCEL_PACKET_LENGTH];

    memset(encoded_initial_data, 0, ICM_ACCEL_PACKET_LENGTH);
    BLE_UUID_BLE_ASSIGN(char_uuid, BLE_UUID_ICM_ACCEL_CHAR);

    ble_gatts_char_md_t char_md;
    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.read = 1;
    char_md.char_props.notify = 1;

    ble_gatts_attr_md_t cccd_md;
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc = BLE_GATTS_VLOC_STACK;
    char_md.p_cccd_md = &cccd_md;

    ble_gatts_attr_md_t attr_md;
    memset(&attr_md, 0, sizeof(attr_md));
    attr_md.vloc = BLE_GATTS_VLOC_STACK;
    attr_md.vlen = 1;
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

    ble_gatts_attr_t attr_char_value;
    memset(&attr_char_value, 0, sizeof(attr_char_value));
    attr_char_value.p_uuid = &char_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len = ICM_ACCEL_PACKET_LENGTH;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len = ICM_ACCEL_PACKET_LENGTH;
    attr_char_value.p_value = encoded_initial_data;

    err_code = sd_ble_gatts_characteristic_add(p_icm->service_handle,
        &char_md,
        &attr_char_value,
        &p_icm->icm_accel_char_handles);

    APP_ERROR_CHECK(err_code);
    return NRF_SUCCESS;
}

/*! @brief Adds the gyroscope data characteristic (read/notify).
 * @param[in]   p_icm   ICM Service structure.
 * @return      NRF_SUCCESS or error code.
 */
static uint32_t icm_gyro_char_add(ble_icm_t *p_icm)
{
    uint32_t err_code = 0;
    ble_uuid_t char_uuid;
    uint8_t encoded_initial_data[ICM_GYRO_PACKET_LENGTH];

    memset(encoded_initial_data, 0, ICM_GYRO_PACKET_LENGTH);
    BLE_UUID_BLE_ASSIGN(char_uuid, BLE_UUID_ICM_GYRO_CHAR);

    ble_gatts_char_md_t char_md;
    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.read = 1;
    char_md.char_props.notify = 1;

    ble_gatts_attr_md_t cccd_md;
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc = BLE_GATTS_VLOC_STACK;
    char_md.p_cccd_md = &cccd_md;

    ble_gatts_attr_md_t attr_md;
    memset(&attr_md, 0, sizeof(attr_md));
    attr_md.vloc = BLE_GATTS_VLOC_STACK;
    attr_md.vlen = 1;
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

    ble_gatts_attr_t attr_char_value;
    memset(&attr_char_value, 0, sizeof(attr_char_value));
    attr_char_value.p_uuid = &char_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len = ICM_GYRO_PACKET_LENGTH;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len = ICM_GYRO_PACKET_LENGTH;
    attr_char_value.p_value = encoded_initial_data;

    err_code = sd_ble_gatts_characteristic_add(p_icm->service_handle,
        &char_md,
        &attr_char_value,
        &p_icm->icm_gyro_char_handles);

    APP_ERROR_CHECK(err_code);
    return NRF_SUCCESS;
}

/*! @brief Adds the temperature data characteristic (read/notify).
 * @param[in]   p_icm   ICM Service structure.
 * @return      NRF_SUCCESS or error code.
 */
static uint32_t icm_temp_char_add(ble_icm_t *p_icm)
{
    uint32_t err_code = 0;
    ble_uuid_t char_uuid;
    uint8_t encoded_initial_data[ICM_TEMP_PACKET_LENGTH];

    memset(encoded_initial_data, 0, ICM_TEMP_PACKET_LENGTH);
    BLE_UUID_BLE_ASSIGN(char_uuid, BLE_UUID_ICM_TEMP_CHAR);

    ble_gatts_char_md_t char_md;
    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.read = 1;
    char_md.char_props.notify = 1;

    ble_gatts_attr_md_t cccd_md;
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc = BLE_GATTS_VLOC_STACK;
    char_md.p_cccd_md = &cccd_md;

    ble_gatts_attr_md_t attr_md;
    memset(&attr_md, 0, sizeof(attr_md));
    attr_md.vloc = BLE_GATTS_VLOC_STACK;
    attr_md.vlen = 1;
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

    ble_gatts_attr_t attr_char_value;
    memset(&attr_char_value, 0, sizeof(attr_char_value));
    attr_char_value.p_uuid = &char_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len = ICM_TEMP_PACKET_LENGTH;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len = ICM_TEMP_PACKET_LENGTH;
    attr_char_value.p_value = encoded_initial_data;

    err_code = sd_ble_gatts_characteristic_add(p_icm->service_handle,
        &char_md,
        &attr_char_value,
        &p_icm->icm_temp_char_handles);

    APP_ERROR_CHECK(err_code);
    return NRF_SUCCESS;
}

/*! @brief Adds the full IMU sample characteristic (accel + gyro + temp, read/notify).
 * @param[in]   p_icm   ICM Service structure.
 * @return      NRF_SUCCESS or error code.
 */
static uint32_t icm_sample_char_add(ble_icm_t *p_icm)
{
    uint32_t err_code = 0;
    ble_uuid_t char_uuid;
    uint8_t encoded_initial_data[ICM_SAMPLE_PACKET_LENGTH];

    memset(encoded_initial_data, 0, ICM_SAMPLE_PACKET_LENGTH);
    BLE_UUID_BLE_ASSIGN(char_uuid, BLE_UUID_ICM_SAMPLE_CHAR);

    ble_gatts_char_md_t char_md;
    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.read = 1;
    char_md.char_props.notify = 1;

    ble_gatts_attr_md_t cccd_md;
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc = BLE_GATTS_VLOC_STACK;
    char_md.p_cccd_md = &cccd_md;

    ble_gatts_attr_md_t attr_md;
    memset(&attr_md, 0, sizeof(attr_md));
    attr_md.vloc = BLE_GATTS_VLOC_STACK;
    attr_md.vlen = 1;
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

    ble_gatts_attr_t attr_char_value;
    memset(&attr_char_value, 0, sizeof(attr_char_value));
    attr_char_value.p_uuid = &char_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len = ICM_SAMPLE_PACKET_LENGTH;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len = ICM_SAMPLE_PACKET_LENGTH;
    attr_char_value.p_value = encoded_initial_data;

    err_code = sd_ble_gatts_characteristic_add(p_icm->service_handle,
        &char_md,
        &attr_char_value,
        &p_icm->icm_sample_char_handles);

    APP_ERROR_CHECK(err_code);
    return NRF_SUCCESS;
}

/* ============================================================================
 * SERVICE INITIALIZATION
 * ============================================================================ */

/*! @brief Function for initializing the IMU Measurement Service.
 * @param[in]   p_icm       IMU Service structure.
 * @param[in]   p_init      Initialization parameters.
 * @return None.
 */
void ble_icm_service_init(ble_icm_t *p_icm, const ble_icm_init_t *p_init)
{
    uint32_t err_code;
    uint16_t service_handle;
    ble_uuid_t service_uuid;
    ble_uuid128_t base_uuid = {IMU_UUID_BASE};

    if ((p_icm == NULL) || (p_init == NULL))
    {
        NRF_LOG_ERROR("ICM Service: NULL pointer passed to initialization.");
        return;
    }

    // Initialize service structure and register custom UUID base
    p_icm->conn_handle = BLE_CONN_HANDLE_INVALID;
    p_icm->icm_config_handler = p_init->icm_config_handler;

    err_code = sd_ble_uuid_vs_add(&base_uuid, &(p_icm->uuid_type));
    APP_ERROR_CHECK(err_code);

    service_uuid.type = p_icm->uuid_type;
    service_uuid.uuid = BLE_UUID_IMU_MEASUREMENT_SERVICE;

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &service_uuid, &service_handle);
    APP_ERROR_CHECK(err_code);

    p_icm->service_handle = service_handle;

    // Add characteristics
    err_code = icm_config_char_add(p_icm, p_init);
    APP_ERROR_CHECK(err_code);

    icm_accel_char_add(p_icm);
    icm_gyro_char_add(p_icm);
    icm_temp_char_add(p_icm);
    icm_sample_char_add(p_icm);

    NRF_LOG_INFO("ICM Service initialized successfully.");
}

/* ============================================================================
 * UPDATE/NOTIFICATION FUNCTIONS
 * ============================================================================ */

/*! @brief Function for updating the ICM configuration characteristic.
 * @param[in]   p_icm       ICM Service structure.
 * @param[in]   notify      Whether to send BLE notification.
 * @return None.
 */
void ble_icm_update_configuration(ble_icm_t *p_icm, bool notify)
{
    uint32_t err_code;

    if ((p_icm == NULL) || (p_icm->conn_handle == BLE_CONN_HANDLE_INVALID))
    {
        return;
    }

    ble_gatts_value_t value;
    value.len = sizeof(p_icm->icm_current_configuration);
    value.offset = 0;
    value.p_value = p_icm->icm_current_configuration;

    err_code = sd_ble_gatts_value_set(p_icm->conn_handle, 
                                      p_icm->icm_config_char_handles.value_handle, 
                                      &value);

    if (notify && err_code == NRF_SUCCESS)
    {
        uint16_t hvx_len = sizeof(p_icm->icm_current_configuration);
        ble_gatts_hvx_params_t const hvx_params = {
            .handle = p_icm->icm_config_char_handles.value_handle,
            .type = BLE_GATT_HVX_NOTIFICATION,
            .offset = 0,
            .p_len = &hvx_len,
            .p_data = p_icm->icm_current_configuration,
        };
        err_code = sd_ble_gatts_hvx(p_icm->conn_handle, &hvx_params);

        if (err_code == NRF_ERROR_RESOURCES)
        {
            NRF_LOG_DEBUG("ICM: Config notification queued (resources busy).");
        }
    }
}

/*! @brief Function for notifying accelerometer data.
 * @param[in]   p_icm   ICM Service structure.
 * @return None.
 */
void ble_icm_update_accel(ble_icm_t *p_icm)
{
    uint32_t err_code;

    if ((p_icm == NULL) || (p_icm->conn_handle == BLE_CONN_HANDLE_INVALID) || (p_icm->icm_accel_count == 0))
    {
        return;
    }

    uint16_t hvx_len = p_icm->icm_accel_count * 6;  /* 6 bytes per accel sample (X, Y, Z) */
    ble_gatts_hvx_params_t const hvx_params = {
        .handle = p_icm->icm_accel_char_handles.value_handle,
        .type = BLE_GATT_HVX_NOTIFICATION,
        .offset = 0,
        .p_len = &hvx_len,
        .p_data = p_icm->icm_accel_buffer,
    };

    err_code = sd_ble_gatts_hvx(p_icm->conn_handle, &hvx_params);

    if (err_code == NRF_SUCCESS)
    {
        p_icm->icm_accel_count = 0;  /* Reset counter after successful notification */
        NRF_LOG_DEBUG("ICM: Accel data notified (%d samples).", p_icm->icm_accel_count);
    }
    else if (err_code == NRF_ERROR_RESOURCES)
    {
        NRF_LOG_DEBUG("ICM: Accel notification queued (resources busy).");
    }
}

/*! @brief Function for notifying gyroscope data.
 * @param[in]   p_icm   ICM Service structure.
 * @return None.
 */
void ble_icm_update_gyro(ble_icm_t *p_icm)
{
    uint32_t err_code;

    if ((p_icm == NULL) || (p_icm->conn_handle == BLE_CONN_HANDLE_INVALID) || (p_icm->icm_gyro_count == 0))
    {
        return;
    }

    uint16_t hvx_len = p_icm->icm_gyro_count * 6;  /* 6 bytes per gyro sample (X, Y, Z) */
    ble_gatts_hvx_params_t const hvx_params = {
        .handle = p_icm->icm_gyro_char_handles.value_handle,
        .type = BLE_GATT_HVX_NOTIFICATION,
        .offset = 0,
        .p_len = &hvx_len,
        .p_data = p_icm->icm_gyro_buffer,
    };

    err_code = sd_ble_gatts_hvx(p_icm->conn_handle, &hvx_params);

    if (err_code == NRF_SUCCESS)
    {
        p_icm->icm_gyro_count = 0;  /* Reset counter after successful notification */
        NRF_LOG_DEBUG("ICM: Gyro data notified.");
    }
    else if (err_code == NRF_ERROR_RESOURCES)
    {
        NRF_LOG_DEBUG("ICM: Gyro notification queued (resources busy).");
    }
}

/*! @brief Function for notifying temperature data.
 * @param[in]   p_icm   ICM Service structure.
 * @return None.
 */
void ble_icm_update_temp(ble_icm_t *p_icm)
{
    uint32_t err_code;

    if ((p_icm == NULL) || (p_icm->conn_handle == BLE_CONN_HANDLE_INVALID) || (p_icm->icm_temp_count == 0))
    {
        return;
    }

    uint16_t hvx_len = p_icm->icm_temp_count * 2;  /* 2 bytes per temp sample */
    ble_gatts_hvx_params_t const hvx_params = {
        .handle = p_icm->icm_temp_char_handles.value_handle,
        .type = BLE_GATT_HVX_NOTIFICATION,
        .offset = 0,
        .p_len = &hvx_len,
        .p_data = p_icm->icm_temp_buffer,
    };

    err_code = sd_ble_gatts_hvx(p_icm->conn_handle, &hvx_params);

    if (err_code == NRF_SUCCESS)
    {
        p_icm->icm_temp_count = 0;  /* Reset counter after successful notification */
        NRF_LOG_DEBUG("ICM: Temp data notified.");
    }
    else if (err_code == NRF_ERROR_RESOURCES)
    {
        NRF_LOG_DEBUG("ICM: Temp notification queued (resources busy).");
    }
}

/*! @brief Function for notifying full IMU sample data.
 * @param[in]   p_icm   ICM Service structure.
 * @return None.
 */
void ble_icm_update_sample(ble_icm_t *p_icm)
{
    uint32_t err_code;

    if ((p_icm == NULL) || (p_icm->conn_handle == BLE_CONN_HANDLE_INVALID) || (p_icm->icm_sample_count == 0))
    {
        return;
    }

    uint16_t hvx_len = p_icm->icm_sample_count * 14;  /* 14 bytes per full sample (accel + gyro + temp) */
    ble_gatts_hvx_params_t const hvx_params = {
        .handle = p_icm->icm_sample_char_handles.value_handle,
        .type = BLE_GATT_HVX_NOTIFICATION,
        .offset = 0,
        .p_len = &hvx_len,
        .p_data = p_icm->icm_sample_buffer,
    };

    err_code = sd_ble_gatts_hvx(p_icm->conn_handle, &hvx_params);

    if (err_code == NRF_SUCCESS)
    {
        p_icm->icm_sample_count = 0;  /* Reset counter after successful notification */
        NRF_LOG_DEBUG("ICM: Full sample data notified.");
    }
    else if (err_code == NRF_ERROR_RESOURCES)
    {
        NRF_LOG_DEBUG("ICM: Sample notification queued (resources busy).");
    }
}

/*! @brief Function for notifying all available IMU data.
 * @param[in]   p_icm   ICM Service structure.
 * @return None.
 */
void ble_icm_update_all(ble_icm_t *p_icm)
{
    if (p_icm == NULL)
    {
        return;
    }

    /* Send all buffered data */
    ble_icm_update_accel(p_icm);
    ble_icm_update_gyro(p_icm);
    ble_icm_update_temp(p_icm);
    ble_icm_update_sample(p_icm);
}

/* End of ble_icm.c */
