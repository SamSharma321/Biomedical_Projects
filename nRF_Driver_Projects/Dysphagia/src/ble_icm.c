/* Created By: Sameera Sharma for BITN */
#include "ble_icm.h"
#include "app_error.h"
#include "nrf_log.h"
#include <string.h>

/* This file handles BLE communication for ICM20649 IMU sample data.
 * The flow is: ICM20649 -> raw sample buffer + parsed sample cache -> BLE notifications.
 */

/* ============================================================================
 * WRITE HANDLER
 * ============================================================================ */
/**@brief Handle writes to the ICM configuration characteristic.
 *
 * @param[in]   p_icm       ICM Service structure.
 * @param[in]   p_ble_evt   BLE event containing write data.
 */
static void on_write(ble_icm_t *p_icm, ble_evt_t *p_ble_evt)
{
    // Get the payload written by the central device to be configured into the device
    ble_gatts_evt_write_t *p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    // check if the write is to the configuration characteristic of the device
    if ((p_evt_write->handle == p_icm->icm_config_char_handles.value_handle) &&
        (p_evt_write->len >= 1) && (p_icm->icm_config_handler != NULL))
    {
        // Pass the raw payload to the application-provided config handler.
        p_icm->icm_config_handler(p_ble_evt->evt.gatts_evt.conn_handle, p_icm, &p_evt_write->data[0]);
    }
}

/* ============================================================================
 * BLE EVENT DISPATCHER
 * ============================================================================ */

/**@brief BLE event dispatcher for this service (connect/disconnect and writes).
 *
 * @param[in]   p_icm       ICM Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
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
        // Once connected, store the connection handle.
        p_icm->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
        NRF_LOG_INFO("ICM Service: BLE connected.");
        break;

    case BLE_GAP_EVT_DISCONNECTED:
        // On disconnection, reset the connection handle.
        p_icm->conn_handle = BLE_CONN_HANDLE_INVALID;
        NRF_LOG_INFO("ICM Service: BLE disconnected.");
        break;

    case BLE_GATTS_EVT_WRITE:
    // When a configuration update has been issued by the central device to the config characteristic.
        on_write(p_icm, p_ble_evt);
        break;

    default:
        break;
    }
}

/* ============================================================================
 * CHARACTERISTIC ADDITION HELPERS
 * ============================================================================ */

/**@brief Adds the ICM configuration characteristic (read/write/notify).
 *
 * @param[in]   p_icm       ICM Service structure.
 *
 * @return      NRF_SUCCESS or error code.
 */
static uint32_t icm_config_char_add(ble_icm_t *p_icm)
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

/**@brief Adds the full IMU sample characteristic (accel + gyro, read/notify).
 *
 * @param[in]   p_icm   ICM Service structure.
 *
 * @return      NRF_SUCCESS or error code.
 */
static uint32_t icm_sample_char_add(ble_icm_t *p_icm)
{
    uint32_t err_code = 0;
    ble_uuid_t char_uuid;
    uint8_t encoded_initial_data[ICM_SAMPLE_PACKET_LENGTH];

    memset(encoded_initial_data, 0, ICM_SAMPLE_PACKET_LENGTH);
    BLE_UUID_BLE_ASSIGN(char_uuid, BLE_UUID_ICM_SAMPLE_CHAR + p_icm->dev_idx);

    ble_gatts_char_md_t char_md;
    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.read = 1;
    char_md.char_props.notify = 1;

    ble_gatts_attr_md_t cccd_md;
    memset(&cccd_md, 0, sizeof(cccd_md));
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
/**@brief Function for initializing the IMU Measurement Service.
 *
 * @param[in]   p_icm       IMU Service structure.
 */
void ble_icm_service_init(ble_icm_t *p_icm)
{
    uint32_t err_code;
    uint16_t service_handle;
    ble_uuid_t service_uuid;
    ble_uuid128_t base_uuid = {IMU_UUID_BASE};
    static bool s_icm_uuid_registered = false;
    static uint8_t s_icm_uuid_type = BLE_UUID_TYPE_UNKNOWN;

    if (p_icm == NULL)
    {
        NRF_LOG_ERROR("ICM Service: NULL pointer passed to initialization.");
        return;
    }

    // Initialize service structure and register custom UUID base
    p_icm->conn_handle = BLE_CONN_HANDLE_INVALID;
    p_icm->icm_config_handler = NULL;
    p_icm->icm_sample_count = 0;
    p_icm->icm_ram_head = 0;
    p_icm->icm_ram_tail = 0;
    p_icm->icm_ram_count = 0;
    p_icm->icm_ram_dropped_samples = 0;
    memset(p_icm->icm_current_configuration, 0, sizeof(p_icm->icm_current_configuration));
    memset(p_icm->icm_sample_buffer, 0, sizeof(p_icm->icm_sample_buffer));
    memset(p_icm->icm_sample_parsed, 0, sizeof(p_icm->icm_sample_parsed));
    memset(p_icm->icm_ram_buffer, 0, sizeof(p_icm->icm_ram_buffer));

    if (!s_icm_uuid_registered)
    {
        err_code = sd_ble_uuid_vs_add(&base_uuid, &s_icm_uuid_type);
        APP_ERROR_CHECK(err_code);
        s_icm_uuid_registered = true;
    }
    p_icm->uuid_type = s_icm_uuid_type;

    service_uuid.type = p_icm->uuid_type;
    service_uuid.uuid = BLE_UUID_IMU_MEASUREMENT_SERVICE;

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &service_uuid, &service_handle);
    APP_ERROR_CHECK(err_code);

    p_icm->service_handle = service_handle;

    // Add characteristics
    err_code = icm_config_char_add(p_icm);
    APP_ERROR_CHECK(err_code);

    err_code = icm_sample_char_add(p_icm); // Required.
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("ICM Service initialized successfully.");
}

/* ============================================================================
 * UPDATE/NOTIFICATION FUNCTIONS
 * ============================================================================ */

/**@brief Function for updating the ICM configuration characteristic.
 *
 * @param[in]   p_icm       ICM Service structure.
 * @param[in]   notify      Whether to send BLE notification.
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

/**@brief Function for notifying full IMU sample data.
 *
 * @param[in]   p_icm   ICM Service structure.
 */
void ble_icm_update_sample(ble_icm_t *p_icm)
{
    uint32_t err_code;

    if ((p_icm == NULL) || (p_icm->conn_handle == BLE_CONN_HANDLE_INVALID) || (p_icm->icm_sample_count == 0))
    {
        return;
    }

    uint16_t hvx_len = p_icm->icm_sample_count * ICM_BYTES_PER_SAMPLE;  /* 12 bytes per full sample (accel + gyro) */
    // NOTE: Make the above number as 14 bytes if temp is included.
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
        // NRF_LOG_INFO("ICM: Full sample data notified.");
    }
    else if (err_code == NRF_ERROR_RESOURCES)
    {
        NRF_LOG_INFO("ICM: Sample notification queued (resources busy).");
    }
}

/* End of ble_icm.c */
