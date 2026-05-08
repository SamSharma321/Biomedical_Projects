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

#include "ble_eeg.h"
#include "ads1299-x.h"
#include "app_error.h"
#include "ble_usr_defined.h"
#include "nordic_common.h"
#include "nrf_log.h"
#include <string.h>

/* This file handles the writes and reads requested by the central BLE.
    The flow is as follows: Central → BLE → your handler → SPI → ADS1299,
    and samples go ADS1299 → SPI → buffer → BLE notifications → Central.
    It allows for central BLE user to configure ADS1299 registers.
*/

// Max payload size for a single ATT notification when using MTU 247 (effective data 244-246 bytes).
#define MAX_LEN_BLE_PACKET_BYTES 246 // 20*3bytes

/* Static Function Prototypes */
static uint32_t eeg_ch_char_add(ble_eeg_t *p_eeg, uint8_t ch_idx, uint16_t char_uuid_val);

/* Function wrappers for each channel */
static uint32_t eeg_ch1_char_add(ble_eeg_t *p_eeg) { return eeg_ch_char_add(p_eeg, 0u, BLE_UUID_EEG_CH1_CHAR); }
static uint32_t eeg_ch2_char_add(ble_eeg_t *p_eeg) { return eeg_ch_char_add(p_eeg, 1u, BLE_UUID_EEG_CH2_CHAR); }
static uint32_t eeg_ch3_char_add(ble_eeg_t *p_eeg) { return eeg_ch_char_add(p_eeg, 2u, BLE_UUID_EEG_CH3_CHAR); }
static uint32_t eeg_ch4_char_add(ble_eeg_t *p_eeg) { return eeg_ch_char_add(p_eeg, 3u, BLE_UUID_EEG_CH4_CHAR); }
static uint32_t eeg_ch5_char_add(ble_eeg_t *p_eeg) { return eeg_ch_char_add(p_eeg, 4u, BLE_UUID_EEG_CH5_CHAR); }
static uint32_t eeg_ch6_char_add(ble_eeg_t *p_eeg) { return eeg_ch_char_add(p_eeg, 5u, BLE_UUID_EEG_CH6_CHAR); }
static uint32_t eeg_ch7_char_add(ble_eeg_t *p_eeg) { return eeg_ch_char_add(p_eeg, 6u, BLE_UUID_EEG_CH7_CHAR); }
static uint32_t eeg_ch8_char_add(ble_eeg_t *p_eeg) { return eeg_ch_char_add(p_eeg, 7u, BLE_UUID_EEG_CH8_CHAR); }

/* typedef function pointer for structure to contain function calls for each channel */
typedef uint32_t (*eeg_char_add_fn_t)(ble_eeg_t *p_eeg);
static eeg_char_add_fn_t const m_eeg_char_add_fns[MAX_NUM_OF_ADS_CH_PER_DEV] = {
    eeg_ch1_char_add,
    eeg_ch2_char_add,
    eeg_ch3_char_add,
    eeg_ch4_char_add,
    eeg_ch5_char_add,
    eeg_ch6_char_add,
    eeg_ch7_char_add,
    eeg_ch8_char_add,
};
    /**< Maximum size in bytes of a transmitted Body Voltage Measurement. */ // 20*3bytes																						 /**< Maximum size in bytes of a transmitted Body Voltage Measurement. */

static uint8_t eeg_enabled_channel_count_get(ble_eeg_t *p_eeg)
{
    if (p_eeg == NULL)
    {
        return 0u;
    }

    uint8_t num_of_enabled_ch = p_eeg->num_of_enabled_ch;
    if (num_of_enabled_ch == 0u)
    {
        uint8_t dev0_enabled = (NUM_OF_ADS_CH > MAX_NUM_OF_ADS_CH_PER_DEV) ? MAX_NUM_OF_ADS_CH_PER_DEV : NUM_OF_ADS_CH;
#if (NUM_OF_ADS1299 == 2)
        if (p_eeg->dev_idx == 0u)
        {
            num_of_enabled_ch = dev0_enabled;
        }
        else
        {
            num_of_enabled_ch = (NUM_OF_ADS_CH > dev0_enabled) ? (uint8_t)(NUM_OF_ADS_CH - dev0_enabled) : 0u;
        }
#else
        num_of_enabled_ch = dev0_enabled;
#endif
    }

    if (num_of_enabled_ch > MAX_NUM_OF_ADS_CH_PER_DEV)
    {
        num_of_enabled_ch = MAX_NUM_OF_ADS_CH_PER_DEV;
    }
    return num_of_enabled_ch;
}

static bool eeg_notify_is_enabled(uint16_t conn_handle, ble_gatts_char_handles_t const * p_handles)
{
    uint8_t cccd_value_buf[2] = {0};
    ble_gatts_value_t cccd_value;
    ret_code_t err_code;

    if ((p_handles == NULL) ||
        (conn_handle == BLE_CONN_HANDLE_INVALID) ||
        (p_handles->cccd_handle == BLE_GATT_HANDLE_INVALID))
    {
        return false;
    }

    memset(&cccd_value, 0, sizeof(cccd_value));
    cccd_value.len = sizeof(cccd_value_buf);
    cccd_value.p_value = cccd_value_buf;
    err_code = sd_ble_gatts_value_get(conn_handle, p_handles->cccd_handle, &cccd_value);
    if (err_code != NRF_SUCCESS)
    {
        return false;
    }

    return ble_srv_is_notification_enabled(cccd_value_buf);
}

/*! @brief Handle writes to the ADS1299 configuration characteristic.
 * @param[in,out] p_eeg      EEG service instance.
 * @param[in]     p_ble_evt  BLE write event.
 * @return None.
 */
static void on_write(ble_eeg_t *p_eeg, ble_evt_t *p_ble_evt)
{
    ble_gatts_evt_write_t *p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    if ((p_evt_write->handle == p_eeg->ads1299_config_char_handles.value_handle) &&
        (p_eeg->eeg_config_handler != NULL))
    {
        if (p_evt_write->len != ADS1299_WRITABLE_REG_COUNT)
        {
            NRF_LOG_WARNING("Ignoring ADS1299 config write: len=%u expected=%u",
                            p_evt_write->len,
                            ADS1299_WRITABLE_REG_COUNT);
            return;
        }

        // Pass validated payload to the application-provided config handler.
        p_eeg->eeg_config_handler(p_ble_evt->evt.gatts_evt.conn_handle,
                                  p_eeg,
                                  &p_evt_write->data[0],
                                  p_evt_write->len);
    }
}

/*! @brief Dispatch BLE events for the EEG service.
 * @param[in,out] p_eeg      EEG service instance.
 * @param[in]     p_ble_evt  BLE event from the SoftDevice.
 * @return None.
 */
void ble_eeg_on_ble_evt(ble_eeg_t *p_eeg, ble_evt_t *p_ble_evt)
{
    switch (p_ble_evt->header.evt_id)
    {
    case BLE_GAP_EVT_CONNECTED:
        p_eeg->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
        break;

    case BLE_GAP_EVT_DISCONNECTED:
        p_eeg->conn_handle = BLE_CONN_HANDLE_INVALID;
        break;

    case BLE_GATTS_EVT_WRITE:
        on_write(p_eeg, p_ble_evt);
        break;
    default:
        break;
    }
}

#if defined(ADS1299)
/*! @brief Function for initializing services that will be used by the
          application received from BLE central device.
 * @param[in]     conn_handle  Connection handle from BLE event.
 * @param[in,out] p_eeg        EEG service instance.
 * @param[in]     data         Incoming ADS register payload.
 * @return None.
 */
void eeg_config_handler(uint16_t conn_handle, ble_eeg_t *p_eeg, uint8_t *data, uint16_t data_len)
{
    UNUSED_PARAMETER(conn_handle);
    if ((p_eeg == NULL) || (data == NULL) || (data_len != ADS1299_WRITABLE_REG_COUNT))
    {
        NRF_LOG_WARNING("Invalid ADS1299 config payload: ptr=%p len=%u", data, data_len);
        return;
    }
    NRF_LOG_INFO("REGISTER DATA RECEIVED: \n");
    NRF_LOG_HEXDUMP_DEBUG(data, ADS1299_WRITABLE_REG_COUNT);
    ads1299_stop_rdatac(p_eeg);
    ads1299_init_regs(p_eeg, data);
    ads1299_read_all_registers(p_eeg);
    ble_eeg_update_configuration(p_eeg, true);
    /* Start measurement on all ADS device */
    ads1299_start_conv();
    ads1299_start_rdatac(p_eeg);
}
#endif

/*! @brief Add ADS1299 configuration characteristic.
 * @param[in,out] p_eeg       EEG service instance.
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t eeg_ads1299_config_char_add(ble_eeg_t *p_eeg)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t attr_char_value;
    ble_uuid_t ble_uuid;
    ble_gatts_attr_md_t attr_md;
    ble_gatts_attr_md_t cccd_md;
    memset(&char_md, 0, sizeof(char_md));
    memset(&cccd_md, 0, sizeof(cccd_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc = BLE_GATTS_VLOC_STACK;
    char_md.char_props.read = 1;
    char_md.char_props.write = 1;
    char_md.char_props.notify = 1;
    //  char_md.p_char_user_desc = NULL;
    //  char_md.p_char_pf = NULL;
    //  char_md.p_user_desc_md = NULL;
    // NOTE: CCCD - Client Characteristic Configuration Descriptor
    char_md.p_cccd_md = &cccd_md;
    //  char_md.p_sccd_md = NULL;
    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_EEG_CONFIG);
    //  ble_uuid.type = p_eeg->uuid_type;
    //  ble_uuid.uuid = BLE_UUID_EEG_CONFIG;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    attr_md.vloc = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len = 23;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len = 23;
    // Back the characteristic with the current ADS1299 config array.
    attr_char_value.p_value = p_eeg->ads1299_current_configuration;

    return sd_ble_gatts_characteristic_add(p_eeg->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_eeg->ads1299_config_char_handles);
}

static uint32_t eeg_ch_char_add(ble_eeg_t *p_eeg, uint8_t ch_idx, uint16_t char_uuid_val)
{
    uint8_t encoded_initial_eeg[EEG_PACKET_LENGTH];
    memset(encoded_initial_eeg, 0, sizeof(encoded_initial_eeg));


    ble_uuid_t char_uuid;
    if (p_eeg->dev_idx == 1) {
        char_uuid_val |= (1 << 8);  // Set bit 8 for device 1
    }

    BLE_UUID_BLE_ASSIGN(char_uuid, char_uuid_val);

    ble_gatts_char_md_t char_md;
    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.read = 1;
    char_md.char_props.write = 0;
    char_md.char_props.notify = 1;

    ble_gatts_attr_md_t cccd_md;
    memset(&cccd_md, 0, sizeof(cccd_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc = BLE_GATTS_VLOC_STACK;
    char_md.p_cccd_md = &cccd_md;

    ble_gatts_attr_md_t attr_md;
    memset(&attr_md, 0, sizeof(attr_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    attr_md.vloc = BLE_GATTS_VLOC_STACK;
    attr_md.vlen = 1;

    ble_gatts_attr_t attr_char_value;
    memset(&attr_char_value, 0, sizeof(attr_char_value));
    attr_char_value.p_uuid = &char_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len = EEG_PACKET_LENGTH;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len = EEG_PACKET_LENGTH;
    attr_char_value.p_value = encoded_initial_eeg;

    return sd_ble_gatts_characteristic_add(p_eeg->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_eeg->eeg_handles[ch_idx]);
}

/*! @brief Initialize EEG BLE service and characteristics.
 * @param[in,out] p_eeg       EEG service instance.
 * @param[in]     p_eeg_init  EEG initialization parameters.
 * @return None.
 */
void ble_eeg_service_init(ble_eeg_t *p_eeg, const ble_eeg_init_t *p_eeg_init)
{
    if ((p_eeg == NULL) || (p_eeg_init == NULL))
    {
        return;
    }

    uint32_t err_code; // Variable to hold return codes from library and soft-device functions
    uint16_t service_handle;
    ble_uuid_t service_uuid;
    ble_uuid128_t base_uuid = {BMS_UUID_BASE};
    uint8_t num_of_enabled_ch = eeg_enabled_channel_count_get(p_eeg);
    p_eeg->num_of_enabled_ch = num_of_enabled_ch;
    p_eeg->conn_handle = BLE_CONN_HANDLE_INVALID;
    // Initialize service structure and register custom UUID base.
    p_eeg->eeg_config_handler = p_eeg_init->eeg_config_handler;

    err_code = sd_ble_uuid_vs_add(&base_uuid, &(p_eeg->uuid_type));
    APP_ERROR_CHECK(err_code);

    service_uuid.type = p_eeg->uuid_type;
    service_uuid.uuid = BLE_UUID_BIOPOTENTIAL_ECG_MEASUREMENT_SERVICE;

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &service_uuid, &service_handle);
    APP_ERROR_CHECK(err_code);
    p_eeg->service_handle = service_handle;

    // Add characteristics (config + channel data).
    err_code = eeg_ads1299_config_char_add(p_eeg);
    APP_ERROR_CHECK(err_code);

    /* Initialize the characteristic UUID for each channel based on whether it is enabled */
    for (uint8_t ch_idx = 0u; ch_idx < num_of_enabled_ch; ch_idx++)
    {
        err_code = m_eeg_char_add_fns[ch_idx](p_eeg);
        APP_ERROR_CHECK(err_code);
    }
}

#if defined(ADS1299)
/*! @brief Update ADS1299 configuration characteristic value.
 * @param[in,out] p_eeg    EEG service instance.
 * @param[in]     notify   True to send notification after update.
 * @return None.
 */
void ble_eeg_update_configuration(ble_eeg_t *p_eeg, bool notify)
{
    if (p_eeg == NULL)
    {
        return;
    }

    uint32_t err_code;
    uint16_t hvx_len = 23;
    if (p_eeg->conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        ble_gatts_value_t value;
        value.len = hvx_len;
        value.offset = 0;
        value.p_value = p_eeg->ads1299_current_configuration;

        err_code = sd_ble_gatts_value_set(p_eeg->conn_handle, p_eeg->ads1299_config_char_handles.value_handle, &value);
        NRF_LOG_INFO("err_code ble_eeg_update::config 0x%x \n", err_code);
        if (notify && eeg_notify_is_enabled(p_eeg->conn_handle, &p_eeg->ads1299_config_char_handles))
        {
            ble_gatts_hvx_params_t const hvx_params = {
                .handle = p_eeg->ads1299_config_char_handles.value_handle,
                .type = BLE_GATT_HVX_NOTIFICATION,
                .offset = 0,
                .p_len = &hvx_len,
                .p_data = p_eeg->ads1299_current_configuration,
            };
            err_code = sd_ble_gatts_hvx(p_eeg->conn_handle, &hvx_params);
        }
    }
}

/*! @brief Notify buffered data for EEG channel 1.
 * @param[in,out] p_eeg  EEG service instance.
 * @return None.
 */
void ble_eeg_update_1ch_v2(ble_eeg_t *p_eeg)
{
    uint32_t err_code = NRF_SUCCESS;
    uint16_t hvx_len = EEG_PACKET_LENGTH;
    if ((p_eeg == NULL) || (p_eeg->eeg_buffer[0] == NULL))
    {
        return;
    }

    if (p_eeg->conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        if (!eeg_notify_is_enabled(p_eeg->conn_handle, &p_eeg->eeg_handles[0]))
        {
            return;
        }

        ble_gatts_hvx_params_t const hvx_params = {
            .handle = p_eeg->eeg_handles[0].value_handle,
            .type = BLE_GATT_HVX_NOTIFICATION,
            .offset = 0,
            .p_len = &hvx_len,
            .p_data = p_eeg->eeg_buffer[0],
        };
        err_code = sd_ble_gatts_hvx(p_eeg->conn_handle, &hvx_params);
    }

    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_WARNING("EEG ch1 hvx failed: err=0x%x len=%u", err_code, hvx_len);
    }
    else if (hvx_len != EEG_PACKET_LENGTH)
    {
        NRF_LOG_WARNING("EEG ch1 hvx truncated: sent=%u req=%u", hvx_len, EEG_PACKET_LENGTH);
    }
}

/*! @brief Notify buffered data for up to four EEG channels.
 * @param[in,out] p_eeg  EEG service instance.
 * @return None.
 */
void ble_eeg_update_4ch(ble_eeg_t *p_eeg)
{
    if ((p_eeg == NULL) || (p_eeg->conn_handle == BLE_CONN_HANDLE_INVALID))
    {
        return;
    }

    uint8_t max_ch = p_eeg->num_of_enabled_ch;
    if (max_ch > 4u)
    {
        max_ch = 4u;
    }

    for (uint8_t idx = 0u; idx < max_ch; idx++)
    {
        if (p_eeg->ads1299_current_configuration[4u + idx] == 0xE1u)
        {
            continue;
        }
        if (p_eeg->eeg_buffer[idx] == NULL)
        {
            continue;
        }
        if (p_eeg->eeg_handles[idx].value_handle == BLE_GATT_HANDLE_INVALID)
        {
            continue;
        }
        if (!eeg_notify_is_enabled(p_eeg->conn_handle, &p_eeg->eeg_handles[idx]))
        {
            continue;
        }

        uint16_t hvx_len = EEG_PACKET_LENGTH;
        ble_gatts_hvx_params_t const hvx_params = {
            .handle = p_eeg->eeg_handles[idx].value_handle,
            .type = BLE_GATT_HVX_NOTIFICATION,
            .offset = 0,
            .p_len = &hvx_len,
            .p_data = p_eeg->eeg_buffer[idx],
        };

        uint32_t err_code = sd_ble_gatts_hvx(p_eeg->conn_handle, &hvx_params);
        if (err_code != NRF_SUCCESS)
        {
            NRF_LOG_WARNING("EEG hvx failed dev=%u ch=%u err=0x%x len=%u",
                            p_eeg->dev_idx,
                            idx,
                            err_code,
                            hvx_len);
            if (err_code == NRF_ERROR_RESOURCES)
            {
                break;
            }
        }
        else if (hvx_len != EEG_PACKET_LENGTH)
        {
            NRF_LOG_WARNING("EEG hvx truncated dev=%u ch=%u sent=%u req=%u",
                            p_eeg->dev_idx,
                            idx,
                            hvx_len,
                            EEG_PACKET_LENGTH);
            break;
        }
    }
}

/*! @brief Notify all enabled EEG channel buffers over BLE.
 * @param[in,out] p_eeg  EEG service instance.
 * @return None.
 */
uint32_t ble_eeg_update_all_ch(ble_eeg_t *p_eeg)
{
    uint32_t err_code = NRF_SUCCESS;
    if ((p_eeg == NULL) || (p_eeg->conn_handle == BLE_CONN_HANDLE_INVALID))
    {
        return NRF_ERROR_NULL;
    }

    uint8_t num_of_enabled_ch = eeg_enabled_channel_count_get(p_eeg);
    if (num_of_enabled_ch == 0u)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    uint8_t start_idx = (uint8_t)(p_eeg->tx_rr_next_ch_idx % num_of_enabled_ch);
    uint8_t next_start_idx = start_idx;
    bool start_idx_advanced = false;

    for (uint8_t offset = 0u; offset < num_of_enabled_ch; offset++)
    {
        uint8_t idx = (uint8_t)((start_idx + offset) % num_of_enabled_ch);
        // If pointer is null or not initialized
        if (p_eeg->eeg_buffer[idx] == NULL)
        {
            continue;
        }
        // If the GATT handle is invalid for the respective channel
        if (p_eeg->eeg_handles[idx].value_handle == BLE_GATT_HANDLE_INVALID)
        {
            continue;
        }
        if (!eeg_notify_is_enabled(p_eeg->conn_handle, &p_eeg->eeg_handles[idx]))
        {
            continue;
        }

        uint16_t hvx_len = EEG_PACKET_LENGTH;
        ble_gatts_hvx_params_t const hvx_params = {
            .handle = p_eeg->eeg_handles[idx].value_handle,
            .type = BLE_GATT_HVX_NOTIFICATION,
            .offset = 0,
            .p_len = &hvx_len,
            .p_data = p_eeg->eeg_buffer[idx],
        };

        err_code = sd_ble_gatts_hvx(p_eeg->conn_handle, &hvx_params);
        if (err_code != NRF_SUCCESS)
        {
            NRF_LOG_WARNING("EEG hvx failed dev=%u ch=%u err=0x%x len=%u",
                            p_eeg->dev_idx,
                            idx,
                            err_code,
                            hvx_len);
            if (err_code == NRF_ERROR_RESOURCES)
            {
                break;
            }
        }
        else if (hvx_len != EEG_PACKET_LENGTH)
        {
            NRF_LOG_WARNING("EEG hvx truncated dev=%u ch=%u sent=%u req=%u",
                            p_eeg->dev_idx,
                            idx,
                            hvx_len,
                            EEG_PACKET_LENGTH);
            break;
        }

        next_start_idx = (uint8_t)((idx + 1u) % num_of_enabled_ch);
        start_idx_advanced = true;
    }

    if (start_idx_advanced)
    {
        p_eeg->tx_rr_next_ch_idx = next_start_idx;
    }
    else
    {
        p_eeg->tx_rr_next_ch_idx = (uint8_t)((start_idx + 1u) % num_of_enabled_ch);
    }
}
#endif //(defined(ADS1299)
