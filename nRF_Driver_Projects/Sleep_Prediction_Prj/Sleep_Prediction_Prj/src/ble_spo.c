#include "ble_spo.h"

#include <string.h>

#include "app_error.h"
#include "ble_srv_common.h"
#include "max30102.h"
#include "nrf_log.h"
#include "nordic_common.h"
#include "sdk_common.h"
#include "usr_config.h"

static void on_write(ble_spo_t *p_spo, ble_evt_t *p_ble_evt)
{
    ble_gatts_evt_write_t *p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    if ((p_evt_write->handle == p_spo->spo_led_config_handles.value_handle) &&
        (p_evt_write->len >= SPO_LED_CONFIG_LENGTH) &&
        (p_spo->spo_write_config_handler != NULL))
    {
        p_spo->spo_write_config_handler(p_ble_evt->evt.gap_evt.conn_handle,
                                        p_spo,
                                        p_evt_write->data,
                                        p_evt_write->len);
    }
}

void ble_spo_on_ble_evt(ble_spo_t *p_spo, ble_evt_t *p_ble_evt)
{
    if ((p_spo == NULL) || (p_ble_evt == NULL))
    {
        return;
    }

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            p_spo->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            p_spo->conn_handle = BLE_CONN_HANDLE_INVALID;
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_spo, p_ble_evt);
            break;

        default:
            break;
    }
}

static uint32_t spo_char_add(ble_spo_t *p_spo,
                             ble_uuid_t const *p_uuid,
                             uint16_t max_len,
                             bool writable,
                             ble_gatts_char_handles_t *p_handles,
                             uint8_t *p_init_value)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t attr_char_value;
    ble_gatts_attr_md_t attr_md;
    ble_gatts_attr_md_t cccd_md;

    memset(&char_md, 0, sizeof(char_md));
    memset(&cccd_md, 0, sizeof(cccd_md));
    memset(&attr_md, 0, sizeof(attr_md));
    memset(&attr_char_value, 0, sizeof(attr_char_value));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc = BLE_GATTS_VLOC_STACK;

    char_md.char_props.read = 1;
    char_md.char_props.notify = 1;
    char_md.char_props.write = writable ? 1 : 0;
    char_md.p_cccd_md = &cccd_md;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    attr_md.vloc = BLE_GATTS_VLOC_STACK;
    attr_md.vlen = 1;

    attr_char_value.p_uuid = p_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len = max_len;
    attr_char_value.max_len = max_len;
    attr_char_value.p_value = p_init_value;

    return sd_ble_gatts_characteristic_add(p_spo->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           p_handles);
}

void ble_spo_service_init(ble_spo_t *p_spo, ble_spo_init_t const *p_init)
{
    APP_ERROR_CHECK_BOOL(p_spo != NULL);
    APP_ERROR_CHECK_BOOL(p_init != NULL);

    memset(p_spo, 0, sizeof(*p_spo));
    p_spo->conn_handle = BLE_CONN_HANDLE_INVALID;
    p_spo->spo_write_config_handler = p_init->spo_write_config_handler;
    p_spo->spo_led_config_buffer[0] = MAX30102_LED_RED_DEFAULT;
    p_spo->spo_led_config_buffer[1] = MAX30102_LED_IR_DEFAULT;

    ble_uuid128_t base_uuid = {SPO_UUID_BASE};
    uint32_t err = sd_ble_uuid_vs_add(&base_uuid, &p_spo->uuid_type);
    APP_ERROR_CHECK(err);

    ble_uuid_t service_uuid;
    service_uuid.type = p_spo->uuid_type;
    service_uuid.uuid = BLE_UUID_BIOPOTENTIAL_SPO_MEASUREMENT_SERVICE;

    err = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &service_uuid, &p_spo->service_handle);
    APP_ERROR_CHECK(err);

    ble_uuid_t char_uuid;

    char_uuid.type = p_spo->uuid_type;
    char_uuid.uuid = BLE_UUID_SPO_CH1_CHAR;
    err = spo_char_add(p_spo, &char_uuid, SPO_PACKET_LENGTH, false, &p_spo->spo_ch1_handles, p_spo->spo_ch1_buffer);
    APP_ERROR_CHECK(err);

    char_uuid.uuid = BLE_UUID_SPO_CH2_CHAR;
    err = spo_char_add(p_spo, &char_uuid, SPO_PACKET_LENGTH, false, &p_spo->spo_ch2_handles, p_spo->spo_ch2_buffer);
    APP_ERROR_CHECK(err);

    char_uuid.uuid = BLE_UUID_SPO_LED_CONFIG;
    err = spo_char_add(p_spo, &char_uuid, SPO_LED_CONFIG_LENGTH, true, &p_spo->spo_led_config_handles, p_spo->spo_led_config_buffer);
    APP_ERROR_CHECK(err);
}

void ble_spo_update_config(ble_spo_t *p_spo, bool notify)
{
    if ((p_spo == NULL) || (p_spo->conn_handle == BLE_CONN_HANDLE_INVALID))
    {
        return;
    }

    uint16_t len = SPO_LED_CONFIG_LENGTH;
    ble_gatts_value_t value = {
        .len = len,
        .offset = 0,
        .p_value = p_spo->spo_led_config_buffer,
    };

    uint32_t err = sd_ble_gatts_value_set(p_spo->conn_handle, p_spo->spo_led_config_handles.value_handle, &value);
    if (err != NRF_SUCCESS)
    {
        NRF_LOG_WARNING("SPO config set failed: 0x%X", err);
        return;
    }

    if (notify)
    {
        ble_gatts_hvx_params_t hvx_params = {
            .handle = p_spo->spo_led_config_handles.value_handle,
            .type = BLE_GATT_HVX_NOTIFICATION,
            .offset = 0,
            .p_len = &len,
            .p_data = p_spo->spo_led_config_buffer,
        };
        err = sd_ble_gatts_hvx(p_spo->conn_handle, &hvx_params);
        if ((err != NRF_SUCCESS) && (err != NRF_ERROR_RESOURCES) && (err != NRF_ERROR_INVALID_STATE))
        {
            NRF_LOG_WARNING("SPO config notify failed: 0x%X", err);
        }
    }
}

void ble_spo_update_1ch(ble_spo_t *p_spo)
{
    if ((p_spo == NULL) || (p_spo->conn_handle == BLE_CONN_HANDLE_INVALID) || (p_spo->spo_ch1_count == 0u))
    {
        return;
    }

    uint16_t len = p_spo->spo_ch1_count;
    ble_gatts_hvx_params_t hvx_params = {
        .handle = p_spo->spo_ch1_handles.value_handle,
        .type = BLE_GATT_HVX_NOTIFICATION,
        .offset = 0,
        .p_len = &len,
        .p_data = p_spo->spo_ch1_buffer,
    };

    uint32_t err = sd_ble_gatts_hvx(p_spo->conn_handle, &hvx_params);
    if (err == NRF_SUCCESS)
    {
        p_spo->spo_ch1_count = 0u;
    }
    else if ((err != NRF_ERROR_RESOURCES) && (err != NRF_ERROR_INVALID_STATE))
    {
        NRF_LOG_WARNING("SPO ch1 notify failed: 0x%X", err);
    }
}

void ble_spo_update_2ch(ble_spo_t *p_spo)
{
    if ((p_spo == NULL) || (p_spo->conn_handle == BLE_CONN_HANDLE_INVALID) ||
        (p_spo->spo_ch1_count == 0u) || (p_spo->spo_ch2_count == 0u))
    {
        return;
    }

    uint16_t len1 = p_spo->spo_ch1_count;
    ble_gatts_hvx_params_t hvx_params1 = {
        .handle = p_spo->spo_ch1_handles.value_handle,
        .type = BLE_GATT_HVX_NOTIFICATION,
        .offset = 0,
        .p_len = &len1,
        .p_data = p_spo->spo_ch1_buffer,
    };

    uint32_t err = sd_ble_gatts_hvx(p_spo->conn_handle, &hvx_params1);
    if (err != NRF_SUCCESS)
    {
        if ((err != NRF_ERROR_RESOURCES) && (err != NRF_ERROR_INVALID_STATE))
        {
            NRF_LOG_WARNING("SPO ch1 notify failed: 0x%X", err);
        }
        return;
    }

    uint16_t len2 = p_spo->spo_ch2_count;
    ble_gatts_hvx_params_t hvx_params2 = {
        .handle = p_spo->spo_ch2_handles.value_handle,
        .type = BLE_GATT_HVX_NOTIFICATION,
        .offset = 0,
        .p_len = &len2,
        .p_data = p_spo->spo_ch2_buffer,
    };

    err = sd_ble_gatts_hvx(p_spo->conn_handle, &hvx_params2);
    if (err == NRF_SUCCESS)
    {
        p_spo->spo_ch1_count = 0u;
        p_spo->spo_ch2_count = 0u;
    }
    else if ((err != NRF_ERROR_RESOURCES) && (err != NRF_ERROR_INVALID_STATE))
    {
        NRF_LOG_WARNING("SPO ch2 notify failed: 0x%X", err);
    }
}

void spo_led_config_handler(uint16_t conn_handle, ble_spo_t *p_spo, uint8_t const *data, uint16_t data_len)
{
    UNUSED_PARAMETER(conn_handle);

    if ((p_spo == NULL) || (data == NULL) || (data_len < SPO_LED_CONFIG_LENGTH))
    {
        return;
    }

    ret_code_t err = max30102_init_custom(data[0], data[1]);
    if (err != NRF_SUCCESS)
    {
        NRF_LOG_WARNING("MAX30102 LED config write failed: 0x%X", err);
        return;
    }

    p_spo->spo_led_config_buffer[0] = data[0];
    p_spo->spo_led_config_buffer[1] = data[1];
    ble_spo_update_config(p_spo, true);
}
