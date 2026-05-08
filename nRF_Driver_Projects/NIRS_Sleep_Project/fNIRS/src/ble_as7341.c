#include "ble_as7341.h"

#if defined(AS7341)

#include <string.h>
#include "app_error.h"
#include "nrf_log.h"
#include "sdk_errors.h"
#include "usr_config.h"

uint32_t ble_as7341_update_sample_device(ble_as7341_t *p_as7341,
                                         uint8_t dev_idx,
                                         uint16_t red_630,
                                         uint16_t red_680,
                                         uint16_t nir,
                                         uint16_t led_location_idx);
uint32_t ble_as7341_update_samples_device(ble_as7341_t *p_as7341,
                                          uint8_t dev_idx,
                                          uint16_t const *p_samples,
                                          uint16_t sample_count);

static bool ble_as7341_notify_is_enabled(uint16_t conn_handle, ble_gatts_char_handles_t const *p_handles)
{
    uint8_t cccd_value_buf[2] = {0};
    ble_gatts_value_t cccd_value;
    ret_code_t err_code;

    if ((conn_handle == BLE_CONN_HANDLE_INVALID) ||
        (p_handles == NULL) ||
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

static uint32_t ble_as7341_config_char_add(ble_as7341_t *p_as7341)
{
    ble_uuid_t char_uuid;
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t attr_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t attr_char_value;

    memset(&char_md, 0, sizeof(char_md));
    memset(&attr_md, 0, sizeof(attr_md));
    memset(&cccd_md, 0, sizeof(cccd_md));
    memset(&attr_char_value, 0, sizeof(attr_char_value));

    char_uuid.uuid = BLE_UUID_AS7341_LED_CONFIG_CHAR;
    char_uuid.type = p_as7341->uuid_type;

    char_md.char_props.read = 1;
    char_md.char_props.write = 1;
    char_md.char_props.notify = 1;
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc = BLE_GATTS_VLOC_STACK;
    char_md.p_cccd_md = &cccd_md;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    attr_md.vloc = BLE_GATTS_VLOC_STACK;
    attr_md.vlen = 1;

    attr_char_value.p_uuid = &char_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len = BLE_AS7341_CONFIG_LEN;
    attr_char_value.init_offs = 0u;
    attr_char_value.max_len = BLE_AS7341_CONFIG_LEN;
    attr_char_value.p_value = p_as7341->config_value;

    return sd_ble_gatts_characteristic_add(p_as7341->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_as7341->config_handles);
}

static uint16_t ble_as7341_data_uuid_for_device(uint8_t dev_idx)
{
    if (dev_idx == 0u)
    {
        return BLE_UUID_AS7341_DATA_CHAR_DEV0;
    }
    if (dev_idx == 1u)
    {
        return BLE_UUID_AS7341_DATA_CHAR_DEV1;
    }

    /* Fallback for unsupported indexes (NUM_OF_AS7341_DEVICES is validated as 1..2). */
    return BLE_UUID_AS7341_DATA_CHAR_DEV0;
}

static uint32_t ble_as7341_data_char_add(ble_as7341_t *p_as7341, uint8_t dev_idx)
{
    ble_uuid_t char_uuid;
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t attr_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t attr_char_value;

    memset(&char_md, 0, sizeof(char_md));
    memset(&attr_md, 0, sizeof(attr_md));
    memset(&cccd_md, 0, sizeof(cccd_md));
    memset(&attr_char_value, 0, sizeof(attr_char_value));

    char_uuid.uuid = ble_as7341_data_uuid_for_device(dev_idx);
    char_uuid.type = p_as7341->uuid_type;

    char_md.char_props.read = 1;
    char_md.char_props.notify = 1;
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc = BLE_GATTS_VLOC_STACK;
    char_md.p_cccd_md = &cccd_md;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    attr_md.vloc = BLE_GATTS_VLOC_STACK;
    attr_md.vlen = 1;

    attr_char_value.p_uuid = &char_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len = 0u;
    attr_char_value.init_offs = 0u;
    attr_char_value.max_len = BLE_AS7341_BATCH_LEN;
    attr_char_value.p_value = (uint8_t *)p_as7341->sample_value[dev_idx];

    return sd_ble_gatts_characteristic_add(p_as7341->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_as7341->data_handles[dev_idx]);
}

static void ble_as7341_on_write(ble_as7341_t *p_as7341, ble_evt_t *p_ble_evt)
{
    ble_gatts_evt_write_t const *p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    if ((p_evt_write->handle == p_as7341->config_handles.value_handle) &&
        (p_evt_write->offset == 0u) &&
        (p_evt_write->len == BLE_AS7341_CONFIG_LEN))
    {
        memcpy(p_as7341->config_value, p_evt_write->data, BLE_AS7341_CONFIG_LEN);
        if (p_as7341->config_handler != NULL)
        {
            p_as7341->config_handler(p_ble_evt->evt.gatts_evt.conn_handle,
                                     p_as7341,
                                     p_evt_write->data,
                                     p_evt_write->len);
        }
    }
}

void ble_as7341_service_init(ble_as7341_t *p_as7341, ble_as7341_init_t const *p_init)
{
    uint32_t err_code;
    ble_uuid_t service_uuid;
    ble_uuid128_t base_uuid = {BLE_AS7341_UUID_BASE};

    if ((p_as7341 == NULL) || (p_init == NULL))
    {
        return;
    }

    memset(p_as7341, 0, sizeof(*p_as7341));
    p_as7341->conn_handle = BLE_CONN_HANDLE_INVALID;
    p_as7341->config_handler = p_init->config_handler;
#if defined(LED_CONTROL)
    p_as7341->config_value[0] = LED_DEFAULT_RED_PERCENT; /* red intensity % */
    p_as7341->config_value[2] = LED_DEFAULT_IR_PERCENT;  /* ir intensity % */
#else
    p_as7341->config_value[0] = 100u;
    p_as7341->config_value[2] = 100u;
#endif
    p_as7341->config_value[1] = AS7341_DEFAULT_INTEGRATION_20MS; /* integration in 20 ms units */
    p_as7341->config_value[3] = AS7341_DEFAULT_LED_LOCATION_S;   /* LED location interval (s) */

    err_code = sd_ble_uuid_vs_add(&base_uuid, &p_as7341->uuid_type);
    APP_ERROR_CHECK(err_code);

    service_uuid.uuid = BLE_UUID_AS7341_SERVICE;
    service_uuid.type = p_as7341->uuid_type;
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &service_uuid,
                                        &p_as7341->service_handle);
    APP_ERROR_CHECK(err_code);

    err_code = ble_as7341_config_char_add(p_as7341);
    APP_ERROR_CHECK(err_code);

    for (uint8_t dev_idx = 0u; dev_idx < NUM_OF_AS7341_DEVICES; dev_idx++)
    {
        err_code = ble_as7341_data_char_add(p_as7341, dev_idx);
        APP_ERROR_CHECK(err_code);
    }
}

void ble_as7341_on_ble_evt(ble_as7341_t *p_as7341, ble_evt_t *p_ble_evt)
{
    if ((p_as7341 == NULL) || (p_ble_evt == NULL))
    {
        return;
    }

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            p_as7341->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            p_as7341->conn_handle = BLE_CONN_HANDLE_INVALID;
            break;

        case BLE_GATTS_EVT_WRITE:
            ble_as7341_on_write(p_as7341, p_ble_evt);
            break;

        default:
            break;
    }
}

uint32_t ble_as7341_update_sample(ble_as7341_t *p_as7341,
                                  uint16_t red_630,
                                  uint16_t red_680,
                                  uint16_t nir,
                                  uint16_t led_location_idx)
{
    return ble_as7341_update_sample_device(p_as7341,
                                           0u,
                                           red_630,
                                           red_680,
                                           nir,
                                           led_location_idx);
}

uint32_t ble_as7341_update_sample_device(ble_as7341_t *p_as7341,
                                         uint8_t dev_idx,
                                         uint16_t red_630,
                                         uint16_t red_680,
                                         uint16_t nir,
                                         uint16_t led_location_idx)
{
    uint16_t sample[BLE_AS7341_FIELDS_PER_SAMPLE];

    sample[0] = red_630;
    sample[1] = red_680;
    sample[2] = nir;
    sample[3] = led_location_idx;

    return ble_as7341_update_samples_device(p_as7341, dev_idx, sample, 1u);
}

uint32_t ble_as7341_update_samples(ble_as7341_t *p_as7341,
                                   uint16_t const *p_samples,
                                   uint16_t sample_count)
{
    return ble_as7341_update_samples_device(p_as7341, 0u, p_samples, sample_count);
}

uint32_t ble_as7341_update_samples_device(ble_as7341_t *p_as7341,
                                          uint8_t dev_idx,
                                          uint16_t const *p_samples,
                                          uint16_t sample_count)
{
    uint32_t err_code;
    uint16_t payload_len;
    uint16_t hvx_len;
    ble_gatts_value_t value;

    if (p_as7341 == NULL)
        return NRF_ERROR_NULL;
    if (p_samples == NULL)
        return NRF_ERROR_NULL;
    if (p_as7341->conn_handle == BLE_CONN_HANDLE_INVALID)
        return NRF_ERROR_INVALID_STATE;
    if (dev_idx >= NUM_OF_AS7341_DEVICES)
        return NRF_ERROR_INVALID_PARAM;

    if ((sample_count == 0u) || (sample_count > BLE_AS7341_QUEUE_SAMPLES))
        return NRF_ERROR_INVALID_PARAM;

    payload_len = (uint16_t)(sample_count * BLE_AS7341_SAMPLE_LEN);
    hvx_len = payload_len;
    memcpy(p_as7341->sample_value[dev_idx], p_samples, payload_len);

    memset(&value, 0, sizeof(value));
    value.len = payload_len;
    value.offset = 0u;
    value.p_value = (uint8_t *)p_as7341->sample_value[dev_idx];
    err_code = sd_ble_gatts_value_set(p_as7341->conn_handle,
                                      p_as7341->data_handles[dev_idx].value_handle,
                                      &value);
    if (err_code != NRF_SUCCESS)
        return err_code;

    if (!ble_as7341_notify_is_enabled(p_as7341->conn_handle, &p_as7341->data_handles[dev_idx]))
        return NRF_SUCCESS;

    ble_gatts_hvx_params_t const hvx_params =
    {
        .handle = p_as7341->data_handles[dev_idx].value_handle,
        .type = BLE_GATT_HVX_NOTIFICATION,
        .offset = 0u,
        .p_len = &hvx_len,
        .p_data = (uint8_t *)p_as7341->sample_value[dev_idx],
    };

    err_code = sd_ble_gatts_hvx(p_as7341->conn_handle, &hvx_params);
    if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_RESOURCES))
    {
        NRF_LOG_WARNING("AS7341[%u] hvx failed: 0x%X (payload=%u)", dev_idx, err_code, payload_len);
    }
    else if (hvx_len != payload_len)
    {
        NRF_LOG_WARNING("AS7341[%u] hvx truncated: sent=%u req=%u", dev_idx, hvx_len, payload_len);
    }

    return err_code;
}

#endif /* defined(AS7341) */
