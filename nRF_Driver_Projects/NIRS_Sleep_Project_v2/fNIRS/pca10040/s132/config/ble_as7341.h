#ifndef BLE_AS7341_H__
#define BLE_AS7341_H__

#include "ble.h"
#include "ble_srv_common.h"
#include <stdint.h>
#include "sdk_config.h"
#include "usr_config.h"

#if defined(AS7341)

#define BLE_AS7341_UUID_BASE                    { 0x23, 0xD1, 0xBC, 0xEA, \
                                                  0x5F, 0x78, 0x23, 0x15, \
                                                  0xDE, 0xEF, 0x12, 0x12, \
                                                  0x00, 0x00, 0x00, 0x00 }

#define BLE_UUID_AS7341_SERVICE                 0x1523
#define BLE_UUID_AS7341_LED_CONFIG_CHAR         0x1525
#define BLE_UUID_AS7341_DATA_CHAR_DEV0          0x1526
#define BLE_UUID_AS7341_DATA_CHAR_DEV1          0x1527
/* Backward-compatible alias: device 0 data UUID. */
#define BLE_UUID_AS7341_DATA_CHAR               BLE_UUID_AS7341_DATA_CHAR_DEV0

#define BLE_AS7341_CONFIG_LEN                   4u
#define BLE_AS7341_SAMPLE_LEN                   8u
#ifndef BLE_AS7341_QUEUE_SAMPLES
#define BLE_AS7341_QUEUE_SAMPLES                10u
#endif
#define BLE_AS7341_FIELDS_PER_SAMPLE            (BLE_AS7341_SAMPLE_LEN / sizeof(uint16_t))
#define BLE_AS7341_BATCH_LEN                    (BLE_AS7341_SAMPLE_LEN * BLE_AS7341_QUEUE_SAMPLES)
#define BLE_AS7341_MAX_NOTIFY_PAYLOAD           (NRF_SDH_BLE_GATT_MAX_MTU_SIZE - 3u)

#if (BLE_AS7341_BATCH_LEN > BLE_AS7341_MAX_NOTIFY_PAYLOAD)
#error "BLE_AS7341_BATCH_LEN exceeds negotiated ATT notification payload."
#endif

typedef struct ble_as7341_s ble_as7341_t;

typedef void (*ble_as7341_write_config_handler_t)(uint16_t conn_handle,
                                                   ble_as7341_t *p_as7341,
                                                   uint8_t const *data,
                                                   uint16_t data_len);

typedef struct
{
    ble_as7341_write_config_handler_t config_handler;
} ble_as7341_init_t;

struct ble_as7341_s
{
    uint8_t uuid_type;
    uint16_t conn_handle;
    uint16_t service_handle;
    ble_gatts_char_handles_t config_handles;
    ble_gatts_char_handles_t data_handles[NUM_OF_AS7341_DEVICES];
    ble_as7341_write_config_handler_t config_handler;
    uint8_t config_value[BLE_AS7341_CONFIG_LEN];
    uint16_t sample_value[NUM_OF_AS7341_DEVICES][BLE_AS7341_BATCH_LEN / sizeof(uint16_t)];
};

void ble_as7341_service_init(ble_as7341_t *p_as7341, ble_as7341_init_t const *p_init);
void ble_as7341_on_ble_evt(ble_as7341_t *p_as7341, ble_evt_t *p_ble_evt);
uint32_t ble_as7341_update_sample(ble_as7341_t *p_as7341,
                                  uint16_t red_630,
                                  uint16_t red_680,
                                  uint16_t nir,
                                  uint16_t led_location_idx);
uint32_t ble_as7341_update_samples(ble_as7341_t *p_as7341,
                                   uint16_t const *p_samples,
                                   uint16_t sample_count);
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

#endif /* defined(AS7341) */

#endif /* BLE_AS7341_H__ */
