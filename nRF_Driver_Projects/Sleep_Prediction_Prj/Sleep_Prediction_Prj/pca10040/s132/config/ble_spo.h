#ifndef BLE_SPO_H__
#define BLE_SPO_H__

#include <stdbool.h>
#include <stdint.h>

#include "ble.h"
#include "ble_srv_common.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SPO_UUID_BASE                    { 0x57, 0x80, 0xD2, 0x94, 0xA3, 0xB2, 0xFE, 0x39, 0x5F, 0x87, 0xFD, 0x35, 0x00, 0x00, 0x8B, 0x22 }
#ifndef BLE_UUID_BIOPOTENTIAL_SPO_MEASUREMENT_SERVICE
#define BLE_UUID_BIOPOTENTIAL_SPO_MEASUREMENT_SERVICE 0xAEC0
#endif
#define BLE_UUID_SPO_CH1_CHAR            0xAEC1
#define BLE_UUID_SPO_CH2_CHAR            0xAEC2
#define BLE_UUID_SPO_LED_CONFIG          0xAECF

#define SPO_PACKET_LENGTH                90u
#define SPO_LED_CONFIG_LENGTH            2u
#define SPO_FIFO_SAMPLE_BYTES            6u

typedef struct ble_spo_s ble_spo_t;

typedef void (*ble_spo_write_config_handler_t)(uint16_t conn_handle, ble_spo_t *p_spo,uint8_t const *data, uint16_t data_len);

typedef struct
{
    ble_spo_write_config_handler_t spo_write_config_handler;
} ble_spo_init_t;

struct ble_spo_s
{
    uint8_t uuid_type;
    uint16_t conn_handle;
    uint16_t service_handle;

    ble_gatts_char_handles_t spo_ch1_handles;
    ble_gatts_char_handles_t spo_ch2_handles;
    ble_gatts_char_handles_t spo_led_config_handles;

    uint8_t spo_ch1_buffer[SPO_PACKET_LENGTH];
    uint8_t spo_ch2_buffer[SPO_PACKET_LENGTH];
    uint8_t spo_led_config_buffer[SPO_LED_CONFIG_LENGTH];

    uint16_t spo_ch1_count;
    uint16_t spo_ch2_count;

    ble_spo_write_config_handler_t spo_write_config_handler;
};

void ble_spo_service_init(ble_spo_t *p_spo, ble_spo_init_t const *p_init);
void ble_spo_on_ble_evt(ble_spo_t *p_spo, ble_evt_t *p_ble_evt);
void ble_spo_update_config(ble_spo_t *p_spo, bool notify);
void ble_spo_update_1ch(ble_spo_t *p_spo);
void ble_spo_update_2ch(ble_spo_t *p_spo);
void spo_led_config_handler(uint16_t conn_handle, ble_spo_t *p_spo, uint8_t const *data, uint16_t data_len);

#ifdef __cplusplus
}
#endif

#endif
