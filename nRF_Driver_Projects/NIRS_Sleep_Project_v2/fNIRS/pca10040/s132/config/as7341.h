#ifndef AS7341_DRIVER_H__
#define AS7341_DRIVER_H__

#include <stdbool.h>
#include <stdint.h>
#include "sdk_errors.h"
#include "usr_config.h"

#if defined(AS7341)

#define AS7341_I2C_ADDR_DEFAULT              0x39u

#ifndef AS7341_ATIME_FIXED
#define AS7341_ATIME_FIXED                   99u
#endif

#ifndef AS7341_ASTEP_DEFAULT
#define AS7341_ASTEP_DEFAULT                 599u
#endif

#ifndef AS7341_BLE_INTERVAL_MS_DEFAULT
#define AS7341_BLE_INTERVAL_MS_DEFAULT       20u
#endif

#define AS7341_MUX_CHANNEL_COUNT             6u
#define AS7341_MUX_IDX_555NM_F5              0u
#define AS7341_MUX_IDX_590NM_F6              1u
#define AS7341_MUX_IDX_630NM_F7              2u
#define AS7341_MUX_IDX_680NM_F8              3u
#define AS7341_MUX_IDX_CLEAR                 4u
#define AS7341_MUX_IDX_NIR                   5u

ret_code_t as7341_init_all(void);
ret_code_t as7341_init_device(uint8_t dev_idx);
ret_code_t as7341_set_integration_20ms_device(uint8_t dev_idx, uint8_t integration_20ms_units);
ret_code_t as7341_read_current_mux_channels_device(uint8_t dev_idx, uint16_t channels[AS7341_MUX_CHANNEL_COUNT]);
ret_code_t as7341_read_red_nir_device(uint8_t dev_idx, uint16_t *p_red_630, uint16_t *p_red_680, uint16_t *p_nir);
uint16_t as7341_get_transfer_interval_ms_device(uint8_t dev_idx);
bool as7341_is_initialized_device(uint8_t dev_idx);

/* Backward-compatible wrappers using device index 0. */
ret_code_t as7341_init(void);
ret_code_t as7341_set_integration_20ms(uint8_t integration_20ms_units);
ret_code_t as7341_read_current_mux_channels(uint16_t channels[AS7341_MUX_CHANNEL_COUNT]);
ret_code_t as7341_read_red_nir(uint16_t *p_red_630, uint16_t *p_red_680, uint16_t *p_nir);
uint16_t as7341_get_transfer_interval_ms(void);

#endif /* defined(AS7341) */

#endif /* AS7341_DRIVER_H__ */
