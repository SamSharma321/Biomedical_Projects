/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef BT_LBS_H_
#define BT_LBS_H_

/**@file
 * @defgroup bt_lbs LED Button Service API
 * @{
 * @brief API for the LED Button Service (LBS).
 */

#ifdef __cplusplus
extern "C" {
#endif

// NOTE: LBS : LED Button Service
#include <zephyr/types.h>

/** @brief LBS Service UUID. */
#define BT_UUID_LBS_VAL BT_UUID_128_ENCODE(0x00001523, 0x1212, 0xefde, 0x1523, 0x785feabcd123)
/** @brief Button Characteristic UUID. */
#define BT_UUID_LBS_BUTTON_VAL	BT_UUID_128_ENCODE(0x00001524, 0x1212, 0xefde, 0x1523, 0x785feabcd123)
/** @brief LED Characteristic UUID. */
#define BT_UUID_LBS_LED_VAL BT_UUID_128_ENCODE(0x00001525, 0x1212, 0xefde, 0x1523, 0x785feabcd123)

/** @brief MySensor Characteristic UUID. */
#define BT_UUID_LBS_MYSENSOR_VAL	BT_UUID_128_ENCODE(0x00001526, 0x1212, 0xefde, 0x1523, 0x785feabcd123)
#define BT_UUID_EEGSENSOR_VAL	    BT_UUID_128_ENCODE(0x00001527, 0x1212, 0xEEF0, 0x1523, 0x785feabcd123)

#define BT_UUID_EEGSENSOR_CH1_VAL	BT_UUID_128_ENCODE(0x00001527, 0x1212, 0xEEF1, 0x1523, 0x785feabcd123)
#define BT_UUID_EEGSENSOR_CH2_VAL	BT_UUID_128_ENCODE(0x00001527, 0x1212, 0xEEF2, 0x1523, 0x785feabcd123)
#define BT_UUID_EEGSENSOR_CH3_VAL	BT_UUID_128_ENCODE(0x00001527, 0x1212, 0xEEF3, 0x1523, 0x785feabcd123)
#define BT_UUID_EEGSENSOR_CH4_VAL	BT_UUID_128_ENCODE(0x00001527, 0x1212, 0xEEF4, 0x1523, 0x785feabcd123)
#define BT_UUID_EEGSENSOR_CH5_VAL	BT_UUID_128_ENCODE(0x00001527, 0x1212, 0xEEF5, 0x1523, 0x785feabcd123)
#define BT_UUID_EEGSENSOR_CH6_VAL	BT_UUID_128_ENCODE(0x00001527, 0x1212, 0xEEF6, 0x1523, 0x785feabcd123)
#define BT_UUID_EEGSENSOR_CH7_VAL	BT_UUID_128_ENCODE(0x00001527, 0x1212, 0xEEF7, 0x1523, 0x785feabcd123)
#define BT_UUID_EEGSENSOR_CH8_VAL	BT_UUID_128_ENCODE(0x00001527, 0x1212, 0xEEF8, 0x1523, 0x785feabcd123)


#define BT_UUID_LBS 			BT_UUID_DECLARE_128(BT_UUID_LBS_VAL)
#define BT_UUID_LBS_BUTTON 		BT_UUID_DECLARE_128(BT_UUID_LBS_BUTTON_VAL)
#define BT_UUID_LBS_LED 		BT_UUID_DECLARE_128(BT_UUID_LBS_LED_VAL)
#define BT_UUID_LBS_MYSENSOR 	BT_UUID_DECLARE_128(BT_UUID_LBS_MYSENSOR_VAL)

#define BT_UUID_EEGSENSOR_CH1   BT_UUID_DECLARE_128(BT_UUID_EEGSENSOR_CH1_VAL)
#define BT_UUID_EEGSENSOR_CH2   BT_UUID_DECLARE_128(BT_UUID_EEGSENSOR_CH2_VAL)
#define BT_UUID_EEGSENSOR_CH3   BT_UUID_DECLARE_128(BT_UUID_EEGSENSOR_CH3_VAL)
#define BT_UUID_EEGSENSOR_CH4   BT_UUID_DECLARE_128(BT_UUID_EEGSENSOR_CH4_VAL)
#define BT_UUID_EEGSENSOR_CH5   BT_UUID_DECLARE_128(BT_UUID_EEGSENSOR_CH5_VAL)
#define BT_UUID_EEGSENSOR_CH6   BT_UUID_DECLARE_128(BT_UUID_EEGSENSOR_CH6_VAL)
#define BT_UUID_EEGSENSOR_CH7   BT_UUID_DECLARE_128(BT_UUID_EEGSENSOR_CH7_VAL)
#define BT_UUID_EEGSENSOR_CH8   BT_UUID_DECLARE_128(BT_UUID_EEGSENSOR_CH8_VAL)

/** @brief Callback type for when an LED state change is received. */
/*
 * Values are delivered from the LED write characteristic payload:
 * redLEDintensity  = payload[0]
 * sensorIntTime    = payload[1]
 * irLEDintensity   = payload[2]
 * payload[3] (LED location interval) is handled separately in my_lbs.c.
 */
typedef void (*led_cb_t)(const uint16_t redLEDintensity, const uint16_t irLEDintensity, const uint16_t sensorIntTime);
// typedef void (*led_cb_t)(const uint16_t LEDintensity, const uint16_t sensorIntTime);
// typedef return_type *func(params...)
typedef void (*ads_cb_t)(const uint8_t allChData, uint8_t numOfEnChannels);

/** @brief Callback type for when the button state is pulled. */
typedef bool (*button_cb_t)(void);

/** @brief Callback struct used by the LBS Service. */
struct my_lbs_cb {
	/** Called on central write to BT_UUID_LBS_LED (control/config update). */
	led_cb_t setSensorAndLED;
	/** Called when central reads BT_UUID_LBS_BUTTON. */
	button_cb_t button_cb;
    // TODO ADS1299 configuration
    
};

/** @brief Initialize the LBS Service.
 *
 * This function registers application callback functions with the My LBS
 * Service
 *
 * @param[in] callbacks Struct containing pointers to callback functions
 *			used by the service. This pointer can be NULL
 *			if no callback functions are defined.
 *
 *
 * @retval 0 If the operation was successful.
 *           Otherwise, a (negative) error code is returned.
 */
int my_lbs_init(struct my_lbs_cb *callbacks);

/** @brief Send the button state as indication.
 *
 * This function sends a binary state, typically the state of a
 * button, to all connected peers.
 *
 * @param[in] button_state The state of the button.
 *
 * @retval 0 If the operation was successful.
 *           Otherwise, a (negative) error code is returned.
 */
int my_lbs_send_button_state_indicate(bool button_state);

/** @brief Send the button state as notification.
 *
 * This function sends a binary state, typically the state of a
 * button, to all connected peers.
 *
 * @param[in] button_state The state of the button.
 *
 * @retval 0 If the operation was successful.
 *           Otherwise, a (negative) error code is returned.
 */
int my_lbs_send_button_state_notify(bool button_state);

/** @brief Send the sensor value as notification.
 *
 * This function sends an uint32_t  value, typically the value
 * of a simulated sensor to all connected peers.
 *
 * @param[in] sensor_value The value of the simulated sensor.
 *
 * @retval 0 If the operation was successful.
 *           Otherwise, a (negative) error code is returned.
 */
int my_lbs_send_sensor_notify(uint32_t sensor_value);

/** @brief Send 3x uint16_t sensor values in one notification.
 *
 * Packet order: [RED630, RED680, NIR].
 */
int my_lbs_send_sensor_notify_RED_NIRS(uint16_t RED630, uint16_t RED680, uint16_t NIR);

/** @brief Send 4x uint16_t values in one notification.
 *
 * Packet order: [RED630, RED680, NIR, LEDlocation].
 */
int my_lbs_send_sensor_notify_RED_NIRS_LEDindex(uint16_t RED630, uint16_t RED680, uint16_t NIR, uint16_t LEDlocation);

/** @brief Send ADS1299 EEG payload as notification. (Handler function)
 *
 * Payload format is application-defined by caller.
 */
int my_lbs_send_eeg_ch_notify(uint8_t ch, const uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* BT_LBS_H_ */
