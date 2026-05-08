#ifndef USR_CONFIG_
/*! Include guard to prevent multiple inclusion of this config header. */
#define USR_CONFIG_

/* Common includes */
#include <stdbool.h>

/* Device Name and other parameters */
/*! Default BLE device name used in advertising packets. */
#define DEVICE_NAME                     "nRF_fNIRS_Sys_v2"
/*! Manufacturer string exposed through the BLE DIS service. */
#define MANUFACTURER_NAME               "BITN Lab"

/* Predefined Macros according to user configuration */
/*! Model number string reported by BLE Device Information Service. */
#define DEVICE_MODEL_NUMBERSTR          "Version 1.0"
/*! Firmware revision string reported by BLE Device Information Service. */
#define DEVICE_FIRMWARE_STRING          "Version 2.0"

#define AS7341

#ifdef AS7341
/* AS7341 related macros*/
/*! Number of AS7341 devices connected and active. */
#define NUM_OF_AS7341_DEVICES           2
/* I2C Pins */
/*! I2C clock pin connected to the AS7341 sensor - Device 1. */
#define AS7341_1_SCL_PIN                9
/*! I2C data pin connected to the AS7341 sensor - Device 1. */
#define AS7341_1_SDA_PIN                10

#if (NUM_OF_AS7341_DEVICES == 2)
/*! I2C clock pin connected to the AS7341 sensor - Device 2. */
#define AS7341_2_SCL_PIN                8
/*! I2C data pin connected to the AS7341 sensor - Device 2. */
#define AS7341_2_SDA_PIN                7
#endif

/*! AS7341 I2C parameters. */
#define AS7341_1_I2C_ADDR               0x39
#define AS7341_2_I2C_ADDR               0x39
#define AS7341_1_I2C_INSTANCE_ID        1
#define AS7341_2_I2C_INSTANCE_ID        0
#define AS7341_STREAM_DEVICE_INDEX      0

/*! AS7341 default BLE payload update cadence and integration settings. */
#define AS7341_DEFAULT_INTEGRATION_20MS 1
#define AS7341_DEFAULT_LED_LOCATION_S   1
#define AS7341_DEFAULT_BLE_INTERVAL_MS  20
#define AS7341_SAMPLE_PERIOD_MS         1000
/*! Number of AS7341 samples to queue before one BLE transmission. */
#define BLE_AS7341_QUEUE_SAMPLES        5
#endif /* #ifdef AS7341 */


#define LED_CONTROL

#ifdef LED_CONTROL
/* White LED Pin 1 */
#define WH_LED_1_PIN                    3
/* White LED Pin 2 */
#define WH_LED_2_PIN                    5
/* IR LED Pin 1 */
#define IR_LED_1_PIN                    4
/* IR LED Pin 2 */
#define IR_LED_2_PIN                    6
/*! LED PWM defaults (0-100%). */
#define LED_DEFAULT_RED_PERCENT         20
#define LED_DEFAULT_IR_PERCENT          20
#define LED_STARTUP_TEST_PERCENT        100
#define LED_STARTUP_TEST_MS             2000
#ifndef LED_PWM_TOP_VALUE
#define LED_PWM_TOP_VALUE               1000
#endif
#endif /* #ifdef LED_CONTROL */

/* DEBUG MODE */
/*! Enables debug-specific behavior/logging when set to nonzero. */
#define DEBUG                          1
/*! Sentinel value used for diagnostics and memory corruption checks. */
#define DEAD_BEEF                      0xDEADBEEFu

/* Macro checks */
#if defined(AS7341)
#if (NUM_OF_AS7341_DEVICES < 1 || NUM_OF_AS7341_DEVICES > 2)
#error "NUM_OF_AS7341_DEVICES must be 1 or 2"
#endif
#if (AS7341_1_I2C_INSTANCE_ID > 1)
#error "AS7341_1_I2C_INSTANCE_ID must be 0 or 1 on nRF52832"
#endif
#if (NUM_OF_AS7341_DEVICES == 2)
#if (AS7341_2_I2C_INSTANCE_ID > 1)
#error "AS7341_2_I2C_INSTANCE_ID must be 0 or 1 on nRF52832"
#endif
#if (AS7341_2_I2C_INSTANCE_ID == AS7341_1_I2C_INSTANCE_ID)
#error "Use different I2C instances for AS7341_1 and AS7341_2"
#endif
#endif
#if (AS7341_STREAM_DEVICE_INDEX >= NUM_OF_AS7341_DEVICES)
#error "AS7341_STREAM_DEVICE_INDEX out of range"
#endif
#if (AS7341_1_I2C_INSTANCE_ID == ADS1299_SPI_INSTANCE_ID)
#error "AS7341_1_I2C_INSTANCE_ID conflicts with ADS1299_SPI_INSTANCE_ID"
#endif
#endif

#endif // USR_CONFIG_
