#ifndef USR_CONFIG_
/*! Include guard to prevent multiple inclusion of this config header. */
#define USR_CONFIG_

/* Common includes */
#include <stdbool.h>

/* Device Name and other parameters */
/*! Default BLE device name used in advertising packets. */
#define DEVICE_NAME                     "nRF_fNIRS_System"
/*! Manufacturer string exposed through the BLE DIS service. */
#define MANUFACTURER_NAME               "BITN Lab"

/* Predefined Macros according to user configuration */
/*! Model number string reported by BLE Device Information Service. */
#define DEVICE_MODEL_NUMBERSTR          "Version 1.0"
/*! Firmware revision string reported by BLE Device Information Service. */
#define DEVICE_FIRMWARE_STRING          "Version 2.0"

/* ADS1299 Related Macros*/
/*! Compile-time flag enabling ADS1299-related code paths. */
#define ADS1299
/*! Number of ADS1299 devices populated and used at runtime. */
#define NUM_OF_ADS1299                 1
/*! Total ADS channel count across all active ADS1299 devices. */
#define NUM_OF_ADS_CH                  2
/*! GPIO used to drive ADS1299 power-down/reset control. */
#define ADS1299_PWDN_RST_PIN           19
/*! GPIO interrupt pin for ADS1299 device 1 data-ready signal. */
#define ADS1299_1_DRDY_PIN             11
/* SPI Pins */
/*! SPI clock pin shared by ADS1299 devices. */
#define ADS1299_SPI_SCLK_PIN           17
/*! SPI MISO pin carrying ADS1299 data to the MCU. */
#define ADS1299_SPI_DOUT_PIN           12
/*! Chip-select GPIO for ADS1299 device 1. */
#define ADS1299_1_SPI_CS_PIN           18
/*! SPI MOSI pin carrying commands from MCU to ADS1299. */
#define ADS1299_SPI_DIN_PIN            20
/*! GPIO controlling ADS1299 START conversion signal. */
#define ADS1299_START_PIN              10
/*! SPI peripheral instance used by ADS1299 (0/1/2). */
#define ADS1299_SPI_INSTANCE_ID        2

#if (NUM_OF_ADS1299 == 2)
/*! GPIO interrupt pin for ADS1299 device 2 data-ready signal. */
#define ADS1299_2_DRDY_PIN             8
/*! Chip-select GPIO for ADS1299 device 2. */
#define ADS1299_2_SPI_CS_PIN           9
#endif

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
/*! GPIO interrupt pin used by AS7341 for event signaling. Device 1 */
#define AS7341_1_INT_PIN                7


#if (NUM_OF_AS7341_DEVICES == 2)
/*! I2C clock pin connected to the AS7341 sensor - Device 2. */
#define AS7341_2_SCL_PIN                14
/*! I2C data pin connected to the AS7341 sensor - Device 2. */
#define AS7341_2_SDA_PIN                13
/*! GPIO interrupt pin used by AS7341 for event signaling. Device 2 */
#define AS7341_2_INT_PIN                15
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
#define LED_DEFAULT_RED_PERCENT         100
#define LED_DEFAULT_IR_PERCENT          100
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
#if (NUM_OF_ADS1299 > 2 || NUM_OF_ADS1299 < 1)
#error "Number of ADS1299 devices should be 1 or 2";
#endif /* #ifdef (ICM20649 && ICM20948) */
#if (ADS1299_SPI_INSTANCE_ID > 2)
#error "ADS1299_SPI_INSTANCE_ID must be 0, 1, or 2 on nRF52832"
#endif

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
#if (NUM_OF_AS7341_DEVICES == 2) && (AS7341_2_I2C_INSTANCE_ID == ADS1299_SPI_INSTANCE_ID)
#error "AS7341_2_I2C_INSTANCE_ID conflicts with ADS1299_SPI_INSTANCE_ID"
#endif
#endif

#endif // USR_CONFIG_
