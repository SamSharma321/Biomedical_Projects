#ifndef USR_CONFIG_
/*! Include guard to prevent multiple inclusion of this config header. */
#define USR_CONFIG_

/* Common includes */
#include <stdbool.h>

/* Device Name and other parameters */
/*! Default BLE device name used in advertising packets. */
#define DEVICE_NAME                     "Sleep_Prediction_Sensor"
#define MANUFACTURER_NAME               "BITN_Lab"

/* Predefined Macros according to user configuration */
/*! Model number string reported by BLE Device Information Service. */
#define DEVICE_MODEL_NUMBERSTR          "Version 1.0"
/*! Firmware revision string reported by BLE Device Information Service. */
#define DEVICE_FIRMWARE_STRING          "Version 3.0"

/* ADS1299 Related Macros*/
/*! Compile-time flag enabling ADS1299-related code paths. */
#define ADS1299
/*! Number of ADS1299 devices populated and used at runtime. */
#define NUM_OF_ADS1299                 1
/*! Total ADS channel count across all active ADS1299 devices. */
#define NUM_OF_ADS_CH                  4

/*! GPIO used to drive ADS1299 power-down/reset control. */
#define ADS1299_PWDN_RST_PIN           15
/*! GPIO interrupt pin for ADS1299 device 1 data-ready signal. */
#define ADS1299_1_DRDY_PIN             11
/*! GPIO interrupt pin for ADS1299 device 2 data-ready signal. */
#define ADS1299_2_DRDY_PIN             8

/* ICM20x related macros*/
/*! Compile-time flag enabling ICM20649-related code paths. */
#define ICM20649
/*! Compile-time flag enabling MAX30102-related code paths. */
#define MAX30102_PRESENT               1

/*! Number of ICM20xxx IMU devices connected and active. */
#define NUM_OF_ICM_DEVICES             1

/* SPI Pins */
/*! SPI clock pin shared by ADS1299 devices. */
#define ADS1299_SPI_SCLK_PIN           13
/*! SPI MISO pin carrying ADS1299 data to the MCU. */
#define ADS1299_SPI_DOUT_PIN           12
/*! Chip-select GPIO for ADS1299 device 1. */
#define ADS1299_1_SPI_CS_PIN           14
/*! Chip-select GPIO for ADS1299 device 2. */
#define ADS1299_2_SPI_CS_PIN           9
/*! SPI MOSI pin carrying commands from MCU to ADS1299. */
#define ADS1299_SPI_DIN_PIN            16
/*! GPIO controlling ADS1299 START conversion signal. */
#define ADS1299_START_PIN              10

/* I2C Pins */
/*! Shared I2C clock pin used by board-level sensor bus. */
#define SENSOR_I2C_SCL_PIN             20
/*! Shared I2C data pin used by board-level sensor bus. */
#define SENSOR_I2C_SDA_PIN             19
/*! I2C clock pin connected to the ICM20649 sensor. */
#define ICM20649_SCL_PIN               SENSOR_I2C_SCL_PIN
/*! I2C data pin connected to the ICM20649 sensor. */
#define ICM20649_SDA_PIN               SENSOR_I2C_SDA_PIN
/*! GPIO interrupt pin used by ICM20649 for event signaling. */
#define ICM20649_INT_PIN               6
/*! I2C clock pin connected to the MAX30102 sensor. */
#define MAX30102_SCL_PIN               SENSOR_I2C_SCL_PIN
/*! I2C data pin connected to the MAX30102 sensor. */
#define MAX30102_SDA_PIN               SENSOR_I2C_SDA_PIN
/*! GPIO interrupt pin used by MAX30102 for FIFO ready signaling. */
#define MAX30102_INT_PIN               22
/*! MAX30102 7-bit I2C address. */
#define MAX30102_I2C_ADDRESS           0x57u
/*! Default MAX30102 red LED current register value. */
#define MAX30102_LED_RED_DEFAULT       0xC6u
/*! Default MAX30102 IR LED current register value. */
#define MAX30102_LED_IR_DEFAULT        0xC6u

/* DEBUG MODE */
/*! Enables debug-specific behavior/logging when set to nonzero. */
#define DEBUG                          1
/*! Sentinel value used for diagnostics and memory corruption checks. */
#define DEAD_BEEF                      0xDEADBEEFu

/* Macro checks */
#if (NUM_OF_ADS1299 > 2 || NUM_OF_ADS1299 < 1)
#error "Number of ADS1299 devices should be 1 or 2";
#endif /* #ifdef (ICM20649 && ICM20948) */

#endif // USR_CONFIG_
