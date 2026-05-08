#ifndef USR_CONFIG_
/*! Include guard to prevent multiple inclusion of this config header. */
#define USR_CONFIG_

/* Common includes */
#include <stdbool.h>

/* Device Name and other parameters */
/*! Default BLE device name used in advertising packets. */
#define DEVICE_NAME                     "nRF_ECG_IMU_Sensor"
/*! BLE device name variant for 500 SPS mode/profile. */
#define DEVICE_NAME_500                 "EEG500PPG"
/*! BLE device name variant for 1 kSPS mode/profile. */
#define DEVICE_NAME_1k                  "EEG1000PPG"
/*! BLE device name variant for 2 kSPS mode/profile. */
#define DEVICE_NAME_2k                  "EEG2000PPG"
/*! BLE device name variant for 4 kSPS mode/profile. */
#define DEVICE_NAME_4k                  "EEG4000PPG"
/*! Manufacturer string exposed through the BLE DIS service. */
#define MANUFACTURER_NAME               "BITN Lab"

/* Predefined Macros according to user configuration */
/*! Model number string reported by BLE Device Information Service. */
#define DEVICE_MODEL_NUMBERSTR          "Version 1.0"
/*! Firmware revision string reported by BLE Device Information Service. */
#define DEVICE_FIRMWARE_STRING          "Version 1.0"

/* ADS1299 Related Macros*/
/*! Compile-time flag enabling ADS1299-related code paths. */
#define ADS1299
/*! Number of ADS1299 devices populated and used at runtime. */
#define NUM_OF_ADS1299                 2
/*! Total ADS channel count across all active ADS1299 devices. */
#define NUM_OF_ADS_CH                  9

/*! GPIO used to drive ADS1299 power-down/reset control. */
#define ADS1299_PWDN_RST_PIN           15
/*! GPIO interrupt pin for ADS1299 device 1 data-ready signal. */
#define ADS1299_1_DRDY_PIN             11
/*! GPIO interrupt pin for ADS1299 device 2 data-ready signal. */
#define ADS1299_2_DRDY_PIN             8

/* ICM20x related macros*/


/*! Compile-time flag enabling ICM20948-related code paths. */
#define ICM20948

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
/*! I2C clock pin connected to the ICM20948 sensor. */
#ifndef ICM20948_SCL_PIN
#define ICM20948_SCL_PIN               6
#endif

/*! I2C data pin connected to the ICM20948 sensor. */
#ifndef ICM20948_SDA_PIN
#define ICM20948_SDA_PIN               5
#endif

/*! GPIO interrupt pin used by ICM20948 for event signaling. */
#ifndef ICM20948_INT_PIN
#define ICM20948_INT_PIN               7
#endif

/*! Sentinel value used for diagnostics and memory corruption checks. */
#define DEAD_BEEF                      0xDEADBEEFu

/* Macro checks */
#if (NUM_OF_ADS1299 > 2 || NUM_OF_ADS1299 < 1)
#error "Number of ADS1299 devices should be 1 or 2";
#endif /* #ifdef (ICM20948 && ICM20948) */

#endif // USR_CONFIG_
