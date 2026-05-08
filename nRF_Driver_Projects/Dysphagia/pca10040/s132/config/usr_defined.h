/* Created By: Sameera Sharma for Dysphagia Project with BITN */
#ifndef USR_DEFINED_H_
#define USR_DEFINED_H_

#define DEVICE_NAME                 "IMU_MIC_Sensor"    /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME           "BITN Lab"          /**< Manufacturer. Will be passed to Device Information Service. */

#define ADS1292                     true

/* ADS1291_2 related macros */
#define ADS1291_2_PWDN_PIN          16                  /**< ADS1291 power-down/reset pin - A3 on Arduino */
#define ADS1291_2_DRDY_PIN          11                  /**< ADS1291 data ready interrupt pin - D3 on Arduino */
#define ADS1291_2_SCK_PIN           13
#define ADS1291_2_MOSI_PIN          14
#define ADS1291_2_MISO_PIN          12
#define ADS1291_2_SS_PIN            15

// Indicate whether ICM20948 present
#define ICM20948_PRESENT            true

/* Convenience: this driver supports up to 2 devices on the same I2C bus. */
#ifndef NUM_ICM_DEVICES
#define NUM_ICM_DEVICES             2u
#endif

/* User defined Macros */
#ifndef ICM20948_SCL_PIN
#define ICM20948_SCL_PIN            25u                   /*!< ICM20948 TWI SCL pin */
#endif

#ifndef ICM20948_SDA_PIN
#define ICM20948_SDA_PIN            26u                   /*!< ICM20948 TWI SDA pin */
#endif


#endif /* #ifndef USR_DEFINED_H_ */
