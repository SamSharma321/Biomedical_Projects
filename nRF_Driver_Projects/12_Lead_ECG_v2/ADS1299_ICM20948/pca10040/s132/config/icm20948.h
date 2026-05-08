/* ICM20948 register definitions and driver API. */
#ifndef ICM20948_H
#define ICM20948_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include "nrf_drv_twi.h"
#include "sdk_errors.h"
#include "usr_config.h"

#ifdef __cplusplus
extern "C"
{
#endif

// Indicate whether ICM20948 present
#define ICM20948_PRESENT true

/* Convenience: this driver supports up to 2 devices on the same I2C bus. */
#ifndef NUM_OF_ICM_DEVICES
#define NUM_OF_ICM_DEVICES 2u
#endif

/* User defined Macros */
#ifndef ICM20948_SCL_PIN
#define ICM20948_SCL_PIN 25u /*!< ICM20948 TWI SCL pin */
#endif

#ifndef ICM20948_SDA_PIN
#define ICM20948_SDA_PIN 26u /*!< ICM20948 TWI SDA pin */
#endif

    /* Register IDs encode the bank in bits 8:7 and the 7-bit address in bits 6:0. */
    typedef uint16_t icm20948_reg_t;

#define ICM20948_I2C_ADDR0 0x68u /* AD0 = 0 */
#define ICM20948_I2C_ADDR1 0x69u /* AD0 = 1 */

#define ICM20948_WHO_AM_I_VAL 0xEAu /* Expected WHO_AM_I response. */

#define ICM20948_REG_BANK_SEL 0x7Fu /* Bank select register (in bank 0). */

/* Bank 0 registers. */
#define ICM20948_REG_WHO_AM_I 0x0000u
#define ICM20948_REG_USER_CTRL 0x0003u
#define ICM20948_REG_LP_CONFIG 0x0005u
#define ICM20948_REG_PWR_MGMT_1 0x0006u
#define ICM20948_REG_PWR_MGMT_2 0x0007u
#define ICM20948_REG_INT_PIN_CFG 0x000Fu
#define ICM20948_REG_INT_ENABLE 0x0010u
#define ICM20948_REG_INT_ENABLE_1 0x0011u
#define ICM20948_REG_INT_ENABLE_2 0x0012u
#define ICM20948_REG_INT_ENABLE_3 0x0013u
#define ICM20948_REG_INT_STATUS 0x0019u
#define ICM20948_REG_INT_STATUS_1 0x001Au
#define ICM20948_REG_INT_STATUS_2 0x001Bu
#define ICM20948_REG_ACCEL_XOUT_H_SH 0x002Du
#define ICM20948_REG_ACCEL_XOUT_L_SH 0x002Eu
#define ICM20948_REG_ACCEL_YOUT_H_SH 0x002Fu
#define ICM20948_REG_ACCEL_YOUT_L_SH 0x0030u
#define ICM20948_REG_ACCEL_ZOUT_H_SH 0x0031u
#define ICM20948_REG_ACCEL_ZOUT_L_SH 0x0032u
#define ICM20948_REG_GYRO_XOUT_H_SH 0x0033u
#define ICM20948_REG_GYRO_XOUT_L_SH 0x0034u
#define ICM20948_REG_GYRO_YOUT_H_SH 0x0035u
#define ICM20948_REG_GYRO_YOUT_L_SH 0x0036u
#define ICM20948_REG_GYRO_ZOUT_H_SH 0x0037u
#define ICM20948_REG_GYRO_ZOUT_L_SH 0x0038u
#define ICM20948_REG_TEMPERATURE_H 0x0039u
#define ICM20948_REG_TEMPERATURE_L 0x003Au
#define ICM20948_REG_EXT_SLV_SENS_DATA_00 0x003Bu
#define ICM20948_REG_TEMP_CONFIG 0x0053u
#define ICM20948_REG_FIFO_EN_1 0x0066u
#define ICM20948_REG_FIFO_EN_2 0x0067u
#define ICM20948_REG_FIFO_RST 0x0068u
#define ICM20948_REG_FIFO_MODE 0x0069u
#define ICM20948_REG_FIFO_COUNT_H 0x0070u
#define ICM20948_REG_FIFO_COUNT_L 0x0071u
#define ICM20948_REG_FIFO_R_W 0x0072u
#define ICM20948_REG_DATA_RDY_STATUS 0x0074u
#define ICM20948_REG_FIFO_CFG 0x0076u

/* Bank 2 registers (gyro/accel configuration). */
#define ICM20948_REG_GYRO_SMPLRT_DIV 0x0100u
#define ICM20948_REG_GYRO_CONFIG_1 0x0101u
#define ICM20948_REG_GYRO_CONFIG_2 0x0102u
#define ICM20948_REG_XG_OFFS_USRH 0x0103u
#define ICM20948_REG_XG_OFFS_USRL 0x0104u
#define ICM20948_REG_YG_OFFS_USRH 0x0105u
#define ICM20948_REG_YG_OFFS_USRL 0x0106u
#define ICM20948_REG_ZG_OFFS_USRH 0x0107u
#define ICM20948_REG_ZG_OFFS_USRL 0x0108u
#define ICM20948_REG_ODR_ALIGN_EN 0x0109u
#define ICM20948_REG_ACCEL_SMPLRT_DIV_1 0x0110u
#define ICM20948_REG_ACCEL_SMPLRT_DIV_2 0x0111u
#define ICM20948_REG_ACCEL_INTEL_CTRL 0x0112u
#define ICM20948_REG_ACCEL_WOM_THR 0x0113u
#define ICM20948_REG_ACCEL_CONFIG 0x0114u
#define ICM20948_REG_ACCEL_CONFIG_2 0x0115u

/* Bank 3 registers (I2C master). */
#define ICM20948_REG_I2C_MST_ODR_CONFIG 0x0180u
#define ICM20948_REG_I2C_MST_CTRL 0x0181u
#define ICM20948_REG_I2C_MST_DELAY_CTRL 0x0182u
#define ICM20948_REG_I2C_SLV0_ADDR 0x0183u
#define ICM20948_REG_I2C_SLV0_REG 0x0184u
#define ICM20948_REG_I2C_SLV0_CTRL 0x0185u
#define ICM20948_REG_I2C_SLV0_DO 0x0186u
#define ICM20948_REG_I2C_SLV1_ADDR 0x0187u
#define ICM20948_REG_I2C_SLV1_REG 0x0188u
#define ICM20948_REG_I2C_SLV1_CTRL 0x0189u
#define ICM20948_REG_I2C_SLV1_DO 0x018Au
#define ICM20948_REG_I2C_SLV2_ADDR 0x018Bu
#define ICM20948_REG_I2C_SLV2_REG 0x018Cu
#define ICM20948_REG_I2C_SLV2_CTRL 0x018Du
#define ICM20948_REG_I2C_SLV2_DO 0x018Eu
#define ICM20948_REG_I2C_SLV3_ADDR 0x018Fu
#define ICM20948_REG_I2C_SLV3_REG 0x0190u
#define ICM20948_REG_I2C_SLV3_CTRL 0x0191u
#define ICM20948_REG_I2C_SLV3_DO 0x0192u
#define ICM20948_REG_I2C_SLV4_ADDR 0x0193u
#define ICM20948_REG_I2C_SLV4_REG 0x0194u
#define ICM20948_REG_I2C_SLV4_CTRL 0x0195u
#define ICM20948_REG_I2C_SLV4_DO 0x0196u
#define ICM20948_REG_I2C_SLV4_DI 0x0197u

/* USER_CTRL bit masks. */
#define ICM20948_USER_CTRL_I2C_MST_EN_MASK 0x20u

    /* Pre-defined Structures */
    /*! @brief: Structure to hold ICM20948 device context */
    typedef struct
    {
        nrf_drv_twi_t const *twi;
        uint8_t address;
        uint8_t current_bank;
    } icm20948_t;

    /*! @brief: Structure to hold 3-axis vector data - for accelerator and gyroscope */
    typedef struct
    {
        int16_t x;
        int16_t y;
        int16_t z;
    } icm20948_vec3_t;

    /*! @brief: Structure to hold a complete sample from the ICM20948 (accel, gyro & temp) */
    typedef struct
    {
        icm20948_vec3_t accel;
        icm20948_vec3_t gyro;
        int16_t temp;
    } icm20948_sample_t;

#define ICM20948_SAMPLE_RAW_LEN 12u

/* ============================================================================
 * Convenience globals
 * ============================================================================
 */
extern icm20948_t m_icm20948[NUM_OF_ICM_DEVICES];
extern nrf_drv_twi_t m_twi;

/* ============================================================================
 * Driver API - Function Prototypes
 * ============================================================================
 */
void icm20948_twi_init(void);
void icm20948_twi_uninit(void);
ret_code_t icm20948_init(icm20948_t *dev, nrf_drv_twi_t* m_twi, uint8_t address);
ret_code_t icm20948_init_auto(nrf_drv_twi_t const *twi,
                              icm20948_t *devs,
                              size_t max_devs,
                              size_t *found_count);
ret_code_t icm20948_select_bank(icm20948_t *dev, uint8_t bank);
ret_code_t icm20948_write_reg(icm20948_t *dev, icm20948_reg_t reg, uint8_t value);
ret_code_t icm20948_write_regs(icm20948_t *dev, icm20948_reg_t reg, const uint8_t *data, size_t len);
ret_code_t icm20948_read_reg(icm20948_t *dev, icm20948_reg_t reg, uint8_t *value);
ret_code_t icm20948_read_regs(icm20948_t *dev, icm20948_reg_t reg, uint8_t *data, size_t len);
ret_code_t icm20948_update_reg(icm20948_t *dev, icm20948_reg_t reg, uint8_t mask, uint8_t value);
ret_code_t icm20948_read_sample(icm20948_t *dev, uint8_t *raw_sample, icm20948_sample_t *sample);

/* Optional feature: enable/disable internal I2C master (required for ICM20948's aux sensors). */
ret_code_t icm20948_i2c_master_enable(icm20948_t *dev, bool enable);

#ifdef __cplusplus
}
#endif

#endif /* ICM20948_H */
