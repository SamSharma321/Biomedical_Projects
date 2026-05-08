/* ICM20649 register definitions, data types, and driver API. */
#ifndef ICM20649_H
#define ICM20649_H                        
#include <stddef.h>
#include <stdint.h>

#include "nrf_drv_twi.h"
#include "usr_config.h"
#include "sdk_errors.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* Register IDs encode bank in bits 8:7 and register address in bits 6:0. */
typedef uint16_t icm20649_reg_t;
#define ICM20649_I2C_ADDR0                0x68u /* AD0 = 0 */
#define ICM20649_I2C_ADDR1                0x69u /* AD0 = 1 */
/* TWI instance ID used by this project for ICM20649. */
#define TWI_INSTANCE_ID                   1
/* Expected value from WHO_AM_I register. */
#define ICM20649_WHO_AM_I_VALUE           0xE1u
#define ICM20649_SAMPLE_RAW_LEN           14u /* accel(6) + gyro(6) + temp(2). */
/* Register used to switch the active user bank. */
#define ICM20649_REG_BANK_SEL             0x7Fu
/* Bank 0 registers. */
#define ICM20649_REG_WHO_AM_I             0x0000u
#define ICM20649_REG_USER_CTRL            0x0003u
#define ICM20649_REG_LP_CONFIG            0x0005u
#define ICM20649_REG_PWR_MGMT_1           0x0006u
#define ICM20649_REG_PWR_MGMT_2           0x0007u
#define ICM20649_REG_INT_PIN_CFG          0x000Fu
#define ICM20649_REG_INT_ENABLE           0x0010u
#define ICM20649_REG_INT_ENABLE_1         0x0011u
#define ICM20649_REG_INT_ENABLE_2         0x0012u
#define ICM20649_REG_INT_ENABLE_3         0x0013u
#define ICM20649_REG_INT_STATUS           0x0019u
#define ICM20649_REG_INT_STATUS_1         0x001Au
#define ICM20649_REG_INT_STATUS_2         0x001Bu
#define ICM20649_REG_ACCEL_XOUT_H_SH      0x002Du
#define ICM20649_REG_ACCEL_XOUT_L_SH      0x002Eu
#define ICM20649_REG_ACCEL_YOUT_H_SH      0x002Fu
#define ICM20649_REG_ACCEL_YOUT_L_SH      0x0030u
#define ICM20649_REG_ACCEL_ZOUT_H_SH      0x0031u
#define ICM20649_REG_ACCEL_ZOUT_L_SH      0x0032u
#define ICM20649_REG_GYRO_XOUT_H_SH       0x0033u
#define ICM20649_REG_GYRO_XOUT_L_SH       0x0034u
#define ICM20649_REG_GYRO_YOUT_H_SH       0x0035u
#define ICM20649_REG_GYRO_YOUT_L_SH       0x0036u
#define ICM20649_REG_GYRO_ZOUT_H_SH       0x0037u
#define ICM20649_REG_GYRO_ZOUT_L_SH       0x0038u
#define ICM20649_REG_TEMPERATURE_H        0x0039u
#define ICM20649_REG_TEMPERATURE_L        0x003Au
#define ICM20649_REG_EXT_SLV_SENS_DATA_00 0x003Bu
#define ICM20649_REG_TEMP_CONFIG          0x0053u
#define ICM20649_REG_FIFO_EN_1            0x0066u
#define ICM20649_REG_FIFO_EN_2            0x0067u
#define ICM20649_REG_FIFO_RST             0x0068u
#define ICM20649_REG_FIFO_MODE            0x0069u
#define ICM20649_REG_FIFO_COUNT_H         0x0070u
#define ICM20649_REG_FIFO_COUNT_L         0x0071u
#define ICM20649_REG_FIFO_R_W             0x0072u
#define ICM20649_REG_DATA_RDY_STATUS      0x0074u
#define ICM20649_REG_FIFO_CFG             0x0076u
#define ICM20649_REG_XA_OFFSET_H          0x0094u
#define ICM20649_REG_XA_OFFSET_L          0x0095u
#define ICM20649_REG_YA_OFFSET_H          0x0097u
#define ICM20649_REG_YA_OFFSET_L          0x0098u
#define ICM20649_REG_ZA_OFFSET_H          0x009Au
#define ICM20649_REG_ZA_OFFSET_L          0x009Bu
#define ICM20649_REG_TIMEBASE_CORR_PLL    0x00A8u
/* Bank 1 registers. */
#define ICM20649_REG_GYRO_SMPLRT_DIV      0x0100u
#define ICM20649_REG_GYRO_CONFIG_1        0x0101u
#define ICM20649_REG_GYRO_CONFIG_2        0x0102u
#define ICM20649_REG_XG_OFFS_USRH         0x0103u
#define ICM20649_REG_XG_OFFS_USRL         0x0104u
#define ICM20649_REG_YG_OFFS_USRH         0x0105u
#define ICM20649_REG_YG_OFFS_USRL         0x0106u
#define ICM20649_REG_ZG_OFFS_USRH         0x0107u
#define ICM20649_REG_ZG_OFFS_USRL         0x0108u
#define ICM20649_REG_ODR_ALIGN_EN         0x0109u
#define ICM20649_REG_ACCEL_SMPLRT_DIV_1   0x0110u
#define ICM20649_REG_ACCEL_SMPLRT_DIV_2   0x0111u
#define ICM20649_REG_ACCEL_INTEL_CTRL     0x0112u
#define ICM20649_REG_ACCEL_WOM_THR        0x0113u
#define ICM20649_REG_ACCEL_CONFIG         0x0114u
#define ICM20649_REG_ACCEL_CONFIG_2       0x0115u
/* Bank 3 registers. */
#define ICM20649_REG_I2C_MST_ODR_CONFIG   0x0180u
#define ICM20649_REG_I2C_MST_CTRL         0x0181u
#define ICM20649_REG_I2C_MST_DELAY_CTRL   0x0182u
#define ICM20649_REG_I2C_SLV0_ADDR        0x0183u
#define ICM20649_REG_I2C_SLV0_REG         0x0184u
#define ICM20649_REG_I2C_SLV0_CTRL        0x0185u
#define ICM20649_REG_I2C_SLV0_DO          0x0186u
#define ICM20649_REG_I2C_SLV1_ADDR        0x0187u
#define ICM20649_REG_I2C_SLV1_REG         0x0188u
#define ICM20649_REG_I2C_SLV1_CTRL        0x0189u
#define ICM20649_REG_I2C_SLV1_DO          0x018Au
#define ICM20649_REG_I2C_SLV2_ADDR        0x018Bu
#define ICM20649_REG_I2C_SLV2_REG         0x018Cu
#define ICM20649_REG_I2C_SLV2_CTRL        0x018Du
#define ICM20649_REG_I2C_SLV2_DO          0x018Eu
#define ICM20649_REG_I2C_SLV3_ADDR        0x018Fu
#define ICM20649_REG_I2C_SLV3_REG         0x0190u
#define ICM20649_REG_I2C_SLV3_CTRL        0x0191u
#define ICM20649_REG_I2C_SLV3_DO          0x0192u
#define ICM20649_REG_I2C_SLV4_ADDR        0x0193u
#define ICM20649_REG_I2C_SLV4_REG         0x0194u
#define ICM20649_REG_I2C_SLV4_CTRL        0x0195u
#define ICM20649_REG_I2C_SLV4_DO          0x0196u
#define ICM20649_REG_I2C_SLV4_DI          0x0197u

/*! Holds the info for each ICM20649 device connected. */
typedef struct
{
    /* Bound Nordic TWI instance used for bus transfers. */
    nrf_drv_twi_t const *twi;
    /* 7-bit I2C address of the sensor. */
    uint8_t address;
    /* Cached active bank to avoid redundant bank-select writes. */
    uint8_t current_bank;
} icm20649_t;

/* Signed 3-axis vector container (raw sensor units). */
typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} icm20649_vec3_t;

/* Combined accel/gyro/temp sample decoded from 14 raw bytes. */
typedef struct
{
    icm20649_vec3_t accel;
    icm20649_vec3_t gyro;
    int16_t temp;
} icm20649_sample_t;

/* Convenience globals. */
extern nrf_drv_twi_t m_twi;
extern icm20649_t m_icm20649[NUM_OF_ICM_DEVICES];

/* Driver API.
 * Functions return `NRF_SUCCESS` on success or a Nordic SDK error code.
 */
/* Initialize/uninitialize the global TWI peripheral used by the driver. */
void twi_init(void);
void twi_uninit(void);
/* Initialize a device context and verify sensor presence. */
ret_code_t icm20649_init(icm20649_t *dev, nrf_drv_twi_t const *twi, uint8_t address);
/* Select active register bank (0..3). */
ret_code_t icm20649_select_bank(icm20649_t *dev, uint8_t bank);
/* Single/multi-byte register write helpers. */
ret_code_t icm20649_write_reg(icm20649_t *dev, icm20649_reg_t reg, uint8_t value);
ret_code_t icm20649_write_regs(icm20649_t *dev, icm20649_reg_t reg, const uint8_t *data, size_t len);
/* Single/multi-byte register read helpers. */
ret_code_t icm20649_read_reg(icm20649_t *dev, icm20649_reg_t reg, uint8_t *value);
ret_code_t icm20649_read_regs(icm20649_t *dev, icm20649_reg_t reg, uint8_t *data, size_t len);
/* Read-modify-write utility: update bits selected by `mask`. */
ret_code_t icm20649_update_reg(icm20649_t *dev, icm20649_reg_t reg, uint8_t mask, uint8_t value);
/* Sensor data convenience readers (raw register units). */
ret_code_t icm20649_read_accel(icm20649_t *dev, icm20649_vec3_t *accel);
ret_code_t icm20649_read_gyro(icm20649_t *dev, icm20649_vec3_t *gyro);
ret_code_t icm20649_read_temp(icm20649_t *dev, int16_t *temp);
/* Read raw sample bytes and optionally decode into `icm20649_sample_t`. */
ret_code_t icm20649_read_sample(icm20649_t *dev, uint8_t *raw_sample, icm20649_sample_t *sample);

#ifdef __cplusplus
}
#endif

#endif /* ICM20649_H */
