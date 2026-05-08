/* ICM40609 register definitions, data types, and driver API. */
#ifndef ICM40609_H
#define ICM40609_H

#include <stddef.h>
#include <stdint.h>

#include "nrf_drv_twi.h"
#include "sdk_errors.h"
#include "usr_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Register IDs encode bank in bits 15:8 and register address in bits 7:0. */
typedef uint16_t icm40609_reg_t;

#define ICM40609_I2C_ADDR0        0x68u /* AD0 = 0 */
#define ICM40609_I2C_ADDR1        0x69u /* AD0 = 1 */

/* TWI instance ID used by this project for the IMU. */
#define TWI_INSTANCE_ID           1

/* Expected value from WHO_AM_I register. */
#define ICM40609_WHO_AM_I_VALUE   0x3Bu

/* Raw sample length expected by this project: accel(6) + gyro(6) + temp(2). */
#define ICM40609_SAMPLE_RAW_LEN   14u

/* Bank select register (BANK_SEL). */
#define ICM40609_REG_BANK_SEL             0x76u

/* Bank 0 registers used by this project. */
#define ICM40609_REG_DEVICE_CONFIG        0x0011u
#define ICM40609_REG_TEMP_DATA1           0x001Du
#define ICM40609_REG_ACCEL_DATA_X1        0x001Fu
#define ICM40609_REG_GYRO_DATA_X1         0x0025u
#define ICM40609_REG_PWR_MGMT0            0x004Eu
#define ICM40609_REG_GYRO_CONFIG0         0x004Fu
#define ICM40609_REG_ACCEL_CONFIG0        0x0050u
#define ICM40609_REG_WHO_AM_I             0x0075u

/*! Holds the info for each ICM40609 device connected. */
typedef struct
{
    /* Bound Nordic TWI instance used for bus transfers. */
    nrf_drv_twi_t const *twi;
    /* 7-bit I2C address of the sensor. */
    uint8_t address;
    /* Cached active bank to avoid redundant bank-select writes. */
    uint8_t current_bank;
} icm40609_t;

/* Signed 3-axis vector container (raw sensor units). */
typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} icm40609_vec3_t;

/* Combined accel/gyro/temp sample decoded from 14 raw bytes. */
typedef struct
{
    icm40609_vec3_t accel;
    icm40609_vec3_t gyro;
    int16_t temp;
} icm40609_sample_t;

/* Convenience globals. */
extern nrf_drv_twi_t m_twi;
extern icm40609_t m_icm40609[NUM_OF_ICM_DEVICES];

/* Initialize/uninitialize the global TWI peripheral used by the driver. */
void twi_init(void);
void twi_uninit(void);

/* Driver API.
 * Functions return `NRF_SUCCESS` on success or a Nordic SDK error code.
 */
ret_code_t icm40609_init(icm40609_t *dev, nrf_drv_twi_t const *twi, uint8_t address);
ret_code_t icm40609_select_bank(icm40609_t *dev, uint8_t bank);
ret_code_t icm40609_write_reg(icm40609_t *dev, icm40609_reg_t reg, uint8_t value);
ret_code_t icm40609_write_regs(icm40609_t *dev, icm40609_reg_t reg, const uint8_t *data, size_t len);
ret_code_t icm40609_read_reg(icm40609_t *dev, icm40609_reg_t reg, uint8_t *value);
ret_code_t icm40609_read_regs(icm40609_t *dev, icm40609_reg_t reg, uint8_t *data, size_t len);
ret_code_t icm40609_update_reg(icm40609_t *dev, icm40609_reg_t reg, uint8_t mask, uint8_t value);
ret_code_t icm40609_read_accel(icm40609_t *dev, icm40609_vec3_t *accel);
ret_code_t icm40609_read_gyro(icm40609_t *dev, icm40609_vec3_t *gyro);
ret_code_t icm40609_read_temp(icm40609_t *dev, int16_t *temp);
ret_code_t icm40609_read_sample(icm40609_t *dev, uint8_t *raw_sample, icm40609_sample_t *sample);

#ifdef __cplusplus
}
#endif

#endif /* ICM40609_H */

