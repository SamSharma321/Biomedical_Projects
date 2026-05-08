/* ICM40609 driver for nRF5 TWI (I2C). */
#include "icm40609.h"
#include "icm40609_config_5ms.h"

#include <string.h>

#include "app_error.h"
#include "nrf_delay.h"
#include "nrf_error.h"
#include "nrf_log.h"
#include "usr_config.h"

#define ICM40609_BANK_SEL_MASK  0x07u
#define ICM40609_MAX_XFER       64u
#define icm40609_reg_bank(reg)  ((uint8_t)((reg >> 8) & 0xFFu))
#define icm40609_reg_addr(reg)  ((uint8_t)(reg & 0xFFu))
#define CHK_FN_ERR_DATA(err)    do { if ((err) != NRF_SUCCESS) return (err); } while (0)

nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);
icm40609_t m_icm40609[NUM_OF_ICM_DEVICES] = {0};

static ret_code_t icm40609_init_chk_reg_data(icm40609_t *dev);

#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
_Static_assert(sizeof(icm40609_vec3_t) == 6u, "icm40609_vec3_t must be 6 bytes");
_Static_assert(sizeof(int16_t) == 2u, "int16_t must be 2 bytes");
#endif

#if defined(__BYTE_ORDER__) && (__BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__)
#error "icm40609.c assumes a little-endian target."
#endif

static void icm40609_copy_i16_from_sensor_bytes(int16_t *dst, uint8_t const *src_be)
{
    uint8_t *dst_bytes = (uint8_t *)dst;
    dst_bytes[0] = src_be[1];
    dst_bytes[1] = src_be[0];
}

static void icm40609_copy_vec3_from_sensor_bytes(icm40609_vec3_t *dst, uint8_t const *src_be)
{
    uint8_t *dst_bytes = (uint8_t *)dst;
    dst_bytes[0] = src_be[1];
    dst_bytes[1] = src_be[0];
    dst_bytes[2] = src_be[3];
    dst_bytes[3] = src_be[2];
    dst_bytes[4] = src_be[5];
    dst_bytes[5] = src_be[4];
}

void twi_init(void)
{
    const nrf_drv_twi_config_t twi_icm_config = {
        .scl = ICM40609_SCL_PIN,
        .sda = ICM40609_SDA_PIN,
        .frequency = NRF_DRV_TWI_FREQ_100K,
        .interrupt_priority = _PRIO_APP_MID,
        .clear_bus_init = false
    };

    ret_code_t err_code = nrf_drv_twi_init(&m_twi, &twi_icm_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);
    nrf_drv_twi_enable(&m_twi);
    NRF_LOG_INFO("ICM40609: TWI interface initialized.");
}

void twi_uninit(void)
{
    nrf_drv_twi_uninit(&m_twi);
    NRF_LOG_INFO("ICM40609: TWI interface uninitialized.");
}

ret_code_t icm40609_select_bank(icm40609_t *dev, uint8_t bank)
{
    uint8_t buf[2];

    if (dev == NULL)
    {
        return NRF_ERROR_NULL;
    }
    if (bank > 7u)
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    if (dev->current_bank == bank)
    {
        return NRF_SUCCESS;
    }

    buf[0] = ICM40609_REG_BANK_SEL;
    buf[1] = (uint8_t)(bank & ICM40609_BANK_SEL_MASK);
    ret_code_t err = nrf_drv_twi_tx(dev->twi, dev->address, buf, sizeof(buf), false);
    if (err == NRF_SUCCESS)
    {
        dev->current_bank = bank;
    }
    return err;
}

ret_code_t icm40609_write_reg(icm40609_t *dev, icm40609_reg_t reg, uint8_t value)
{
    return icm40609_write_regs(dev, reg, &value, 1u);
}

ret_code_t icm40609_write_regs(icm40609_t *dev, icm40609_reg_t reg, const uint8_t *data, size_t len)
{
    uint8_t buf[ICM40609_MAX_XFER + 1u];

    if ((dev == NULL) || (data == NULL))
    {
        return NRF_ERROR_NULL;
    }
    if (len == 0u)
    {
        return NRF_SUCCESS;
    }
    if (len > ICM40609_MAX_XFER)
    {
        return NRF_ERROR_INVALID_LENGTH;
    }

    ret_code_t err = icm40609_select_bank(dev, icm40609_reg_bank(reg));
    CHK_FN_ERR_DATA(err);

    buf[0] = icm40609_reg_addr(reg);
    memcpy(&buf[1], data, len);
    return nrf_drv_twi_tx(dev->twi, dev->address, buf, len + 1u, false);
}

ret_code_t icm40609_read_reg(icm40609_t *dev, icm40609_reg_t reg, uint8_t *value)
{
    return icm40609_read_regs(dev, reg, value, 1u);
}

ret_code_t icm40609_read_regs(icm40609_t *dev, icm40609_reg_t reg, uint8_t *data, size_t len)
{
    uint8_t addr;

    if ((dev == NULL) || (data == NULL))
    {
        return NRF_ERROR_NULL;
    }
    if (len == 0u)
    {
        return NRF_SUCCESS;
    }

    ret_code_t err = icm40609_select_bank(dev, icm40609_reg_bank(reg));
    CHK_FN_ERR_DATA(err);

    addr = icm40609_reg_addr(reg);
    err = nrf_drv_twi_tx(dev->twi, dev->address, &addr, 1u, true);
    CHK_FN_ERR_DATA(err);

    return nrf_drv_twi_rx(dev->twi, dev->address, data, len);
}

ret_code_t icm40609_update_reg(icm40609_t *dev, icm40609_reg_t reg, uint8_t mask, uint8_t value)
{
    uint8_t reg_val = 0u;
    ret_code_t err = icm40609_read_reg(dev, reg, &reg_val);
    CHK_FN_ERR_DATA(err);

    reg_val = (uint8_t)((reg_val & ~mask) | (value & mask));
    return icm40609_write_reg(dev, reg, reg_val);
}

ret_code_t icm40609_read_accel(icm40609_t *dev, icm40609_vec3_t *accel)
{
    uint8_t raw[6];
    ret_code_t err;

    if (accel == NULL)
    {
        return NRF_ERROR_NULL;
    }

    err = icm40609_read_regs(dev, ICM40609_REG_ACCEL_DATA_X1, raw, sizeof(raw));
    CHK_FN_ERR_DATA(err);

    icm40609_copy_vec3_from_sensor_bytes(accel, raw);
    return NRF_SUCCESS;
}

ret_code_t icm40609_read_gyro(icm40609_t *dev, icm40609_vec3_t *gyro)
{
    uint8_t raw[6];
    ret_code_t err;

    if (gyro == NULL)
    {
        return NRF_ERROR_NULL;
    }

    err = icm40609_read_regs(dev, ICM40609_REG_GYRO_DATA_X1, raw, sizeof(raw));
    CHK_FN_ERR_DATA(err);

    icm40609_copy_vec3_from_sensor_bytes(gyro, raw);
    return NRF_SUCCESS;
}

ret_code_t icm40609_read_temp(icm40609_t *dev, int16_t *temp)
{
    uint8_t raw[2];
    ret_code_t err;

    if (temp == NULL)
    {
        return NRF_ERROR_NULL;
    }

    err = icm40609_read_regs(dev, ICM40609_REG_TEMP_DATA1, raw, sizeof(raw));
    CHK_FN_ERR_DATA(err);

    icm40609_copy_i16_from_sensor_bytes(temp, raw);
    return NRF_SUCCESS;
}

ret_code_t icm40609_read_sample(icm40609_t *dev, uint8_t *raw_sample, icm40609_sample_t *sample)
{
    uint8_t raw_sensor[ICM40609_SAMPLE_RAW_LEN];

    if ((dev == NULL) || (raw_sample == NULL))
    {
        return NRF_ERROR_NULL;
    }

    ret_code_t err = icm40609_read_regs(dev, ICM40609_REG_TEMP_DATA1, raw_sensor, sizeof(raw_sensor));
    CHK_FN_ERR_DATA(err);

    /* Sensor register order is: TEMP (2), ACCEL (6), GYRO (6).
     * This project expects the raw payload order: ACCEL (6), GYRO (6), TEMP (2).
     */
    memcpy(&raw_sample[0],  &raw_sensor[2],  6u);
    memcpy(&raw_sample[6],  &raw_sensor[8],  6u);
    memcpy(&raw_sample[12], &raw_sensor[0],  2u);

    if (sample != NULL)
    {
        icm40609_copy_vec3_from_sensor_bytes(&sample->accel, &raw_sample[0]);
        icm40609_copy_vec3_from_sensor_bytes(&sample->gyro, &raw_sample[6]);
        icm40609_copy_i16_from_sensor_bytes(&sample->temp, &raw_sample[12]);
    }

    return NRF_SUCCESS;
}

ret_code_t icm40609_init(icm40609_t *dev, nrf_drv_twi_t const *twi, uint8_t address)
{
    if ((dev == NULL) || (twi == NULL))
    {
        return NRF_ERROR_NULL;
    }

    dev->twi = twi;
    dev->address = address;
    dev->current_bank = 0xFFu;

    ret_code_t err = icm40609_select_bank(dev, 0u);
    CHK_FN_ERR_DATA(err);

    return icm40609_init_chk_reg_data(dev);
}

static ret_code_t icm40609_init_chk_reg_data(icm40609_t *dev)
{
    uint8_t data_holder = 0u;
    ret_code_t err;

    /* Reset device. */
    err = icm40609_write_reg(dev, ICM40609_REG_DEVICE_CONFIG, ICM40609_DEVICE_CONFIG_SOFT_RESET);
    CHK_FN_ERR_DATA(err);
    nrf_delay_ms(10);

    dev->current_bank = 0xFFu;
    err = icm40609_select_bank(dev, 0u);
    CHK_FN_ERR_DATA(err);

    /* Verify device identity. */
    err = icm40609_read_reg(dev, ICM40609_REG_WHO_AM_I, &data_holder);
    CHK_FN_ERR_DATA(err);
    if (data_holder != ICM40609_WHO_AM_I_VAL)
    {
        NRF_LOG_ERROR("ICM40609 WHO_AM_I mismatch: got 0x%02X exp 0x%02X", data_holder, ICM40609_WHO_AM_I_VAL);
        return NRF_ERROR_INVALID_DATA;
    }

    /* Power up sensors and configure ODR/FS. */
    err = icm40609_write_reg(dev, ICM40609_REG_PWR_MGMT0, ICM40609_PWR_MGMT0_ACCEL_GYRO_ON);
    CHK_FN_ERR_DATA(err);
    nrf_delay_ms(1);

    err = icm40609_write_reg(dev, ICM40609_REG_GYRO_CONFIG0, ICM40609_GYRO_CONFIG0_VAL_200HZ_1000DPS);
    CHK_FN_ERR_DATA(err);

    err = icm40609_write_reg(dev, ICM40609_REG_ACCEL_CONFIG0, ICM40609_ACCEL_CONFIG0_VAL_200HZ_4G);
    CHK_FN_ERR_DATA(err);

    /* Verify. */
    err = icm40609_read_reg(dev, ICM40609_REG_PWR_MGMT0, &data_holder);
    CHK_FN_ERR_DATA(err);
    if (data_holder != ICM40609_PWR_MGMT0_ACCEL_GYRO_ON)
    {
        NRF_LOG_ERROR("ICM40609 PWR_MGMT0 mismatch: got 0x%02X exp 0x%02X",
                      data_holder, ICM40609_PWR_MGMT0_ACCEL_GYRO_ON);
        return NRF_ERROR_INVALID_DATA;
    }

    err = icm40609_read_reg(dev, ICM40609_REG_GYRO_CONFIG0, &data_holder);
    CHK_FN_ERR_DATA(err);
    if (data_holder != ICM40609_GYRO_CONFIG0_VAL_200HZ_1000DPS)
    {
        NRF_LOG_ERROR("ICM40609 GYRO_CONFIG0 mismatch: got 0x%02X exp 0x%02X",
                      data_holder, ICM40609_GYRO_CONFIG0_VAL_200HZ_1000DPS);
        return NRF_ERROR_INVALID_DATA;
    }

    err = icm40609_read_reg(dev, ICM40609_REG_ACCEL_CONFIG0, &data_holder);
    CHK_FN_ERR_DATA(err);
    if (data_holder != ICM40609_ACCEL_CONFIG0_VAL_200HZ_4G)
    {
        NRF_LOG_ERROR("ICM40609 ACCEL_CONFIG0 mismatch: got 0x%02X exp 0x%02X",
                      data_holder, ICM40609_ACCEL_CONFIG0_VAL_200HZ_4G);
        return NRF_ERROR_INVALID_DATA;
    }

    NRF_LOG_INFO("ICM40609 initialization check passed.");
    return NRF_SUCCESS;
}
