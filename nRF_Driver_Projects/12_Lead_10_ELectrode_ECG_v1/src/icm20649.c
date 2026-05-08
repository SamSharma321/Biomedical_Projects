/* ICM20649 driver for nRF5 TWI (I2C). */
#include "icm20649.h"
#include "icm20649_config_5ms.h"
#include <string.h>
#include "app_error.h"
#include "nrf_error.h"
#include "nrf_log.h"
#include "usr_config.h"
#define ICM20649_BANK_SEL_SHIFT 4u
#define ICM20649_BANK_SEL_MASK  0x30u
#define ICM20649_MAX_XFER       64u
#define icm20649_reg_bank(reg)  ((uint8_t)((reg >> 7) & 0x03u))
#define icm20649_reg_addr(reg)  ((uint8_t)(reg & 0x7Fu))
#define CHK_FN_ERR_DATA(err)    do { if ((err) != NRF_SUCCESS) return (err); } while (0)
#define VERIFY_DATA(data,       val) do { if ((data) != (val)) return NRF_ERROR_INVALID_DATA; } while (0)
nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);
icm20649_t m_icm20649[NUM_OF_ICM_DEVICES] = {0};

static ret_code_t icm20649_init_chk_reg_data(icm20649_t *dev);

#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
_Static_assert(sizeof(icm20649_vec3_t) == 6u, "icm20649_vec3_t must be 6 bytes");
_Static_assert(sizeof(int16_t) == 2u, "int16_t must be 2 bytes");
#endif

#if defined(__BYTE_ORDER__) && (__BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__)
#error "icm20649.c assumes a little-endian target."
#endif

/*! @brief Convert a big-endian 16-bit sensor value into host-endian int16.
 * @param[out] dst     Destination int16 value.
 * @param[in]  src_be  Source bytes in sensor byte order (MSB first).
 * @return None.
 */
static void icm20649_copy_i16_from_sensor_bytes(int16_t *dst, uint8_t const *src_be)
{
    uint8_t *dst_bytes = (uint8_t *)dst;
    dst_bytes[0] = src_be[1];
    dst_bytes[1] = src_be[0];
}

/*! @brief Convert a big-endian XYZ vector payload into host-endian struct fields.
 * @param[out] dst     Destination vector.
 * @param[in]  src_be  Source bytes (6 bytes, MSB first per axis).
 * @return None.
 */
static void icm20649_copy_vec3_from_sensor_bytes(icm20649_vec3_t *dst, uint8_t const *src_be)
{
    uint8_t *dst_bytes = (uint8_t *)dst;
    dst_bytes[0] = src_be[1];
    dst_bytes[1] = src_be[0];
    dst_bytes[2] = src_be[3];
    dst_bytes[3] = src_be[2];
    dst_bytes[4] = src_be[5];
    dst_bytes[5] = src_be[4];
}

/*! @brief Initialize TWI peripheral used by the ICM20649 driver.
 * @return None.
 */
void twi_init(void)
{
    const nrf_drv_twi_config_t twi_icm_config = {
        .scl = ICM20649_SCL_PIN,
        .sda = ICM20649_SDA_PIN,
        .frequency = NRF_DRV_TWI_FREQ_400K,
        .interrupt_priority = _PRIO_APP_MID,
        .clear_bus_init = false
    };

    ret_code_t err_code = nrf_drv_twi_init(&m_twi, &twi_icm_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);
    nrf_drv_twi_enable(&m_twi);
    NRF_LOG_INFO("ICM20649: TWI interface initialized.");
}

/*! @brief Uninitialize TWI peripheral used by the ICM20649 driver.
 * @return None.
 */
void twi_uninit(void)
{
    nrf_drv_twi_uninit(&m_twi);
    NRF_LOG_INFO("ICM20649: TWI interface uninitialized.");
}

/*! @brief Select active register bank on the ICM20649.
 * @param[in,out] dev   ICM device context.
 * @param[in]     bank  Register bank index (0-3).
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
ret_code_t icm20649_select_bank(icm20649_t *dev, uint8_t bank)
{
    uint8_t buf[2];

    if (dev == NULL) {
        return NRF_ERROR_NULL;
    }
    if (bank > 3u) {
        return NRF_ERROR_INVALID_PARAM;
    }
    if (dev->current_bank == bank) {
        return NRF_SUCCESS;
    }

    buf[0] = ICM20649_REG_BANK_SEL;
    buf[1] = (uint8_t)((bank << ICM20649_BANK_SEL_SHIFT) & ICM20649_BANK_SEL_MASK);
    ret_code_t err = nrf_drv_twi_tx(dev->twi, dev->address, buf, sizeof(buf), false);
    if (err == NRF_SUCCESS)
    {
        dev->current_bank = bank;
    }
    return err;
}

/*! @brief Write one register on ICM20649.
 * @param[in,out] dev    ICM device context.
 * @param[in]     reg    Register identifier.
 * @param[in]     value  Register value to write.
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
ret_code_t icm20649_write_reg(icm20649_t *dev, icm20649_reg_t reg, uint8_t value)
{
    return icm20649_write_regs(dev, reg, &value, 1u);
}

/*! @brief Write multiple consecutive registers on ICM20649.
 * @param[in,out] dev   ICM device context.
 * @param[in]     reg   Starting register identifier.
 * @param[in]     data  Pointer to bytes to write.
 * @param[in]     len   Number of bytes to write.
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
ret_code_t icm20649_write_regs(icm20649_t *dev, icm20649_reg_t reg, const uint8_t *data, size_t len)
{
    uint8_t buf[ICM20649_MAX_XFER + 1u];

    if ((dev == NULL) || (data == NULL))
    {
        return NRF_ERROR_NULL;
    }
    if (len == 0u)
    {
        return NRF_SUCCESS;
    }
    if (len > ICM20649_MAX_XFER)
    {
        return NRF_ERROR_INVALID_LENGTH;
    }

    ret_code_t err = icm20649_select_bank(dev, icm20649_reg_bank(reg));
    CHK_FN_ERR_DATA(err);

    buf[0] = icm20649_reg_addr(reg);
    memcpy(&buf[1], data, len);
    return nrf_drv_twi_tx(dev->twi, dev->address, buf, len + 1u, false);
}

/*! @brief Read one register from ICM20649.
 * @param[in,out] dev    ICM device context.
 * @param[in]     reg    Register identifier.
 * @param[out]    value  Output register value.
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
ret_code_t icm20649_read_reg(icm20649_t *dev, icm20649_reg_t reg, uint8_t *value)
{
    return icm20649_read_regs(dev, reg, value, 1u);
}

/*! @brief Read multiple consecutive registers from ICM20649.
 * @param[in,out] dev   ICM device context.
 * @param[in]     reg   Starting register identifier.
 * @param[out]    data  Destination byte buffer.
 * @param[in]     len   Number of bytes to read.
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
ret_code_t icm20649_read_regs(icm20649_t *dev, icm20649_reg_t reg, uint8_t *data, size_t len)
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

    ret_code_t err = icm20649_select_bank(dev, icm20649_reg_bank(reg));
    CHK_FN_ERR_DATA(err);

    addr = icm20649_reg_addr(reg);
    err = nrf_drv_twi_tx(dev->twi, dev->address, &addr, 1u, true);
    CHK_FN_ERR_DATA(err);

    return nrf_drv_twi_rx(dev->twi, dev->address, data, len);
}

/*! @brief Update masked bits in an ICM20649 register.
 * @param[in,out] dev    ICM device context.
 * @param[in]     reg    Register identifier.
 * @param[in]     mask   Bit mask to update.
 * @param[in]     value  New bit values (masked).
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
ret_code_t icm20649_update_reg(icm20649_t *dev, icm20649_reg_t reg, uint8_t mask, uint8_t value)
{
    uint8_t reg_val = 0u;
    ret_code_t err = icm20649_read_reg(dev, reg, &reg_val);
    CHK_FN_ERR_DATA(err);

    reg_val = (uint8_t)((reg_val & ~mask) | (value & mask));
    return icm20649_write_reg(dev, reg, reg_val);
}

/*! @brief Read accelerometer sample from ICM20649.
 * @param[in,out] dev    ICM device context.
 * @param[out]    accel  Output accelerometer vector.
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
ret_code_t icm20649_read_accel(icm20649_t *dev, icm20649_vec3_t *accel)
{
    uint8_t raw[6];
    ret_code_t err;

    if (accel == NULL)
    {
        return NRF_ERROR_NULL;
    }

    err = icm20649_read_regs(dev, ICM20649_REG_ACCEL_XOUT_H_SH, raw, sizeof(raw));
    CHK_FN_ERR_DATA(err);

    icm20649_copy_vec3_from_sensor_bytes(accel, raw);
    return NRF_SUCCESS;
}

/*! @brief Read gyroscope sample from ICM20649.
 * @param[in,out] dev   ICM device context.
 * @param[out]    gyro  Output gyroscope vector.
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
ret_code_t icm20649_read_gyro(icm20649_t *dev, icm20649_vec3_t *gyro)
{
    uint8_t raw[6];
    ret_code_t err;

    if (gyro == NULL)
    {
        return NRF_ERROR_NULL;
    }

    err = icm20649_read_regs(dev, ICM20649_REG_GYRO_XOUT_H_SH, raw, sizeof(raw));
    CHK_FN_ERR_DATA(err);

    icm20649_copy_vec3_from_sensor_bytes(gyro, raw);
    return NRF_SUCCESS;
}

/*! @brief Read temperature sample from ICM20649.
 * @param[in,out] dev   ICM device context.
 * @param[out]    temp  Output temperature sample.
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
ret_code_t icm20649_read_temp(icm20649_t *dev, int16_t *temp)
{
    uint8_t raw[2];
    ret_code_t err;

    if (temp == NULL)
    {
        return NRF_ERROR_NULL;
    }

    err = icm20649_read_regs(dev, ICM20649_REG_TEMPERATURE_H, raw, sizeof(raw));
    CHK_FN_ERR_DATA(err);

    icm20649_copy_i16_from_sensor_bytes(temp, raw);
    return NRF_SUCCESS;
}

/*! @brief Read one full raw IMU sample (accel + gyro + temp).
 * @param[in,out] dev         ICM device context.
 * @param[out]    raw_sample  Raw sample byte buffer (14 bytes).
 * @param[out]    sample      Optional parsed sample structure, may be NULL.
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
ret_code_t icm20649_read_sample(icm20649_t *dev, uint8_t *raw_sample, icm20649_sample_t *sample)
{
    if ((dev == NULL) || (raw_sample == NULL))
    {
        return NRF_ERROR_NULL;
    }

    ret_code_t err = icm20649_read_regs(dev,
                                        ICM20649_REG_ACCEL_XOUT_H_SH,
                                        raw_sample,
                                        ICM20649_SAMPLE_RAW_LEN);
    CHK_FN_ERR_DATA(err);

    if (sample != NULL)
    {
        icm20649_copy_vec3_from_sensor_bytes(&sample->accel, &raw_sample[0]);
        icm20649_copy_vec3_from_sensor_bytes(&sample->gyro, &raw_sample[6]);
        icm20649_copy_i16_from_sensor_bytes(&sample->temp, &raw_sample[12]);
    }

    return NRF_SUCCESS;
}

/*! @brief Initialize ICM20649 context and apply startup register configuration.
 * @param[out]    dev      ICM device context.
 * @param[in]     twi      TWI instance used for communication.
 * @param[in]     address  I2C address of the ICM20649 device.
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
ret_code_t icm20649_init(icm20649_t *dev, nrf_drv_twi_t const *twi, uint8_t address)
{
    if ((dev == NULL) || (twi == NULL))
    {
        return NRF_ERROR_NULL;
    }

    dev->twi = twi;
    dev->address = address;
    dev->current_bank = 0xFFu;

    ret_code_t err = icm20649_select_bank(dev, 0u);
    CHK_FN_ERR_DATA(err);

    return icm20649_init_chk_reg_data(dev);
}

/*! @brief Program and verify ICM20649 startup register set.
 * @param[in,out] dev  ICM device context.
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
static ret_code_t icm20649_init_chk_reg_data(icm20649_t *dev)
{
    uint8_t data_holder = 0u;
    uint8_t status_reg_data = 0u;
    uint8_t status1_reg_data = 0u;
    ret_code_t err;

    /* Bank 0 configuration. */
    err = icm20649_write_reg(dev, ICM20649_REG_USER_CTRL, ICM20649_USER_CTRL_VAL);
    CHK_FN_ERR_DATA(err);
    err = icm20649_write_reg(dev, ICM20649_REG_PWR_MGMT_1, ICM20649_PWR_MGMT_1_NORMAL_OP);
    CHK_FN_ERR_DATA(err);
    err = icm20649_write_reg(dev, ICM20649_REG_PWR_MGMT_2, ICM20649_PWR_MGMT_2_NORMAL_OP);
    CHK_FN_ERR_DATA(err);
    err = icm20649_write_reg(dev, ICM20649_REG_LP_CONFIG, ICM20649_LP_CONFIG_NORMAL_OP);
    CHK_FN_ERR_DATA(err);
    err = icm20649_write_reg(dev, ICM20649_REG_INT_PIN_CFG, ICM20649_INT_PIN_CFG_DATA_RDY);
    CHK_FN_ERR_DATA(err);
    err = icm20649_write_reg(dev, ICM20649_REG_INT_ENABLE, ICM20649_INT_ENABLE_DATA_RDY);
    CHK_FN_ERR_DATA(err);
    err = icm20649_write_reg(dev, ICM20649_REG_INT_ENABLE_1, ICM20649_INT_ENABLE_1_DATA_RDY);
    CHK_FN_ERR_DATA(err);

    /* Verify bank 0. */
    err = icm20649_read_reg(dev, ICM20649_REG_WHO_AM_I, &data_holder);
    CHK_FN_ERR_DATA(err);
    VERIFY_DATA(data_holder, ICM20649_WHO_AM_I_VAL);

    err = icm20649_read_reg(dev, ICM20649_REG_USER_CTRL, &data_holder);
    CHK_FN_ERR_DATA(err);
    VERIFY_DATA(data_holder, ICM20649_USER_CTRL_VAL);

    err = icm20649_read_reg(dev, ICM20649_REG_PWR_MGMT_1, &data_holder);
    CHK_FN_ERR_DATA(err);
    VERIFY_DATA(data_holder, ICM20649_PWR_MGMT_1_NORMAL_OP);

    err = icm20649_read_reg(dev, ICM20649_REG_PWR_MGMT_2, &data_holder);
    CHK_FN_ERR_DATA(err);
    VERIFY_DATA(data_holder, ICM20649_PWR_MGMT_2_NORMAL_OP);

    err = icm20649_read_reg(dev, ICM20649_REG_INT_PIN_CFG, &data_holder);
    CHK_FN_ERR_DATA(err);
    VERIFY_DATA(data_holder, ICM20649_INT_PIN_CFG_DATA_RDY);

    err = icm20649_read_reg(dev, ICM20649_REG_INT_ENABLE, &data_holder);
    CHK_FN_ERR_DATA(err);
    VERIFY_DATA(data_holder, ICM20649_INT_ENABLE_DATA_RDY);

    err = icm20649_read_reg(dev, ICM20649_REG_INT_ENABLE_1, &data_holder);
    CHK_FN_ERR_DATA(err);
    VERIFY_DATA(data_holder, ICM20649_INT_ENABLE_1_DATA_RDY);

    (void)icm20649_read_reg(dev, ICM20649_REG_INT_STATUS, &status_reg_data);
    (void)icm20649_read_reg(dev, ICM20649_REG_INT_STATUS_1, &status1_reg_data);

    /* Bank 1 configuration. */
    err = icm20649_write_reg(dev, ICM20649_REG_GYRO_SMPLRT_DIV, ICM20649_GYRO_SMPLRT_DIV_5MS);
    CHK_FN_ERR_DATA(err);
    err = icm20649_write_reg(dev, ICM20649_REG_GYRO_CONFIG_1, ICM20649_GYRO_CONFIG_1_VAL);
    CHK_FN_ERR_DATA(err);
    err = icm20649_write_reg(dev, ICM20649_REG_GYRO_CONFIG_2, ICM20649_GYRO_CONFIG_2_DEFAULT);
    CHK_FN_ERR_DATA(err);

    uint8_t accel_div[2] = {ICM20649_ACCEL_SMPLRT_DIV_MSB_5MS, ICM20649_ACCEL_SMPLRT_DIV_LSB_5MS};
    err = icm20649_write_regs(dev, ICM20649_REG_ACCEL_SMPLRT_DIV_1, accel_div, sizeof(accel_div));
    CHK_FN_ERR_DATA(err);
    err = icm20649_write_reg(dev, ICM20649_REG_ACCEL_CONFIG, ICM20649_ACCEL_CONFIG_1_VAL);
    CHK_FN_ERR_DATA(err);
    err = icm20649_write_reg(dev, ICM20649_REG_ACCEL_CONFIG_2, ICM20649_ACCEL_CONFIG_2_VAL);
    CHK_FN_ERR_DATA(err);

    /* Verify bank 1. */
    err = icm20649_read_reg(dev, ICM20649_REG_GYRO_SMPLRT_DIV, &data_holder);
    CHK_FN_ERR_DATA(err);
    VERIFY_DATA(data_holder, ICM20649_GYRO_SMPLRT_DIV_5MS);

    err = icm20649_read_reg(dev, ICM20649_REG_GYRO_CONFIG_1, &data_holder);
    CHK_FN_ERR_DATA(err);
    VERIFY_DATA(data_holder, ICM20649_GYRO_CONFIG_1_VAL);

    err = icm20649_read_reg(dev, ICM20649_REG_GYRO_CONFIG_2, &data_holder);
    CHK_FN_ERR_DATA(err);
    VERIFY_DATA(data_holder, ICM20649_GYRO_CONFIG_2_DEFAULT);

    uint8_t accel_div_read[2] = {0u, 0u};
    err = icm20649_read_regs(dev, ICM20649_REG_ACCEL_SMPLRT_DIV_1, accel_div_read, sizeof(accel_div_read));
    CHK_FN_ERR_DATA(err);
    VERIFY_DATA(accel_div_read[0], ICM20649_ACCEL_SMPLRT_DIV_MSB_5MS);
    VERIFY_DATA(accel_div_read[1], ICM20649_ACCEL_SMPLRT_DIV_LSB_5MS);

    err = icm20649_read_reg(dev, ICM20649_REG_ACCEL_CONFIG, &data_holder);
    CHK_FN_ERR_DATA(err);
    VERIFY_DATA(data_holder, ICM20649_ACCEL_CONFIG_1_VAL);

    err = icm20649_read_reg(dev, ICM20649_REG_ACCEL_CONFIG_2, &data_holder);
    CHK_FN_ERR_DATA(err);
    VERIFY_DATA(data_holder, ICM20649_ACCEL_CONFIG_2_VAL);

    NRF_LOG_INFO("ICM20649 initialization check passed.");
    return NRF_SUCCESS;
}
