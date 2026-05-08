/* ICM20948 driver for nRF5 TWI (I2C). */
#include "icm20948.h"
#include <string.h>
#include "app_error.h"
#include "nrf_error.h"
#include "nrf_log.h"
//#include "usr_config.h"
#include "icm20948_config.h"

#define ICM20948_BANK_SEL_SHIFT 4u
#define ICM20948_BANK_SEL_MASK 0x30u /*! Mask to select the bank (0 - 3) */
#define ICM20948_MAX_XFER 64u /* Device supports 8-bit address + up to 64 bytes -> No limit mentioned in DS */

/* Function Macros */
#define icm20948_reg_bank(reg) ((uint8_t)((reg >> 7) & 0x03u))
#define icm20948_reg_addr(reg) ((uint8_t)(reg & 0x7Fu))
#define CHK_FN_ERR_DATA(err) if (err != NRF_SUCCESS) return err;
#define VERIFY_DATA(data, val)    if ((data) != (val))  return NRF_ERROR_INVALID_DATA


/* TWI instance used by the ICM20948 driver (convenience). */
#ifndef ICM20948_TWI_INSTANCE_ID
#define ICM20948_TWI_INSTANCE_ID 1 // Default Instance 0 since SPIM might be set on instance 1
#endif

// Initialize TWI instance for ICM20948
nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(ICM20948_TWI_INSTANCE_ID);

/* Convenience device array for storing device instances + info.
    NOTE: up to 2 devices on the I2C bus at max (Addr1 = 0x68 & Addr2 = 0x69). */
icm20948_t m_icm20948[NUM_OF_ICM_DEVICES] = {0};

/* Function Prototypes */
static ret_code_t icm20948_init_chk_reg_data(icm20948_t *dev);

#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
_Static_assert(sizeof(icm20948_vec3_t) == 6u, "icm20948_vec3_t must be 6 bytes");
_Static_assert(sizeof(int16_t) == 2u, "int16_t must be 2 bytes");
#endif

#if defined(__BYTE_ORDER__) && (__BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__)
#error "icm20948.c assumes a little-endian target."
#endif

static void icm20948_copy_i16_from_sensor_bytes(int16_t *dst, uint8_t const *src_be)
{
    uint8_t *dst_bytes = (uint8_t *)dst;
    // Sensor outputs MSB first; memory layout is little-endian.
    dst_bytes[0] = src_be[1];
    dst_bytes[1] = src_be[0];
}

static void icm20948_copy_vec3_from_sensor_bytes(icm20948_vec3_t *dst, uint8_t const *src_be)
{
    uint8_t *dst_bytes = (uint8_t *)dst;
    // X
    dst_bytes[0] = src_be[1];
    dst_bytes[1] = src_be[0];
    // Y
    dst_bytes[2] = src_be[3];
    dst_bytes[3] = src_be[2];
    // Z
    dst_bytes[4] = src_be[5];
    dst_bytes[5] = src_be[4];
}


/* Function Definitions*/
/*! @brief: Initializes the TWI interface for all the ICM20948 devices (according to the set ICM20948_TWI_INSTANCE_ID).
            This needs to be called before any other ICM20948 functions. */
void icm20948_twi_init(void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_icm_config = {
        .scl = ICM20948_SCL_PIN,
        .sda = ICM20948_SDA_PIN,
        .frequency = NRF_DRV_TWI_FREQ_100K,
        .interrupt_priority = _PRIO_APP_MID,
        .clear_bus_init = false,
        .hold_bus_uninit = false
    };
    /* Initialize the TWI interface -> Initialized in non-blocking mode */
    err_code = nrf_drv_twi_init(&m_twi, &twi_icm_config, NULL, NULL);
    /* Enable TWI interface */
    if (!err_code) {
        nrf_drv_twi_enable(&m_twi);
        NRF_LOG_INFO("ICM20948: TWI interface initialized.");
    }
} 

void icm20948_twi_uninit(void) {
  nrf_drv_twi_uninit(&m_twi);
  NRF_LOG_INFO("ICM20948: TWI interface initialized.");
}

/*! @brief: Selects the register bank (to write to specific registers) for the ICM20948 device */
ret_code_t icm20948_select_bank(icm20948_t *dev, uint8_t bank)
{
    uint8_t buf[2];

    if (dev == NULL) {
        return NRF_ERROR_NULL;
    } if (bank > 3u) {
        return NRF_ERROR_INVALID_PARAM;
    } if (dev->current_bank == bank) {
        return NRF_SUCCESS;
    }

    buf[0] = ICM20948_REG_BANK_SEL;
    buf[1] = (uint8_t)((bank << ICM20948_BANK_SEL_SHIFT) & ICM20948_BANK_SEL_MASK);
    ret_code_t err = nrf_drv_twi_tx(dev->twi, dev->address, buf, sizeof(buf), false);
    if (err == NRF_SUCCESS) {
        dev->current_bank = bank;
    }
    return err;
}

/*! @brief: Wrapper function to write a single register */
ret_code_t icm20948_write_reg(icm20948_t *dev, icm20948_reg_t reg, uint8_t value)
{
    return icm20948_write_regs(dev, reg, &value, 1u);
}

/*! @brief: Function to write multiple registers starting from the specified 'reg' register (continuously addessed registers).
    NOTE: All register macros are provided in the icm20948.h file which can be used. */
ret_code_t icm20948_write_regs(icm20948_t *dev, icm20948_reg_t reg, const uint8_t *data, size_t len)
{
    uint8_t buf[ICM20948_MAX_XFER + 1u];

    if ((dev == NULL) || (data == NULL)) {
        return NRF_ERROR_NULL;
    } if (len == 0u) {
        return NRF_SUCCESS;
    } if (len > ICM20948_MAX_XFER) {
        return NRF_ERROR_INVALID_LENGTH;
    }
    /* Select the bank to which the register belongs to */
    ret_code_t err = icm20948_select_bank(dev, icm20948_reg_bank(reg));
    CHK_FN_ERR_DATA(err);

    buf[0] = icm20948_reg_addr(reg);
    memcpy(&buf[1], data, len);
    return nrf_drv_twi_tx(dev->twi, dev->address, buf, len + 1u, false);
}

/*! @brief: Wrapper function to read a single register for the specified ICM device on the I2C bus. */
ret_code_t icm20948_read_reg(icm20948_t *dev, icm20948_reg_t reg, uint8_t *value)
{
    return icm20948_read_regs(dev, reg, value, 1u);
}

/*! @brief: Function to read from multiple registers starting from the specified 'reg' register
            for the specified ICM device (continuously addessed registers).
    NOTE:   All register macros are provided in the icm20948.h file which can be used. */
ret_code_t icm20948_read_regs(icm20948_t *dev, icm20948_reg_t reg, uint8_t *data, size_t len)
{
    uint8_t addr;

    if ((dev == NULL) || (data == NULL)) {
        return NRF_ERROR_NULL;
    } if (len == 0u) {
        return NRF_SUCCESS;
    }
    /* Select the bank to which the register belongs to */
    ret_code_t err = icm20948_select_bank(dev, icm20948_reg_bank(reg));
    CHK_FN_ERR_DATA(err);

    addr = icm20948_reg_addr(reg);
    /* Select the device and register to read from by providing appropriate address */
    err = nrf_drv_twi_tx(dev->twi, dev->address, &addr, 1u, true);
    CHK_FN_ERR_DATA(err);
    
    /* Blocking reception */
    return nrf_drv_twi_rx(dev->twi, dev->address, data, len);
}

/*! @brief: Function to update specific bits of a register (using mask) */
ret_code_t icm20948_update_reg(icm20948_t *dev, icm20948_reg_t reg, uint8_t mask, uint8_t value)
{
    uint8_t reg_val = 0u;
    ret_code_t err = icm20948_read_reg(dev, reg, &reg_val);
    CHK_FN_ERR_DATA(err);
    reg_val = (uint8_t)((reg_val & ~mask) | (value & mask));
    return icm20948_write_reg(dev, reg, reg_val);
}

/*! @brief: Function to read sample data (Accelerometer, Gyroscope, Temperature) together. 
            Data buffer of appropriate buffer needs to be passed for this function. */
ret_code_t icm20948_read_sample(icm20948_t *dev, uint8_t *raw_sample, icm20948_sample_t *sample)
{
    if ((dev == NULL) || (raw_sample == NULL))
    {
        return NRF_ERROR_NULL;
    }

    // Continuous read from ACC -> GYRO (temp omitted for bandwidth).
    ret_code_t err = icm20948_read_regs(dev, ICM20948_REG_ACCEL_XOUT_H_SH, raw_sample, ICM20948_SAMPLE_RAW_LEN);
    CHK_FN_ERR_DATA(err);

    if (sample != NULL)
    {
        // Populate parsed 16-bit values from raw byte stream.
        icm20948_copy_vec3_from_sensor_bytes(&sample->accel, &raw_sample[0]);
        icm20948_copy_vec3_from_sensor_bytes(&sample->gyro, &raw_sample[6]);
#if 0 // Skipping temp measurement for now
        /* Temp Data */
        icm20948_copy_i16_from_sensor_bytes(&sample->temp, &raw_sample[12]);
#endif
    }
    return NRF_SUCCESS;
}

/* Will mostly be not used. Experimental function to enable/disable I2C master mode */
#if 0
ret_code_t icm20948_i2c_master_enable(icm20948_t *dev, bool enable)
{
    if (dev == NULL)
    {
        return NRF_ERROR_NULL;
    }

    return icm20948_update_reg(dev,
                               ICM20948_REG_USER_CTRL,
                               ICM20948_USER_CTRL_I2C_MST_EN_MASK,
                               enable ? ICM20948_USER_CTRL_I2C_MST_EN_MASK : 0u);
}
#endif

/*! @brief: Initializes the ICM20948 device (basic init + TWI init + register data check) */
ret_code_t icm20948_init(icm20948_t *dev, nrf_drv_twi_t* m_twi, uint8_t address)
{
    if (dev == NULL)
    {
        return NRF_ERROR_NULL;
    }

    dev->twi = m_twi;
    dev->address = address;
    dev->current_bank = 0xFFu;
    /* Select bank 0 */
    ret_code_t err = icm20948_select_bank(dev, 0u);
    CHK_FN_ERR_DATA(err);
    return icm20948_init_chk_reg_data(dev);
}

/* Function which searches for ICM devices present on the TWI bus and initializes them.
    Also updates the device info structure, with number of devices connected info. 
    Experiemntal Function. */
#if 0
ret_code_t icm20948_init_auto(nrf_drv_twi_t const *twi,
                              icm20948_t *devs,
                              size_t max_devs,
                              size_t *found_count)
{
    if ((twi == NULL) || (devs == NULL) || (found_count == NULL))
    {
        return NRF_ERROR_NULL;
    }

    *found_count = 0u;

    const uint8_t candidates[] = {ICM20948_I2C_ADDR0, ICM20948_I2C_ADDR1};
    for (size_t i = 0; i < (sizeof(candidates) / sizeof(candidates[0])); i++)
    {
        if (*found_count >= max_devs)
        {
            break;
        }

        icm20948_t probe = {
            .twi = twi,
            .address = candidates[i],
            .current_bank = 0xFFu,
        };

        uint8_t who = 0u;
        ret_code_t err = icm20948_select_bank(&probe, 0u);
        if (err != NRF_SUCCESS)
        {
            continue;
        }

        err = icm20948_read_reg(&probe, ICM20948_REG_WHO_AM_I, &who);
        if (err != NRF_SUCCESS)
        {
            continue;
        }

        if (who != ICM20948_WHO_AM_I_VAL)
        {
            continue;
        }

        devs[*found_count] = probe;
        err = icm20948_init_chk_reg_data(&devs[*found_count]);
        if (err != NRF_SUCCESS)
        {
            continue;
        }

        (*found_count)++;
    }

    return (*found_count > 0u) ? NRF_SUCCESS : NRF_ERROR_NOT_FOUND;
}
#endif

static ret_code_t icm20948_init_chk_reg_data(icm20948_t *dev)
{
    uint8_t data_holder = 0u;
    uint8_t status_reg_data = 0u;
    uint8_t status1_reg_data = 0u;

    ret_code_t err;

    /* Bank 0 configuration. */
    err = icm20948_write_reg(dev, ICM20948_REG_USER_CTRL, ICM20948_USER_CTRL_VAL);
    if (err != NRF_SUCCESS) return err;
    err = icm20948_write_reg(dev, ICM20948_REG_PWR_MGMT_1, ICM20948_PWR_MGMT_1_NORMAL_OP);
    if (err != NRF_SUCCESS) return err;
    err = icm20948_write_reg(dev, ICM20948_REG_PWR_MGMT_2, ICM20948_PWR_MGMT_2_NORMAL_OP);
    if (err != NRF_SUCCESS) return err;
    err = icm20948_write_reg(dev, ICM20948_REG_LP_CONFIG, ICM20948_LP_CONFIG_NORMAL_OP);
    if (err != NRF_SUCCESS) return err;
    err = icm20948_write_reg(dev, ICM20948_REG_INT_PIN_CFG, ICM20948_INT_PIN_CFG_DATA_RDY);
    if (err != NRF_SUCCESS) return err;
    err = icm20948_write_reg(dev, ICM20948_REG_INT_ENABLE, ICM20948_INT_ENABLE_DATA_RDY);
    if (err != NRF_SUCCESS) return err;
    err = icm20948_write_reg(dev, ICM20948_REG_INT_ENABLE_1, ICM20948_INT_ENABLE_1_DATA_RDY);
    if (err != NRF_SUCCESS) return err;

    /* Verify bank 0. */
    err = icm20948_read_reg(dev, ICM20948_REG_WHO_AM_I, &data_holder);
    if (err != NRF_SUCCESS) return err;
    VERIFY_DATA(data_holder, ICM20948_WHO_AM_I_VAL);

    err = icm20948_read_reg(dev, ICM20948_REG_USER_CTRL, &data_holder);
    if (err != NRF_SUCCESS) return err;
    VERIFY_DATA(data_holder, ICM20948_USER_CTRL_VAL);

    err = icm20948_read_reg(dev, ICM20948_REG_PWR_MGMT_1, &data_holder);
    if (err != NRF_SUCCESS) return err;
    VERIFY_DATA(data_holder, ICM20948_PWR_MGMT_1_NORMAL_OP);

    err = icm20948_read_reg(dev, ICM20948_REG_PWR_MGMT_2, &data_holder);
    if (err != NRF_SUCCESS) return err;
    VERIFY_DATA(data_holder, ICM20948_PWR_MGMT_2_NORMAL_OP);

    err = icm20948_read_reg(dev, ICM20948_REG_INT_PIN_CFG, &data_holder);
    if (err != NRF_SUCCESS) return err;
    VERIFY_DATA(data_holder, ICM20948_INT_PIN_CFG_DATA_RDY);

    err = icm20948_read_reg(dev, ICM20948_REG_INT_ENABLE, &data_holder);
    if (err != NRF_SUCCESS) return err;
    VERIFY_DATA(data_holder, ICM20948_INT_ENABLE_DATA_RDY);

    err = icm20948_read_reg(dev, ICM20948_REG_INT_ENABLE_1, &data_holder);
    if (err != NRF_SUCCESS) return err;
    VERIFY_DATA(data_holder, ICM20948_INT_ENABLE_1_DATA_RDY);

    /* Read INT_STATUS registers to clear any pending interrupts. */
    (void)icm20948_read_reg(dev, ICM20948_REG_INT_STATUS, &status_reg_data);
    (void)icm20948_read_reg(dev, ICM20948_REG_INT_STATUS_1, &status1_reg_data);

    /* Bank 2 configuration. */
    err = icm20948_write_reg(dev, ICM20948_REG_GYRO_SMPLRT_DIV, ICM20948_GYRO_SMPLRT_DIV_5MS);
    if (err != NRF_SUCCESS) return err;
    err = icm20948_write_reg(dev, ICM20948_REG_GYRO_CONFIG_1, ICM20948_GYRO_CONFIG_1_VAL);
    if (err != NRF_SUCCESS) return err;
    err = icm20948_write_reg(dev, ICM20948_REG_GYRO_CONFIG_2, ICM20948_GYRO_CONFIG_2_DEFAULT);
    if (err != NRF_SUCCESS) return err;

    uint8_t accel_div[2] = {ICM20948_ACCEL_SMPLRT_DIV_MSB_5MS, ICM20948_ACCEL_SMPLRT_DIV_LSB_5MS};
    err = icm20948_write_regs(dev, ICM20948_REG_ACCEL_SMPLRT_DIV_1, accel_div, sizeof(accel_div));
    if (err != NRF_SUCCESS) return err;
    err = icm20948_write_reg(dev, ICM20948_REG_ACCEL_CONFIG, ICM20948_ACCEL_CONFIG_1_VAL);
    if (err != NRF_SUCCESS) return err;
    err = icm20948_write_reg(dev, ICM20948_REG_ACCEL_CONFIG_2, ICM20948_ACCEL_CONFIG_2_VAL);
    if (err != NRF_SUCCESS) return err;

    /* Verify bank 2. */
    err = icm20948_read_reg(dev, ICM20948_REG_GYRO_SMPLRT_DIV, &data_holder);
    if (err != NRF_SUCCESS) return err;
    VERIFY_DATA(data_holder, ICM20948_GYRO_SMPLRT_DIV_5MS);

    err = icm20948_read_reg(dev, ICM20948_REG_GYRO_CONFIG_1, &data_holder);
    if (err != NRF_SUCCESS) return err;
    VERIFY_DATA(data_holder, ICM20948_GYRO_CONFIG_1_VAL);

    err = icm20948_read_reg(dev, ICM20948_REG_GYRO_CONFIG_2, &data_holder);
    if (err != NRF_SUCCESS) return err;
    VERIFY_DATA(data_holder, ICM20948_GYRO_CONFIG_2_DEFAULT);

    uint8_t accel_div_read[2] = {0u, 0u};
    err = icm20948_read_regs(dev, ICM20948_REG_ACCEL_SMPLRT_DIV_1, accel_div_read, sizeof(accel_div_read));
    if (err != NRF_SUCCESS) return err;
    VERIFY_DATA(accel_div_read[0], ICM20948_ACCEL_SMPLRT_DIV_MSB_5MS);
    VERIFY_DATA(accel_div_read[1], ICM20948_ACCEL_SMPLRT_DIV_LSB_5MS);

    err = icm20948_read_reg(dev, ICM20948_REG_ACCEL_CONFIG, &data_holder);
    if (err != NRF_SUCCESS) return err;
    VERIFY_DATA(data_holder, ICM20948_ACCEL_CONFIG_1_VAL);

    err = icm20948_read_reg(dev, ICM20948_REG_ACCEL_CONFIG_2, &data_holder);
    if (err != NRF_SUCCESS) return err;
    VERIFY_DATA(data_holder, ICM20948_ACCEL_CONFIG_2_VAL);

    NRF_LOG_INFO("ICM20948 initialization check passed.");
    return NRF_SUCCESS;
}
