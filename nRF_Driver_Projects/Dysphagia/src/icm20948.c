/* ICM20948 driver for nRF5 TWI (I2C). */
#include "icm20948.h"
#include <string.h>
#include "app_error.h"
#include "nrf_error.h"
#include "nrf_log.h"
#include "nrf_delay.h"
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
nrf_drv_twi_t m_twi_icm20948 = NRF_DRV_TWI_INSTANCE(ICM20948_TWI_INSTANCE_ID);

/* Convenience device array for storing device instances + info.
    NOTE: up to 2 devices on the I2C bus at max (Addr1 = 0x68 & Addr2 = 0x69). */
icm20948_t m_icm20948[NUM_ICM_DEVICES] = {0};
static bool m_icm_twi_initialized = false;

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
    if (m_icm_twi_initialized)
    {
        return;
    }

    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_icm_config = {
        .scl = ICM20948_SCL_PIN,
        .sda = ICM20948_SDA_PIN,
        .frequency = NRF_DRV_TWI_FREQ_400K,
        .interrupt_priority = _PRIO_APP_MID,
        .clear_bus_init = false
    };
    /* Initialize the TWI interface -> Initialized in non-blocking mode */
    err_code = nrf_drv_twi_init(&m_twi_icm20948, &twi_icm_config, NULL, NULL);
    if (err_code == NRF_ERROR_INVALID_STATE)
    {
        // Driver already initialized by another path.
        m_icm_twi_initialized = true;
        nrf_drv_twi_enable(&m_twi_icm20948);
        return;
    }
    APP_ERROR_CHECK(err_code);
    /* Enable TWI interface */
    nrf_drv_twi_enable(&m_twi_icm20948);
    m_icm_twi_initialized = true;
    NRF_LOG_INFO("ICM20948: TWI interface initialized.");
}

void icm20948_twi_uninit(void) {
  if (!m_icm_twi_initialized)
  {
      return;
  }
  nrf_drv_twi_uninit(&m_twi_icm20948);
  m_icm_twi_initialized = false;
  NRF_LOG_INFO("ICM20948: TWI interface uninitialized.");
}

/*! @brief: Selects the register bank (to write to specific registers) for the ICM20948 device */
ret_code_t icm20948_select_bank(icm20948_t *dev, uint8_t bank)
{
    uint8_t buf[2];

    if (dev == NULL) {
        return NRF_ERROR_NULL;
    } if (dev->twi == NULL) {
        return NRF_ERROR_INVALID_STATE;
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

static int16_t icm20948_s32_to_i16_clamped(int32_t value)
{
    if (value > 32767)
    {
        return 32767;
    }
    if (value < -32768)
    {
        return -32768;
    }
    return (int16_t)value;
}

static void icm20948_store_i16_to_be(uint8_t *dst, int16_t value)
{
    dst[0] = (uint8_t)(((uint16_t)value >> 8) & 0xFFu);
    dst[1] = (uint8_t)((uint16_t)value & 0xFFu);
}

static void icm20948_apply_bias_to_sample(icm20948_sample_t *sample,
                                          icm20948_vec3_t const *accel_bias,
                                          icm20948_vec3_t const *gyro_bias)
{
    sample->accel.x = icm20948_s32_to_i16_clamped((int32_t)sample->accel.x - accel_bias->x);
    sample->accel.y = icm20948_s32_to_i16_clamped((int32_t)sample->accel.y - accel_bias->y);
    sample->accel.z = icm20948_s32_to_i16_clamped((int32_t)sample->accel.z - accel_bias->z);

    sample->gyro.x = icm20948_s32_to_i16_clamped((int32_t)sample->gyro.x - gyro_bias->x);
    sample->gyro.y = icm20948_s32_to_i16_clamped((int32_t)sample->gyro.y - gyro_bias->y);
    sample->gyro.z = icm20948_s32_to_i16_clamped((int32_t)sample->gyro.z - gyro_bias->z);
}

static void icm20948_pack_sample_to_raw(uint8_t *raw_sample, icm20948_sample_t const *sample)
{
    icm20948_store_i16_to_be(&raw_sample[0], sample->accel.x);
    icm20948_store_i16_to_be(&raw_sample[2], sample->accel.y);
    icm20948_store_i16_to_be(&raw_sample[4], sample->accel.z);
    icm20948_store_i16_to_be(&raw_sample[6], sample->gyro.x);
    icm20948_store_i16_to_be(&raw_sample[8], sample->gyro.y);
    icm20948_store_i16_to_be(&raw_sample[10], sample->gyro.z);
}

void icm20948_convert_raw_sample(icm20948_t const *dev,
                                 uint8_t const *raw_sample,
                                 uint8_t *processed_raw_sample,
                                 icm20948_sample_t *sample)
{
    icm20948_sample_t local_sample;
    icm20948_sample_t *sample_ptr = (sample != NULL) ? sample : &local_sample;
    uint8_t local_raw[ICM20948_SAMPLE_RAW_LEN];
    uint8_t *raw_out = processed_raw_sample;

    if ((raw_sample == NULL) || ((sample == NULL) && (processed_raw_sample == NULL)))
    {
        return;
    }

    if (raw_out == NULL)
    {
        raw_out = local_raw;
    }

    memcpy(raw_out, raw_sample, ICM20948_SAMPLE_RAW_LEN);
    icm20948_copy_vec3_from_sensor_bytes(&sample_ptr->accel, &raw_out[0]);
    icm20948_copy_vec3_from_sensor_bytes(&sample_ptr->gyro, &raw_out[6]);

    if ((dev != NULL) && dev->bias_valid)
    {
        icm20948_apply_bias_to_sample(sample_ptr, &dev->accel_bias, &dev->gyro_bias);
        icm20948_pack_sample_to_raw(raw_out, sample_ptr);
    }
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

    if ((sample != NULL) || dev->bias_valid)
    {
        icm20948_convert_raw_sample(dev, raw_sample, raw_sample, sample);
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

typedef struct
{
    uint8_t gyro_config_1;
    uint8_t accel_config;
    uint8_t gyro_smplrt_div;
    uint8_t accel_smplrt_div_msb;
    uint8_t accel_smplrt_div_lsb;
    uint8_t odr_align_en;
    uint8_t gyro_config_2;
    uint8_t accel_config_2;
} icm20948_profile_regs_t;

static icm20948_profile_regs_t const s_icm20948_profiles[] = {
    // Default profile from icm20948_config.h.
    {
        .gyro_config_1 = ICM20948_GYRO_CONFIG_1_VAL,
        .accel_config = ICM20948_ACCEL_CONFIG_1_VAL,
        .gyro_smplrt_div = ICM20948_GYRO_SMPLRT_DIV_5MS,
        .accel_smplrt_div_msb = ICM20948_ACCEL_SMPLRT_DIV_MSB_5MS,
        .accel_smplrt_div_lsb = ICM20948_ACCEL_SMPLRT_DIV_LSB_5MS,
        .odr_align_en = 0x01u,
        .gyro_config_2 = ICM20948_GYRO_CONFIG_2_DEFAULT,
        .accel_config_2 = ICM20948_ACCEL_CONFIG_2_VAL,
    },
    // IMU1 swallow profile: closest DLPF-enabled divider setting to a 300 Hz target, +/-4g and +/-500 dps.
    {
        .gyro_config_1 = 0x23u,
        .accel_config = 0x23u,
        .gyro_smplrt_div = 0x03u,
        .accel_smplrt_div_msb = 0x00u,
        .accel_smplrt_div_lsb = 0x03u,
        .odr_align_en = 0x01u,
        .gyro_config_2 = 0x00u,
        .accel_config_2 = 0x00u,
    },
    // IMU2 context profile: DLPF enabled, 5 Hz output, +/-2g and +/-250 dps.
    {
        .gyro_config_1 = 0x31u,
        .accel_config = 0x31u,
        .gyro_smplrt_div = 0xDBu,
        .accel_smplrt_div_msb = 0x00u,
        .accel_smplrt_div_lsb = 0xE0u,
        .odr_align_en = 0x01u,
        .gyro_config_2 = 0x00u,
        .accel_config_2 = 0x00u,
    },
};

ret_code_t icm20948_apply_profile(icm20948_t *dev, icm20948_profile_t profile)
{
    if (dev == NULL)
    {
        return NRF_ERROR_NULL;
    }
    if ((uint32_t)profile >= (uint32_t)(sizeof(s_icm20948_profiles) / sizeof(s_icm20948_profiles[0])))
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    icm20948_profile_regs_t const *cfg = &s_icm20948_profiles[(uint32_t)profile];
    ret_code_t err = icm20948_write_reg(dev, ICM20948_REG_GYRO_SMPLRT_DIV, cfg->gyro_smplrt_div);
    CHK_FN_ERR_DATA(err);
    err = icm20948_write_reg(dev, ICM20948_REG_GYRO_CONFIG_1, cfg->gyro_config_1);
    CHK_FN_ERR_DATA(err);
    err = icm20948_write_reg(dev, ICM20948_REG_GYRO_CONFIG_2, cfg->gyro_config_2);
    CHK_FN_ERR_DATA(err);

    {
        uint8_t accel_div[2] = {cfg->accel_smplrt_div_msb, cfg->accel_smplrt_div_lsb};
        err = icm20948_write_regs(dev, ICM20948_REG_ACCEL_SMPLRT_DIV_1, accel_div, sizeof(accel_div));
        CHK_FN_ERR_DATA(err);
    }
    err = icm20948_write_reg(dev, ICM20948_REG_ODR_ALIGN_EN, cfg->odr_align_en);
    CHK_FN_ERR_DATA(err);
    err = icm20948_write_reg(dev, ICM20948_REG_ACCEL_CONFIG, cfg->accel_config);
    CHK_FN_ERR_DATA(err);
    err = icm20948_write_reg(dev, ICM20948_REG_ACCEL_CONFIG_2, cfg->accel_config_2);
    CHK_FN_ERR_DATA(err);

    dev->bias_valid = false;
    NRF_LOG_INFO("ICM20948 profile applied: %u", (unsigned)profile);
    return NRF_SUCCESS;
}

ret_code_t icm20948_calibrate_bias(icm20948_t *dev, uint16_t sample_count, uint16_t inter_sample_delay_ms)
{
    if (dev == NULL)
    {
        return NRF_ERROR_NULL;
    }
    if (sample_count == 0u)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    int64_t gyro_x_sum = 0;
    int64_t gyro_y_sum = 0;
    int64_t gyro_z_sum = 0;
    uint8_t raw_sample[ICM20948_SAMPLE_RAW_LEN];
    icm20948_sample_t sample;

    dev->bias_valid = false;
    for (uint16_t i = 0u; i < sample_count; i++)
    {
        ret_code_t err = icm20948_read_sample(dev, raw_sample, &sample);
        CHK_FN_ERR_DATA(err);
        gyro_x_sum += sample.gyro.x;
        gyro_y_sum += sample.gyro.y;
        gyro_z_sum += sample.gyro.z;
        if (inter_sample_delay_ms > 0u)
        {
            nrf_delay_ms(inter_sample_delay_ms);
        }
    }

    // Preserve accelerometer gravity component by default; compensate gyro zero-rate bias only.
    dev->accel_bias.x = 0;
    dev->accel_bias.y = 0;
    dev->accel_bias.z = 0;
    dev->gyro_bias.x = (int16_t)(gyro_x_sum / sample_count);
    dev->gyro_bias.y = (int16_t)(gyro_y_sum / sample_count);
    dev->gyro_bias.z = (int16_t)(gyro_z_sum / sample_count);
    dev->bias_valid = true;

    NRF_LOG_INFO("ICM20948 gyro bias calibrated: x=%d y=%d z=%d",
                 dev->gyro_bias.x, dev->gyro_bias.y, dev->gyro_bias.z);
    return NRF_SUCCESS;
}

ret_code_t icm20948_fifo_reset(icm20948_t *dev)
{
    ret_code_t err;

    if (dev == NULL)
    {
        return NRF_ERROR_NULL;
    }

    err = icm20948_write_reg(dev, ICM20948_REG_FIFO_RST, ICM20948_FIFO_RESET_MASK);
    CHK_FN_ERR_DATA(err);
    return icm20948_write_reg(dev, ICM20948_REG_FIFO_RST, 0x00u);
}

ret_code_t icm20948_fifo_configure(icm20948_t *dev, bool enable_accel, bool enable_gyro, bool status_required)
{
    uint8_t fifo_en_2 = 0u;
    ret_code_t err;

    if (dev == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if (enable_accel)
    {
        fifo_en_2 |= ICM20948_FIFO_EN_2_ACCEL_MASK;
    }
    if (enable_gyro)
    {
        fifo_en_2 |= (ICM20948_FIFO_EN_2_GYRO_X_MASK |
                      ICM20948_FIFO_EN_2_GYRO_Y_MASK |
                      ICM20948_FIFO_EN_2_GYRO_Z_MASK);
    }

    err = icm20948_write_reg(dev, ICM20948_REG_FIFO_EN_1, 0x00u);
    CHK_FN_ERR_DATA(err);
    err = icm20948_write_reg(dev, ICM20948_REG_FIFO_EN_2, 0x00u);
    CHK_FN_ERR_DATA(err);
    err = icm20948_write_reg(dev, ICM20948_REG_FIFO_CFG, status_required ? ICM20948_FIFO_CFG_STATUS_EN : 0x00u);
    CHK_FN_ERR_DATA(err);
    err = icm20948_write_reg(dev, ICM20948_REG_FIFO_MODE, ICM20948_FIFO_MODE_STREAM);
    CHK_FN_ERR_DATA(err);
    err = icm20948_fifo_reset(dev);
    CHK_FN_ERR_DATA(err);
    err = icm20948_update_reg(dev,
                              ICM20948_REG_USER_CTRL,
                              ICM20948_USER_CTRL_FIFO_EN_MASK,
                              (fifo_en_2 != 0u) ? ICM20948_USER_CTRL_FIFO_EN_MASK : 0x00u);
    CHK_FN_ERR_DATA(err);
    return icm20948_write_reg(dev, ICM20948_REG_FIFO_EN_2, fifo_en_2);
}

ret_code_t icm20948_fifo_get_count(icm20948_t *dev, uint16_t *fifo_count_bytes)
{
    uint8_t fifo_count_raw[2];
    ret_code_t err;

    if ((dev == NULL) || (fifo_count_bytes == NULL))
    {
        return NRF_ERROR_NULL;
    }

    err = icm20948_read_regs(dev, ICM20948_REG_FIFO_COUNT_H, fifo_count_raw, sizeof(fifo_count_raw));
    CHK_FN_ERR_DATA(err);
    *fifo_count_bytes = (uint16_t)(((uint16_t)(fifo_count_raw[0] & 0x1Fu) << 8) | fifo_count_raw[1]);
    return NRF_SUCCESS;
}

ret_code_t icm20948_fifo_read(icm20948_t *dev, uint8_t *data, size_t len)
{
    if ((dev == NULL) || (data == NULL))
    {
        return NRF_ERROR_NULL;
    }
    if (len == 0u)
    {
        return NRF_SUCCESS;
    }

    return icm20948_read_regs(dev, ICM20948_REG_FIFO_R_W, data, len);
}
/*! @brief: Initializes the ICM20948 device (basic init + TWI init + register data check) */
ret_code_t icm20948_init(icm20948_t *dev, uint8_t address)
{
    if (dev == NULL)
    {
        return NRF_ERROR_NULL;
    }

    dev->twi = &m_twi_icm20948;
    dev->address = address;
    dev->current_bank = 0xFFu;
    dev->accel_bias.x = 0;
    dev->accel_bias.y = 0;
    dev->accel_bias.z = 0;
    dev->gyro_bias.x = 0;
    dev->gyro_bias.y = 0;
    dev->gyro_bias.z = 0;
    dev->bias_valid = false;
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
    err = icm20948_write_reg(dev, ICM20948_REG_INT_ENABLE_1, ICM20948_INT_ENABLE_1_RAW_DATA_RDY_MASK);
    if (err != NRF_SUCCESS) return err;
    err = icm20948_write_reg(dev, ICM20948_REG_INT_ENABLE_2, 0x00u);
    if (err != NRF_SUCCESS) return err;
    err = icm20948_write_reg(dev, ICM20948_REG_INT_ENABLE_3, 0x00u);
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
    VERIFY_DATA(data_holder, ICM20948_INT_ENABLE_1_RAW_DATA_RDY_MASK);

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

