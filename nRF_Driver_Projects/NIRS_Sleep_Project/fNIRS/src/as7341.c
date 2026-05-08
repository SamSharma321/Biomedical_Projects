#include "as7341.h"

#if defined(AS7341)

#include <stddef.h>
#include "app_util_platform.h"
#include "nrf_delay.h"
#include "nrf_drv_twi.h"
#include "nrf_log.h"

#define AS7341_CHIP_ID                       0x09u    // Device ID

#define AS7341_REG_ID                        0x92u    // Device ID regsiter address
#define AS7341_REG_ENABLE                    0x80u    // Enable regsiter - SPEN, SMUXEN, PON, SPEN, etc
#define AS7341_REG_ATIME                     0x81u    // Sets the number of integration steps from 1 to 256
#define AS7341_REG_CFG6                      0xAFu    // Configuration Regsiter 6 address
#define AS7341_REG_STATUS2                   0xA3u    // Status 2 regsiter address
#define AS7341_REG_CH0_DATA_L                0x95u    // Channel 0 data regsiter adress. Can be used for continuous reading
#define AS7341_REG_ASTEP_L                   0xCAu    
#define AS7341_REG_ASTEP_H                   0xCBu
#define AS7341_ENABLE_PON_BIT                0u       // On setting this bit, the device enters the IDLE state in which the internal oscillator and attendant circuitry are active - low power consumption
#define AS7341_ENABLE_SP_EN_BIT              1u       // Enable Spectral measurement - ACTIVE STATE - can be disabled to save power
#define AS7341_ENABLE_SMUX_EN_BIT            4u       // Starts SMUX command - clears once the operation is finished
#define AS7341_SMUX_CMD_WRITE                0x02u    // Value to be updated into CFG6 register
#define AS7341_DATA_READY_MASK               0x40u    // Mask to check if data is ready from stat regsiter
#define AS7341_DATA_TIMEOUT_MS               250u     // Dtata time out value

/* defining multiple instances of I2C for each AS7341 (I2C address is same for all devices) */
#if (NUM_OF_AS7341_DEVICES >= 1)
static const nrf_drv_twi_t m_as7341_twi_0 = NRF_DRV_TWI_INSTANCE(AS7341_1_I2C_INSTANCE_ID);
#endif
#if (NUM_OF_AS7341_DEVICES >= 2)
static const nrf_drv_twi_t m_as7341_twi_1 = NRF_DRV_TWI_INSTANCE(AS7341_2_I2C_INSTANCE_ID);
#endif

/*! Structure to hold AS7341 device parameter info */
typedef struct
{
    nrf_drv_twi_t const *p_twi;
    uint8_t scl_pin;
    uint8_t sda_pin;
    uint8_t i2c_addr;
    bool initialized;
    uint16_t transfer_interval_ms;
} as7341_dev_ctx_t;

static as7341_dev_ctx_t m_as7341_ctx[NUM_OF_AS7341_DEVICES] =
{
#if (NUM_OF_AS7341_DEVICES >= 1)
    {
        .p_twi = &m_as7341_twi_0,
        .scl_pin = AS7341_1_SCL_PIN,
        .sda_pin = AS7341_1_SDA_PIN,
        .i2c_addr = AS7341_1_I2C_ADDR,
        .initialized = false,
        .transfer_interval_ms = AS7341_DEFAULT_BLE_INTERVAL_MS,
    },
#endif
#if (NUM_OF_AS7341_DEVICES >= 2)
    {
        .p_twi = &m_as7341_twi_1,
        .scl_pin = AS7341_2_SCL_PIN,
        .sda_pin = AS7341_2_SDA_PIN,
        .i2c_addr = AS7341_2_I2C_ADDR,
        .initialized = false,
        .transfer_interval_ms = AS7341_DEFAULT_BLE_INTERVAL_MS,
    },
#endif
};

/*! @brief: Returns the stucture based on device index */
static as7341_dev_ctx_t *as7341_ctx_get(uint8_t dev_idx)
{
    if (dev_idx >= NUM_OF_AS7341_DEVICES)
    {
        return NULL;
    }
    return &m_as7341_ctx[dev_idx];
}

/* @brief: Writes to desired AS741 resgiter */
static ret_code_t as7341_reg_write(as7341_dev_ctx_t const *p_ctx, uint8_t reg, uint8_t value)
{
    uint8_t tx_buf[2] = {reg, value};
    return nrf_drv_twi_tx(p_ctx->p_twi, p_ctx->i2c_addr, tx_buf, sizeof(tx_buf), false); // End of transaction - no_stop = false
}

/*! 
*   @brief: Read register data from AS7341 device(s).
*   @return: err_code - success or failure code
*/
static ret_code_t as7341_reg_read(as7341_dev_ctx_t const *p_ctx, uint8_t reg, uint8_t *p_value)
{
    ret_code_t err_code;

    if ((p_ctx == NULL) || (p_value == NULL))
    {
        return NRF_ERROR_NULL;
    }

    err_code = nrf_drv_twi_tx(p_ctx->p_twi, p_ctx->i2c_addr, &reg, 1u, true); // End of transaction - no_stop = true
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return nrf_drv_twi_rx(p_ctx->p_twi, p_ctx->i2c_addr, p_value, 1u);
}

/*! 
*   @brief: Read continuous data registers from the AS7341. (auto-register increment). 
            The start_reg denotes the starting resgiter from which the data has to be read.
*   @return: err_code - success or failure code
*/
static ret_code_t as7341_reg_read_bytes(as7341_dev_ctx_t const *p_ctx, uint8_t start_reg, uint8_t *p_buffer, size_t length) {
    ret_code_t err_code;
    if ((p_ctx == NULL) || (p_buffer == NULL) || (length == 0u)) {
        return NRF_ERROR_NULL;
    }

    err_code = nrf_drv_twi_tx(p_ctx->p_twi, p_ctx->i2c_addr, &start_reg, 1u, true);
    if (err_code != NRF_SUCCESS) {
        return err_code;
    }
    return nrf_drv_twi_rx(p_ctx->p_twi, p_ctx->i2c_addr, p_buffer, length);
}


/*! 
*   @brief: Function which flips or sets the desired respective bit position of a AS7341 regsiter.
*   @return: err_code - success or failure code
*/
static ret_code_t as7341_reg_write_bit(as7341_dev_ctx_t const *p_ctx, uint8_t reg, uint8_t bit_pos, bool enable)
{
    uint8_t value = 0u;
    ret_code_t err_code = as7341_reg_read(p_ctx, reg, &value);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    if (enable) {
        // Set the bit
        value |= (uint8_t)(1u << bit_pos);
    }
    else {
        // Reset the bit
        value &= (uint8_t)(~(1u << bit_pos));
    }
    // Write back updated data to the register
    return as7341_reg_write(p_ctx, reg, value);
}

/*! 
*   @brief: Function to set or reset the SPEN (Spectral Measurement Enable) bit for AS7341.
*   @return: err_code - success or failure code
*/
static ret_code_t as7341_config_spen(as7341_dev_ctx_t const *p_ctx, bool enable)
{
    return as7341_reg_write_bit(p_ctx, AS7341_REG_ENABLE, AS7341_ENABLE_SP_EN_BIT, enable);
}

/*! 
*   @brief: Function to set or reset the SPEN (Spectral Measurement Enable) bit for AS7341.
*   @return: err_code - success or failure code
*/
static ret_code_t as7341_enable_smux(as7341_dev_ctx_t const *p_ctx)
{
    ret_code_t err_code = as7341_reg_write_bit(p_ctx, AS7341_REG_ENABLE, AS7341_ENABLE_SMUX_EN_BIT, true);
    if (err_code != NRF_SUCCESS)
        return err_code;
    nrf_delay_ms(1u);
    return NRF_SUCCESS;
}


/*! 
*   @brief: Function to set or reset the SPEN (Spectral Measurement Enable) bit for AS7341.
*   @return: err_code - success or failure code
*/
static ret_code_t as7341_set_smux_command(as7341_dev_ctx_t const *p_ctx, uint8_t command)
{
    uint8_t cfg6 = 0u;
    ret_code_t err_code = as7341_reg_read(p_ctx, AS7341_REG_CFG6, &cfg6);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    cfg6 = (uint8_t)((cfg6 & 0xF3u) | ((command & 0x03u) << 3u));
    return as7341_reg_write(p_ctx, AS7341_REG_CFG6, cfg6);
}


static ret_code_t as7341_setup_f5f8_clear_nir(as7341_dev_ctx_t const *p_ctx)
{
    /* 
      0x00=0x00, 0x01=0x00, 0x02=0x00: disable F3/F1/reserved-left paths
      0x03=0x40: route F8-left -> ADC3
      0x04=0x02: route F6-left -> ADC1
      0x05=0x00: disable F4/F2-left
      0x06=0x10: route F5-left -> ADC0
      0x07=0x03: route F7-left -> ADC2
      0x08=0x50: route CLEAR -> ADC4
      0x09=0x10: route F5-right -> ADC0
      0x0A=0x03: route F7-right -> ADC2
      0x0B=0x00: reserved/disabled
      0x0C=0x00, 0x0D=0x00: disable F2/F4-right
      0x0E=0x24: route F6/F8-right paths (combined register for those two)
      0x0F=0x00, 0x10=0x00: disable F3/F1-right
      0x11=0x50: route CLEAR-right -> ADC4
      0x12=0x00: reserved/disabled
      0x13=0x06: route NIR -> ADC5
    */
    static const uint8_t smux_cfg[][2] =
    {   
        /* {Register, Value} */
        {0x00u, 0x00u}, {0x01u, 0x00u}, {0x02u, 0x00u}, {0x03u, 0x40u},
        {0x04u, 0x02u}, {0x05u, 0x00u}, {0x06u, 0x10u}, {0x07u, 0x03u},
        {0x08u, 0x50u}, {0x09u, 0x10u}, {0x0Au, 0x03u}, {0x0Bu, 0x00u},
        {0x0Cu, 0x00u}, {0x0Du, 0x00u}, {0x0Eu, 0x24u}, {0x0Fu, 0x00u},
        {0x10u, 0x00u}, {0x11u, 0x50u}, {0x12u, 0x00u}, {0x13u, 0x06u},
    };

    for (uint8_t i = 0u; i < (uint8_t)(sizeof(smux_cfg) / sizeof(smux_cfg[0])); i++)
    {
        ret_code_t err_code = as7341_reg_write(p_ctx, smux_cfg[i][0], smux_cfg[i][1]);
        if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }
    }

    return NRF_SUCCESS;
}

static ret_code_t as7341_setup_f1f4_clear_nir(as7341_dev_ctx_t const *p_ctx)
{
    static const uint8_t smux_cfg[][2] =
    {
        {0x00u, 0x30u}, {0x01u, 0x01u}, {0x02u, 0x00u}, {0x03u, 0x00u},
        {0x04u, 0x00u}, {0x05u, 0x42u}, {0x06u, 0x00u}, {0x07u, 0x00u},
        {0x08u, 0x50u}, {0x09u, 0x00u}, {0x0Au, 0x00u}, {0x0Bu, 0x00u},
        {0x0Cu, 0x20u}, {0x0Du, 0x04u}, {0x0Eu, 0x00u}, {0x0Fu, 0x30u},
        {0x10u, 0x01u}, {0x11u, 0x50u}, {0x12u, 0x00u}, {0x13u, 0x06u},
    };

    for (uint8_t i = 0u; i < (uint8_t)(sizeof(smux_cfg) / sizeof(smux_cfg[0])); i++)
    {
        ret_code_t err_code = as7341_reg_write(p_ctx, smux_cfg[i][0], smux_cfg[i][1]);
        if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }
    }

    return NRF_SUCCESS;
}

static ret_code_t as7341_set_mux_high_channels(as7341_dev_ctx_t const *p_ctx)
{
    ret_code_t err_code;

    err_code = as7341_config_spen(p_ctx, false);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    err_code = as7341_set_smux_command(p_ctx, AS7341_SMUX_CMD_WRITE);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    err_code = as7341_setup_f5f8_clear_nir(p_ctx);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    err_code = as7341_enable_smux(p_ctx);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return as7341_config_spen(p_ctx, true);
}

static ret_code_t as7341_wait_for_data_ready(as7341_dev_ctx_t const *p_ctx, uint16_t timeout_ms)
{
    uint8_t status2 = 0u;

    for (uint16_t elapsed = 0u; elapsed < timeout_ms; elapsed++)
    {
        ret_code_t err_code = as7341_reg_read(p_ctx, AS7341_REG_STATUS2, &status2);
        if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }
        if ((status2 & AS7341_DATA_READY_MASK) != 0u)
        {
            return NRF_SUCCESS;
        }
        nrf_delay_ms(1u);
    }

    return NRF_ERROR_TIMEOUT;
}

ret_code_t as7341_set_integration_20ms_device(uint8_t dev_idx, uint8_t integration_20ms_units)
{
    as7341_dev_ctx_t *p_ctx = as7341_ctx_get(dev_idx);
    uint32_t astep_plus_one;
    uint32_t astep;
    ret_code_t err_code;

    if (p_ctx == NULL)
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    if (!p_ctx->initialized)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    if (integration_20ms_units == 0u)
    {
        integration_20ms_units = AS7341_DEFAULT_INTEGRATION_20MS;
    }

    astep_plus_one = ((uint32_t)integration_20ms_units * 20000u) / 278u;
    if (astep_plus_one == 0u)
    {
        astep_plus_one = 1u;
    }

    astep = astep_plus_one - 1u;
    if (astep > 0xFFFFu)
    {
        astep = 0xFFFFu;
    }

    err_code = as7341_reg_write(p_ctx, AS7341_REG_ATIME, AS7341_ATIME_FIXED);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    err_code = as7341_reg_write(p_ctx, AS7341_REG_ASTEP_L, (uint8_t)(astep & 0xFFu));
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    err_code = as7341_reg_write(p_ctx, AS7341_REG_ASTEP_H, (uint8_t)((astep >> 8u) & 0xFFu));
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    p_ctx->transfer_interval_ms = (uint16_t)integration_20ms_units * 20u;
    return NRF_SUCCESS;
}


/*! 
    @brief: Function to initialize specifc connected AS7341 device (based on the provided device index).
    @param [in] devIdx: Device Index
    @return NRF_SUCCESS is succeded, err_code if not.
*/
ret_code_t as7341_init_device(uint8_t dev_idx)
{   
    /* Get device parameters info based on device index */
    as7341_dev_ctx_t *p_ctx = as7341_ctx_get(dev_idx);
    ret_code_t err_code;
    uint8_t chip_id = 0u;

    if (p_ctx == NULL) {
        return NRF_ERROR_INVALID_PARAM;
    }
    
    // Initialize I2C instance if not already done
    if (!p_ctx->initialized) {
        nrf_drv_twi_config_t twi_cfg = NRF_DRV_TWI_DEFAULT_CONFIG;
        twi_cfg.scl = p_ctx->scl_pin;
        twi_cfg.sda = p_ctx->sda_pin;
        // 400 kHz Frequency
        twi_cfg.frequency = NRF_DRV_TWI_FREQ_400K;
        twi_cfg.interrupt_priority = APP_IRQ_PRIORITY_LOW;
        // When a slave device hangs and holds the I2C SDA line low, the master forcefully toggles the SCL 9 times and sends stop signal to release the slave
        twi_cfg.clear_bus_init = false; 
        // Initializing I2C in blocking fashion
        err_code = nrf_drv_twi_init(p_ctx->p_twi, &twi_cfg, NULL, NULL);
        if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_INVALID_STATE)) {
            return err_code;
        }
        nrf_drv_twi_enable(p_ctx->p_twi);
        p_ctx->initialized = true;
    }
    // Read device ID register to confirm
    err_code = as7341_reg_read(p_ctx, AS7341_REG_ID, &chip_id);
    if (err_code != NRF_SUCCESS) {
        NRF_LOG_WARNING("AS7341[%u] ID read failed: 0x%X", dev_idx, err_code);
        p_ctx->initialized = false;
        return err_code;
    }

    if ((chip_id & 0xFCu) != (uint8_t)(AS7341_CHIP_ID << 2u)) {
        NRF_LOG_WARNING("AS7341[%u] ID mismatch: 0x%02X", dev_idx, chip_id);
        p_ctx->initialized = false;
        return NRF_ERROR_NOT_FOUND;
    } else {
        NRF_LOG_INFO("AS7341 device found on I2C bus (index = %d)", dev_idx);
    }

    err_code = as7341_reg_write_bit(p_ctx, AS7341_REG_ENABLE, AS7341_ENABLE_PON_BIT, true);
    if (err_code != NRF_SUCCESS) {
        p_ctx->initialized = false;
        return err_code;
    }

    err_code = as7341_set_integration_20ms_device(dev_idx, AS7341_DEFAULT_INTEGRATION_20MS);
    if (err_code != NRF_SUCCESS) {
        p_ctx->initialized = false;
        return err_code;
    }

    err_code = as7341_set_mux_high_channels(p_ctx);
    if (err_code != NRF_SUCCESS) {
        p_ctx->initialized = false;
        return err_code;
    }

    err_code = as7341_config_spen(p_ctx, true);
    if (err_code != NRF_SUCCESS) {
        p_ctx->initialized = false;
        return err_code;
    }

    NRF_LOG_INFO("AS7341[%u] initialized on TWI instance %u (SCL=%u SDA=%u)",
                 dev_idx,
                 p_ctx->p_twi->inst_idx,
                 p_ctx->scl_pin,
                 p_ctx->sda_pin);
    return NRF_SUCCESS;
}

/*! 
    @brief: Function to initialize all connected AS7341 devices.
    NOTE: Since only one I2C address is available for AS7341, different 
    I2C busses should be allocated for each AS7341 device  (maximum 2).
    
    @return NRF_SUCCESS is succeded, err_code if not.
*/
ret_code_t as7341_init_all(void)
{
    ret_code_t err_code = NRF_SUCCESS;
    ret_code_t first_error = NRF_SUCCESS;
    uint8_t initialized_count = 0u;

    for (uint8_t dev_idx = 0u; dev_idx < NUM_OF_AS7341_DEVICES; dev_idx++)
    {
        err_code = as7341_init_device(dev_idx);
        if (err_code == NRF_SUCCESS)
        {
            initialized_count++;
        }
        else
        {
            if (first_error == NRF_SUCCESS)
            {
                first_error = err_code;
            }
            NRF_LOG_WARNING("AS7341[%u] init skipped (err=0x%X)", dev_idx, err_code);
        }
    }

    if (initialized_count > 0u)
    {
        return NRF_SUCCESS;
    }

    return (first_error == NRF_SUCCESS) ? NRF_ERROR_NOT_FOUND : first_error;
}

ret_code_t as7341_read_current_mux_channels_device(uint8_t dev_idx,
                                                   uint16_t channels[AS7341_MUX_CHANNEL_COUNT])
{
    as7341_dev_ctx_t *p_ctx = as7341_ctx_get(dev_idx);
    uint8_t raw_buf[12];
    ret_code_t err_code;

    if ((p_ctx == NULL) || (channels == NULL))
    {
        return NRF_ERROR_NULL;
    }
    if (!p_ctx->initialized)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    err_code = as7341_wait_for_data_ready(p_ctx, AS7341_DATA_TIMEOUT_MS);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    err_code = as7341_reg_read_bytes(p_ctx, AS7341_REG_CH0_DATA_L, raw_buf, sizeof(raw_buf));
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    for (uint8_t i = 0u; i < AS7341_MUX_CHANNEL_COUNT; i++)
    {
        uint8_t lo = raw_buf[(2u * i)];
        uint8_t hi = raw_buf[(2u * i) + 1u];
        channels[i] = (uint16_t)(((uint16_t)hi << 8u) | lo);
    }

    return NRF_SUCCESS;
}

ret_code_t as7341_read_red_nir_device(uint8_t dev_idx,
                                      uint16_t *p_red_630,
                                      uint16_t *p_red_680,
                                      uint16_t *p_nir)
{
    uint16_t channels[AS7341_MUX_CHANNEL_COUNT];
    ret_code_t err_code;

    if ((p_red_630 == NULL) || (p_red_680 == NULL) || (p_nir == NULL))
    {
        return NRF_ERROR_NULL;
    }

    err_code = as7341_read_current_mux_channels_device(dev_idx, channels);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    *p_red_630 = channels[AS7341_MUX_IDX_630NM_F7];
    *p_red_680 = channels[AS7341_MUX_IDX_680NM_F8];
    *p_nir = channels[AS7341_MUX_IDX_NIR];

    return NRF_SUCCESS;
}

uint16_t as7341_get_transfer_interval_ms_device(uint8_t dev_idx)
{
    as7341_dev_ctx_t *p_ctx = as7341_ctx_get(dev_idx);
    if (p_ctx == NULL)
    {
        return AS7341_DEFAULT_BLE_INTERVAL_MS;
    }
    return p_ctx->transfer_interval_ms;
}

bool as7341_is_initialized_device(uint8_t dev_idx)
{
    as7341_dev_ctx_t *p_ctx = as7341_ctx_get(dev_idx);
    if (p_ctx == NULL)
    {
        return false;
    }
    return p_ctx->initialized;
}

ret_code_t as7341_init(void)
{
    return as7341_init_device(0u);
}

ret_code_t as7341_set_integration_20ms(uint8_t integration_20ms_units)
{
    return as7341_set_integration_20ms_device(0u, integration_20ms_units);
}

ret_code_t as7341_read_current_mux_channels(uint16_t channels[AS7341_MUX_CHANNEL_COUNT])
{
    return as7341_read_current_mux_channels_device(0u, channels);
}

ret_code_t as7341_read_red_nir(uint16_t *p_red_630, uint16_t *p_red_680, uint16_t *p_nir)
{
    return as7341_read_red_nir_device(0u, p_red_630, p_red_680, p_nir);
}

uint16_t as7341_get_transfer_interval_ms(void)
{
    return as7341_get_transfer_interval_ms_device(0u);
}

#endif /* defined(AS7341) */
