#include "ads1299-x.h"
#include "app_error.h"
#include "ble_eeg.h"
#include "nrf_delay.h"
#include "nrf_drv_spi.h"
#include "nrf_gpio.h"
#include "nrf_log.h"
#include "stdlib.h"
#include <string.h>

#if defined(FAST_SPI_ENABLED) && FAST_SPI_ENABLED == 1
#include "spi_master_fast.h"
#endif
#define ADS1299_STATUS_BYTES      3u
#define BYTES_PER_CHANNEL         3u
#define ADS1299_MAX_DATA_CHANNELS 8u
#define RX_DATA_LEN               (ADS1299_STATUS_BYTES + ADS1299_MAX_DATA_CHANNELS * BYTES_PER_CHANNEL)
#define ADS1299_REGIDX_CONFIG2    1u
#define ADS1299_REGIDX_CH1SET     4u
#define ADS1299_REGIDX_MISC1      20u
#define ADS1299_CHSET_MUX_Msk     0x07u
#define ADS1299_CHSET_MUX_TEST    0x05u
#define ADS1299_MISC1_SRB1_Msk    0x20u
#define ADS1299_CONFIG2_INT_CAL_Msk (1u << 4)
#if defined(FAST_SPI_ENABLED) && FAST_SPI_ENABLED == 1
#else
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(ADS1299_SPI_INSTANCE_ID);
#endif

static uint8_t rx_data_dev0[RX_DATA_LEN];
static uint8_t rx_data_dev1[RX_DATA_LEN];

static const uint8_t ads1299_default_registers[ADS1299_WRITABLE_REG_COUNT] = {
    ADS1299_REGDEFAULT_CONFIG1,
    ADS1299_REGDEFAULT_CONFIG2,
    ADS1299_REGDEFAULT_CONFIG3,
    ADS1299_REGDEFAULT_LOFF,
    ADS1299_REGDEFAULT_CH1SET,
    ADS1299_REGDEFAULT_CH2SET,
    ADS1299_REGDEFAULT_CH3SET,
    ADS1299_REGDEFAULT_CH4SET,
    ADS1299_REGDEFAULT_CH5SET,
    ADS1299_REGDEFAULT_CH6SET,
    ADS1299_REGDEFAULT_CH7SET,
    ADS1299_REGDEFAULT_CH8SET,
    ADS1299_REGDEFAULT_BIAS_SENSP,
    ADS1299_REGDEFAULT_BIAS_SENSN,
    ADS1299_REGDEFAULT_LOFF_SENSP,
    ADS1299_REGDEFAULT_LOFF_SENSN,
    ADS1299_REGDEFAULT_LOFF_FLIP,
    ADS1299_REGDEFAULT_LOFF_STATP,
    ADS1299_REGDEFAULT_LOFF_STATN,
    ADS1299_REGDEFAULT_GPIO,
    ADS1299_REGDEFAULT_MISC1,
    ADS1299_REGDEFAULT_MISC2,
    ADS1299_REGDEFAULT_CONFIG4
};

/*! @brief Apply ADS clock-output policy for multi-device topology.
 * @param[in]     p_eeg       EEG device context.
 * @param[in,out] reg_values  Register image to update.
 * @return None.
 */
static void ads1299_apply_clock_output_policy(ble_eeg_t *p_eeg, uint8_t *reg_values)
{
    if ((p_eeg == NULL) || (reg_values == NULL))
    {
        return;
    }

#if (NUM_OF_ADS1299 == 2)
    if (p_eeg->dev_idx == 0u)
    {
        /* ADS#1 is the clock master: drive CLK to ADS#2. */
        reg_values[0] |= ADS1299_CONFIG1_CLK_EN_Msk;
    }
    else
    {
        /* ADS#2 must not drive the shared clock line. */
        reg_values[0] &= (uint8_t)~ADS1299_CONFIG1_CLK_EN_Msk;
    }
#endif
}

/*! @brief Enforce safe register constraints for ADS1299 writes.
 * @param[in]     p_eeg       EEG device context.
 * @param[in,out] reg_values  Register image to sanitize.
 * @return None.
 */
static void ads1299_apply_register_safety_policy(ble_eeg_t *p_eeg, uint8_t *reg_values)
{
    if ((p_eeg == NULL) || (reg_values == NULL))
    {
        return;
    }

    /* MISC1 only exposes SRB1 at bit 5; keep reserved bits cleared. */
    reg_values[ADS1299_REGIDX_MISC1] &= ADS1299_MISC1_SRB1_Msk;
#if COMMON_REF
    if (p_eeg->dev_idx == 0u)
    {
        reg_values[ADS1299_REGIDX_MISC1] |= ADS1299_MISC1_SRB1_Msk;
    }
#endif

    /* If any enabled channel uses test-signal MUX, force INT_CAL on CONFIG2. */
    bool test_signal_in_use = false;
    uint8_t channels_to_check = p_eeg->num_of_enabled_ch;
    if (channels_to_check > ADS1299_MAX_DATA_CHANNELS)
    {
        channels_to_check = ADS1299_MAX_DATA_CHANNELS;
    }

    for (uint8_t ch_idx = 0u; ch_idx < channels_to_check; ch_idx++)
    {
        uint8_t chset = reg_values[ADS1299_REGIDX_CH1SET + ch_idx];
        if (((chset & CH_PD_MASK) == 0u) &&
            ((chset & ADS1299_CHSET_MUX_Msk) == ADS1299_CHSET_MUX_TEST))
        {
            test_signal_in_use = true;
            break;
        }
    }

    if (test_signal_in_use &&
        ((reg_values[ADS1299_REGIDX_CONFIG2] & ADS1299_CONFIG2_INT_CAL_Msk) == 0u))
    {
        reg_values[ADS1299_REGIDX_CONFIG2] |= ADS1299_CONFIG2_INT_CAL_Msk;
        NRF_LOG_WARNING("ADS%u: CONFIG2.INT_CAL forced to 1 (test MUX active).",
                        (unsigned)(p_eeg->dev_idx + 1u));
    }
}

/*! @brief Get read-data offset for ADS register readback frames.
 * @return Byte offset where register data starts in received frame.
 */
static uint8_t ads1299_reg_read_data_offset(void)
{
//#if defined(SPI0_USE_EASY_DMA) && (SPI0_USE_EASY_DMA == 1)
//    return 3u;
//#else
    return 2u;
//#endif
}

/*! @brief Configure ADS1299 SPI peripheral parameters.
 * @param[in] frequency  SPI clock frequency to apply.
 * @return None.
 */
static void ads1299_spi_configure(nrf_drv_spi_frequency_t frequency)
{
    nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
    spi_config.bit_order = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST;
    spi_config.frequency = frequency;
    spi_config.irq_priority = APP_IRQ_PRIORITY_HIGHEST;
    spi_config.mode = NRF_DRV_SPI_MODE_1;
    spi_config.miso_pin = ADS1299_SPI_DOUT_PIN;
    spi_config.sck_pin = ADS1299_SPI_SCLK_PIN;
    spi_config.mosi_pin = ADS1299_SPI_DIN_PIN;
    spi_config.ss_pin = NRFX_SPIM_PIN_NOT_USED;
    /* Keep RX-only transfers command-safe outside RDATAC mode (0x00 = NOP). */
    spi_config.orc = 0x00;

#if defined(FAST_SPI_ENABLED) && FAST_SPI_ENABLED == 1
    APP_ERROR_CHECK_BOOL(0);
#else
    ret_code_t err_code = nrf_drv_spi_init(&spi, &spi_config, NULL, NULL);
    if (err_code == NRF_ERROR_INVALID_STATE)
    {
        // SPI already initialized: reconfigure by uninitializing then retrying.
        nrf_drv_spi_uninit(&spi);
        err_code = nrf_drv_spi_init(&spi, &spi_config, NULL, NULL);
    }
    APP_ERROR_CHECK(err_code);
#endif
}

/*! @brief Perform one SPI transaction with ADS1299 device.
 * @param[in]  p_eeg    EEG device context.
 * @param[in]  tx_data  TX payload pointer (may be NULL when tx_len is 0).
 * @param[in]  tx_len   Number of TX bytes.
 * @param[out] rx_buf   RX buffer pointer (may be NULL when rx_len is 0).
 * @param[in]  rx_len   Number of RX bytes.
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
static ret_code_t ads1299_spi_transfer(ble_eeg_t *p_eeg,
                                       uint8_t const *tx_data,
                                       uint8_t tx_len,
                                       uint8_t *rx_buf,
                                       uint8_t rx_len)
{
    if (p_eeg == NULL)
    {
        return NRF_ERROR_NULL;
    }
    if ((tx_len == 0u) && (rx_len == 0u))
    {
        return NRF_ERROR_INVALID_LENGTH;
    }

#if defined(FAST_SPI_ENABLED) && FAST_SPI_ENABLED == 1
    return NRF_ERROR_NOT_SUPPORTED;
#else
    nrf_gpio_pin_clear(p_eeg->dev_cs);
    ret_code_t err_code = nrf_drv_spi_transfer(&spi, tx_data, tx_len, rx_buf, rx_len);
    nrf_gpio_pin_set(p_eeg->dev_cs);
    return err_code;
#endif
}

/*! @brief Drive START pin high to begin ADS conversions.
 * @return None.
 */
void ads1299_start_conv(void)
{
    nrf_gpio_pin_set(ADS1299_START_PIN);
    nrf_delay_us(10);
    NRF_LOG_INFO("ADS1299-x conversion started using START pin.");
}

/*! @brief Drive START pin low to stop ADS conversions.
 * @return None.
 */
void ads1299_stop_conv(void)
{
    nrf_gpio_pin_clear(ADS1299_START_PIN);
    NRF_LOG_INFO("ADS1299-x conversion stopped using START pin.");
}

/*! @brief Initialize ADS SPI interface with default frequency.
 * @return None.
 */
void ads1299_spi_init(void)
{
    ads1299_spi_configure(NRF_DRV_SPI_FREQ_4M);
    NRF_LOG_INFO("ADS1299 SPI initialized.");
}

/*! @brief Uninitialize ADS SPI interface.
 * @return None.
 */
void ads1299_spi_uninit(void)
{
#if defined(FAST_SPI_ENABLED) && FAST_SPI_ENABLED == 1
#else
    nrf_drv_spi_uninit(&spi);
#endif
    NRF_LOG_INFO("ADS1299 SPI uninitialized.");
}

/*! @brief Initialize ADS SPI interface using selectable SCLK preset.
 * @param[in] spi_sclk  SCLK selector (0/1/2/4/8 maps to 0.5/1/2/4/8 MHz).
 * @return None.
 */
void ads1299_spi_init_with_sample_freq(uint8_t spi_sclk)
{
    nrf_drv_spi_frequency_t frequency = NRF_DRV_SPI_FREQ_1M;
    switch (spi_sclk)
    {
    case 0:
        frequency = NRF_DRV_SPI_FREQ_500K;
        break;
    case 1:
        frequency = NRF_DRV_SPI_FREQ_1M;
        break;
    case 2:
        frequency = NRF_DRV_SPI_FREQ_2M;
        break;
    case 4:
        frequency = NRF_DRV_SPI_FREQ_4M;
        break;
    case 8:
        frequency = NRF_DRV_SPI_FREQ_8M;
        break;
    default:
        NRF_LOG_WARNING("Unsupported ADS SPI SCLK selector: %d. Falling back to 1 MHz.", spi_sclk);
        break;
    }

    ads1299_spi_configure(frequency);
    NRF_LOG_INFO("ADS1299 SPI initialized (SCLK selector = %d).", spi_sclk);
}

/*! @brief Initialize ADS device structures and channel buffers.
 * @return None.
 */
void ads1299_struct_init(void)
{
    uint8_t dev0_enabled_ch = (NUM_OF_ADS_CH > MAX_NUM_OF_ADS_CH_PER_DEV) ? MAX_NUM_OF_ADS_CH_PER_DEV : NUM_OF_ADS_CH;

    m_eeg[0].dev_idx = 0;
    m_eeg[0].dev_cs = ADS1299_1_SPI_CS_PIN;
    m_eeg[0].drdy_status = false;
    m_eeg[0].eeg_ch1_count = 0;
    m_eeg[0].tx_rr_next_ch_idx = 0u;
    m_eeg[0].num_of_enabled_ch = dev0_enabled_ch;
    for (uint8_t ch_idx = 0u; ch_idx < MAX_NUM_OF_ADS_CH_PER_DEV; ch_idx++) {
        m_eeg[0].eeg_buffer[ch_idx] = NULL;
    }
    for (uint8_t ch_idx = 0u; ch_idx < m_eeg[0].num_of_enabled_ch; ch_idx++) {
        m_eeg[0].eeg_buffer[ch_idx] = (uint8_t *)malloc(EEG_PACKET_LENGTH);
        APP_ERROR_CHECK_BOOL(m_eeg[0].eeg_buffer[ch_idx] != NULL);
    }

#if (NUM_OF_ADS1299 == 2)
    m_eeg[1].dev_idx = 1;
    m_eeg[1].dev_cs = ADS1299_2_SPI_CS_PIN;
    m_eeg[1].drdy_status = false;
    m_eeg[1].eeg_ch1_count = 0;
    m_eeg[1].tx_rr_next_ch_idx = 0u;
    m_eeg[1].num_of_enabled_ch = (NUM_OF_ADS_CH > dev0_enabled_ch) ? (uint8_t)(NUM_OF_ADS_CH - dev0_enabled_ch) : 0u;
    if (m_eeg[1].num_of_enabled_ch > MAX_NUM_OF_ADS_CH_PER_DEV) {
        m_eeg[1].num_of_enabled_ch = MAX_NUM_OF_ADS_CH_PER_DEV;
    }
    for (uint8_t ch_idx = 0u; ch_idx < MAX_NUM_OF_ADS_CH_PER_DEV; ch_idx++) {
        m_eeg[1].eeg_buffer[ch_idx] = NULL;
    }
    for (uint8_t ch_idx = 0u; ch_idx < m_eeg[1].num_of_enabled_ch; ch_idx++) {
        m_eeg[1].eeg_buffer[ch_idx] = (uint8_t *)malloc(EEG_PACKET_LENGTH);
        APP_ERROR_CHECK_BOOL(m_eeg[1].eeg_buffer[ch_idx] != NULL);
    }
#endif
}

/*! @brief Power down ADS1299 hardware pins.
 * @return None.
 */
void ads1299_powerdn(void)
{
#if defined(BOARD_PCA10028) || defined(BOARD_NRF_BREAKOUT) || defined(BOARD_PCA10040) || defined(BOARD_ADS1299_MAX30102)
    nrf_gpio_pin_clear(ADS1299_PWDN_RST_PIN);
#endif
#if defined(BOARD_FULL_EEG_V1)
    nrf_gpio_pin_clear(ADS1299_RESET_PIN);
    nrf_gpio_pin_clear(ADS1299_PWDN_PIN);
#endif
    nrf_delay_us(20);
    NRF_LOG_INFO("ADS1299-x powered down.");
}

/*! @brief Execute ADS hardware power-up reset sequence.
 * @return None.
 */
void ads1299_powerup_reset(void)
{
#if defined(BOARD_PCA10028) || defined(BOARD_NRF_BREAKOUT) || defined(BOARD_PCA10040) || defined(BOARD_ADS1299_MAX30102)
    nrf_gpio_pin_clear(ADS1299_PWDN_RST_PIN);
    nrf_delay_us(20);
    nrf_gpio_pin_set(ADS1299_PWDN_RST_PIN);
#endif
#if defined(BOARD_FULL_EEG_V1)
    nrf_gpio_pin_clear(ADS1299_RESET_PIN);
    nrf_gpio_pin_clear(ADS1299_PWDN_PIN);
    nrf_delay_us(20);
    nrf_gpio_pin_set(ADS1299_RESET_PIN);
    nrf_gpio_pin_set(ADS1299_PWDN_PIN);
#endif
    nrf_delay_ms(150);
    NRF_LOG_INFO("ADS1299-x power-up reset complete.");
}

/*! @brief Power up ADS1299 hardware pins.
 * @return None.
 */
void ads1299_powerup(void)
{
#if defined(BOARD_PCA10028) || defined(BOARD_NRF_BREAKOUT) || defined(BOARD_PCA10040) || defined(BOARD_ADS1299_MAX30102)
    nrf_gpio_pin_set(ADS1299_PWDN_RST_PIN);
#endif
#if defined(BOARD_FULL_EEG_V1)
    nrf_gpio_pin_set(ADS1299_RESET_PIN);
    nrf_gpio_pin_set(ADS1299_PWDN_PIN);
#endif
    nrf_delay_ms(150);
    NRF_LOG_INFO("ADS1299-x powered up.");
}

/*! @brief Send ADS standby opcode.
 * @param[in] p_eeg  EEG device context.
 * @return None.
 */
void ads1299_standby(ble_eeg_t *p_eeg)
{
    uint8_t cmd = ADS1299_OPC_STANDBY;
    APP_ERROR_CHECK(ads1299_spi_transfer(p_eeg, &cmd, (uint8_t)sizeof(cmd), NULL, 0u));
    NRF_LOG_INFO("ADS1299-x standby.");
}

/*! @brief Send ADS wake opcode.
 * @param[in] p_eeg  EEG device context.
 * @return None.
 */
void ads1299_wake(ble_eeg_t *p_eeg)
{
    uint8_t cmd = ADS1299_OPC_WAKEUP;
    APP_ERROR_CHECK(ads1299_spi_transfer(p_eeg, &cmd, (uint8_t)sizeof(cmd), NULL, 0u));
    nrf_delay_ms(10);
    NRF_LOG_INFO("ADS1299-x wake.");
}

/*! @brief Send ADS reset opcode.
 * @param[in] p_eeg  EEG device context.
 * @return None.
 */
void ads1299_reset(ble_eeg_t *p_eeg)
{
    uint8_t cmd = ADS1299_OPC_RESET;
    APP_ERROR_CHECK(ads1299_spi_transfer(p_eeg, &cmd, (uint8_t)sizeof(cmd), NULL, 0u));
    nrf_delay_ms(10);
    NRF_LOG_INFO("ADS1299-x reset.");
}

/*! @brief Send ADS START opcode (software start).
 * @param[in] p_eeg  EEG device context.
 * @return None.
 */
void ads1299_soft_start_conversion(ble_eeg_t *p_eeg)
{
    uint8_t cmd = ADS1299_OPC_START;
    APP_ERROR_CHECK(ads1299_spi_transfer(p_eeg, &cmd, (uint8_t)sizeof(cmd), NULL, 0u));
    NRF_LOG_INFO("ADS1299-x start conversion opcode sent.");
}

/*! @brief Disable ADS continuous read mode (SDATAC).
 * @param[in] p_eeg  EEG device context.
 * @return None.
 */
void ads1299_stop_rdatac(ble_eeg_t *p_eeg)
{
    uint8_t cmd = ADS1299_OPC_SDATAC;
    APP_ERROR_CHECK(ads1299_spi_transfer(p_eeg, &cmd, (uint8_t)sizeof(cmd), NULL, 0u));
    nrf_delay_us(3);
    NRF_LOG_INFO("ADS1299-x RDATAC disabled.");
}

/*! @brief Enable ADS continuous read mode (RDATAC).
 * @param[in] p_eeg  EEG device context.
 * @return None.
 */
void ads1299_start_rdatac(ble_eeg_t *p_eeg)
{
    uint8_t cmd = ADS1299_OPC_RDATAC;
    APP_ERROR_CHECK(ads1299_spi_transfer(p_eeg, &cmd, (uint8_t)sizeof(cmd), NULL, 0u));
    nrf_delay_us(3);
    NRF_LOG_INFO("ADS1299-x RDATAC enabled.");
}

/*! @brief Read and validate ADS device ID register.
 * @param[in] p_eeg  EEG device context.
 * @return None.
 */
void ads1299_check_id(ble_eeg_t *p_eeg)
{
    uint8_t tx_data_spi[3];
    uint8_t rx_data_spi[8] = {0};
    uint8_t rx_offset = ads1299_reg_read_data_offset();

    tx_data_spi[0] = (uint8_t)(ADS1299_OPC_RREG | ADS1299_REGADDR_ID);
    tx_data_spi[1] = 0x00u;
    tx_data_spi[2] = 0x00u;

    APP_ERROR_CHECK(ads1299_spi_transfer(p_eeg,
                                         tx_data_spi,
                                         (uint8_t)sizeof(tx_data_spi),
                                         rx_data_spi,
                                         (uint8_t)(rx_offset + 1u)));

    uint8_t device_id_reg_value = rx_data_spi[rx_offset];
    bool is_ads_1299_4 = ((device_id_reg_value & 0x1Fu) == ADS1299_4_DEVICE_ID);
    bool is_ads_1299_6 = ((device_id_reg_value & 0x1Fu) == ADS1299_6_DEVICE_ID);
    bool is_ads_1299 = ((device_id_reg_value & 0x1Fu) == ADS1299_DEVICE_ID);
    uint8_t revision_version = (uint8_t)((device_id_reg_value & 0xE0u) >> 5);

    if (is_ads_1299 || is_ads_1299_6 || is_ads_1299_4)
    {
        NRF_LOG_INFO("ADS1299 device detected: ID=0x%X rev=%u", device_id_reg_value, revision_version);
    }
    else
    {
        NRF_LOG_ERROR("ADS1299 device ID mismatch: 0x%X", device_id_reg_value);
    }
}

/*! @brief Write ADS register image from caller-provided values.
 * @param[in,out] p_eeg                 EEG device context.
 * @param[in]     new_register_values   Register image to program.
 * @return None.
 */
void ads1299_init_regs(ble_eeg_t *p_eeg, uint8_t *new_register_values)
{
    if ((p_eeg == NULL) || (new_register_values == NULL))
    {
        return;
    }

    uint8_t tx_data_spi[ADS1299_WRITABLE_REG_COUNT + 2u];

    memcpy(p_eeg->ads1299_current_configuration, new_register_values, ADS1299_WRITABLE_REG_COUNT);
    ads1299_apply_clock_output_policy(p_eeg, p_eeg->ads1299_current_configuration);
    ads1299_apply_register_safety_policy(p_eeg, p_eeg->ads1299_current_configuration);
    tx_data_spi[0] = (uint8_t)(ADS1299_OPC_WREG | ADS1299_REGADDR_CONFIG1);
    tx_data_spi[1] = ADS1299_WRITABLE_REG_COUNT - 1u;
    memcpy(&tx_data_spi[2], p_eeg->ads1299_current_configuration, ADS1299_WRITABLE_REG_COUNT);

    APP_ERROR_CHECK(ads1299_spi_transfer(p_eeg, tx_data_spi, (uint8_t)sizeof(tx_data_spi), NULL, 0u));
    nrf_delay_us(5);
    NRF_LOG_INFO("ADS1299 register configuration updated (device CS pin: %u).", p_eeg->dev_cs);
}

/*! @brief Program ADS default register image.
 * @param[in,out] p_eeg  EEG device context.
 * @return None.
 */
void ads1299_init_regs_default(ble_eeg_t *p_eeg)
{
    if (p_eeg == NULL)
    {
        return;
    }

    uint8_t tx_data_spi[ADS1299_WRITABLE_REG_COUNT + 2u];

    memcpy(p_eeg->ads1299_current_configuration, ads1299_default_registers, ADS1299_WRITABLE_REG_COUNT);
    ads1299_apply_clock_output_policy(p_eeg, p_eeg->ads1299_current_configuration);
#if COMMON_REF
    if (p_eeg->dev_idx == 0) // ADS1299 device 1
    {
        p_eeg->ads1299_current_configuration[20u] = 0x20;
    }
#if (NUM_OF_ADS1299 == 2)
    // turn off BIAS
    if (p_eeg->dev_idx == 1)
    {
        p_eeg->ads1299_current_configuration[2u] = 0xE8u;
        for (int regidx = 4; regidx < 4 + p_eeg->num_of_enabled_ch; regidx++) {
          // SRB2 is the reference for the 2nd device
          p_eeg->ads1299_current_configuration[regidx] |= 0x30u | SRB2_MASK; 
        }

        for (int ch_idx = 4 + p_eeg->num_of_enabled_ch; ch_idx < 12; ch_idx++) {
            // turn off all non-required channels
            p_eeg->ads1299_current_configuration[ch_idx] |= CH_PD_MASK;
        }
    }
#endif 
                 #endif
    ads1299_apply_register_safety_policy(p_eeg, p_eeg->ads1299_current_configuration);
    tx_data_spi[0] = (uint8_t)(ADS1299_OPC_WREG | ADS1299_REGADDR_CONFIG1);
    tx_data_spi[1] = ADS1299_WRITABLE_REG_COUNT - 1u;
    memcpy(&tx_data_spi[2], p_eeg->ads1299_current_configuration, ADS1299_WRITABLE_REG_COUNT);

    APP_ERROR_CHECK(ads1299_spi_transfer(p_eeg, tx_data_spi, (uint8_t)sizeof(tx_data_spi), NULL, 0u));
    nrf_delay_us(5);
    NRF_LOG_INFO("ADS1299 default registers programmed.");
}

/*! @brief Read all ADS writable registers and verify against shadow copy.
 * @param[in,out] p_eeg  EEG device context.
 * @return None.
 */
void ads1299_read_all_registers(ble_eeg_t *p_eeg)
{
    if (p_eeg == NULL)
    {
        return;
    }

    uint8_t rx_offset = ads1299_reg_read_data_offset();
    uint8_t rx_len = (uint8_t)(ADS1299_WRITABLE_REG_COUNT + rx_offset);
    uint8_t tx_data_spi[ADS1299_WRITABLE_REG_COUNT + 3u] = {0u};
    uint8_t rx_data_spi[ADS1299_WRITABLE_REG_COUNT + 3u] = {0u};

    tx_data_spi[0] = (uint8_t)(ADS1299_OPC_RREG | ADS1299_REGADDR_CONFIG1);
    tx_data_spi[1] = ADS1299_WRITABLE_REG_COUNT - 1u;

    APP_ERROR_CHECK(ads1299_spi_transfer(p_eeg, tx_data_spi, rx_len, rx_data_spi, rx_len));
    NRF_LOG_HEXDUMP_DEBUG(rx_data_spi, rx_len);

    uint8_t mismatch_count = 0u;
    for (uint8_t i = 0; i < ADS1299_WRITABLE_REG_COUNT; i++)
    {
        if (rx_data_spi[rx_offset + i] != p_eeg->ads1299_current_configuration[i])
        {
            mismatch_count++;
        }
    }

    if (mismatch_count != 0u)
    {
        NRF_LOG_WARNING("ADS1299 register verify failed (%u mismatches).", mismatch_count);
        NRF_LOG_HEXDUMP_DEBUG(&rx_data_spi[rx_offset], ADS1299_WRITABLE_REG_COUNT);
    }
    else
    {
        NRF_LOG_INFO("ADS1299 register verify successful.");
    }

    // memcpy(p_eeg->ads1299_current_configuration, &rx_data_spi[rx_offset], ADS1299_WRITABLE_REG_COUNT);
}

/*! @brief Read one-channel EEG sample frame into channel buffer.
 * @param[in,out] p_eeg  EEG device context.
 * @return None.
 */
void get_eeg_voltage_array(ble_eeg_t *p_eeg)
{
    if (p_eeg == NULL)
    {
        return;
    }
    uint8_t *rx_data = (p_eeg->dev_idx == 0u) ? rx_data_dev0 : rx_data_dev1;

    if (p_eeg->eeg_ch1_count > (EEG_PACKET_LENGTH - BYTES_PER_CHANNEL))
    {
        return;
    }

    memset(rx_data, 0, RX_DATA_LEN);
    APP_ERROR_CHECK(ads1299_spi_transfer(p_eeg, NULL, 0u, rx_data, (uint8_t)RX_DATA_LEN));
    memcpy(&p_eeg->eeg_buffer[0][p_eeg->eeg_ch1_count], &rx_data[3], BYTES_PER_CHANNEL);
    p_eeg->eeg_ch1_count += BYTES_PER_CHANNEL;
}

/*! @brief Read four-channel EEG sample frame into channel buffers.
 * @param[in,out] p_eeg  EEG device context.
 * @return None.
 */
void get_eeg_voltage_array_4ch(ble_eeg_t *p_eeg)
{
    if (p_eeg == NULL)
    {
        return;
    }
    uint8_t *rx_data = (p_eeg->dev_idx == 0u) ? rx_data_dev0 : rx_data_dev1;

    if (p_eeg->eeg_ch1_count > (EEG_PACKET_LENGTH - BYTES_PER_CHANNEL))
    {
        return;
    }

    memset(rx_data, 0, RX_DATA_LEN);
    APP_ERROR_CHECK(ads1299_spi_transfer(p_eeg, NULL, 0u, rx_data, (uint8_t)RX_DATA_LEN));
    memcpy(&p_eeg->eeg_buffer[0][p_eeg->eeg_ch1_count], &rx_data[3], BYTES_PER_CHANNEL);
    memcpy(&p_eeg->eeg_buffer[1][p_eeg->eeg_ch1_count], &rx_data[6], BYTES_PER_CHANNEL);
    memcpy(&p_eeg->eeg_buffer[2][p_eeg->eeg_ch1_count], &rx_data[9], BYTES_PER_CHANNEL);
    memcpy(&p_eeg->eeg_buffer[3][p_eeg->eeg_ch1_count], &rx_data[12], BYTES_PER_CHANNEL);
    p_eeg->eeg_ch1_count += BYTES_PER_CHANNEL;
}

/*! @brief Read all enabled EEG channels into channel buffers.
 * @param[in,out] p_eeg  EEG device context.
 * @return None.
 */
void get_eeg_voltage_array_all_ch(ble_eeg_t *p_eeg)
{
    if ((p_eeg == NULL) || (p_eeg->num_of_enabled_ch == 0u) || (p_eeg->num_of_enabled_ch > NUM_OF_ADS_CH))
    {
        return;
    }
    uint8_t *rx_data = (p_eeg->dev_idx == 0u) ? rx_data_dev0 : rx_data_dev1;

    if (p_eeg->eeg_ch1_count > (EEG_PACKET_LENGTH - BYTES_PER_CHANNEL))
    {
        return;
    }

    /* RDATAC frame is fixed: 3 status bytes + 8 channels x 3 bytes.
     * Always read full frame to preserve byte alignment. */
    memset(rx_data, 0, RX_DATA_LEN);
    APP_ERROR_CHECK(ads1299_spi_transfer(p_eeg, NULL, 0u, rx_data, (uint8_t)RX_DATA_LEN));

    for (uint16_t i = 0; i < p_eeg->num_of_enabled_ch; i++)
    {
        memcpy(&p_eeg->eeg_buffer[i][p_eeg->eeg_ch1_count], &rx_data[BYTES_PER_CHANNEL * i + ADS1299_STATUS_BYTES], BYTES_PER_CHANNEL);
    }
    p_eeg->eeg_ch1_count += BYTES_PER_CHANNEL;
}

/*! @brief Read specific channel subset into primary EEG buffer.
 * @param[in,out] p_eeg  EEG device context.
 * @return None.
 */
void get_eeg_voltage_array_ch1_2(ble_eeg_t *p_eeg)
{
    if (p_eeg == NULL)
    {
        return;
    }
    uint8_t *rx_data = (p_eeg->dev_idx == 0u) ? rx_data_dev0 : rx_data_dev1;

    if (p_eeg->eeg_ch1_count > (EEG_PACKET_LENGTH - BYTES_PER_CHANNEL))
    {
        return;
    }

    memset(rx_data, 0, RX_DATA_LEN);
    APP_ERROR_CHECK(ads1299_spi_transfer(p_eeg, NULL, 0u, rx_data, (uint8_t)RX_DATA_LEN));

    for (uint8_t ch = 0; ch < 4u; ch++)
    {
        if (p_eeg->ads1299_current_configuration[4u + ch] != 0xF1u)
        {
            memcpy(&p_eeg->eeg_buffer[0][p_eeg->eeg_ch1_count], &rx_data[3u + (BYTES_PER_CHANNEL * ch)], BYTES_PER_CHANNEL);
            p_eeg->eeg_ch1_count += BYTES_PER_CHANNEL;
        }
    }
}
