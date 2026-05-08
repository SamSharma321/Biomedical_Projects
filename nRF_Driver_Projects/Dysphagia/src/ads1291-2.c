/* Edited By: Sameera Sharma for BITN */
#ifdef __cplusplus
extern "C" {
#endif

#include "ads1291-2.h"

#include "app_error.h"
#include "app_util_platform.h"
#include "ble_eeg.h"
#include "nrf_delay.h"
#include "nrf_drv_spi.h"
#include "nrf_gpio.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "usr_defined.h"
/**headers for s delay:*/
#include <string.h>

uint8_t ads1291_2_default_regs[] = {
    ADS1291_2_REGDEFAULT_CONFIG1,   ADS1291_2_REGDEFAULT_CONFIG2,
    ADS1291_2_REGDEFAULT_LOFF,      ADS1291_2_REGDEFAULT_CH1SET,
    ADS1291_2_REGDEFAULT_CH2SET,    ADS1291_2_REGDEFAULT_RLD_SENS,
    ADS1291_2_REGDEFAULT_LOFF_SENS, ADS1291_2_REGDEFAULT_LOFF_STAT,
    ADS1291_2_REGDEFAULT_RESP1,     ADS1291_2_REGDEFAULT_RESP2,
    ADS1291_2_REGDEFAULT_GPIO};
#if defined(FAST_SPI_ENABLED) && FAST_SPI_ENABLED == 1
#else
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(0);
#endif
#define RX_DATA_LEN 9
static uint8_t rx_data[RX_DATA_LEN];
static volatile bool spi_xfer_done;
static uint8_t ads_frame_len_bytes = RX_DATA_LEN;
static uint16_t ch1_zero_streak = 0;
static uint16_t debug_frame_counter = 0;

static void ads1291_2_write_reg_once(uint8_t reg_addr, uint8_t reg_val) {
  uint8_t tx_data_spi[3] = {0};
  uint8_t rx_data_spi[3] = {0};
  tx_data_spi[0] = ADS1291_2_OPC_WREG | reg_addr;
  tx_data_spi[1] = 0x00;
  tx_data_spi[2] = reg_val;

  spi_xfer_done = false;
#if defined(FAST_SPI_ENABLED) && FAST_SPI_ENABLED == 1
  spi_master_tx_rx(SPI0, 3, tx_data_spi, rx_data_spi);
#else
  APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, tx_data_spi, 3, rx_data_spi, 3));
  while (!spi_xfer_done) {
    __WFE();
  }
#endif
}

static uint8_t ads1291_2_read_reg_once(uint8_t reg_addr) {
  uint8_t tx_data_spi[3] = {0};
  uint8_t rx_data_spi[3] = {0};
  tx_data_spi[0] = ADS1291_2_OPC_RREG | reg_addr;
  tx_data_spi[1] = 0x00;
  tx_data_spi[2] = 0x00;

  spi_xfer_done = false;
#if defined(FAST_SPI_ENABLED) && FAST_SPI_ENABLED == 1
  spi_master_tx_rx(SPI0, 3, tx_data_spi, rx_data_spi);
#else
  APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, tx_data_spi, 3, rx_data_spi, 3));
  while (!spi_xfer_done) {
    __WFE();
  }
#endif

  return rx_data_spi[2];
}

static uint8_t ads1291_2_verify_mask_for_reg(uint8_t reg_addr) {
  switch (reg_addr) {
    case ADS1291_2_REGADDR_LOFF_STAT:
      // LOFF_STAT contains live lead-off state; only CLK_DIV (bit6) is
      // writable.
      return 0x40;
    default:
      return 0xFF;
  }
}

/**
 * @brief SPI user event handler.
 * @param event
 */
void spi_event_handler(nrf_drv_spi_evt_t const *p_event, void *p_context) {
  spi_xfer_done = true;
}
#if defined(FAST_SPI_ENABLED) && FAST_SPI_ENABLED == 1
#else
#endif
/**@INITIALIZE SPI INSTANCE */
void ads_spi_init(void) {
#if defined(FAST_SPI_ENABLED) && FAST_SPI_ENABLED == 1
#else
  nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
  spi_config.bit_order = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST;
  // SCLK = 1MHz is right speed because fCLK = (1/2)*SCLK, and fMOD = fCLK/4,
  // and fMOD MUST BE 128kHz. Do the math.
  spi_config.frequency = NRF_DRV_SPI_FREQ_1M;
  spi_config.irq_priority =
      APP_IRQ_PRIORITY_HIGHEST;  // APP_IRQ_PRIORITY_HIGHEST;
  spi_config.mode =
      NRF_DRV_SPI_MODE_1;  // CPOL = 0 (Active High); CPHA = TRAILING (1)
  spi_config.miso_pin = ADS1291_2_MISO_PIN;
  spi_config.sck_pin = ADS1291_2_SCK_PIN;
  spi_config.mosi_pin = ADS1291_2_MOSI_PIN;
  spi_config.ss_pin = ADS1291_2_SS_PIN;
  spi_config.orc = 0x00;
  APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL));
#endif
}

void ads_spi_uninit(void) {
#if defined(FAST_SPI_ENABLED) && FAST_SPI_ENABLED == 1

#else
  nrf_drv_spi_uninit(&spi);
  NRF_LOG_INFO(" SPI UNinitialized \r\n");
#endif
}

void ads_spi_init_with_sample_freq(uint8_t spi_sclk) {
#if defined(FAST_SPI_ENABLED) && FAST_SPI_ENABLED == 1
  SPI_config_t spi_config = {.pin_SCK = ADS1291_2_SCK_PIN,
                             .pin_MOSI = ADS1291_2_MOSI_PIN,
                             .pin_MISO = ADS1291_2_MISO_PIN,
                             .pin_CSN = ADS1291_2_SS_PIN,
                             .frequency = SPI_FREQ_4MBPS,
                             .config.fields.mode = 1,
                             .config.fields.bit_order = SPI_BITORDER_MSB_LSB};

  if (spi_sclk == 0) {
    spi_config.frequency = SPI_FREQ_500KBPS;
    spi_master_init(SPI0, &spi_config);
  } else if (spi_sclk == 4 || spi_sclk == 8) {
    spi_config.frequency = SPI_FREQ_4MBPS;
    spi_set_frequency(&spi_config);
  }

#else
  nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
  switch (spi_sclk) {
    case 0:
      spi_config.frequency = NRF_DRV_SPI_FREQ_500K;
      break;
    case 1:
      spi_config.frequency = NRF_DRV_SPI_FREQ_1M;
      break;
    case 2:
      spi_config.frequency = NRF_DRV_SPI_FREQ_2M;
      break;
    case 4:
      spi_config.frequency = NRF_DRV_SPI_FREQ_4M;
      break;
    case 8:
      spi_config.frequency = NRF_DRV_SPI_FREQ_8M;
      break;
    default:
      break;
  }
  spi_config.irq_priority =
      APP_IRQ_PRIORITY_HIGHEST;  // APP_IRQ_PRIORITY_HIGHEST;
  spi_config.mode =
      NRF_DRV_SPI_MODE_1;  // CPOL = 0 (Active High); CPHA = TRAILING (1)
  spi_config.miso_pin = ADS1291_2_MISO_PIN;
  spi_config.sck_pin = ADS1291_2_SCK_PIN;
  spi_config.mosi_pin = ADS1291_2_MOSI_PIN;
  spi_config.ss_pin = ADS1291_2_SS_PIN;
  spi_config.orc = 0x00;
  APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL));
  NRF_LOG_INFO(" SPI Initialized @ %d MHz\r\n", spi_sclk);
#endif
}

/* SYSTEM CONTROL FUNCTIONS
 * **********************************************************************************************************************/
void ads1291_2_verify_reg(void) {
  uint8_t num_registers = (uint8_t)(sizeof(ads1291_2_default_regs) /
                                    sizeof(ads1291_2_default_regs[0]));
  uint8_t reg_addr = 0;
  uint8_t expected = 0;
  uint8_t actual = 0;
  uint8_t verify_mask = 0xFF;
  // Verify each register readback once all writes are complete.
  for (int i = 0; i < num_registers; i++) {
    reg_addr = (uint8_t)(ADS1291_2_REGADDR_CONFIG1 + i);
    expected = ads1291_2_default_regs[i];
    actual = ads1291_2_read_reg_once(reg_addr);
    verify_mask = ads1291_2_verify_mask_for_reg(reg_addr);
    if ((actual & verify_mask) != (expected & verify_mask)) {
      NRF_LOG_ERROR(
          "ADS reg verify failed: addr=0x%x expected=0x%x "
          "actual=0x%x mask=0x%x",
          reg_addr, expected, actual, verify_mask);
      NRF_LOG_FLUSH();
      APP_ERROR_CHECK_BOOL(false);
    }
    if (reg_addr == ADS1291_2_REGADDR_LOFF_STAT) {
      NRF_LOG_INFO("ADS LOFF_STAT read=0x%x (verify mask=0x40)", actual);
    }
  }
  NRF_LOG_INFO(" ADS register initialization + verify passed.\r\n");
}

void ads1291_2_init_regs(void) {
  uint8_t i = 0;
  uint8_t num_registers = (uint8_t)(sizeof(ads1291_2_default_regs) /
                                    sizeof(ads1291_2_default_regs[0]));

  // Write each register individually.
  for (i = 0; i < num_registers; i++) {
    ads1291_2_write_reg_once(ADS1291_2_REGADDR_CONFIG1 + i,
                             ads1291_2_default_regs[i]);
  }
}

void ads1291_2_rreg(uint8_t reg_addr, uint8_t num_to_read,
                    uint8_t *read_reg_val_ptr) {
  uint8_t i = 0;
  if (read_reg_val_ptr == NULL || num_to_read == 0) {
    return;
  }
  for (i = 0; i < num_to_read; i++) {
    read_reg_val_ptr[i] = ads1291_2_read_reg_once(reg_addr + i);
  }
}

void ads1291_2_wreg(uint8_t reg_addr, uint8_t num_to_write,
                    uint8_t *write_reg_val_ptr) {
  uint8_t i = 0;
  if (write_reg_val_ptr == NULL || num_to_write == 0) {
    return;
  }
  for (i = 0; i < num_to_write; i++) {
    ads1291_2_write_reg_once(reg_addr + i, write_reg_val_ptr[i]);
  }
}

void ads1291_2_standby(void) {
  uint8_t tx_data_spi;
  uint8_t rx_data_spi;

  tx_data_spi = ADS1291_2_OPC_STANDBY;
  spi_xfer_done = false;
#if defined(FAST_SPI_ENABLED) && FAST_SPI_ENABLED == 1
  spi_master_tx_rx(SPI0, 1, &tx_data_spi, &rx_data_spi);
#else
  APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, &tx_data_spi, 1, &rx_data_spi, 1));

  while (!spi_xfer_done) {
    __WFE();
  }
#endif
#if LOG_LOW_DETAIL == 1
  NRF_LOG_INFO(" ADS1292 placed in standby mode...\r\n");
#endif
}

void ads1291_2_wake(void) {
  uint8_t tx_data_spi;
  uint8_t rx_data_spi;

  tx_data_spi = ADS1291_2_OPC_WAKEUP;
  spi_xfer_done = false;
#if defined(FAST_SPI_ENABLED) && FAST_SPI_ENABLED == 1
  spi_master_tx_rx(SPI0, 1, &tx_data_spi, &rx_data_spi);
#else
  APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, &tx_data_spi, 1, &rx_data_spi, 1));
  while (!spi_xfer_done) {
    __WFE();
  }
#endif
  nrf_delay_ms(10);  // Allow time to wake up - 10ms
#if LOG_LOW_DETAIL == 1
  NRF_LOG_INFO(" ADS1292 Wakeup..\r\n");
#endif
}

void ads1291_2_soft_start_conversion(void) {
  uint8_t tx_data_spi;
  uint8_t rx_data_spi;

  tx_data_spi = ADS1291_2_OPC_START;
  spi_xfer_done = false;
#if defined(FAST_SPI_ENABLED) && FAST_SPI_ENABLED == 1
  spi_master_tx_rx(SPI0, 1, &tx_data_spi, &rx_data_spi);
#else
  APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, &tx_data_spi, 1, &rx_data_spi, 1));
  while (!spi_xfer_done) {
    __WFE();
  }
#endif
#if LOG_LOW_DETAIL == 1
  NRF_LOG_INFO(" Start ADC conversion..\r\n");
#endif
}

void ads1291_2_stop_rdatac(void) {
  uint8_t tx_data_spi;
  uint8_t rx_data_spi;

  tx_data_spi = ADS1291_2_OPC_SDATAC;
  spi_xfer_done = false;
#if defined(FAST_SPI_ENABLED) && FAST_SPI_ENABLED == 1
  spi_master_tx_rx(SPI0, 1, &tx_data_spi, &rx_data_spi);
#else
  APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, &tx_data_spi, 1, &rx_data_spi, 1));

  while (!spi_xfer_done) {
    __WFE();
  }
#endif
#if LOG_LOW_DETAIL == 1
  NRF_LOG_INFO(" Continuous Data Output Disabled..\r\n");
#endif
}

void ads1291_2_soft_reset(void) {
  uint8_t tx_data_spi;
  uint8_t rx_data_spi;

  tx_data_spi = ADS1291_2_OPC_RESET;
  spi_xfer_done = false;
#if defined(FAST_SPI_ENABLED) && FAST_SPI_ENABLED == 1
  spi_master_tx_rx(SPI0, 1, &tx_data_spi, &rx_data_spi);
#else
  APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, &tx_data_spi, 1, &rx_data_spi, 1));

  while (!spi_xfer_done) {
    __WFE();
  }
#endif
#if LOG_LOW_DETAIL == 1
  NRF_LOG_INFO(" Continuous Data Output Disabled..\r\n");
#endif
}

void ads1291_2_start_rdatac(void) {
  uint8_t tx_data_spi;
  uint8_t rx_data_spi;

  tx_data_spi = ADS1291_2_OPC_RDATAC;
  spi_xfer_done = false;
#if defined(FAST_SPI_ENABLED) && FAST_SPI_ENABLED == 1
  spi_master_tx_rx(SPI0, 1, &tx_data_spi, &rx_data_spi);
#else
  APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, &tx_data_spi, 1, &rx_data_spi, 1));
  while (!spi_xfer_done) {
    __WFE();
  }
#endif
#if LOG_LOW_DETAIL == 1
  NRF_LOG_INFO(" Continuous Data Output Enabled..\r\n");
#endif
}

void ads1291_2_powerdn(void) {
  nrf_gpio_pin_clear(ADS1291_2_PWDN_PIN);
  nrf_delay_ms(10);
#if LOG_LOW_DETAIL == 1
  NRF_LOG_INFO(" ADS1292 POWERED DOWN..\r\n");
#endif
}

void ads1291_2_powerup(void) {
  nrf_gpio_pin_set(ADS1291_2_PWDN_PIN);
  nrf_delay_ms(1000);  // Allow time for power-on reset
#if LOG_LOW_DETAIL == 1
  NRF_LOG_INFO(" ADS1292 POWERED UP...\r\n");
#endif
}

/* DATA RETRIEVAL FUNCTIONS
 * **********************************************************************************************************************/
void ads1291_2_check_id(void) {
  uint8_t device_id;
#if defined(ADS1291)
  device_id = ADS1291_DEVICE_ID;
#elif defined(ADS1292)
  device_id = ADS1292R_DEVICE_ID;
#endif
  uint8_t device_id_reg_value = ads1291_2_read_reg_once(ADS1291_2_REGADDR_ID);
  if (device_id_reg_value != ADS1291_DEVICE_ID &&
      device_id_reg_value != ADS1292_DEVICE_ID &&
      device_id_reg_value != ADS1292R_DEVICE_ID) {
    // Fallback path for SPI/EasyDMA timing variants.
    uint8_t tx_data_spi[6] = {0x20, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t rx_data_spi[6] = {0};
    spi_xfer_done = false;
#if defined(FAST_SPI_ENABLED) && FAST_SPI_ENABLED == 1
    spi_master_tx_rx(SPI0, 6, tx_data_spi, rx_data_spi);
#else
    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, tx_data_spi, 2, rx_data_spi, 6));
    while (!spi_xfer_done) {
      __WFE();
    }
#endif
    if (rx_data_spi[2] == ADS1291_DEVICE_ID ||
        rx_data_spi[2] == ADS1292_DEVICE_ID ||
        rx_data_spi[2] == ADS1292R_DEVICE_ID) {
      device_id_reg_value = rx_data_spi[2];
    } else if (rx_data_spi[3] == ADS1291_DEVICE_ID ||
               rx_data_spi[3] == ADS1292_DEVICE_ID ||
               rx_data_spi[3] == ADS1292R_DEVICE_ID) {
      device_id_reg_value = rx_data_spi[3];
    }
  }
  // Select frame length from actual silicon ID (ADS1291: 6 bytes, ADS1292: 9
  // bytes).
  if (device_id_reg_value == ADS1291_DEVICE_ID) {
    ads_frame_len_bytes = 6;
  } else if ((device_id_reg_value == ADS1292R_DEVICE_ID) || \
   (device_id_reg_value == ADS1292_DEVICE_ID)) {
    ads_frame_len_bytes = 9;
  } else {
    NRF_LOG_INFO("ERROR: Check ID (not match): 0x%x \r\n",
                 device_id_reg_value);
    return;
  }
  NRF_LOG_INFO("SUCCESS: Check ID (match): 0x%x \r\n", device_id_reg_value);
}

void get_eeg_voltage_array_2ch(ble_eeg_t *p_eeg) {
  spi_xfer_done = false;
#if defined(FAST_SPI_ENABLED) && FAST_SPI_ENABLED == 1
  spi_master_tx_rx(SPI0, ads_frame_len_bytes, rx_data, rx_data);
#else
  nrf_drv_spi_transfer(&spi, NULL, 0, rx_data, ads_frame_len_bytes);
  while (!spi_xfer_done) __WFE();
#endif

  if ((rx_data[0] & 0xF0u) == 0xC0u) {
    if (ads_frame_len_bytes >= 9) {
      memcpy(&p_eeg->eeg_ch2_buffer[p_eeg->eeg_ch1_count], &rx_data[6], 3);
      p_eeg->eeg_ch1_count += 3;
    } else {
      // CH2 is unavailable in 6-byte ADS1291 frames; ignore this sample.
    }
  }
}

void get_eeg_voltage_array_2ch_low_resolution(ble_eeg_t *p_eeg) {
  //  memset(rx_data, 0, RX_DATA_LEN);
  spi_xfer_done = false;
//  APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, NULL, 0, rx_data, 8));
#if defined(FAST_SPI_ENABLED) && FAST_SPI_ENABLED == 1
  uint8_t tx_data[9];
  spi_master_tx_rx(SPI0, 9, tx_data, rx_data);
#else
  nrf_drv_spi_transfer(&spi, NULL, NULL, rx_data, 9);
  while (!spi_xfer_done) __WFE();
//    __WFE();
#endif
  p_eeg->eeg_ch2_buffer[p_eeg->eeg_ch1_count] = rx_data[6];
  p_eeg->eeg_ch2_buffer[p_eeg->eeg_ch1_count + 1] = rx_data[7];
  //  if (rx_data[0]!=0xC0)
  NRF_LOG_HEXDUMP_INFO(rx_data, 12 * sizeof(uint8_t));
}

void ads1291_2_main_init(bool startup) {
  // SPI STUFF FOR ADS:
  // Stop continuous data conversion and initialize registers to default values
  ads1291_2_powerdn();
  ads1291_2_powerup();
  nrf_delay_ms(5);
  ads1291_2_stop_rdatac();
  if (startup) ads1291_2_check_id();
  ads1291_2_init_regs();
  if (startup) ads1291_2_verify_reg();
  ads1291_2_soft_start_conversion();
  ads1291_2_start_rdatac();
  nrf_delay_ms(10);
  m_eeg.eeg_ch1_count = 0;
  NRF_LOG_FLUSH();
}
