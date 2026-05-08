#ifndef MAX30102_H_
#define MAX30102_H_

#include <stdint.h>

#include "ble_spo.h"
#include "sdk_errors.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MAX30102_NUM_REGS                  11u

#define MAX30102_RATE_50SPS                (0u << 2u)
#define MAX30102_RATE_100SPS               (1u << 2u)
#define MAX30102_RATE_200SPS               (2u << 2u)
#define MAX30102_RATE_400SPS               (3u << 2u)
#define MAX30102_RATE_800SPS               (4u << 2u)
#define MAX30102_RATE_1000SPS              (5u << 2u)
#define MAX30102_RATE_1600SPS              (6u << 2u)
#define MAX30102_RATE_3200SPS              (7u << 2u)

#define MAX30102_SPO2_ADC_RANGE_2048NA     (0u << 5u)
#define MAX30102_SPO2_ADC_RANGE_4096NA     (1u << 5u)
#define MAX30102_SPO2_ADC_RANGE_8192NA     (2u << 5u)
#define MAX30102_SPO2_ADC_RANGE_16384NA    (3u << 5u)

#define MAX30102_LED_PW_69US               0u
#define MAX30102_LED_PW_118US              1u
#define MAX30102_LED_PW_215US              2u
#define MAX30102_LED_PW_411US              3u

#define MAX30102_INT_A_FULL_EN             0x80u
#define MAX30102_INT_PPG_RDY_EN            0x40u

/* SpO2 mode setup tuned for waveform visibility:
 * - max ADC range to avoid clipping on stronger contact
 * - 100 sps effective sample rate
 * - 411 us LED pulse width for 18-bit resolution
 */
#define MAX30102_SPO2_SAMPLING_RATE        (MAX30102_SPO2_ADC_RANGE_16384NA | \
                                            MAX30102_RATE_100SPS |              \
                                            MAX30102_LED_PW_411US)


#define MAX30102_ADDR_INT_STAT1            0x00u
#define MAX30102_ADDR_INT_STAT2            0x01u
#define MAX30102_ADDR_INT_EN1              0x02u
#define MAX30102_ADDR_INT_EN2              0x03u
#define MAX30102_ADDR_FIFO_WRITE_POINTER   0x04u
#define MAX30102_ADDR_OVERFLOW_CNT         0x05u
#define MAX30102_ADDR_FIFO_RD_POINTER      0x06u
#define MAX30102_ADDR_FIFO_RD_REGISTER     0x07u
#define MAX30102_ADDR_FIFO_CNFG            0x08u
#define MAX30102_ADDR_MODE_CNFG            0x09u
#define MAX30102_ADDR_SPO2_CNFG            0x0Au
#define MAX30102_ADDR_LED_PULSE_AMP1       0x0Cu
#define MAX30102_ADDR_LED_PULSE_AMP2       0x0Du
#define MAX30102_ADDR_MULTI_LED_MODE_CTRL1 0x11u
#define MAX30102_ADDR_MULTI_LED_MODE_CTRL2 0x12u
#define MAX30102_ADDR_DIE_TEMP_INT         0x1Fu
#define MAX30102_ADDR_DIE_TEMP_FRAC        0x20u
#define MAX30102_ADDR_DIE_TEMP_CONFIG      0x21u
#define MAX30102_ADDR_REV_ID               0xFEu
#define MAX30102_ADDR_PART_ID              0xFFu

ret_code_t max30102_write_reg(uint8_t reg_addr, uint8_t new_value);
ret_code_t max30102_read_reg(uint8_t reg_addr, uint8_t *value);
ret_code_t max30102_reset(void);
ret_code_t max30102_init(void);
ret_code_t max30102_init_custom(uint8_t red_led, uint8_t ir_led);
ret_code_t max30102_read_fifo(ble_spo_t *p_spo);
ret_code_t max30102_clear_interrupts(void);
void max30102_debug_get_stats(uint32_t *p_irq_count,
                              uint32_t *p_read_count,
                              uint32_t *p_avg_period_us,
                              uint8_t *p_last_status1,
                              uint8_t *p_last_status2);

#ifdef __cplusplus
}
#endif

#endif
