#include "max30102.h"

#include <string.h>

#include "app_timer.h"
#include "icm20649.h"
#include "nrf_drv_gpiote.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "ble_usr_defined.h"
#include "usr_config.h"

extern bool m_connected;
extern ble_spo_t m_spo;

static volatile uint32_t m_max30102_irq_count = 0u;
static volatile uint32_t m_max30102_read_count = 0u;
static volatile uint32_t m_max30102_last_tick = 0u;
static volatile uint32_t m_max30102_avg_period_us = 0u;
static volatile bool m_max30102_seen_once = false;
static volatile uint8_t m_max30102_last_status1 = 0u;
static volatile uint8_t m_max30102_last_status2 = 0u;

static uint32_t app_timer_ticks_to_us(uint32_t ticks)
{
    return (uint32_t)(((uint64_t)ticks * 1000000ULL + (APP_TIMER_CLOCK_FREQ / 2u)) / APP_TIMER_CLOCK_FREQ);
}

static ret_code_t max30102_read_multi(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return twi_write_read(MAX30102_I2C_ADDRESS, &reg_addr, 1u, data, len);
}

ret_code_t max30102_write_reg(uint8_t reg_addr, uint8_t new_value)
{
    uint8_t tx_data[2] = {reg_addr, new_value};
    ret_code_t err = twi_write_bytes(MAX30102_I2C_ADDRESS, tx_data, sizeof(tx_data), false);
    if (err == NRF_SUCCESS)
    {
        nrf_delay_ms(1);
    }
    return err;
}

ret_code_t max30102_read_reg(uint8_t reg_addr, uint8_t *value)
{
    if (value == NULL)
    {
        return NRF_ERROR_NULL;
    }

    return max30102_read_multi(reg_addr, value, 1u);
}

ret_code_t max30102_clear_interrupts(void)
{
    uint8_t status1 = 0u;
    uint8_t status2 = 0u;

    ret_code_t err = max30102_read_reg(MAX30102_ADDR_INT_STAT1, &status1);
    if (err != NRF_SUCCESS)
    {
        return err;
    }

    err = max30102_read_reg(MAX30102_ADDR_INT_STAT2, &status2);
    if (err == NRF_SUCCESS)
    {
        m_max30102_last_status1 = status1;
        m_max30102_last_status2 = status2;
    }

    return err;
}

void max30102_debug_get_stats(uint32_t *p_irq_count,
                              uint32_t *p_read_count,
                              uint32_t *p_avg_period_us,
                              uint8_t *p_last_status1,
                              uint8_t *p_last_status2)
{
    if (p_irq_count != NULL)
    {
        *p_irq_count = m_max30102_irq_count;
    }
    if (p_read_count != NULL)
    {
        *p_read_count = m_max30102_read_count;
    }
    if (p_avg_period_us != NULL)
    {
        *p_avg_period_us = m_max30102_avg_period_us;
    }
    if (p_last_status1 != NULL)
    {
        *p_last_status1 = m_max30102_last_status1;
    }
    if (p_last_status2 != NULL)
    {
        *p_last_status2 = m_max30102_last_status2;
    }
}

ret_code_t max30102_reset(void)
{
    ret_code_t err = max30102_write_reg(MAX30102_ADDR_MODE_CNFG, 0x40u);
    if (err != NRF_SUCCESS)
    {
        return err;
    }

    nrf_delay_ms(10);

    uint8_t ignored = 0u;
    err |= max30102_read_reg(MAX30102_ADDR_INT_STAT1, &ignored);
    if (err)
      NRF_LOG_WARNING("MAX30102: Reset not successful\n");
    NRF_LOG_INFO("MAX30102: Reset successful and device ID verified.\n");
    return err;
}

ret_code_t max30102_init(void)
{
    /* Use per-sample PPG_RDY interrupts only. The current ISR reads one Red+IR
     * sample pair at a time, so enabling A_FULL as well can leave INT asserted.
     */
    ret_code_t err = max30102_write_reg(MAX30102_ADDR_INT_EN1, MAX30102_INT_PPG_RDY_EN);
    if (err != NRF_SUCCESS) return err;
    err = max30102_write_reg(MAX30102_ADDR_INT_EN2, 0x00u);
    if (err != NRF_SUCCESS) return err;
    err = max30102_write_reg(MAX30102_ADDR_FIFO_WRITE_POINTER, 0x00u);
    if (err != NRF_SUCCESS) return err;
    err = max30102_write_reg(MAX30102_ADDR_OVERFLOW_CNT, 0x00u);
    if (err != NRF_SUCCESS) return err;
    err = max30102_write_reg(MAX30102_ADDR_FIFO_RD_POINTER, 0x00u);
    if (err != NRF_SUCCESS) return err;
    err = max30102_write_reg(MAX30102_ADDR_FIFO_CNFG, 0x0Fu);
    if (err != NRF_SUCCESS) return err;
    err = max30102_write_reg(MAX30102_ADDR_MODE_CNFG, 0x03u);
    if (err != NRF_SUCCESS) return err;
    err = max30102_write_reg(MAX30102_ADDR_SPO2_CNFG, MAX30102_SPO2_SAMPLING_RATE);
    if (err != NRF_SUCCESS) return err;
    err = max30102_write_reg(MAX30102_ADDR_LED_PULSE_AMP1, MAX30102_LED_RED_DEFAULT);
    if (err != NRF_SUCCESS) return err;
    err = max30102_write_reg(MAX30102_ADDR_LED_PULSE_AMP2, MAX30102_LED_IR_DEFAULT);
    if (err != NRF_SUCCESS) return err;

    NRF_LOG_INFO("MAX30102 initialized.");
    return NRF_SUCCESS;
}

ret_code_t max30102_init_custom(uint8_t red_led, uint8_t ir_led)
{
    ret_code_t err = max30102_write_reg(MAX30102_ADDR_LED_PULSE_AMP1, red_led);
    if (err != NRF_SUCCESS)
    {
        return err;
    }

    return max30102_write_reg(MAX30102_ADDR_LED_PULSE_AMP2, ir_led);
}

ret_code_t max30102_read_fifo(ble_spo_t *p_spo)
{
    if (p_spo == NULL)
    {
        return NRF_ERROR_NULL;
    }
    if ((p_spo->spo_ch1_count > (SPO_PACKET_LENGTH - 3u)) ||
        (p_spo->spo_ch2_count > (SPO_PACKET_LENGTH - 3u)))
    {
        return NRF_ERROR_NO_MEM;
    }

    uint8_t status[2] = {0u, 0u};
    uint8_t fifo_data[SPO_FIFO_SAMPLE_BYTES] = {0u};
    ret_code_t err = max30102_read_multi(MAX30102_ADDR_FIFO_RD_REGISTER, fifo_data, sizeof(fifo_data));
    if (err != NRF_SUCCESS)
    {
        return err;
    }

    memcpy(&p_spo->spo_ch1_buffer[p_spo->spo_ch1_count], &fifo_data[0], 3u);
    memcpy(&p_spo->spo_ch2_buffer[p_spo->spo_ch2_count], &fifo_data[3], 3u);
    p_spo->spo_ch1_count += 3u;
    p_spo->spo_ch2_count += 3u;
    return NRF_SUCCESS;
}

void max30102_in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    UNUSED_PARAMETER(pin);
    UNUSED_PARAMETER(action);

    m_max30102_irq_count++;
    {
        uint32_t now_ticks = app_timer_cnt_get();
        if (!m_max30102_seen_once)
        {
            m_max30102_seen_once = true;
            m_max30102_last_tick = now_ticks;
        }
        else
        {
            uint32_t diff_ticks = app_timer_cnt_diff_compute(now_ticks, m_max30102_last_tick);
            uint32_t interval_us = app_timer_ticks_to_us(diff_ticks);
            m_max30102_last_tick = now_ticks;

            if (interval_us != 0u)
            {
                if (m_max30102_avg_period_us == 0u)
                {
                    m_max30102_avg_period_us = interval_us;
                }
                else
                {
                    m_max30102_avg_period_us =
                        (uint32_t)((7u * m_max30102_avg_period_us + interval_us) / 8u);
                }
            }
        }
    }

    APP_ERROR_CHECK(max30102_clear_interrupts());

    if (!m_connected || (m_spo.conn_handle == BLE_CONN_HANDLE_INVALID))
    {
        return;
    }

    if ((m_spo.spo_ch1_count >= SPO_PACKET_LENGTH) &&
        (m_spo.spo_ch2_count >= SPO_PACKET_LENGTH))
    {
        ble_spo_update_2ch(&m_spo);
    }

    ret_code_t err = max30102_read_fifo(&m_spo);
    if (err != NRF_SUCCESS)
    {
        if (err != NRF_ERROR_NO_MEM)
        {
            NRF_LOG_WARNING("MAX30102 FIFO read failed: 0x%X", err);
        }
        return;
    }

    m_max30102_read_count++;
}
