#include "led.h"

#if defined(LED_CONTROL)

#include "app_error.h"
#include "app_timer.h"
#include "nrf_delay.h"
#include "sdk_config.h"
#include "nrfx_pwm.h"
#include "usr_config.h"

#define LED_LOCATION_COUNT            2u

#if !defined(NRFX_PWM_ENABLED) || (NRFX_PWM_ENABLED == 0)
#error "NRFX_PWM_ENABLED must be 1 to build LED_CONTROL (see sdk_config.h / project defines)."
#endif

#if !defined(NRFX_PWM0_ENABLED) || (NRFX_PWM0_ENABLED == 0)
#error "NRFX_PWM0_ENABLED must be 1 to build LED_CONTROL (see sdk_config.h / project defines)."
#endif

#ifndef AS7341_DEFAULT_LED_LOCATION_S
#define AS7341_DEFAULT_LED_LOCATION_S LED_LOCATION_INTERVAL_DEFAULT_S
#endif

APP_TIMER_DEF(m_led_location_timer_id);

static nrfx_pwm_t const m_led_pwm = NRFX_PWM_INSTANCE(0);
static bool m_led_pwm_initialized = false;
static bool m_led_location_timer_initialized = false;

static uint8_t m_red_percent = 0u;
static uint8_t m_ir_percent = 0u;
static uint8_t m_location_interval_s = AS7341_DEFAULT_LED_LOCATION_S;
static uint8_t m_location_index = 0u;

static nrf_pwm_values_individual_t m_led_pwm_values = {0u, 0u, 0u, 0u};
static nrf_pwm_sequence_t const m_led_pwm_sequence =
{
    .values.p_individual = &m_led_pwm_values,
    .length = NRF_PWM_VALUES_LENGTH(m_led_pwm_values),
    .repeats = 0u,
    .end_delay = 0u
};

static uint16_t led_percent_to_pwm_ticks(uint8_t percent)
{
    uint32_t duty = percent;
    if (duty > 100u)
    {
        duty = 100u;
    }

    return (uint16_t)((duty * LED_PWM_TOP_VALUE) / 100u);
}

static void led_pwm_apply(void)
{
    uint16_t red_ticks = led_percent_to_pwm_ticks(m_red_percent);
    uint16_t ir_ticks = led_percent_to_pwm_ticks(m_ir_percent);

    m_led_pwm_values.channel_0 = red_ticks;
    m_led_pwm_values.channel_1 = ir_ticks;
    m_led_pwm_values.channel_2 = red_ticks;
    m_led_pwm_values.channel_3 = ir_ticks;
}

static void led_location_timeout_handler(void *p_context)
{
    UNUSED_PARAMETER(p_context);
    m_location_index = (uint8_t)((m_location_index + 1u) % LED_LOCATION_COUNT);
}

static void led_location_timer_restart(void)
{
    uint32_t ticks;
    ret_code_t err_code;

    if (!m_led_location_timer_initialized)
    {
        return;
    }

    ticks = APP_TIMER_TICKS((uint32_t)m_location_interval_s * 1000u);
    err_code = app_timer_stop(m_led_location_timer_id);
    if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_INVALID_STATE))
    {
        APP_ERROR_CHECK(err_code);
    }

    err_code = app_timer_start(m_led_location_timer_id, ticks, NULL);
    APP_ERROR_CHECK(err_code);
}

ret_code_t led_init(void)
{
    ret_code_t err_code;
    nrfx_err_t pwm_err;

    if (!m_led_pwm_initialized)
    {
        nrfx_pwm_config_t pwm_cfg = NRFX_PWM_DEFAULT_CONFIG;
        pwm_cfg.output_pins[0] = WH_LED_1_PIN;
        pwm_cfg.output_pins[1] = IR_LED_1_PIN;
        pwm_cfg.output_pins[2] = WH_LED_2_PIN;
        pwm_cfg.output_pins[3] = IR_LED_2_PIN;
        pwm_cfg.base_clock = NRF_PWM_CLK_1MHz;
        pwm_cfg.count_mode = NRF_PWM_MODE_UP;
        pwm_cfg.top_value = LED_PWM_TOP_VALUE;
        pwm_cfg.load_mode = NRF_PWM_LOAD_INDIVIDUAL;
        pwm_cfg.step_mode = NRF_PWM_STEP_AUTO;
        pwm_cfg.irq_priority = APP_IRQ_PRIORITY_LOWEST;

        pwm_err = nrfx_pwm_init(&m_led_pwm, &pwm_cfg, NULL);
        if ((pwm_err != NRFX_SUCCESS) && (pwm_err != NRFX_ERROR_INVALID_STATE))
        {
            return (ret_code_t)pwm_err;
        }

        led_pwm_apply();
        pwm_err = nrfx_pwm_simple_playback(&m_led_pwm,
                                           &m_led_pwm_sequence,
                                           1u,
                                           NRFX_PWM_FLAG_LOOP);
        if (pwm_err != NRFX_SUCCESS)
        {
            return (ret_code_t)pwm_err;
        }

        m_led_pwm_initialized = true;
    }

    if (!m_led_location_timer_initialized)
    {
        err_code = app_timer_create(&m_led_location_timer_id,
                                    APP_TIMER_MODE_REPEATED,
                                    led_location_timeout_handler);
        APP_ERROR_CHECK(err_code);
        m_led_location_timer_initialized = true;
    }

    led_location_timer_restart();
    return NRF_SUCCESS;
}

void led_run_startup_sequence(uint32_t duration_ms)
{
    uint8_t saved_red_percent = m_red_percent;
    uint8_t saved_ir_percent = m_ir_percent;

    m_red_percent = 0;
    m_ir_percent = 0;
    led_pwm_apply();

    if (duration_ms > 0u)
    {
        nrf_delay_ms(duration_ms);
    }

    m_red_percent = 100;
    m_ir_percent = 100;
    led_pwm_apply();
}

void led_set_intensity(uint8_t red_percent, uint8_t ir_percent)
{
    m_red_percent = (red_percent > 100u) ? 100u : red_percent;
    m_ir_percent = (ir_percent > 100u) ? 100u : ir_percent;
    led_pwm_apply();
}

void led_get_intensity(uint8_t *p_red_percent, uint8_t *p_ir_percent)
{
    if (p_red_percent != NULL)
    {
        *p_red_percent = m_red_percent;
    }
    if (p_ir_percent != NULL)
    {
        *p_ir_percent = m_ir_percent;
    }
}

void led_set_location_interval(uint8_t interval_s)
{
    if (interval_s == 0u)
    {
        interval_s = 1u;
    }

    m_location_interval_s = interval_s;
    led_location_timer_restart();
}

uint8_t led_get_location_interval(void)
{
    return m_location_interval_s;
}

uint8_t led_get_location_index(void)
{
    return m_location_index;
}

#endif /* defined(LED_CONTROL) */
