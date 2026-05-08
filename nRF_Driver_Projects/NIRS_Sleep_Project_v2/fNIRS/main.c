/**
 * Copyright (c) 2014 - 2019, Nordic Semiconductor ASA
 * All rights reserved.
 * This file contains code to run the fNIRS application using spectral sensors (AS7341).
 */

#include "nordic_common.h"
#include "app_error.h"
#include "app_timer.h"
#include "ble_srv_common.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_delay.h"
#include "nrf_drv_gpiote.h"

/* User defined includes */
#include "usr_config.h"
#include "ble_usr_defined.h"
#include "nrf_usr_defined.h"

/* Variable to indicated whether the device is currently connected to a BLE central device */
bool m_connected = false;

#if defined(AS7341)
#include "as7341.h"
#include "ble_as7341.h"
ble_as7341_t m_as7341 = {0};
static uint16_t m_as7341_sample_queue[NUM_OF_AS7341_DEVICES][BLE_AS7341_QUEUE_SAMPLES * BLE_AS7341_FIELDS_PER_SAMPLE];
static uint16_t m_as7341_queue_count[NUM_OF_AS7341_DEVICES] = {0u};
static uint8_t m_as7341_rr_dev_idx = 0u;
#endif /* #if defined(AS7341) */

#if defined(LED_CONTROL)
#include "led.h"
#endif

APP_TIMER_DEF(m_sampling_timer_id);
#define TICKS_SAMPLING_INTERVAL APP_TIMER_TICKS(1000)

#if defined(AS7341)
APP_TIMER_DEF(m_as7341_timer_id);
#endif

/* Number of samples recorded in a time interval (BLE) */
static uint16_t m_samples[NUM_OF_AS7341_DEVICES] = {0};

#if defined(SAADC_ENABLED) && SAADC_ENABLED == 1
#include "nrf_drv_saadc.h"
#define SAMPLES_IN_BUFFER 4
#define SAADC_BURST_MODE 1 // Set to 1 to enable BURST mode, otherwise set to 0.
static nrf_saadc_value_t m_buffer_pool[SAMPLES_IN_BUFFER];
static uint32_t m_adc_evt_counter;
#endif

#if defined(BLE_BAS_ENABLED) && BLE_BAS_ENABLED == 1
#include "ble_bas2.h"
static ble_bas_t m_bas;
// Timer stuff:
APP_TIMER_DEF(m_battery_timer_id);
#define BATTERY_TIMER_INTERVAL APP_TIMER_TICKS(60000) /**< Battery level measurement interval (ticks). */
#endif

#if defined(BLE_DIS_ENABLED) && BLE_DIS_ENABLED == 1
#include "ble_dis.h"
#endif

/*! @brief Periodic diagnostics callback for sample-rate logging.
 * @param[in] p_context  Unused context pointer.
 * @return None.
 */
static void m_sampling_timeout_handler(void *p_context)
{
    UNUSED_PARAMETER(p_context);
    for (uint8_t i = 0; i < NUM_OF_AS7341_DEVICES; i++)
    {
        // NRF_LOG_INFO("Device %u SAMPLE RATE = %dHz | DRDY avg us: = %u", i, m_samples[i], (uint32_t)m_drdy_avg_period_us[i]);
        m_samples[i] = 0;
    }
}

#if defined(AS7341)
static void as7341_queue_reset_device(uint8_t dev_idx)
{
    if (dev_idx < NUM_OF_AS7341_DEVICES)
    {
        m_as7341_queue_count[dev_idx] = 0u;
    }
}

static void as7341_queue_reset_all(void)
{
    for (uint8_t dev_idx = 0u; dev_idx < NUM_OF_AS7341_DEVICES; dev_idx++)
    {
        as7341_queue_reset_device(dev_idx);
    }
}

static bool as7341_next_initialized_device(uint8_t start_idx, uint8_t *p_dev_idx)
{
    if (p_dev_idx == NULL)
    {
        return false;
    }

    for (uint8_t offset = 0u; offset < NUM_OF_AS7341_DEVICES; offset++)
    {
        uint8_t idx = (uint8_t)((start_idx + offset) % NUM_OF_AS7341_DEVICES);
        if (as7341_is_initialized_device(idx))
        {
            *p_dev_idx = idx;
            return true;
        }
    }

    return false;
}

static ret_code_t as7341_queue_flush_device(uint8_t dev_idx)
{
    if (dev_idx >= NUM_OF_AS7341_DEVICES)
        return NRF_ERROR_INVALID_PARAM;

    if (m_as7341_queue_count[dev_idx] == 0u)
        return NRF_SUCCESS;

    ret_code_t err_code = (ret_code_t)ble_as7341_update_samples_device(&m_as7341,
                                                                       dev_idx,
                                                                       m_as7341_sample_queue[dev_idx],
                                                                       m_as7341_queue_count[dev_idx]);
    if (err_code == NRF_SUCCESS)
        as7341_queue_reset_device(dev_idx);

    return err_code;
}

static void as7341_config_handler(uint16_t conn_handle,
                                  ble_as7341_t *p_as7341,
                                  uint8_t const *data,
                                  uint16_t data_len)
{
    UNUSED_PARAMETER(conn_handle);
    UNUSED_PARAMETER(p_as7341);

    if ((data == NULL) || (data_len != BLE_AS7341_CONFIG_LEN))
    {
        return;
    }

    for (uint8_t dev_idx = 0u; dev_idx < NUM_OF_AS7341_DEVICES; dev_idx++)
    {
        (void)as7341_set_integration_20ms_device(dev_idx, data[1]);
    }

#if defined(LED_CONTROL)
    led_set_intensity(100 - data[0], 100 - data[2]);
    led_set_location_interval(data[3]);
#endif

    ret_code_t err_code = app_timer_stop(m_as7341_timer_id);
    if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_INVALID_STATE))
    {
        APP_ERROR_CHECK(err_code);
    }

    err_code = app_timer_start(m_as7341_timer_id,
                               APP_TIMER_TICKS(as7341_get_transfer_interval_ms_device(AS7341_STREAM_DEVICE_INDEX)),
                               NULL);
    APP_ERROR_CHECK(err_code);

    as7341_queue_reset_all();
    m_as7341_rr_dev_idx = 0u;
}

static void as7341_timer_timeout_handler(void *p_context)
{
    UNUSED_PARAMETER(p_context);

    if (!m_connected || (m_as7341.conn_handle == BLE_CONN_HANDLE_INVALID))
    {
        return;
    }

    uint16_t led_location_idx = 0u;
#if defined(LED_CONTROL)
    led_location_idx = led_get_location_index();
#endif

    uint8_t dev_idx = 0u;
    if (!as7341_next_initialized_device(m_as7341_rr_dev_idx, &dev_idx))
    {
        return;
    }

    m_as7341_rr_dev_idx = (uint8_t)((dev_idx + 1u) % NUM_OF_AS7341_DEVICES);

    /* Workaround: process one AS7341 device per tick to avoid dual-stream HVX bursts. */
    if (m_as7341_queue_count[dev_idx] >= BLE_AS7341_QUEUE_SAMPLES)
    {
        ret_code_t flush_err = as7341_queue_flush_device(dev_idx);
        if (flush_err != NRF_SUCCESS)
        {
            NRF_LOG_WARNING("AS7341[%u] queue flush failed: 0x%X", dev_idx, flush_err);
            return;
        }
    }

    uint16_t red_630 = 0u;
    uint16_t red_680 = 0u;
    uint16_t nir = 0u;
    ret_code_t err_code = as7341_read_red_nir_device(dev_idx, &red_630, &red_680, &nir);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_INFO("AS7341[%u] read failed: 0x%X", dev_idx, err_code);
        return;
    }

    uint16_t offset = (uint16_t)(m_as7341_queue_count[dev_idx] * BLE_AS7341_FIELDS_PER_SAMPLE);
    m_as7341_sample_queue[dev_idx][offset + 0u] = red_630;
    m_as7341_sample_queue[dev_idx][offset + 1u] = red_680;
    m_as7341_sample_queue[dev_idx][offset + 2u] = nir;
    m_as7341_sample_queue[dev_idx][offset + 3u] = led_location_idx;
    m_as7341_queue_count[dev_idx]++;

    if (m_as7341_queue_count[dev_idx] >= BLE_AS7341_QUEUE_SAMPLES)
    {
        ret_code_t flush_err = as7341_queue_flush_device(dev_idx);
        if (flush_err != NRF_SUCCESS)
        {
            NRF_LOG_WARNING("AS7341[%u] queue flush failed: 0x%X", dev_idx, flush_err);
        }
    }
}
#endif /* #if defined(AS7341) */

#if defined(BLE_BAS_ENABLED) && BLE_BAS_ENABLED == 1
/*! @brief Trigger one battery-level SAADC measurement.
 * @return None.
 */
static void battery_level_update(void)
{
    nrf_gpio_pin_set(BATTERY_LOAD_SWITCH_CTRL_PIN);
    nrf_drv_saadc_sample();
}

/*! @brief Periodic timer callback for battery measurements.
 * @param[in] p_context  Unused context pointer.
 * @return None.
 */
static void battery_timeout_handler(void *p_context)
{
    UNUSED_PARAMETER(p_context);
    battery_level_update();
}
#endif /* #if defined(BLE_BAS_ENABLED) && BLE_BAS_ENABLED == 1 */

/*! @brief Function for the Timer initialization.
 * @details Initializes the timer module. This creates and starts application timers.
 * @return None.
 */
static void timers_init(void)
{
    // Initialize timer module.
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
    // Create timers.
#if defined(BLE_BAS_ENABLED) && BLE_BAS_ENABLED == 1
    err_code = app_timer_create(&m_battery_timer_id, APP_TIMER_MODE_REPEATED, battery_timeout_handler);
    APP_ERROR_CHECK(err_code);
#endif
#if defined(AS7341)
    err_code = app_timer_create(&m_as7341_timer_id, APP_TIMER_MODE_REPEATED, as7341_timer_timeout_handler);
    APP_ERROR_CHECK(err_code);
#endif
    err_code = app_timer_create(&m_sampling_timer_id, APP_TIMER_MODE_REPEATED, m_sampling_timeout_handler);
    APP_ERROR_CHECK(err_code);
}

/*! @brief Initialize all enabled BLE services.
 * @return None.
 */
static void services_init(void)
{
    ret_code_t err_code;

#if defined(AS7341)
    ble_as7341_init_t as7341_init_cfg = {0};
    as7341_init_cfg.config_handler = as7341_config_handler;
    ble_as7341_service_init(&m_as7341, &as7341_init_cfg);
#endif

#if defined(BLE_BAS_ENABLED) && BLE_BAS_ENABLED == 1
    ble_bas_init_t bas_init;
    // Initialize Battery Service.
    memset(&bas_init, 0, sizeof(bas_init));

    bas_init.evt_handler = NULL;
    bas_init.support_notification = true;
    bas_init.p_report_ref = NULL;
    bas_init.initial_batt_level = 100;

    // Here the sec level for the Battery Service can be changed/increased.
    bas_init.bl_rd_sec = SEC_OPEN;
    bas_init.bl_cccd_wr_sec = SEC_OPEN;
    bas_init.bl_report_rd_sec = SEC_OPEN;

    err_code = ble_bas_init(&m_bas, &bas_init);
    APP_ERROR_CHECK(err_code);
#endif
#if defined(BLE_DIS_ENABLED) && BLE_DIS_ENABLED == 1
    ble_dis_init_t dis_init;
    // Initialize Device Information Service.
    memset(&dis_init, 0, sizeof(dis_init));

    ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, (char *)MANUFACTURER_NAME);
    ble_srv_ascii_to_utf8(&dis_init.model_num_str, (char *)DEVICE_MODEL_NUMBERSTR);
    ble_srv_ascii_to_utf8(&dis_init.fw_rev_str, (char *)DEVICE_FIRMWARE_STRING);

    dis_init.dis_char_rd_sec = SEC_OPEN;

    err_code = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);
#endif
}

/*! @brief Function for starting timers.
 * @return None.
 */
static void application_timers_start(void)
{
    ret_code_t err_code;
#if defined(BLE_BAS_ENABLED) && BLE_BAS_ENABLED == 1
    err_code = app_timer_start(m_battery_timer_id, BATTERY_TIMER_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
#endif
#if defined(AS7341)
    err_code = app_timer_start(m_as7341_timer_id,
                               APP_TIMER_TICKS(as7341_get_transfer_interval_ms_device(AS7341_STREAM_DEVICE_INDEX)),
                               NULL);
    APP_ERROR_CHECK(err_code);
#endif

    err_code = app_timer_start(m_sampling_timer_id, TICKS_SAMPLING_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
}

#if defined(SAADC_ENABLED) && SAADC_ENABLED == 1
/*! @brief SAADC event callback for battery conversion completion.
 * @param[in] p_event  SAADC event payload.
 * @return None.
 */
void saadc_callback(nrf_drv_saadc_evt_t const *p_event)
{
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
        ret_code_t err_code;
        uint8_t battery_level = 0;
        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);
        APP_ERROR_CHECK(err_code);

        int i;
        NRF_LOG_INFO("ADC event number: %d\r\n", (int)m_adc_evt_counter);

        for (i = 3; i < SAMPLES_IN_BUFFER; i++)
        {
            NRF_LOG_INFO("Batt: 0x%x\r\n", p_event->data.done.p_buffer[i]);
        }
        // TRANSMIT UPDATED BATTERY VALUE.
        m_bas.battery_level = p_event->data.done.p_buffer[3];
        err_code = ble_bas_battery_level_update(&m_bas, p_event->data.done.p_buffer[3], BLE_CONN_HANDLE_ALL);
        if ((err_code != NRF_SUCCESS) &&
            (err_code != NRF_ERROR_INVALID_STATE) &&
            (err_code != NRF_ERROR_RESOURCES) &&
            (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING))
        {
            APP_ERROR_HANDLER(err_code);
        }
        m_adc_evt_counter++;
        nrf_gpio_pin_clear(BATTERY_LOAD_SWITCH_CTRL_PIN); // LOAD SWITCH OFF
    }
}

/*! @brief Initialize SAADC resources for battery measurement.
 * @return None.
 */
void saadc_init(void)
{
    ret_code_t err_code;
    nrf_drv_saadc_config_t saadc_config;
    // Configure SAADC
    saadc_config.low_power_mode = true;                     // Enable low power mode.
    saadc_config.resolution = NRF_SAADC_RESOLUTION_12BIT;   // Set SAADC resolution to 12-bit. This will make the SAADC output values from 0 (when input voltage is 0V) to 2^12=2048 (when input voltage is 3.6V for channel gain setting of 1/6).
    saadc_config.oversample = NRF_SAADC_OVERSAMPLE_4X;      // Set oversample to 4x. This will make the SAADC output a single averaged value when the SAMPLE task is triggered 4 times.
    saadc_config.interrupt_priority = APP_IRQ_PRIORITY_LOW; // Set SAADC interrupt to low priority.

    nrf_saadc_channel_config_t channel_config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN2);

    err_code = nrf_drv_saadc_init(&saadc_config, saadc_callback);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(0, &channel_config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(&m_buffer_pool[0], SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);
}
#endif /* #if defined(SAADC_ENABLED) && SAADC_ENABLED == 1 */


/*! @brief Function for application main entry.
 * @return Application exit code (never reached in normal operation).
 */
int main(void)
{
    bool erase_bonds = false;
    // Initialize.
    log_init();
    /* INitialize required timers for the application */
    timers_init();
    /* nRF Power management module initializations */
    power_management_init();
    /* Initialize BLE stack before drivers that may enable interrupts/peripherals reserved by SoftDevice. */
    ble_stack_init();
    /* Initialize nRF GPIO Pins for interrupts/inputs/outputs */
    gpio_init();
#if defined(LED_CONTROL)
    APP_ERROR_CHECK(led_init());
    led_run_startup_sequence(LED_STARTUP_TEST_MS);
#endif
#if defined(AS7341)
    {
        ret_code_t as7341_err = as7341_init_all();
        if (as7341_err != NRF_SUCCESS)
        {
            NRF_LOG_WARNING("AS7341 init failed (0x%X); continuing with available sensors only", as7341_err);
        }
    }
#endif
    /* Initialize GAP - Generic Attribute Profile */
    gap_params_init();
    /* Generic Attribute initialization */
    gatt_init();
    services_init();
#if (EXPOSE_BLE_PARAM == 1)
    ble_param_service_init();
#endif
    /* Initiale BLE advertisement and other PPCP parameters */
    advertising_init();
    conn_params_init();

#if (PEER_MANAGER_ENABLED == 1)
    peer_manager_init();
#endif

// Initialize SAADC
#if defined(SAADC_ENABLED) && SAADC_ENABLED == 1
    saadc_init();
#endif
    NRF_LOG_INFO("ECG Device Advertising Start! \r\n");
    /* Start Adv BLE */
    application_timers_start();

    while (1)
    {
        /* Put device in no process mode until event (interrupt or timer) occurs to save power */
        while (m_connected)
        {
            __WFE();
            NRF_LOG_FLUSH();
        }
        advertising_start(erase_bonds); // reconnect to BLE central if device disconnects
        while (!m_connected)
        {
            __WFE();
            NRF_LOG_FLUSH();
        }
    }
}

/**
 * @}
 */
