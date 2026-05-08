/* Created By: Sameera Sharma for Dysphagia Project with BITN */
/** @file
 *
 * @defgroup Dysphagia MIC + IMU main.c
 * @{
 * @brief Dysphagia MIC + IMU main file.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define APP_TIMER_SAMPLING 1
#define BATTERY_LOAD_SWITCH_CTRL_PIN 1

#include "app_error.h"
#if defined(APP_TIMER_SAMPLING)
#include "app_timer.h"
#endif
#include "nrf_pwr_mgmt.h"
#include "ble_srv_common.h"
#include "nrf_log.h"
#include "nrf_error.h"
#include "app_error.h"
#include "nordic_common.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrf_log.h"
#include "nrf_log_default_backends.h"
#include "nrf_log_ctrl.h"
#include "usr_defined.h"
#include "ble_usr_defined.h"
#include "ble_eeg.h"

#warning ("CHECK LFCLK & LED DEFS IN custom_board.h")

#if defined(ADS1292) // first EMG for stetho
#include "ads1291-2.h"
#include "ble_dis.h"
#include "ble_eeg.h"
#include "nrf_delay.h"
#include "nrf_drv_gpiote.h"

#define DEVICE_MODEL_NUMBERSTR "Version 3.1"
#define DEVICE_FIRMWARE_STRING "Version 13.1.0"

#define LEDS_ENABLE 0
#define LED2_PIN 11
#define LED3_PIN 12

ble_eeg_t m_eeg;
bool m_connected = false;
static bool m_EMG_flag = false;
#define SPI_SCLK_WRITE_REG 2
#define SPI_SCLK_SAMPLING 2
static volatile bool m_drdy_flag = false;
#endif

#include "icm20948.h"
#if (ICM20948_PRESENT)
#include "ble_icm.h"
ble_icm_t m_icm[NUM_ICM_DEVICES];
uint8_t m_icm_rr_next_idx = 0u;

// Creating timer IDs
APP_TIMER_DEF(m_icm1_timer_id);
#if (NUM_ICM_DEVICES == 2)
APP_TIMER_DEF(m_icm2_timer_id);
#endif

// Timers for getting data from ICM devices
uint32_t nICMTimerTicks[NUM_ICM_DEVICES] = {
    APP_TIMER_TICKS(20) // Drain IMU1 FIFO in bursts of roughly 5-6 accel/gyro samples.
#if (NUM_ICM_DEVICES == 2)
    ,
    APP_TIMER_TICKS(200) // Drain IMU2 FIFO about once per 5 Hz sample period.
#endif
};
#endif /* #if (ICM20948_PRESENT) */

#if defined(SAADC_ENABLED) && SAADC_ENABLED == 1
#include "nrf_drv_saadc.h"
#define SAMPLES_IN_BUFFER 4
#define SAADC_BURST_MODE 1 // Set to 1 to enable BURST mode, otherwise set to 0.`
static nrf_saadc_value_t m_buffer_pool[SAMPLES_IN_BUFFER];
static uint32_t m_adc_evt_counter;
#endif

#if defined(BLE_BAS_ENABLED) && BLE_BAS_ENABLED == 1
#include "ble_bas2.h"
#define BATTERY_LEVEL_MEAS_INTERVAL APP_TIMER_TICKS(5000) /**< Battery level measurement interval (ticks). */
APP_TIMER_DEF(m_battery_timer_id);                        /**< Battery timer. */
static ble_bas_t m_bas;                                   /**< Structure used to identify the battery service. */
#endif

#if defined(APP_TIMER_SAMPLING) && APP_TIMER_SAMPLING == 1
#define TICKS_SAMPLING_INTERVAL APP_TIMER_TICKS(1000)
APP_TIMER_DEF(m_sampling_timer_id);
static uint16_t m_samples;
#endif

#define DEAD_BEEF 0xDEADBEEF /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *            how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name    File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t *p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

#if defined(APP_TIMER_SAMPLING) && APP_TIMER_SAMPLING == 1
static void m_sampling_timeout_handler(void *p_context)
{
    UNUSED_PARAMETER(p_context);
#if defined(APP_TIMER_SAMPLING) && APP_TIMER_SAMPLING == 1
#if LOG_LOW_DETAIL == 1
    NRF_LOG_INFO("SAMPLE RATE = %dHz \r\n", m_samples);
#endif
    m_samples = 0;
#endif
}
#endif
#if (defined(MPU60x0) || defined(MPU9150) || defined(MPU9250) || defined(MPU9255))
static void mpu_send_timeout_handler(void *p_context)
{
    // DEPENDS ON SAMPLING RATE
    mpu_read_accel_array(&m_mpu);
    mpu_read_gyro_array(&m_mpu);
    if (m_mpu.mpu_count == 24)
    {
        m_mpu.mpu_count = 0;
        ble_mpu_combined_update_v2(&m_mpu);
    }
}
#endif /**@(defined(MPU60x0) || defined(MPU9150) || defined(MPU9255))*/

#if ICM20948_PRESENT
static uint16_t const s_icm_fifo_burst_samples[NUM_ICM_DEVICES] = {
    6u
#if (NUM_ICM_DEVICES == 2)
    ,
    1u
#endif
};
static volatile bool m_icm_service_active = false;

static void icm_ram_queue_reset(ble_icm_t *p_icm)
{
    if (p_icm == NULL)
    {
        return;
    }

    CRITICAL_REGION_ENTER();
    p_icm->icm_sample_count = 0u;
    p_icm->icm_ram_head = 0u;
    p_icm->icm_ram_tail = 0u;
    p_icm->icm_ram_count = 0u;
    p_icm->icm_ram_dropped_samples = 0u;
    CRITICAL_REGION_EXIT();
}

static void icm_ram_queue_push_samples(ble_icm_t *p_icm, uint8_t const *raw_samples, uint16_t sample_count)
{
    uint16_t pushed = 0u;

    if ((p_icm == NULL) || (raw_samples == NULL) || (sample_count == 0u))
    {
        return;
    }

    CRITICAL_REGION_ENTER();
    while (pushed < sample_count)
    {
        if (p_icm->icm_ram_count >= ICM_RAM_BUFFER_SAMPLES)
        {
            p_icm->icm_ram_tail = (uint16_t)((p_icm->icm_ram_tail + 1u) % ICM_RAM_BUFFER_SAMPLES);
            p_icm->icm_ram_count--;
            p_icm->icm_ram_dropped_samples++;
        }

        memcpy(&p_icm->icm_ram_buffer[p_icm->icm_ram_head * ICM_BYTES_PER_SAMPLE],
               &raw_samples[pushed * ICM_BYTES_PER_SAMPLE],
               ICM_BYTES_PER_SAMPLE);
        p_icm->icm_ram_head = (uint16_t)((p_icm->icm_ram_head + 1u) % ICM_RAM_BUFFER_SAMPLES);
        p_icm->icm_ram_count++;
        pushed++;
    }
    CRITICAL_REGION_EXIT();
}

static bool icm_prepare_ble_packet_from_ram(uint32_t dev_idx)
{
    ble_icm_t *p_icm = &m_icm[dev_idx];
    icm20948_t *p_dev = &m_icm20948[dev_idx];
    uint16_t sample_idx = 0u;

    if (p_icm->icm_sample_count != 0u)
    {
        return true;
    }

    while (sample_idx < ICM_NUM_OF_SAMPLES)
    {
        uint8_t raw_sample[ICM_BYTES_PER_SAMPLE];
        uint8_t nested_critical = 0u;

        app_util_critical_region_enter(&nested_critical);
        if (p_icm->icm_ram_count == 0u)
        {
            app_util_critical_region_exit(nested_critical);
            break;
        }

        memcpy(raw_sample,
               &p_icm->icm_ram_buffer[p_icm->icm_ram_tail * ICM_BYTES_PER_SAMPLE],
               ICM_BYTES_PER_SAMPLE);
        p_icm->icm_ram_tail = (uint16_t)((p_icm->icm_ram_tail + 1u) % ICM_RAM_BUFFER_SAMPLES);
        p_icm->icm_ram_count--;
        app_util_critical_region_exit(nested_critical);

        icm20948_convert_raw_sample(p_dev,
                                    raw_sample,
                                    &p_icm->icm_sample_buffer[sample_idx * ICM_BYTES_PER_SAMPLE],
                                    &p_icm->icm_sample_parsed[sample_idx]);
        sample_idx++;
    }

    p_icm->icm_sample_count = sample_idx;
    return (sample_idx > 0u);
}

static bool icm_service_one_device(uint32_t dev_idx)
{
    ble_icm_t *p_icm = &m_icm[dev_idx];
    uint16_t queued_before;

    if (p_icm->conn_handle == BLE_CONN_HANDLE_INVALID)
    {
        return false;
    }

    if ((p_icm->icm_sample_count == 0u) && !icm_prepare_ble_packet_from_ram(dev_idx))
    {
        return false;
    }

    queued_before = p_icm->icm_sample_count;
    ble_icm_update_sample(p_icm);
    return (queued_before > 0u) && (p_icm->icm_sample_count == 0u);
}

static void icm_service_pending_buffers(void)
{
    bool progress = false;
    uint32_t attempts = 0u;
    uint8_t nested_critical = 0u;

    app_util_critical_region_enter(&nested_critical);
    if (m_icm_service_active)
    {
        app_util_critical_region_exit(nested_critical);
        return;
    }
    m_icm_service_active = true;
    app_util_critical_region_exit(nested_critical);

    do
    {
        progress = false;

        for (uint32_t n = 0u; n < NUM_ICM_DEVICES; n++)
        {
            uint32_t idx = (m_icm_rr_next_idx + n) % NUM_ICM_DEVICES;
            if (icm_service_one_device(idx))
            {
                m_icm_rr_next_idx = (uint8_t)((idx + 1u) % NUM_ICM_DEVICES);
                progress = true;
            }
        }

        attempts++;
    } while (progress && (attempts < (NUM_ICM_DEVICES * 4u)));

    app_util_critical_region_enter(&nested_critical);
    m_icm_service_active = false;
    app_util_critical_region_exit(nested_critical);
}

void icm_timeout_handler(void *p_context)
{
    uint32_t dev_idx = (uint32_t)(uintptr_t)p_context;
    uint8_t fifo_burst_buffer[ICM_BYTES_PER_SAMPLE * 6u];
    uint16_t fifo_count_bytes = 0u;
    uint16_t available_samples;
    ret_code_t err_code;
    ble_icm_t *p_icm = &m_icm[dev_idx];
    icm20948_t *p_dev = &m_icm20948[dev_idx];

    if ((p_dev->twi == NULL) || (p_icm->conn_handle == BLE_CONN_HANDLE_INVALID))
    {
        return;
    }

    err_code = icm20948_fifo_get_count(p_dev, &fifo_count_bytes);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_INFO("ICM%u FIFO count read failed: 0x%x", (unsigned)(dev_idx + 1u), err_code);
        return;
    }

    available_samples = (uint16_t)(fifo_count_bytes / ICM_BYTES_PER_SAMPLE);
    while (available_samples > 0u)
    {
        uint16_t burst_samples = available_samples;
        uint16_t burst_bytes;

        if (burst_samples > s_icm_fifo_burst_samples[dev_idx])
        {
            burst_samples = s_icm_fifo_burst_samples[dev_idx];
        }

        burst_bytes = (uint16_t)(burst_samples * ICM_BYTES_PER_SAMPLE);
        err_code = icm20948_fifo_read(p_dev, fifo_burst_buffer, burst_bytes);
        if (err_code != NRF_SUCCESS)
        {
            NRF_LOG_INFO("ICM%u FIFO burst read failed: 0x%x", (unsigned)(dev_idx + 1u), err_code);
            return;
        }

        icm_ram_queue_push_samples(p_icm, fifo_burst_buffer, burst_samples);
        available_samples -= burst_samples;
    }
}

void icm_flush_pending_round_robin(void)
{
    icm_service_pending_buffers();
}

void icm_timers_start(void)
{
    ret_code_t err_code;

    if (m_icm20948[0u].twi != NULL)
    {
        err_code = app_timer_start(m_icm1_timer_id, nICMTimerTicks[0], (void *)(uintptr_t)0u);
        if (err_code != NRF_SUCCESS && err_code != NRF_ERROR_INVALID_STATE)
        {
            APP_ERROR_CHECK(err_code);
        }
    }

#if (NUM_ICM_DEVICES == 2)
    if (m_icm20948[1u].twi != NULL)
    {
        err_code = app_timer_start(m_icm2_timer_id, nICMTimerTicks[1], (void *)(uintptr_t)1u);
        if (err_code != NRF_SUCCESS && err_code != NRF_ERROR_INVALID_STATE)
        {
            APP_ERROR_CHECK(err_code);
        }
    }
#endif
}

void icm_timers_stop(void)
{
    ret_code_t err_code;

    err_code = app_timer_stop(m_icm1_timer_id);
    if (err_code != NRF_SUCCESS && err_code != NRF_ERROR_INVALID_STATE)
    {
        APP_ERROR_CHECK(err_code);
    }

#if (NUM_ICM_DEVICES == 2)
    err_code = app_timer_stop(m_icm2_timer_id);
    if (err_code != NRF_SUCCESS && err_code != NRF_ERROR_INVALID_STATE)
    {
        APP_ERROR_CHECK(err_code);
    }
#endif

    for (uint32_t i = 0u; i < NUM_ICM_DEVICES; i++)
    {
        icm_ram_queue_reset(&m_icm[i]);
    }
}
#endif

#if defined(BLE_BAS_ENABLED) && BLE_BAS_ENABLED == 1
static void battery_level_update(void)
{
    ret_code_t err_code;
#if defined(SAADC_ENABLED) && SAADC_ENABLED == 1
    // Enable load switch:
    nrf_gpio_pin_set(BATTERY_LOAD_SWITCH_CTRL_PIN);
    // Sample with ADC
    err_code = nrf_drv_saadc_sample();
    APP_ERROR_CHECK(err_code);
#endif
}

// Timer timeout handler to measure battery level
static void battery_level_meas_timeout_handler(void *p_context)
{
    UNUSED_PARAMETER(p_context);
    battery_level_update();
}
#endif /* #if defined(BLE_BAS_ENABLED) && BLE_BAS_ENABLED == 1 */

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    // Initialize timer module..
    // Create timers
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
#if ICM20948_PRESENT
    err_code = app_timer_create(&m_icm1_timer_id, APP_TIMER_MODE_REPEATED, icm_timeout_handler);
    APP_ERROR_CHECK(err_code);

#if (NUM_ICM_DEVICES == 2)
    err_code = app_timer_create(&m_icm2_timer_id, APP_TIMER_MODE_REPEATED, icm_timeout_handler);
    APP_ERROR_CHECK(err_code);
#endif
#endif

#if defined(APP_TIMER_SAMPLING) && APP_TIMER_SAMPLING == 1
    err_code = app_timer_create(&m_sampling_timer_id, APP_TIMER_MODE_REPEATED, m_sampling_timeout_handler);
    APP_ERROR_CHECK(err_code);
#endif

#if defined(BLE_BAS_ENABLED) && BLE_BAS_ENABLED == 1
    err_code = app_timer_create(&m_battery_timer_id, APP_TIMER_MODE_REPEATED, battery_level_meas_timeout_handler);
    APP_ERROR_CHECK(err_code);
#endif
}

/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t err_code;
    ble_eeg_service_init(&m_eeg);
/**@Device Information Service:*/
#if (defined(MPU60x0) || defined(MPU9150) || defined(MPU9250) || defined(MPU9255))
    ble_mpu_service_init(&m_mpu);
#endif

#if ICM20948_PRESENT
    m_icm[0].dev_idx = 0;
    ble_icm_service_init(&m_icm[0]);

#if (NUM_ICM_DEVICES == 2)
    m_icm[1].dev_idx = 1;
    ble_icm_service_init(&m_icm[1]);
#endif
#endif

#if defined(BLE_BAS_ENABLED) && BLE_BAS_ENABLED == 1
    ble_bas_init_t bas_init;
    // Initialize Battery Service.
    memset(&bas_init, 0, sizeof(bas_init));

    // Here the sec level for the Battery Service can be changed/increased.
    bas_init.evt_handler = NULL;
    bas_init.support_notification = true;
    bas_init.p_report_ref = NULL;
    bas_init.initial_batt_level = 0x64;
    bas_init.bl_rd_sec = SEC_OPEN;
    bas_init.bl_cccd_wr_sec = SEC_OPEN;
    bas_init.bl_report_rd_sec = SEC_OPEN;

    err_code = ble_bas_init(&m_bas, &bas_init);
    APP_ERROR_CHECK(err_code);
#endif

    ble_dis_init_t dis_init;
    memset(&dis_init, 0, sizeof(dis_init));
    ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, (char *)MANUFACTURER_NAME);
    ble_srv_ascii_to_utf8(&dis_init.model_num_str, (char *)DEVICE_MODEL_NUMBERSTR);
    ble_srv_ascii_to_utf8(&dis_init.fw_rev_str, (char *)DEVICE_FIRMWARE_STRING);
    dis_init.dis_char_rd_sec = SEC_OPEN;
    err_code = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for starting timers.
 */
static void application_timers_start(void)
{
    /* YOUR_JOB: Start your timers. below is an example of how to start a timer.
           ret_code_t err_code;
           err_code = app_timer_start(m_app_timer_id, TIMER_INTERVAL, NULL);
           APP_ERROR_CHECK(err_code); */
    ret_code_t err_code;
#if defined(APP_TIMER_SAMPLING) && APP_TIMER_SAMPLING == 1
    err_code = app_timer_start(m_sampling_timer_id, TICKS_SAMPLING_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
#endif

#if (defined(MPU60x0) || defined(MPU9150) || defined(MPU9250) || defined(MPU9255))
    err_code = app_timer_start(m_mpu_send_timer_id, TICKS_MPU_SAMPLING_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
#endif

#if defined(BLE_BAS_ENABLED) && BLE_BAS_ENABLED == 1
    err_code = app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
#endif
}

/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

#if (defined(MPU60x0) || defined(MPU9150) || defined(MPU9250) || defined(MPU9255))
///*
// MPU9250;MPU9255
void mpu_setup(void)
{
    ret_code_t ret_code;
    // Initiate MPU driver
    ret_code = mpu_init();
    APP_ERROR_CHECK(ret_code); // Check for errors in return value

    // Setup and configure the MPU with intial values
    mpu_config_t p_mpu_config = MPU_DEFAULT_CONFIG(); // Load default values
    p_mpu_config.smplrt_div = 19;                     // Change sampelrate. Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV). 19 gives a sample rate of 50Hz
    p_mpu_config.accel_config.afs_sel = AFS_16G;      // Set accelerometer full scale range to 2G
    ret_code = mpu_config(&p_mpu_config);             // Configure the MPU with above values
    APP_ERROR_CHECK(ret_code);                        // Check for errors in return value
}
//*/
#endif

#if defined(SAADC_ENABLED) && SAADC_ENABLED == 1
/*! @brief: Call back for the SAADC initialized on the nRF board */
void saadc_callback(nrf_drv_saadc_evt_t const *p_event)
{
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
        ret_code_t err_code;
        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);
        APP_ERROR_CHECK(err_code);

        int i;
        NRF_LOG_INFO("SAADC: ADC event number: %d\r\n", (int)m_adc_evt_counter);

        for (i = 0; i < SAMPLES_IN_BUFFER; i++)
        {
            NRF_LOG_INFO("SAADC: %d\r\n", p_event->data.done.p_buffer[i]);
        }
        m_bas.battery_level = p_event->data.done.p_buffer[3];
        err_code = ble_bas_battery_level_update(&m_bas, m_bas.battery_level, BLE_CONN_HANDLE_ALL);
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

/*! @brief: Initialize the SA-ADC on board for BAS measurement */
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
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN3);

    err_code = nrf_drv_saadc_init(&saadc_config, saadc_callback);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(0, &channel_config);
    APP_ERROR_CHECK(err_code);

    if (SAADC_BURST_MODE)
    {
        NRF_SAADC->CH[0].CONFIG |= 0x01000000; // Configure burst mode for channel 0. Burst is useful together with oversampling. When triggering the SAMPLE task in burst mode, the SAADC will sample "Oversample" number of times as fast as it can and then output a single averaged value to the RAM buffer. If burst mode is not enabled, the SAMPLE task needs to be triggered "Oversample" number of times to output a single averaged value to the RAM buffer.
    }

    err_code = nrf_drv_saadc_buffer_convert(&m_buffer_pool[0], SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);
}
#endif

/*! Callback function for DRDY configured pin interrupt to collect ADS1291 Stetho data */
static void ads_in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    UNUSED_PARAMETER(pin);
    UNUSED_PARAMETER(action);
    if (m_connected)
    {
        get_eeg_voltage_array_2ch(&m_eeg); // first ads1292 stetho // 192
        m_samples++;

        if (m_eeg.eeg_ch1_count == EEG_PACKET_LENGTH)
        {
            m_eeg.eeg_ch1_count = 0;
            ble_eeg_update_2ch(&m_eeg);
        }
    }
}

static void ads1291_gpio_init(void)
{
#if defined(BOARD_NRF_BREAKOUT) | defined(BOARD_PCA10028) | defined(BOARD_PCA10040) | defined(BOARD_2CH_ECG_RAT)
//    nrf_gpio_pin_dir_set(ADS1291_2_DRDY_PIN, NRF_GPIO_PIN_DIR_INPUT); //sets 'direction' = input/output
//    nrf_gpio_pin_dir_set(ADS1291_2_PWDN_PIN, NRF_GPIO_PIN_DIR_OUTPUT);
#endif
#ifdef BATTERY_LOAD_SWITCH_CTRL_PIN
//    nrf_gpio_cfg_output(BATTERY_LOAD_SWITCH_CTRL_PIN);
//    nrf_gpio_pin_set(BATTERY_LOAD_SWITCH_CTRL_PIN); //OFF
#endif

    uint32_t err_code = NRF_SUCCESS;
    if (!nrf_drv_gpiote_is_init())
    {
        err_code = nrf_drv_gpiote_init();
    }

    nrf_gpio_pin_dir_set(ADS1291_2_DRDY_PIN, NRF_GPIO_PIN_DIR_INPUT); // sets 'direction' = input/output
    nrf_gpio_pin_dir_set(ADS1291_2_PWDN_PIN, NRF_GPIO_PIN_DIR_OUTPUT);

    NRF_LOG_RAW_INFO(" nrf_drv_gpiote_init(a): %d\r\n", err_code);
    NRF_LOG_FLUSH();
    APP_ERROR_CHECK(err_code);
    bool is_high_accuracy = true;
    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(is_high_accuracy);
    in_config.is_watcher = false;
    in_config.pull = NRF_GPIO_PIN_NOPULL;
    err_code = nrf_drv_gpiote_in_init(ADS1291_2_DRDY_PIN, &in_config, ads_in_pin_handler);
    NRF_LOG_RAW_INFO(" nrf_drv_gpiote_in_init(b): %d: \r\n", err_code);
    NRF_LOG_FLUSH();
    APP_ERROR_CHECK(err_code);
    nrf_drv_gpiote_in_event_enable(ADS1291_2_DRDY_PIN, true);
}

#if (ICM20948_PRESENT)
static void icm20948_main_init(void)
{
    ret_code_t err_code = NRF_SUCCESS;
    icm20948_twi_init();
    nrf_delay_ms(1000);
    err_code = icm20948_init(&m_icm20948[0u], ICM20948_I2C_ADDR0); // ICM device 1
    APP_ERROR_CHECK(err_code);
#if (NUM_ICM_DEVICES == 2u)
    err_code = icm20948_init(&m_icm20948[1u], ICM20948_I2C_ADDR1); // ICM device 2
    APP_ERROR_CHECK(err_code);
#endif /* #if (NUM_ICM_DEVICES == 2) */
}
#endif /* #if (ICM20948_PRESENT) */

/*! @brief Initialize power-management module.
 * @return None.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}

/*! @brief Function for application main entry.
 */
int main(void)
{
    ret_code_t err_code = NRF_SUCCESS;
    // Initialize.
    log_init();
    /* INitialize required timers for the application */
    timers_init();
    /* nRF Power management module initializations */
    power_management_init();
    /* Initialize BLE stack*/

    ble_stack_init();
    /* Initialize GAP - Generic Attribute Profile */
    gap_params_init();
    /* Generic Attribute initialization */
    gatt_init();
    services_init();
#if (EXPOSE_BLE_PARAM == 1)
    ble_param_service_init();
#endif
    advertising_init();
    conn_params_init();

#if (PEER_MANAGER_ENABLED == 1)
    peer_manager_init();
#endif

#if defined(ADS1292)
    // Initialize SPI for communication
    ads_spi_init_with_sample_freq(1);
    // Configure required GPIOs for input/output/interrupt triggers
    ads1291_gpio_init();
    ads1291_2_main_init(true);
    ads1291_2_powerdn();
#endif

#if (defined(MPU60x0) || defined(MPU9150) || defined(MPU9250) || defined(MPU9255))
    mpu_setup();
#endif

#if (ICM20948_PRESENT)
    icm20948_main_init();
#endif /* #if (ICM20948_PRESENT) */

#if defined(SAADC_ENABLED) && SAADC_ENABLED == 1
    saadc_init();
#endif
    // Start execution.
    NRF_LOG_RAW_INFO("BLE Advertising Start! \r\n");
    application_timers_start();
    NRF_LOG_FLUSH();

    while (1)
    {
        /* Put device in no process mode until event (interrupt or timer) occurs to save power */
        while (m_connected)
        {
            icm_service_pending_buffers();
            __WFE();
            NRF_LOG_FLUSH();
        }
        advertising_start(false); // reconnect to BLE central if device disconnects
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
