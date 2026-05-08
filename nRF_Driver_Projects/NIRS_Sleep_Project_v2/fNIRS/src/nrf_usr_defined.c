#include "nrf_usr_defined.h"
#include "app_error.h"
#include "nrf_pwr_mgmt.h"
#include "../../../../../../components/softdevice/s112/headers/nrf_soc.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "ble_bas2.h"

#if defined(SAADC_ENABLED) && SAADC_ENABLED == 1
#include "nrf_drv_saadc.h"
#define SAMPLES_IN_BUFFER 4
#define SAADC_BURST_MODE  1 //Set to 1 to enable BURST mode, otherwise set to 0.static nrf_saadc_value_t m_buffer_pool[SAMPLES_IN_BUFFER];
static uint32_t m_adc_evt_counter;
#endif

/*! @brief Callback function for asserts in the SoftDevice.
 * @details This function will be called in case of an assert in the SoftDevice.
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 * @return None.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/*! @brief Function for putting the chip into sleep mode.
 * @note This function will not return.
 * @return None.
 */
void sleep_mode_enter(void)
{
    ret_code_t err_code;
#if 0 // Enable this code in case you have connected LEDs for indication
    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);
#endif /* #if 0 */

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/*! @brief Initialize NRF logging backends.
 * @return None.
 */
void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/*! @brief Initialize power-management module.
 * @return None.
 */
void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/*! @brief Function for handling the idle state (main loop).
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 * @return None.
 */
void idle_state_handle(void)
{
    // If no pending logs, let nrf_pwr_mgmt put the CPU in low power until the next interrupt/event.
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}

#if defined(SAADC_ENABLED) && SAADC_ENABLED == 1
/*! @brief Handle SAADC completion events.
 * @param[in] p_event  SAADC event payload.
 * @return None.
 */
static void saadc_callback(nrf_drv_saadc_evt_t const *p_event) {
    // SAADC ISR callback: called when an ADC conversion completes; this code updates Battery Service with the measured value.

    if (p_event->type == NRF_DRV_SAADC_EVT_DONE) {
        ret_code_t err_code;
        uint8_t battery_level = 0;
        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);
        APP_ERROR_CHECK(err_code);

        int i;
        NRF_LOG_INFO("ADC event number: %d\r\n", (int)m_adc_evt_counter);

        for (i = 3; i < SAMPLES_IN_BUFFER; i++) {
            NRF_LOG_INFO("Batt: 0x%x\r\n", p_event->data.done.p_buffer[i]);
        }
        //TRANSMIT UPDATED BATTERY VALUE.
        m_bas.battery_level = p_event->data.done.p_buffer[3];
        err_code = ble_bas_battery_level_update(&m_bas, p_event->data.done.p_buffer[3], BLE_CONN_HANDLE_ALL);
        if (err_code != NRF_SUCCESS) 
              //&& (err_code != NRF_ERROR_INVALID_STATE) &&
              //(err_code != NRF_ERROR_RESOURCES) &&
              //(err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING))
        {
            APP_ERROR_HANDLER(err_code);
        }
        m_adc_evt_counter++;
        nrf_gpio_pin_clear(BATTERY_LOAD_SWITCH_CTRL_PIN); //LOAD SWITCH OFF
    }
}

/*! @brief Initialize SAADC channel and buffers for battery measurements.
 * @return None.
 */
void saadc_init(void)
{
    ret_code_t err_code;
    nrf_drv_saadc_config_t saadc_config;
    //Configure SAADC
    saadc_config.low_power_mode = true;                     //Enable low power mode.
    saadc_config.resolution = NRF_SAADC_RESOLUTION_12BIT;   //Set SAADC resolution to 12-bit. This will make the SAADC output values from 0 (when input voltage is 0V) to 2^12=2048 (when input voltage is 3.6V for channel gain setting of 1/6).
    saadc_config.oversample = NRF_SAADC_OVERSAMPLE_4X;      //Set oversample to 4x. This will make the SAADC output a single averaged value when the SAMPLE task is triggered 4 times.
    saadc_config.interrupt_priority = APP_IRQ_PRIORITY_LOW; //Set SAADC interrupt to low priority.

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


/*! @brief: Initialize all required GPIOs for the applications and enable handlers for pins */
/*! @return None. */
void gpio_init(void)
{
    uint32_t err_code;
    if (!nrf_drv_gpiote_is_init())
    {
        APP_ERROR_CHECK(nrf_drv_gpiote_init());
    }
    NRF_LOG_INFO(" [NRF DRV] GPIOTE Init\r\n");
    // Set interrupt pins as inputs
    bool is_high_accuracy = true;
#if defined(ADS1299)
    nrf_gpio_pin_dir_set(ADS1299_1_DRDY_PIN, NRF_GPIO_PIN_DIR_INPUT); // sets 'direction' = input/output
#if (NUM_OF_ADS1299 == 2)
    nrf_gpio_pin_dir_set(ADS1299_2_DRDY_PIN, NRF_GPIO_PIN_DIR_INPUT);
#endif
    nrf_gpio_pin_dir_set(ADS1299_START_PIN, NRF_GPIO_PIN_DIR_OUTPUT);
    nrf_gpio_pin_dir_set(ADS1299_PWDN_RST_PIN, NRF_GPIO_PIN_DIR_OUTPUT);

    // SPI CS pins will be controlled manually
    nrf_gpio_cfg_output(ADS1299_1_SPI_CS_PIN);
    nrf_gpio_pin_dir_set(ADS1299_1_SPI_CS_PIN, NRF_GPIO_PIN_DIR_OUTPUT);
    nrf_gpio_pin_set(ADS1299_1_SPI_CS_PIN);

#if (NUM_OF_ADS1299 == 2)
    nrf_gpio_cfg_output(ADS1299_2_SPI_CS_PIN);
    nrf_gpio_pin_dir_set(ADS1299_2_SPI_CS_PIN, NRF_GPIO_PIN_DIR_OUTPUT);
    nrf_gpio_pin_set(ADS1299_2_SPI_CS_PIN);
#endif

    // Set GPIOTE as input:
    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(is_high_accuracy);
    in_config.is_watcher = true;
    in_config.pull = NRF_GPIO_PIN_NOPULL;

    // Configure DRDY signals to call the handler
    err_code = nrf_drv_gpiote_in_init(ADS1299_1_DRDY_PIN, &in_config, ads_in_pin_handler);
    APP_ERROR_CHECK(err_code);
#if (NUM_OF_ADS1299 == 2)
    err_code = nrf_drv_gpiote_in_init(ADS1299_2_DRDY_PIN, &in_config, ads_in_pin_handler);
    APP_ERROR_CHECK(err_code);
#endif
    if (err_code)
      NRF_LOG_INFO(" [NRF DRV] ADS1299 Int Pin Setup errcode: 0x%x!\r\n", err_code);
    NRF_LOG_FLUSH();
    nrf_drv_gpiote_in_event_enable(ADS1299_1_DRDY_PIN, true);
#if (NUM_OF_ADS1299 == 2)
    nrf_drv_gpiote_in_event_enable(ADS1299_2_DRDY_PIN, true);
#endif
#endif

#ifdef BATTERY_LOAD_SWITCH_CTRL_PIN
    nrf_gpio_cfg_output(BATTERY_LOAD_SWITCH_CTRL_PIN);
    nrf_gpio_pin_clear(BATTERY_LOAD_SWITCH_CTRL_PIN); // OFF
#endif
}


/*! @brief Function for handling Queued Write Module errors.
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 * @return None.
 */
void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}
