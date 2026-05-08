#ifndef NRF_USR_DEFINED_
#define NRF_USR_DEFINED_ 
#include "stdint.h"
#include "nrf_drv_gpiote.h"
#include "usr_config.h"

void sleep_mode_enter(void);
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name);
void log_init(void);
void power_management_init(void);
void idle_state_handle(void);
void saadc_init(void);
void gpio_init(void);
void nrf_qwr_error_handler(uint32_t nrf_error);

#if defined(ADS1299)
void ads_in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action);
#endif

#if defined(MAX30102_PRESENT) && (MAX30102_PRESENT == 1)
void max30102_in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action);
#endif

#endif // NRF_USR_DEFINED_
