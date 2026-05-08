#ifndef LED_CONTROL_H__
#define LED_CONTROL_H__

#include <stdint.h>
#include "sdk_errors.h"
#include "usr_config.h"

#if defined(LED_CONTROL)

#ifndef LED_PWM_TOP_VALUE
#define LED_PWM_TOP_VALUE                 1000u
#endif

#ifndef LED_LOCATION_INTERVAL_DEFAULT_S
#define LED_LOCATION_INTERVAL_DEFAULT_S   1u
#endif

ret_code_t led_init(void);
void led_run_startup_sequence(uint32_t duration_ms);
void led_set_intensity(uint8_t red_percent, uint8_t ir_percent);
void led_get_intensity(uint8_t *p_red_percent, uint8_t *p_ir_percent);
void led_set_location_interval(uint8_t interval_s);
uint8_t led_get_location_interval(void);
uint8_t led_get_location_index(void);

#endif /* defined(LED_CONTROL) */

#endif /* LED_CONTROL_H__ */
