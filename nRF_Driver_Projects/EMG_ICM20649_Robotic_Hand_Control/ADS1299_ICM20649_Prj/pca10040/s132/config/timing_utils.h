/**
 * @file timing_utils.h
 * @brief Timing utilities for absolute timestamp and clock management
 * 
 * This module provides functions to get absolute timestamps with rollover handling.
 */

#ifndef TIMING_UTILS_H__
#define TIMING_UTILS_H__

#include <stdint.h>
#include <stdbool.h>
#include "sdk_errors.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * STRUCTURES
 * ============================================================================ */

/**
 * @brief Timing system information structure
 * 
 * Contains information about the current timing state, useful for debugging
 * and system monitoring.
 */
typedef struct {
    uint32_t rtc_frequency;          /**< RTC clock frequency in Hz (32768) */
    uint32_t rtc_counter_max;        /**< Maximum 24-bit counter value */
    uint32_t rollover_count;         /**< Number of RTC counter rollovers */
    uint32_t current_rtc_ticks;      /**< Current RTC counter value (24-bit) */
    uint64_t absolute_ticks;         /**< Absolute timestamp in RTC ticks (64-bit) */
    uint64_t absolute_ms;            /**< Absolute timestamp in milliseconds */
    float rollover_period_seconds;   /**< Time between rollovers in seconds */
    bool is_initialized;             /**< Whether timing system is initialized */
} timing_info_t;

/* ============================================================================
 * PUBLIC FUNCTION DECLARATIONS
 * ============================================================================ */

/**
 * @brief Initialize timing utilities
 * 
 * Initializes RTC1 overflow interrupt detection for tracking rollovers.
 * Must be called early in application startup, after app_timer_init().
 * 
 * Typical usage:
 * @code
 *   app_timer_init();
 *   // ... other initializations ...
 *   timing_init();  // Initialize timing utilities
 * @endcode
 * 
 * @return NRF_SUCCESS if successful, error code otherwise
 */
ret_code_t timing_init(void);

/**
 * @brief Get current RTC counter value (raw ticks)
 * 
 * Returns the raw 24-bit RTC counter value without rollover compensation.
 * Useful for very short intervals where rollover is impossible.
 * 
 * Range: 0 to 16,777,215
 * Period: Rolls over every ~512 seconds
 * 
 * @return Raw RTC counter value
 */
uint32_t timing_get_raw_ticks(void);

/**
 * @brief Get absolute timestamp in RTC ticks (with rollover handling)
 * 
 * Returns a 64-bit absolute timestamp that accounts for counter rollovers.
 * This is the primary function for getting timestamps in your application.
 * 
 * The timestamp combines:
 * - Upper 32 bits: Number of rollovers
 * - Lower 24 bits: Current RTC counter value
 * 
 * This function should be called at least once every ~256 seconds
 * to ensure rollover detection. More frequent calls are safe and recommended.
 * 
 * Typical usage:
 * @code
 *   uint64_t start_time = timing_get_ticks();
 *   // ... perform task ...
 *   uint64_t end_time = timing_get_ticks();
 *   uint64_t elapsed_ticks = timing_tick_diff(start_time, end_time);
 *   uint64_t elapsed_ms = timing_ticks_to_ms(elapsed_ticks);
 * @endcode
 * 
 * @return 64-bit absolute timestamp in RTC ticks
 */
uint64_t timing_get_ticks(void);

/**
 * @brief Get absolute timestamp in milliseconds (with rollover handling)
 * 
 * Converts absolute RTC timestamp to milliseconds.
 * Accuracy: ±30 microseconds per sample
 * 
 * Typical usage:
 * @code
 *   uint64_t current_ms = timing_get_milliseconds();
 *   NRF_LOG_INFO("Current time: %llu ms", current_ms);
 * @endcode
 * 
 * @return Absolute timestamp in milliseconds (64-bit value)
 * 
 * @note Maximum range: ~584 million years before 64-bit overflow
 */
uint64_t timing_get_milliseconds(void);

/**
 * @brief Get absolute timestamp in microseconds (approximate, with rollover handling)
 * 
 * Converts absolute RTC timestamp to microseconds.
 * Resolution is limited by the RTC frequency (approximately 30.5 μs per tick).
 * 
 * Note: This is an integer approximation and may lose precision.
 * For maximum precision, use timing_get_ticks() and convert as needed.
 * 
 * @return Approximate timestamp in microseconds (64-bit value)
 */
uint64_t timing_get_microseconds(void);

/**
 * @brief Get time difference in ticks between two timestamps
 * 
 * Safely calculates the difference between two 64-bit timestamps,
 * accounting for rollover and wraparound.
 * 
 * Typical usage:
 * @code
 *   uint64_t start = timing_get_ticks();
 *   // ... do something ...
 *   uint64_t end = timing_get_ticks();
 *   uint64_t elapsed = timing_tick_diff(start, end);
 * @endcode
 * 
 * @param[in] from_ticks  Starting timestamp
 * @param[in] to_ticks    Ending timestamp
 * 
 * @return Time elapsed in ticks (to_ticks - from_ticks)
 */
uint64_t timing_tick_diff(uint64_t from_ticks, uint64_t to_ticks);

/**
 * @brief Convert RTC ticks to milliseconds
 * 
 * Utility function to convert a tick value to milliseconds.
 * 
 * Typical usage:
 * @code
 *   uint64_t ticks = 65536;
 *   uint64_t ms = timing_ticks_to_ms(ticks);  // ~2000 ms
 * @endcode
 * 
 * @param[in] ticks  Number of RTC ticks to convert
 * 
 * @return Equivalent time in milliseconds
 */
uint64_t timing_ticks_to_ms(uint64_t ticks);

/**
 * @brief Convert RTC ticks to microseconds (approximate)
 * 
 * Utility function to convert a tick value to microseconds.
 * Resolution is limited by RTC frequency (~30.5 μs per tick).
 * 
 * @param[in] ticks  Number of RTC ticks to convert
 * 
 * @return Approximate time in microseconds (integer division)
 */
uint64_t timing_ticks_to_us(uint64_t ticks);

/**
 * @brief Convert RTC ticks to seconds (floating point)
 * 
 * Utility function to convert a tick value to seconds with decimal places.
 * 
 * Typical usage:
 * @code
 *   uint64_t ticks = timing_get_ticks();
 *   float seconds = timing_ticks_to_seconds(ticks);
 * @endcode
 * 
 * @param[in] ticks  Number of RTC ticks to convert
 * 
 * @return Time in seconds (floating point)
 */
float timing_ticks_to_seconds(uint64_t ticks);

/**
 * @brief Convert milliseconds to RTC ticks
 * 
 * Utility function to convert milliseconds to tick count.
 * Useful for creating delays or comparing with timestamps.
 * 
 * Typical usage:
 * @code
 *   uint64_t one_second_ticks = timing_ms_to_ticks(1000);
 *   uint64_t now = timing_get_ticks();
 *   if ((now - start) > one_second_ticks) { ... }
 * @endcode
 * 
 * @param[in] milliseconds  Time in milliseconds
 * 
 * @return Equivalent number of RTC ticks
 */
uint64_t timing_ms_to_ticks(uint64_t milliseconds);

/**
 * @brief Get timing system information for debugging
 * 
 * Returns current timing state information useful for debugging and monitoring.
 * 
 * Typical usage:
 * @code
 *   timing_info_t info;
 *   timing_get_info(&info);
 *   NRF_LOG_INFO("Current time: %llu ms", info.absolute_ms);
 * @endcode
 * 
 * @param[out] p_info  Pointer to timing_info_t structure to fill
 */
void timing_get_info(timing_info_t *p_info);

/**
 * @brief Print timing information to logs
 * 
 * Convenience function to log current timing state for debugging.
 * Produces formatted output with all timing information.
 * 
 * Typical usage:
 * @code
 *   timing_print_info();  // Logs timing state
 * @endcode
 */
void timing_print_info(void);

#ifdef __cplusplus
}
#endif

#endif  // TIMING_UTILS_H__
