/**
 * @file timing_utils.c
 * @brief Timing utilities for absolute timestamp and clock management
 * 
 * This module provides functions to get absolute timestamps with rollover handling.
 * It uses app_timer (which relies on RTC1 at 32.768 kHz) as the clock source.
 * 
 * Clock Information:
 * - Timer Source: RTC1 (Real Time Clock)
 * - Frequency: 32.768 kHz
 * - Tick Period: 30.517 μs (approximately)
 * - Counter Width: 24-bit (0 to 16,777,215)
 * - Max Ticks Before Rollover: 16,777,215 (~512 seconds or ~8.5 minutes)
 * - Prescaler: 1 (APP_TIMER_CONFIG_RTC_FREQUENCY = 1 means 1 Hz resolution after app_timer)
 * 
 * Usage:
 * 1. Call timing_init() at startup
 * 2. Call timing_get_ticks() to get current timestamp with rollover tracking
 * 3. Call timing_get_milliseconds() for millisecond timestamps
 * 4. Call timing_get_microseconds() for microsecond timestamps (approximate)
 */

#include "timing_utils.h"
#include "app_timer.h"
#include "nrf_drv_rtc.h"
#include "nrf_log.h"
#include "nrf_atomic.h"
#include <stdint.h>
#include <stdbool.h>

/* ============================================================================
 * DEFINES AND CONSTANTS
 * ============================================================================ */

/** @brief RTC1 is used as the clock source for app_timer */
#define TIMING_RTC_INSTANCE (&nrf_drv_rtc_instances[1])

/** @brief RTC1 counter width: 24 bits */
#define RTC_COUNTER_MAX 0x00FFFFFF  /* 16,777,215 */
#define RTC_COUNTER_BITS 24

/** @brief RTC1 frequency: 32,768 Hz nominal */
#define RTC_FREQUENCY 32768

/** @brief Ticks per millisecond (approximately 32.768 ticks = 1 ms) */
#define TICKS_PER_MILLISECOND (RTC_FREQUENCY / 1000)

/** @brief Ticks per microsecond (approximately 0.033 ticks = 1 μs) */
#define TICKS_PER_MICROSECOND (RTC_FREQUENCY / 1000000.0f)

/* ============================================================================
 * STATIC VARIABLES
 * ============================================================================ */

/** @brief Global state for tracking time with rollover handling */
static struct {
    uint32_t rollover_count;      /**< Number of times counter has rolled over */
    uint32_t last_rtc_value;      /**< Last known RTC counter value */
    nrf_atomic_flag_t lock;       /**< Spinlock for atomic access */
    bool initialized;             /**< Initialization flag */
} m_timing_state = {
    .rollover_count = 0,
    .last_rtc_value = 0,
    .lock = 0,
    .initialized = false
};

/* ============================================================================
 * INTERRUPT HANDLER (for RTC overflow)
 * ============================================================================ */

/**
 * @brief RTC overflow interrupt handler
 * 
 * Called when RTC counter rolls over (every ~512 seconds)
 * 
 * @param[in] instance  RTC instance (unused)
 */
static void rtc_overflow_handler(nrf_drv_rtc_int_type_t int_type, void *p_context)
{
    UNUSED_PARAMETER(p_context);
    
    if (int_type == NRF_DRV_RTC_INT_OVERFLOW)
    {
        // Atomic increment without disabling interrupts
        nrf_atomic_u32_add(&m_timing_state.rollover_count, 1);
        NRF_LOG_DEBUG("RTC overflow detected. Rollover count: %u", 
                      nrf_atomic_u32_fetch_store(&m_timing_state.rollover_count, 
                      nrf_atomic_u32_fetch_store(&m_timing_state.rollover_count, 0)));
    }
}

/* ============================================================================
 * PUBLIC FUNCTIONS
 * ============================================================================ */

/**
 * @brief Initialize timing utilities
 * 
 * Initializes RTC1 overflow interrupt detection for tracking rollovers.
 * Must be called early in application startup, after app_timer_init().
 * 
 * @return NRF_SUCCESS if successful, error code otherwise
 */
ret_code_t timing_init(void)
{
    if (m_timing_state.initialized)
    {
        return NRF_SUCCESS;  // Already initialized
    }

    ret_code_t err_code;

    // Configure RTC for overflow interrupt only
    nrf_drv_rtc_config_t config = NRF_DRV_RTC_DEFAULT_CONFIG;
    config.prescaler = 0;  // Default prescaler (32.768 kHz clock)

    // Initialize RTC1 (used by app_timer)
    err_code = nrf_drv_rtc_init(TIMING_RTC_INSTANCE, &config, rtc_overflow_handler);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("Failed to initialize RTC: 0x%08X", err_code);
        return err_code;
    }

    // Enable overflow interrupt
    nrf_drv_rtc_overflow_enable(TIMING_RTC_INSTANCE, true);
    nrf_drv_rtc_enable(TIMING_RTC_INSTANCE);

    // Read initial RTC value
    m_timing_state.last_rtc_value = nrf_drv_rtc_counter_get(TIMING_RTC_INSTANCE);
    m_timing_state.rollover_count = 0;
    m_timing_state.initialized = true;

    NRF_LOG_INFO("Timing utilities initialized (RTC freq: %u Hz, period: ~512 sec)",
                 RTC_FREQUENCY);

    return NRF_SUCCESS;
}

/**
 * @brief Get current RTC counter value (raw ticks)
 * 
 * Returns the raw 24-bit RTC counter value without rollover compensation.
 * Useful for very short intervals where rollover is impossible.
 * 
 * @return Raw RTC counter value (0 to 16,777,215)
 */
uint32_t timing_get_raw_ticks(void)
{
    if (!m_timing_state.initialized)
    {
        NRF_LOG_WARNING("Timing not initialized. Call timing_init() first.");
        return 0;
    }

    return nrf_drv_rtc_counter_get(TIMING_RTC_INSTANCE);
}

/**
 * @brief Get absolute timestamp in RTC ticks (with rollover handling)
 * 
 * Returns a 64-bit absolute timestamp that accounts for counter rollovers.
 * Combines the 24-bit RTC counter with rollover count for seamless timing.
 * 
 * Example:
 *   - At startup (0 rollovers):  0x00000000_12345678
 *   - After 512 seconds (1 rollover): 0x00000001_87654321
 * 
 * @return 64-bit absolute timestamp in RTC ticks
 * 
 * @note This function is designed to be called regularly (at least once per
 *       ~256 seconds) to catch rollovers. Calling more frequently is safer.
 */
uint64_t timing_get_ticks(void)
{
    if (!m_timing_state.initialized)
    {
        NRF_LOG_WARNING("Timing not initialized. Call timing_init() first.");
        return 0;
    }

    uint32_t current_rtc = nrf_drv_rtc_counter_get(TIMING_RTC_INSTANCE);
    uint32_t rollover_count;

    // Check for rollover (atomic read)
    CRITICAL_REGION_ENTER();
    
    // If current counter is less than last recorded, a rollover occurred
    if (current_rtc < m_timing_state.last_rtc_value)
    {
        // Rollover detected - this should match the interrupt, but handle it anyway
        m_timing_state.rollover_count++;
        NRF_LOG_DEBUG("Rollover detected via counter comparison");
    }

    m_timing_state.last_rtc_value = current_rtc;
    rollover_count = m_timing_state.rollover_count;

    CRITICAL_REGION_EXIT();

    // Combine rollovers and current tick into 64-bit value
    // Upper 32 bits: rollover count
    // Lower 32 bits: current RTC value (24-bit, but extended to 32)
    uint64_t absolute_ticks = ((uint64_t)rollover_count << 24) | current_rtc;

    return absolute_ticks;
}

/**
 * @brief Get absolute timestamp in milliseconds (with rollover handling)
 * 
 * Converts absolute RTC timestamp to milliseconds.
 * 
 * Conversion: 1 millisecond = 32.768 RTC ticks (approximately)
 * 
 * Example:
 *   - 0 ticks = 0 ms
 *   - 32,768 ticks = 1,000 ms (1 second)
 *   - 16,777,216 ticks = 512,000 ms (512 seconds, after 1 rollover)
 * 
 * @return Absolute timestamp in milliseconds (64-bit value)
 */
uint64_t timing_get_milliseconds(void)
{
    uint64_t ticks = timing_get_ticks();
    
    // Convert ticks to milliseconds
    // Formula: ms = ticks / (RTC_FREQUENCY / 1000)
    //             = ticks * 1000 / RTC_FREQUENCY
    //             = ticks * 1000 / 32768
    // Using division to avoid float operations
    return (ticks * 1000) / RTC_FREQUENCY;
}

/**
 * @brief Get absolute timestamp in microseconds (approximate, with rollover handling)
 * 
 * Converts absolute RTC timestamp to microseconds.
 * Resolution is limited by the RTC frequency (approximately 30.5 μs per tick).
 * 
 * Note: This is an integer approximation and may lose some precision.
 * For maximum precision, use timing_get_ticks() and convert as needed.
 * 
 * Example:
 *   - 0 ticks = 0 μs
 *   - 33 ticks ≈ 1,000 μs (1 ms)
 *   - 32,768,000 ticks ≈ 1,000,000 μs (1 second)
 * 
 * @return Approximate timestamp in microseconds (64-bit value)
 */
uint64_t timing_get_microseconds(void)
{
    uint64_t ticks = timing_get_ticks();
    
    // Convert ticks to microseconds
    // Formula: μs = ticks / (RTC_FREQUENCY / 1000000)
    //              = ticks * 1000000 / RTC_FREQUENCY
    //              = ticks * 1000000 / 32768
    return (ticks * 1000000) / RTC_FREQUENCY;
}

/**
 * @brief Get time difference in ticks between two timestamps
 * 
 * Safely calculates the difference between two 64-bit timestamps,
 * accounting for rollover and wraparound.
 * 
 * @param[in] from_ticks  Starting timestamp (from timing_get_ticks())
 * @param[in] to_ticks    Ending timestamp (from timing_get_ticks())
 * 
 * @return Time elapsed in ticks (to_ticks - from_ticks)
 * 
 * Example:
 *   uint64_t start = timing_get_ticks();
 *   // ... do something ...
 *   uint64_t end = timing_get_ticks();
 *   uint64_t elapsed_ticks = timing_tick_diff(start, end);
 *   uint64_t elapsed_ms = timing_ticks_to_ms(elapsed_ticks);
 */
uint64_t timing_tick_diff(uint64_t from_ticks, uint64_t to_ticks)
{
    if (to_ticks >= from_ticks)
    {
        return to_ticks - from_ticks;
    }
    else
    {
        // This shouldn't happen with 64-bit timestamps, but handle it safely
        NRF_LOG_WARNING("Time went backwards: from=0x%016llX, to=0x%016llX",
                        from_ticks, to_ticks);
        return 0;
    }
}

/**
 * @brief Convert RTC ticks to milliseconds
 * 
 * Utility function to convert a tick value to milliseconds.
 * 
 * @param[in] ticks  Number of RTC ticks to convert
 * 
 * @return Equivalent time in milliseconds
 */
uint64_t timing_ticks_to_ms(uint64_t ticks)
{
    return (ticks * 1000) / RTC_FREQUENCY;
}

/**
 * @brief Convert RTC ticks to microseconds (approximate)
 * 
 * Utility function to convert a tick value to microseconds.
 * Resolution is limited by RTC frequency.
 * 
 * @param[in] ticks  Number of RTC ticks to convert
 * 
 * @return Approximate time in microseconds (integer division)
 */
uint64_t timing_ticks_to_us(uint64_t ticks)
{
    return (ticks * 1000000) / RTC_FREQUENCY;
}

/**
 * @brief Convert RTC ticks to seconds (floating point)
 * 
 * Utility function to convert a tick value to seconds with decimal places.
 * 
 * @param[in] ticks  Number of RTC ticks to convert
 * 
 * @return Time in seconds (floating point)
 */
float timing_ticks_to_seconds(uint64_t ticks)
{
    return (float)ticks / RTC_FREQUENCY;
}

/**
 * @brief Convert milliseconds to RTC ticks
 * 
 * Utility function to convert milliseconds to tick count.
 * 
 * @param[in] milliseconds  Time in milliseconds
 * 
 * @return Equivalent number of RTC ticks
 */
uint64_t timing_ms_to_ticks(uint64_t milliseconds)
{
    return (milliseconds * RTC_FREQUENCY) / 1000;
}

/**
 * @brief Get timing system information for debugging
 * 
 * Returns current timing state information useful for debugging.
 * 
 * @param[out] p_info  Pointer to timing_info_t structure to fill
 */
void timing_get_info(timing_info_t *p_info)
{
    if (p_info == NULL)
    {
        return;
    }

    CRITICAL_REGION_ENTER();
    p_info->rtc_frequency = RTC_FREQUENCY;
    p_info->rtc_counter_max = RTC_COUNTER_MAX;
    p_info->rollover_count = nrf_atomic_u32_fetch_store(&m_timing_state.rollover_count, 
                                                        m_timing_state.rollover_count);
    p_info->current_rtc_ticks = nrf_drv_rtc_counter_get(TIMING_RTC_INSTANCE);
    p_info->is_initialized = m_timing_state.initialized;
    CRITICAL_REGION_EXIT();

    p_info->absolute_ticks = ((uint64_t)p_info->rollover_count << RTC_COUNTER_BITS) | 
                            p_info->current_rtc_ticks;
    p_info->absolute_ms = timing_ticks_to_ms(p_info->absolute_ticks);
    p_info->rollover_period_seconds = (float)(RTC_COUNTER_MAX + 1) / RTC_FREQUENCY;
}

/**
 * @brief Print timing information to logs
 * 
 * Convenience function to log current timing state.
 */
void timing_print_info(void)
{
    timing_info_t info;
    timing_get_info(&info);

    if (!info.is_initialized)
    {
        NRF_LOG_INFO("Timing: NOT INITIALIZED");
        return;
    }

    NRF_LOG_INFO("=== Timing Information ===");
    NRF_LOG_INFO("RTC Frequency: %u Hz", info.rtc_frequency);
    NRF_LOG_INFO("Counter Max: %u (0x%08X)", info.rtc_counter_max, info.rtc_counter_max);
    NRF_LOG_INFO("Current RTC Ticks: %u", info.current_rtc_ticks);
    NRF_LOG_INFO("Rollover Count: %u", info.rollover_count);
    NRF_LOG_INFO("Absolute Ticks: 0x%016llX", info.absolute_ticks);
    NRF_LOG_INFO("Absolute Time: %llu ms (%.2f sec)", info.absolute_ms, 
                 timing_ticks_to_seconds(info.absolute_ticks));
    NRF_LOG_INFO("Rollover Period: %.1f seconds", info.rollover_period_seconds);
    NRF_LOG_INFO("==========================");
}
