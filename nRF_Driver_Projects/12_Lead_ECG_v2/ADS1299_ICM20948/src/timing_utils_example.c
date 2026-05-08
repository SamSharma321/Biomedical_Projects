/**
 * @file timing_utils_example.c
 * @brief Example implementations of timing_utils integration with sensor data
 * 
 * This file demonstrates how to use the timing_utils module in your application
 * for timestamping ICM20649 and EEG samples, measuring performance, and more.
 * 
 * These are reference implementations - adapt them to your specific needs.
 */

#include "timing_utils.h"
#include "nrf_log.h"
#include <stdint.h>

/* ============================================================================
 * EXAMPLE 1: Timestamped ICM20649 Sample Structure
 * ============================================================================ */

/**
 * @brief ICM20649 sample with absolute timestamp
 * 
 * Use this structure to bundle sensor data with the exact time it was collected.
 */
typedef struct {
    uint64_t timestamp_ms;           /**< Absolute time sample was collected (ms) */
    uint32_t timestamp_us;           /**< Sub-millisecond precision timestamp (μs) */
    int16_t accel_x;                 /**< X-axis acceleration (mg) */
    int16_t accel_y;                 /**< Y-axis acceleration (mg) */
    int16_t accel_z;                 /**< Z-axis acceleration (mg) */
    int16_t gyro_x;                  /**< X-axis angular velocity (°/s) */
    int16_t gyro_y;                  /**< Y-axis angular velocity (°/s) */
    int16_t gyro_z;                  /**< Z-axis angular velocity (°/s) */
    int16_t temp;                    /**< Temperature sensor (°C / 100) */
} icm_sample_timestamped_t;

/**
 * @brief Read ICM20649 sample with timestamp
 * 
 * Example function showing how to timestamp sensor data at collection time.
 * The timestamp is taken BEFORE reading the sensor to minimize latency.
 * 
 * @param[out] p_sample  Pointer to ICM sample structure
 * @return True if sample was successfully read
 */
bool icm_read_sample_timestamped(icm_sample_timestamped_t *p_sample)
{
    if (p_sample == NULL)
    {
        return false;
    }

    // Timestamp FIRST (before any other operations)
    // This gives the most accurate collection time
    p_sample->timestamp_ms = timing_get_milliseconds();
    
    // Also capture sub-millisecond time for future use
    // Note: You may want to store this separately for higher precision
    uint64_t ticks = timing_get_ticks();
    uint64_t total_us = timing_ticks_to_us(ticks);
    p_sample->timestamp_us = (uint32_t)(total_us % 1000);  // Microseconds within ms

    // Now read sensor data (actual collection time is ~1-2ms after timestamp)
    // TODO: Replace with your actual ICM read function
    // icm_read_accelerometer(&p_sample->accel_x, &p_sample->accel_y, &p_sample->accel_z);
    // icm_read_gyroscope(&p_sample->gyro_x, &p_sample->gyro_y, &p_sample->gyro_z);
    // icm_read_temperature(&p_sample->temp);

    return true;
}

/* ============================================================================
 * EXAMPLE 2: Timestamped EEG Sample Structure
 * ============================================================================ */

/**
 * @brief EEG sample with absolute timestamp
 */
typedef struct {
    uint64_t timestamp_ms;           /**< Time sample was collected (ms) */
    uint32_t channel_count;          /**< Number of channels in this sample */
    uint32_t channels[8];            /**< EEG data for up to 8 channels */
} eeg_sample_timestamped_t;

/**
 * @brief Record EEG sample with timestamp
 * 
 * Example function for timestamping multi-channel EEG data.
 * 
 * @param[out] p_sample  Pointer to EEG sample structure
 * @param[in] channel_count Number of channels to read
 * @return True if sample was successfully read
 */
bool eeg_read_sample_timestamped(eeg_sample_timestamped_t *p_sample, 
                                  uint32_t channel_count)
{
    if (p_sample == NULL || channel_count > 8)
    {
        return false;
    }

    // Timestamp at collection time
    p_sample->timestamp_ms = timing_get_milliseconds();
    p_sample->channel_count = channel_count;

    // TODO: Read EEG data from ADS1299
    // for (uint32_t i = 0; i < channel_count; i++)
    // {
    //     p_sample->channels[i] = ads_read_channel(i);
    // }

    return true;
}

/* ============================================================================
 * EXAMPLE 3: Performance Monitoring
 * ============================================================================ */

/**
 * @brief Structure for tracking function performance
 */
typedef struct {
    const char *function_name;       /**< Name of function being measured */
    uint64_t call_count;             /**< Number of times called */
    uint64_t total_time_ticks;       /**< Total execution time (ticks) */
    uint64_t min_time_ticks;         /**< Minimum execution time (ticks) */
    uint64_t max_time_ticks;         /**< Maximum execution time (ticks) */
} perf_metrics_t;

/**
 * @brief Initialize performance metrics
 */
void perf_metrics_init(perf_metrics_t *p_metrics, const char *name)
{
    if (p_metrics != NULL)
    {
        p_metrics->function_name = name;
        p_metrics->call_count = 0;
        p_metrics->total_time_ticks = 0;
        p_metrics->min_time_ticks = UINT64_MAX;
        p_metrics->max_time_ticks = 0;
    }
}

/**
 * @brief Record a function execution time
 */
void perf_metrics_record(perf_metrics_t *p_metrics, uint64_t execution_ticks)
{
    if (p_metrics != NULL)
    {
        p_metrics->call_count++;
        p_metrics->total_time_ticks += execution_ticks;
        
        if (execution_ticks < p_metrics->min_time_ticks)
        {
            p_metrics->min_time_ticks = execution_ticks;
        }
        if (execution_ticks > p_metrics->max_time_ticks)
        {
            p_metrics->max_time_ticks = execution_ticks;
        }
    }
}

/**
 * @brief Print performance metrics
 */
void perf_metrics_print(const perf_metrics_t *p_metrics)
{
    if (p_metrics == NULL || p_metrics->call_count == 0)
    {
        return;
    }

    uint64_t avg_ticks = p_metrics->total_time_ticks / p_metrics->call_count;
    float avg_ms = timing_ticks_to_ms(avg_ticks) / 1000.0f;
    float min_ms = timing_ticks_to_ms(p_metrics->min_time_ticks) / 1000.0f;
    float max_ms = timing_ticks_to_ms(p_metrics->max_time_ticks) / 1000.0f;

    NRF_LOG_INFO("=== Performance: %s ===", p_metrics->function_name);
    NRF_LOG_INFO("Calls: %llu", p_metrics->call_count);
    NRF_LOG_INFO("Avg: %.3f ms, Min: %.3f ms, Max: %.3f ms", 
                 avg_ms, min_ms, max_ms);
}

/**
 * @brief Example: Measure GPIO interrupt handler performance
 */
void gpio_handler_performance_example(void)
{
    static perf_metrics_t gpio_metrics = {0};
    static bool initialized = false;

    if (!initialized)
    {
        perf_metrics_init(&gpio_metrics, "GPIO_IRQ_Handler");
        initialized = true;
    }

    // At START of GPIO handler
    uint64_t handler_start = timing_get_ticks();
    
    // ... GPIO handler code ...
    
    // At END of GPIO handler
    uint64_t handler_end = timing_get_ticks();
    uint64_t handler_duration = timing_tick_diff(handler_start, handler_end);
    perf_metrics_record(&gpio_metrics, handler_duration);

    // Periodically print metrics
    static uint32_t print_counter = 0;
    if (++print_counter >= 1000)
    {
        perf_metrics_print(&gpio_metrics);
        print_counter = 0;
    }
}

/* ============================================================================
 * EXAMPLE 4: Rate Limiting / Throttling
 * ============================================================================ */

/**
 * @brief Structure for rate limiting
 */
typedef struct {
    uint64_t last_event_ms;          /**< Time of last event (ms) */
    uint64_t min_interval_ms;        /**< Minimum interval between events (ms) */
    uint32_t event_count;            /**< Number of events since last reset */
} rate_limiter_t;

/**
 * @brief Initialize rate limiter
 */
void rate_limiter_init(rate_limiter_t *p_limiter, uint64_t min_interval_ms)
{
    if (p_limiter != NULL)
    {
        p_limiter->last_event_ms = timing_get_milliseconds();
        p_limiter->min_interval_ms = min_interval_ms;
        p_limiter->event_count = 0;
    }
}

/**
 * @brief Check if sufficient time has elapsed for next event
 * 
 * Returns true if the minimum interval has passed since the last event.
 * If true, updates the last event time.
 * 
 * Usage example:
 * @code
 *   static rate_limiter_t log_limiter;
 *   static bool init = false;
 *   
 *   if (!init) {
 *       rate_limiter_init(&log_limiter, 1000);  // Limit to 1 per second
 *       init = true;
 *   }
 *   
 *   if (rate_limiter_allow(&log_limiter)) {
 *       NRF_LOG_INFO("Throttled message");
 *   }
 * @endcode
 */
bool rate_limiter_allow(rate_limiter_t *p_limiter)
{
    if (p_limiter == NULL)
    {
        return false;
    }

    uint64_t now = timing_get_milliseconds();
    uint64_t elapsed = timing_tick_diff(p_limiter->last_event_ms, now);

    if (elapsed >= p_limiter->min_interval_ms)
    {
        p_limiter->last_event_ms = now;
        p_limiter->event_count++;
        return true;
    }

    return false;
}

/* ============================================================================
 * EXAMPLE 5: Sample Buffer with Timestamps
 * ============================================================================ */

#define SAMPLE_BUFFER_SIZE 256

/**
 * @brief Timestamped sample buffer
 */
typedef struct {
    uint64_t timestamp_ms;           /**< When sample was collected */
    uint32_t sample_id;              /**< Sample sequence number */
    uint32_t raw_data;               /**< Raw sensor data */
} sample_entry_t;

typedef struct {
    sample_entry_t buffer[SAMPLE_BUFFER_SIZE];
    uint32_t write_index;            /**< Next write position */
    uint32_t entry_count;            /**< Total entries logged */
} sample_buffer_t;

/**
 * @brief Initialize sample buffer
 */
void sample_buffer_init(sample_buffer_t *p_buf)
{
    if (p_buf != NULL)
    {
        p_buf->write_index = 0;
        p_buf->entry_count = 0;
    }
}

/**
 * @brief Add timestamped sample to buffer
 * 
 * Records a sample with its collection timestamp.
 * Wraps around if buffer is full.
 */
void sample_buffer_add(sample_buffer_t *p_buf, uint32_t raw_data)
{
    if (p_buf == NULL)
    {
        return;
    }

    // Write entry
    p_buf->buffer[p_buf->write_index].timestamp_ms = timing_get_milliseconds();
    p_buf->buffer[p_buf->write_index].sample_id = p_buf->entry_count;
    p_buf->buffer[p_buf->write_index].raw_data = raw_data;

    // Update indices
    p_buf->write_index = (p_buf->write_index + 1) % SAMPLE_BUFFER_SIZE;
    p_buf->entry_count++;
}

/**
 * @brief Print buffer contents for debugging
 */
void sample_buffer_print(const sample_buffer_t *p_buf, uint32_t max_entries)
{
    if (p_buf == NULL || max_entries == 0)
    {
        return;
    }

    uint32_t entries_to_print = (p_buf->entry_count < max_entries) ? 
                                 p_buf->entry_count : max_entries;

    NRF_LOG_INFO("Sample Buffer (showing last %u of %llu entries):",
                 entries_to_print, p_buf->entry_count);

    uint32_t start_index = (p_buf->write_index - entries_to_print + SAMPLE_BUFFER_SIZE) 
                           % SAMPLE_BUFFER_SIZE;

    for (uint32_t i = 0; i < entries_to_print; i++)
    {
        uint32_t idx = (start_index + i) % SAMPLE_BUFFER_SIZE;
        const sample_entry_t *entry = &p_buf->buffer[idx];
        
        NRF_LOG_INFO("[%u] Time: %llu ms, ID: %u, Data: 0x%08X",
                     i, entry->timestamp_ms, entry->sample_id, entry->raw_data);
    }
}

/* ============================================================================
 * EXAMPLE 6: Periodic Task Scheduler
 * ============================================================================ */

/**
 * @brief Periodic task with timestamp tracking
 */
typedef struct {
    const char *task_name;
    uint64_t last_run_ms;            /**< Time of last execution */
    uint64_t period_ms;              /**< Execution period */
    uint32_t run_count;              /**< Number of times executed */
} periodic_task_t;

/**
 * @brief Initialize periodic task
 */
void periodic_task_init(periodic_task_t *p_task, const char *name, 
                        uint64_t period_ms)
{
    if (p_task != NULL)
    {
        p_task->task_name = name;
        p_task->last_run_ms = timing_get_milliseconds();
        p_task->period_ms = period_ms;
        p_task->run_count = 0;
    }
}

/**
 * @brief Check if it's time to run the task
 * 
 * Returns true if the specified period has elapsed since last run.
 * Call this in your main loop or timer handler.
 * 
 * Usage:
 * @code
 *   static periodic_task_t batt_task;
 *   
 *   void init() {
 *       periodic_task_init(&batt_task, "Battery", 60000);  // Every 60 seconds
 *   }
 *   
 *   void main_loop() {
 *       if (periodic_task_should_run(&batt_task)) {
 *           update_battery_level();
 *       }
 *   }
 * @endcode
 */
bool periodic_task_should_run(periodic_task_t *p_task)
{
    if (p_task == NULL)
    {
        return false;
    }

    uint64_t now = timing_get_milliseconds();
    uint64_t elapsed = timing_tick_diff(p_task->last_run_ms, now);

    if (elapsed >= p_task->period_ms)
    {
        p_task->last_run_ms = now;
        p_task->run_count++;
        return true;
    }

    return false;
}

/**
 * @brief Get statistics about periodic task
 */
float periodic_task_get_actual_period_ms(const periodic_task_t *p_task)
{
    if (p_task == NULL || p_task->run_count < 2)
    {
        return 0.0f;
    }

    // Return target period (actual might vary due to scheduling)
    return (float)p_task->period_ms;
}

/* ============================================================================
 * EXAMPLE 7: System Startup Timing
 * ============================================================================ */

/**
 * @brief Record major startup milestones
 */
typedef struct {
    uint64_t power_on_ms;            /**< System power-on time (0) */
    uint64_t clock_init_ms;          /**< When clock initialized */
    uint64_t ble_init_ms;            /**< When BLE stack ready */
    uint64_t services_ready_ms;      /**< When all services initialized */
    uint64_t advertising_start_ms;   /**< When BLE advertising started */
} startup_timing_t;

static startup_timing_t startup_times = {0};

/**
 * @brief Record clock initialization time
 * 
 * Call this right after timing_init()
 */
void startup_record_clock_init(void)
{
    startup_times.power_on_ms = 0;  // Reference point
    startup_times.clock_init_ms = timing_get_milliseconds();
    NRF_LOG_INFO("Clock initialized at +%llu ms", startup_times.clock_init_ms);
}

/**
 * @brief Record BLE initialization time
 */
void startup_record_ble_init(void)
{
    startup_times.ble_init_ms = timing_get_milliseconds();
    uint64_t elapsed = startup_times.ble_init_ms - startup_times.clock_init_ms;
    NRF_LOG_INFO("BLE initialized at +%llu ms (clock to BLE: %llu ms)",
                 startup_times.ble_init_ms, elapsed);
}

/**
 * @brief Record services ready time
 */
void startup_record_services_ready(void)
{
    startup_times.services_ready_ms = timing_get_milliseconds();
    uint64_t elapsed = startup_times.services_ready_ms - startup_times.clock_init_ms;
    NRF_LOG_INFO("Services ready at +%llu ms (clock to ready: %llu ms)",
                 startup_times.services_ready_ms, elapsed);
}

/**
 * @brief Record advertising start time
 */
void startup_record_advertising_start(void)
{
    startup_times.advertising_start_ms = timing_get_milliseconds();
    uint64_t elapsed = startup_times.advertising_start_ms - startup_times.clock_init_ms;
    NRF_LOG_INFO("Advertising started at +%llu ms (total startup: %llu ms)",
                 startup_times.advertising_start_ms, elapsed);
    
    // Print full startup summary
    NRF_LOG_INFO("=== STARTUP TIMELINE ===");
    NRF_LOG_INFO("Clock init: %llu ms", startup_times.clock_init_ms);
    NRF_LOG_INFO("BLE init: %llu ms", startup_times.ble_init_ms);
    NRF_LOG_INFO("Services: %llu ms", startup_times.services_ready_ms);
    NRF_LOG_INFO("Advertising: %llu ms", startup_times.advertising_start_ms);
    NRF_LOG_INFO("========================");
}

/* ============================================================================
 * INTEGRATION WITH main.c
 * ============================================================================ */

/**
 * @brief Example main() initialization with timing
 * 
 * Demonstrates the proper sequence for initializing timing utilities
 * and recording startup milestones.
 */
void main_example(void)
{
    // Initialize NRF_LOG
    // log_init();
    
    // Initialize Bluetooth stack FIRST
    // ble_stack_init();
    // gap_params_init();
    // gatt_init();
    
    // Initialize app_timer (required before timing_utils)
    // timers_init();
    
    // Initialize timing utilities
    ret_code_t err_code = timing_init();
    APP_ERROR_CHECK(err_code);
    startup_record_clock_init();
    
    // Initialize BLE services
    // services_init();
    startup_record_ble_init();
    
    // Setup advertising
    // advertising_init();
    startup_record_services_ready();
    
    // Start advertising
    // advertising_start();
    startup_record_advertising_start();
    
    // Main loop
    NRF_LOG_INFO("System ready. Entering main loop...");
    
    // Print timing info at startup
    timing_print_info();
}
