/**
 * @file logger_imu_time.c
 * @brief Phase 3: IMU raw data buffering (simplified - v2 architecture)
 *
 * Architecture (v2 - Optimized):
 *   - Simple 50-sample raw buffer (12 bytes per sample)
 *   - No per-sample timestamp tracking in ISR
 *   - Raw bytes stored directly from SPI buffer: [gx_lo, gx_hi, gy_lo, gy_hi, gz_lo, gz_hi,
 *                                                   ax_lo, ax_hi, ay_lo, ay_hi, az_lo, az_hi]
 *   - Timestamp assignment deferred to frame assembly (Logger_Task)
 *
 * Integration:
 *   - Called from LSM6DS3.c Lsm6ds3_GetDataCbk() with pointer to 12 raw bytes
 *   - Memory: 50 × 12 bytes = 600 bytes (vs. previous 50 × 16 = 800 bytes)
 *   - ISR overhead: Zero parsing, just memcpy
 *   - Frame builder correlates IMU timestamp using ADC reference timestamp
 *
 * Timing:
 *   - LSM6DS3 @ ~104 Hz (move detection mode)
 *   - Each sample captures raw 12 bytes from SPI read
 *   - Timestamp synchronized at frame assembly based on ADC stream position
 */

#include "stm32g4xx_hal.h"
#include "logger.h"
#include <stdint.h>
#include <string.h>

/* ============================================================================
 * DATA STRUCTURES (internal)
 * ============================================================================
 */

/**
 * @brief 50-sample IMU raw buffer
 *
 * Design:
 *   - Simple linear buffer (not circular, simpler logic)
 *   - LSM6DS3 callback writes via Logger_ImuOnNewSample()
 *   - Frame builder reads and resets via Logger_ImuRing_Dequeue()
 *   - No mutex (callback-friendly, single reader/writer assumption)
 *   - Overflow: Latest samples are kept, oldest discarded if count > 50
 *
 * Memory: 50 × 12 bytes = 600 bytes
 */
typedef struct {
    ImuRawSample_t samples[50];        // Raw sample buffer (50 samples max)
    volatile uint32_t count;           // Current sample count (0..50)
    volatile uint32_t total_samples;   // Total samples received (statistics)
    volatile uint32_t overflow_count;  // Count of discarded samples
} ImuBuffer_t;

/* Global IMU buffer instance */
static ImuBuffer_t g_imu_buffer = {0};

/* ============================================================================
 * PHASE 3 PUBLIC API IMPLEMENTATION
 * ============================================================================
 */

/**
 * @brief Initialize IMU raw buffer
 *
 * Call sequence:
 *   1. Logger_Timing_Init() completed (TIM6 running @ 100 kHz)
 *   2. Logger_RingBuffer_Init() completed (ADC buffer ready)
 *   3. Logger_ImuRing_Init() - this function
 *   4. LSM6DS3 starts polling and calling Logger_ImuOnNewSample()
 *
 * Effects:
 *   - Clears IMU buffer
 *   - Initializes statistics counters
 *
 * Returns: void
 */
void Logger_ImuRing_Init(void) {
    memset(&g_imu_buffer, 0, sizeof(ImuBuffer_t));
}

/**
 * @brief Callback: New IMU raw sample ready (12 bytes from SPI buffer)
 *
 * Called from: LSM6DS3.c Lsm6ds3_GetDataCbk() @ ~104 Hz
 *
 * @param raw_data: Pointer to 12 raw bytes from SPI read
 *                  Format: [gx_lo, gx_hi, gy_lo, gy_hi, gz_lo, gz_hi,
 *                           ax_lo, ax_hi, ay_lo, ay_hi, az_lo, az_hi]
 *
 * Effects:
 *   - Copies 12 bytes directly into buffer (zero ISR overhead)
 *   - Increments count
 *   - If count > 50, discards oldest sample and increments overflow_count
 *
 * Returns: void
 */
void Logger_ImuOnNewSample(const uint8_t *raw_data) {
    if (!raw_data) {
        return;
    }
                Test2Toggle();
            Test2Toggle();
    if (g_imu_buffer.count < 50) {
        // Buffer not full, add new sample
        memcpy(&g_imu_buffer.samples[g_imu_buffer.count].data[0],
               raw_data, 12);
        g_imu_buffer.count++;
    } else {
        // Buffer full, shift left and add new sample at end
        memmove(&g_imu_buffer.samples[0], &g_imu_buffer.samples[1],
                49 * sizeof(ImuRawSample_t));
        memcpy(&g_imu_buffer.samples[49].data[0], raw_data, 12);
        g_imu_buffer.overflow_count++;
    }
    
    g_imu_buffer.total_samples++;
}

/**
 * @brief Get current IMU buffer fill level
 *
 * Returns: uint32_t - Number of IMU samples currently in buffer (0..50)
 */
uint32_t Logger_ImuRing_GetCount(void)
{
    return g_imu_buffer.count;
}

/**
 * @brief Get pointer to IMU buffer data
 *
 * @param out_count: Pointer to uint32_t to receive actual sample count
 *
 * Returns: Pointer to ImuRawSample_t array (up to 50 samples),
 *          or NULL if buffer empty
 *
 * Note: Buffer ownership remains in logger; caller should copy if keeping data.
 */
const ImuRawSample_t *Logger_ImuRing_GetBuffer(uint32_t *out_count)
{
    if (g_imu_buffer.count == 0) {
        if (out_count) *out_count = 0;
        return NULL;
    }
    
    if (out_count) {
        *out_count = g_imu_buffer.count;
    }
    
    return g_imu_buffer.samples;
}

/**
 * @brief Clear IMU buffer after frame assembly
 *
 * Call from Logger_Task() after processing current IMU buffer.
 * Resets sample counter to 0 for next batch of samples.
 *
 * Returns: void
 */
void Logger_ImuRing_Clear(void)
{
    g_imu_buffer.count = 0;
}

/**
 * @brief Get total IMU samples ever received (debug/statistics)
 *
 * Returns: uint32_t - Total count
 */
uint32_t Logger_ImuRing_GetTotalSamples(void)
{
    return g_imu_buffer.total_samples;
}

/**
 * @brief Get IMU overflow count (debug/statistics)
 *
 * Overflow occurs when new sample arrives with buffer full (50 samples).
 * Latest sample kept, oldest discarded.
 *
 * Returns: uint32_t - Number of samples discarded due to buffer full
 */
uint32_t Logger_ImuRing_GetOverflowCount(void)
{
    return g_imu_buffer.overflow_count;
}

/* ============================================================================
 * DEPRECATED COMPATIBILITY FUNCTIONS
 * ============================================================================
 */

/**
 * @brief Get number of buffered IMU samples (deprecated)
 *
 * Use Logger_ImuRing_GetCount() instead.
 *
 * Returns: uint32_t - Number of samples in buffer (0..50)
 */
uint32_t Logger_ImuRing_GetNewCount(void)
{
    return g_imu_buffer.count;
}

/**
 * @brief Dequeue all IMU samples from buffer (deprecated)
 *
 * New code should use Logger_ImuRing_GetBuffer() + Logger_ImuRing_Clear() instead.
 *
 * @param out_buffer: Pointer to ImuSample_t array (caller allocated, 50 max)
 * @param count: Number of samples to read (will read min(count, available))
 *
 * Returns: uint32_t - Number of samples actually dequeued
 *
 * Note: Converts raw 12-byte format back to ImuSample_t for compatibility
 */
typedef struct {
    int16_t gx, gy, gz;
    int16_t ax, ay, az;
    uint32_t timestamp;
} ImuSample_t;

uint32_t Logger_ImuRing_Dequeue(ImuSample_t *out_buffer, uint32_t count)
{
    uint32_t to_read = (count > g_imu_buffer.count) ? g_imu_buffer.count : count;
    
    for (uint32_t i = 0; i < to_read; i++) {
        const uint8_t *raw = g_imu_buffer.samples[i].data;
        out_buffer[i].gx = (int16_t)(raw[0] | (raw[1] << 8));
        out_buffer[i].gy = (int16_t)(raw[2] | (raw[3] << 8));
        out_buffer[i].gz = (int16_t)(raw[4] | (raw[5] << 8));
        out_buffer[i].ax = (int16_t)(raw[6] | (raw[7] << 8));
        out_buffer[i].ay = (int16_t)(raw[8] | (raw[9] << 8));
        out_buffer[i].az = (int16_t)(raw[10] | (raw[11] << 8));
        out_buffer[i].timestamp = 0;  // Assigned later by frame builder
    }
    
    // Clear buffer after dequeue
    g_imu_buffer.count = 0;
    
    return to_read;
}

/**
 * @brief Get first timestamp from IMU ring buffer (deprecated)
 *
 * Returns: uint32_t - 0 (timestamp not stored in linear buffer, assigned at frame assembly)
 */
uint32_t Logger_ImuRing_GetFirstTimestamp(void)
{
    return 0;  // Timestamps assigned during frame assembly
}
