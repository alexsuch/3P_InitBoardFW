/**
 * @file logger_test.c
 * @brief Testing and debugging utilities for logger data flow
 *
 * Provides diagnostic functions to verify that:
 * 1. ADC2 DMA is running and filling buffer
 * 2. LSM6DS3 is reading and calling callbacks
 * 3. Logger ring buffers are receiving data
 * 4. Frame builder is processing data
 */

#include "logger.h"
#include <stdint.h>

/* Test counters - increment in callbacks to verify data flow */
static volatile uint32_t test_adc_callback_count = 0;
static volatile uint32_t test_imu_callback_count = 0;
static volatile uint32_t test_frame_build_count = 0;

/**
 * @brief Get ADC callback invocation counter
 * Returns how many times ADC DMA callbacks have fired
 */
uint32_t Logger_Test_GetAdcCallbackCount(void)
{
    return test_adc_callback_count;
}

/**
 * @brief Get IMU callback invocation counter
 * Returns how many times LSM6DS3 callback has fired
 */
uint32_t Logger_Test_GetImuCallbackCount(void)
{
    return test_imu_callback_count;
}

/**
 * @brief Get frame build counter
 * Returns how many frames have been built
 */
uint32_t Logger_Test_GetFrameCount(void)
{
    return test_frame_build_count;
}

/**
 * @brief Increment ADC callback counter (called from logger_adc_ring.c)
 */
void Logger_Test_IncrementAdcCount(void)
{
    test_adc_callback_count++;
}

/**
 * @brief Increment IMU callback counter (called from logger_imu_time.c)
 */
void Logger_Test_IncrementImuCount(void)
{
    test_imu_callback_count++;
}

/**
 * @brief Increment frame build counter (called from logger_frame.c)
 */
void Logger_Test_IncrementFrameCount(void)
{
    test_frame_build_count++;
}

/**
 * @brief Reset all test counters
 */
void Logger_Test_ResetCounters(void)
{
    test_adc_callback_count = 0;
    test_imu_callback_count = 0;
    test_frame_build_count = 0;
}

/**
 * @brief Diagnostic status snapshot
 *
 * Usage in GDB:
 *   break app.c:2391    # In App_Task()
 *   commands
 *   call Logger_Test_GetAdcCallbackCount()
 *   call Logger_Test_GetImuCallbackCount()
 *   call Logger_Test_GetFrameCount()
 *   continue
 *   end
 */
typedef struct {
    uint32_t adc_callbacks;      /* Number of ADC DMA callbacks */
    uint32_t imu_callbacks;      /* Number of LSM6DS3 callbacks */
    uint32_t frames_built;       /* Number of frames built */
    uint32_t adc_ring_count;     /* Current ADC ring fill level */
    uint32_t imu_ring_count;     /* Current IMU ring fill level */
} LoggerTestStatus_t;

void Logger_Test_GetStatus(LoggerTestStatus_t *status)
{
    if (!status) return;
    
    status->adc_callbacks = test_adc_callback_count;
    status->imu_callbacks = test_imu_callback_count;
    status->frames_built = test_frame_build_count;
    status->adc_ring_count = Logger_AdcBuffer_IsReady() ? 256 : 0;
    status->imu_ring_count = Logger_ImuRing_GetCount();
    
    /* Optional: Print to UART for monitoring
       printf("Logger Test Status:\r\n"
              "  ADC callbacks: %lu\r\n"
              "  IMU callbacks: %lu\r\n"
              "  Frames built: %lu\r\n"
              "  ADC ring: %lu samples\r\n"
              "  IMU ring: %lu samples\r\n",
              status->adc_callbacks, status->imu_callbacks, status->frames_built,
              status->adc_ring_count, status->imu_ring_count);
    */
}
