/**
 * @file logger_adc_buffer.c
 * @brief Phase 2: ADC linear buffer (configurable-sample, ready flag architecture)
 *
 * Architecture:
 *   - LOGGER_ADC_BLOCK_SIZE-sample linear buffer (simple, not circular)
 *   - Ready flag set when LOGGER_ADC_BLOCK_SIZE samples accumulated
 *   - No per-sample timestamps (only block reference timestamp)
 *   - Main loop polls ready flag and processes entire buffer
 *
 * Data Flow:
 *   DMA Complete Interrupt (every block_time ms @ ADC_SAMPLING_FREQ_KHZ kHz):
 *   └─ Logger_AdcBuffer_OnComplete() → Set ready flag
 *
 *   Main Loop:
 *   └─ while (Logger_AdcBuffer_IsReady()) {
 *        adc_buffer = Logger_AdcBuffer_GetBuffer(&count)
 *        // Process LOGGER_ADC_BLOCK_SIZE samples with reference timestamp
 *        Logger_AdcBuffer_ClearReady()
 *      }
 *
 * Timing:
 *   - DMA circular buffer: ADC_DMA_BUFFER_SIZE samples, two halves
 *   - We use only complete (2nd half) callback: ADC_DMA_HALF_SIZE samples
 *   - Block time: (ADC_DMA_HALF_SIZE / ADC_SAMPLING_FREQ_KHZ) ms
 *   - Timestamp captured at start of LOGGER_ADC_BLOCK_SIZE-sample block
 *   - Configuration in prj_config.h (ADC_SAMPLING_FREQ_KHZ, ADC_DMA_BUFFER_SIZE)
 */

#include "stm32g4xx_hal.h"
#include "logger.h"
#include "prj_config.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* ============================================================================
 * DATA STRUCTURES (internal)
 * ============================================================================
 */

/**
 * @brief LOGGER_ADC_BLOCK_SIZE-sample ADC linear buffer
 *
 * Design:
 *   - Simple linear buffer (ready flag indicates valid data)
 *   - DMA interrupt writes via Logger_AdcBuffer_OnComplete()
 *   - Main loop reads via Logger_AdcBuffer_GetBuffer()
 *   - After processing, main clears flag via Logger_AdcBuffer_ClearReady()
 *   - Next DMA complete will fill buffer again
 *
 * Memory: LOGGER_ADC_BLOCK_SIZE × 2 bytes (int16_t)
 * LOGGER_ADC_BLOCK_SIZE defined in prj_config.h (typically 256 samples)
 * Note: Buffer size is fixed to maximum expected size (256 samples)
 *       Actual data count is tracked separately if needed
 */
#define ADC_BUFFER_MAX_SAMPLES 256  /* Maximum ADC buffer size - cannot be VLA in struct */

typedef struct {
    int16_t samples[ADC_BUFFER_MAX_SAMPLES];  // ADC samples array (int16_t), max 256
    volatile uint32_t ready;                  // 1 = buffer ready, 0 = processed
    volatile uint32_t block_timestamp;        // Reference timestamp of first sample
} AdcBuffer_t;

/* Global ADC buffer instance */
static AdcBuffer_t g_adc_buffer = {0};

/* ============================================================================
 * PHASE 2 PUBLIC API IMPLEMENTATION
 * ============================================================================
 */

/**
 * @brief Initialize ADC2 linear buffer
 *
 * Call sequence:
 *   1. Logger_Timing_Init() completed (TIM6 running @ ADC_SAMPLING_FREQ_KHZ kHz)
 *   2. ADC2 DMA configured and running (from main.c / CubeMX)
 *   3. Logger_AdcBuffer_Init() - this function
 *   4. Main loop starts polling Logger_AdcBuffer_IsReady()
 *
 * Effects:
 *   - Clears buffer
 *   - Resets ready flag to 0
 *   - Resets statistics
 *
 * Returns: void
 */
void Logger_AdcBuffer_Init(void)
{
    memset(&g_adc_buffer, 0, sizeof(AdcBuffer_t));
}

/**
 * @brief Check if new ADC buffer is ready (LOGGER_ADC_BLOCK_SIZE samples accumulated)
 *
 * Returns: bool - true if ready flag set, false otherwise
 */
bool Logger_AdcBuffer_IsReady(void)
{
    return g_adc_buffer.ready != 0;
}

/**
 * @brief Get pointer to ready ADC buffer (LOGGER_ADC_BLOCK_SIZE samples)
 *
 * @param out_count: Pointer to uint32_t to receive sample count (always LOGGER_ADC_BLOCK_SIZE)
 *
 * Returns: Pointer to int16_t array (LOGGER_ADC_BLOCK_SIZE samples), or NULL if not ready
 *
 * Note: Must call Logger_AdcBuffer_IsReady() first to check readiness.
 *       Buffer remains valid until next DMA_COMPLETE interrupt.
 *
 * Usage:
 *   if (Logger_AdcBuffer_IsReady()) {
 *       uint32_t count = 0;
 *       const int16_t *samples = Logger_AdcBuffer_GetBuffer(&count);
 *       // samples[0..LOGGER_ADC_BLOCK_SIZE-1] contain ADC values
 *       // count == LOGGER_ADC_BLOCK_SIZE
 *   }
 */
const int16_t *Logger_AdcBuffer_GetBuffer(uint32_t *out_count)
{
    if (!g_adc_buffer.ready) {
        if (out_count) *out_count = 0;
        return NULL;
    }
    
    if (out_count) {
        *out_count = 256;
    }
    
    return g_adc_buffer.samples;
}

/**
 * @brief Clear ready flag after processing current ADC buffer
 *
 * Call from main loop after Logger_Task() completes processing.
 * Next DMA_COMPLETE interrupt will fill buffer again and set ready flag.
 *
 * Returns: void
 *
 * Usage:
 *   Logger_AdcBuffer_ClearReady();  // After frame assembly
 *   // DMA will set ready flag again when next LOGGER_ADC_BLOCK_SIZE samples ready
 */
void Logger_AdcBuffer_ClearReady(void)
{
    g_adc_buffer.ready = 0;
}

/**
 * @brief Get total samples ever transferred (debug/statistics)
 *
 * Returns: uint32_t - Total count (wraps at 2^32)
 *
 * Useful for monitoring ADC data flow and detecting stalls.
 */
// REMOVED - not used in this architecture

/**
 * @brief DMA complete callback (called from HAL_ADC_ConvCpltCallback)
 *
 * Called when LOGGER_ADC_BLOCK_SIZE samples accumulated in DMA circular buffer.
 * Sets ready flag and stores reference timestamp for this block.
 *
 * Integration:
 *   In solution_wrapper.c HAL_ADC_ConvCpltCallback():
 *   if (hadc == &ADC_PIEZO_HANDLE) {
 *       extern uint16_t adc_dma_buffer[];  // From CubeMX (ADC_DMA_BUFFER_SIZE samples total)
 *       Logger_AdcBuffer_OnComplete(&adc_dma_buffer[ADC_DMA_HALF_SIZE], ADC_DMA_HALF_SIZE);
 *   }
 *
 * @param dma_buffer: Pointer to DMA result buffer (LOGGER_ADC_BLOCK_SIZE uint16_t samples)
 * @param count: Number of samples (should be LOGGER_ADC_BLOCK_SIZE, typically 256)
 *
 * Returns: void
 */
void Logger_AdcBuffer_OnComplete(const uint16_t *dma_buffer, uint32_t count)
{
    if (!dma_buffer || count != LOGGER_ADC_BLOCK_SIZE) {
        return;
    }
    
    // Copy LOGGER_ADC_BLOCK_SIZE samples from DMA buffer to internal storage using memcpy
    memcpy(g_adc_buffer.samples, dma_buffer, LOGGER_ADC_BLOCK_SIZE * sizeof(int16_t));
    
    // Store reference timestamp (first sample of this block)
    g_adc_buffer.block_timestamp = Logger_GetTimestamp();

    // Set ready flag
    g_adc_buffer.ready = 1;
}

/* ============================================================================
 * DIAGNOSTIC / DEBUG
 * ============================================================================
 */

/**
 * @brief Get reference timestamp for current ADC block (first sample)
 *
 * This is the timestamp of the first sample in the current buffer.
 * Useful for frame assembly to correlate ADC with IMU samples.
 *
 * Returns: uint32_t - Timestamp in TIM6 ticks (ADC_SAMPLING_FREQ_KHZ kHz = (1000 / ADC_SAMPLING_FREQ_KHZ) µs per tick)
 *
 * Note: Return value only valid when Logger_RingBuffer_IsReady() == 1
 */
uint32_t Logger_AdcBuffer_GetFirstTimestamp(void)
{
    return g_adc_buffer.block_timestamp;
}
