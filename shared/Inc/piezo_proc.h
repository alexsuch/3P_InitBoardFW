/**
 * @file piezo_proc.h
 * @brief Real-time ADC processing for piezo sensor (release version, not logger)
 *
 * Architecture:
 *   - ADC runs at ~100 kHz via DMA (16-sample blocks)
 *   - ISR callbacks push 8 samples per half/full-transfer into ring buffer (512 samples)
 *   - Quick trigger logic in ISR detects suspicious events
 *   - Snapshot mechanism captures 64-sample window for heavy analysis in main loop
 *
 * Design constraints:
 *   - No RTOS, pure C
 *   - ISR execution time < 10-20 µs (minimal work only)
 *   - Ring buffer size = power of 2 for fast modulo
 *   - Heavy processing happens in main loop via piezo_poll()
 *
 * Integration:
 *   1. Call piezo_init(callback) from App_InitRun()
 *   2. Route HAL_ADC_ConvHalfCpltCallback() -> piezo_on_dma_half()
 *   3. Route HAL_ADC_ConvCpltCallback() -> piezo_on_dma_full()
 *   4. Call piezo_poll() from App_Task()
 *   5. Callback will be called on snapshot ready for custom analysis
 */

#ifndef SHARED_INC_PIEZO_PROC_H_
#define SHARED_INC_PIEZO_PROC_H_

#include <stdbool.h>
#include <stdint.h>

#include "init_brd.h"
#include "prj_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#if PIEZO_DETECTION_ENABLE

/* ============================================================================
 * CONFIGURATION CONSTANTS
 * ============================================================================
 */

/* Compile-time check: PIEZO_HIST_N must be power of 2 */
#if (PIEZO_HIST_N & (PIEZO_HIST_N - 1)) != 0
#error "PIEZO_HIST_N must be power of two (256, 512, 1024, etc.)"
#endif

/* ============================================================================
 * DATA STRUCTURES
 * ============================================================================
 */

/**
 * @brief Piezo detection event types
 */
typedef enum {
    PIEZO_EVT_IDLE = 0,
    PIEZO_EVT_INIT_OK,
    PIEZO_EVT_SNAPSHOT_READY,
    PIEZO_EVT_TRIGGER_DETECTED,
} piezo_evt_t;

/**
 * @brief Piezo real-time processing status
 *
 * Contains ring buffer, snapshot buffers, and event flags for ISR<->main communication.
 * All fields accessed from ISR are marked volatile.
 */
typedef struct {
    // Pointer to ADC DMA buffer (owned by HAL, size = PIEZO_ADC_DMA_BLOCK samples)
    volatile int16_t* adc_dma_buf;

    // History ring buffer (512 samples for look-back)
    int16_t hist[PIEZO_HIST_N];
    volatile uint16_t hist_w;  // Write index (next write position)
    uint16_t hist_mask;        // PIEZO_HIST_N - 1 (for fast modulo)

    // ISR -> main loop event flags
    volatile bool need_snapshot;      // ISR sets when trigger detected
    volatile uint16_t snap_w_anchor;  // Captured hist_w at trigger time

    // Snapshot linear buffer (64 samples extracted in main loop)
    int16_t snap[PIEZO_SNAP_N];
    volatile bool snap_ready;  // Main loop sets after snapshot extraction

    // Diagnostic counters
    volatile uint32_t isr_half_count;  // Half-transfer callback count
    volatile uint32_t isr_full_count;  // Full-transfer callback count
    volatile uint32_t snapshot_count;  // Total snapshots captured
    volatile uint32_t trigger_count;   // Total trigger events
} piezo_status_t;

/* ============================================================================
 * PUBLIC API
 * ============================================================================
 */

/**
 * @brief Initialize piezo processing subsystem
 *
 * Must be called once from App_InitRun() before ADC DMA starts.
 * Resets all buffers, counters, and flags. Stores callback for event notification.
 *
 * @param sys_cbk: System callback (app_cbk_fn) for piezo events (PIEZO_EVT_xxx)
 */
void Piezo_Init(app_cbk_fn sys_cbk);

/**
 * @brief DMA half-transfer callback (processes first 8 samples)
 *
 * Called from HAL_ADC_ConvHalfCpltCallback() when first half of DMA buffer is ready.
 * Pushes samples [0..7] into ring buffer and checks trigger condition.
 *
 * ISR execution time: < 10 µs @ 170 MHz
 */
void Piezo_OnDmaHalf(void);

/**
 * @brief DMA full-transfer callback (processes second 8 samples)
 *
 * Called from HAL_ADC_ConvCpltCallback() when second half of DMA buffer is ready.
 * Pushes samples [8..15] into ring buffer and checks trigger condition.
 *
 * ISR execution time: < 10 µs @ 170 MHz
 */
void Piezo_OnDmaFull(void);

/**
 * @brief Main loop polling function
 *
 * Call from App_Task() (every iteration or every few ms).
 * Handles snapshot extraction and calls callback when ready.
 *
 * Processing:
 *   1. If need_snapshot flag set: extract PIEZO_SNAP_N samples into snap[] buffer
 *   2. Call registered callback with PIEZO_EVT_SNAPSHOT_READY
 *   3. Clear flags and wait for next trigger
 */
void Piezo_Task(void);

/**
 * @brief Get relative sample from history (x[-k])
 *
 * Access previous samples relative to current write position.
 * k_back = 1 means previous sample, k_back = 2 means two samples ago, etc.
 *
 * @param k_back: Number of samples to look back (1..PIEZO_HIST_N)
 *
 * @return Sample value at position (hist_w - k_back)
 */
int16_t Piezo_GetRel(uint16_t k_back);

/**
 * @brief Try to extract snapshot into user buffer
 *
 * Copies the most recent snapshot if available.
 * Returns false if no snapshot ready or buffer too small.
 *
 * @param out: Output buffer (must be at least n samples)
 * @param n: Number of samples to copy (should be <= PIEZO_SNAP_N)
 *
 * @return true if snapshot copied, false otherwise
 */
bool Piezo_TryGetSnapshot(int16_t* out, uint16_t n);

/**
 * @brief Stop piezo detection and reset state
 *
 * Resets all buffers, counters, and flags. Does not clear callback.
 */
void Piezo_Stop(void);

#endif /* PIEZO_DETECTION_ENABLE */

#ifdef __cplusplus
}
#endif

#endif  // SHARED_INC_PIEZO_PROC_H_
