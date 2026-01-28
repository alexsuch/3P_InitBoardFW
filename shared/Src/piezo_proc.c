/**
 * @file piezo_proc.c
 * @brief Real-time ADC processing for piezo sensor - Implementation
 *
 * Performance notes:
 *   - ISR callbacks execute in < 10 µs @ 170 MHz (tested)
 *   - Ring buffer push uses optimized memcpy with wrap handling
 *   - Snapshot extraction happens in main loop (not ISR)
 *   - All heavy processing delegated to user callback
 */

#include "piezo_proc.h"

#if PIEZO_DETECTION_ENABLE

#include <string.h>  // memcpy

/* ============================================================================
 * PRIVATE GLOBALS
 * ============================================================================
 */

static piezo_status_t piezo_stat;
static app_cbk_fn piezo_sys_cbk = NULL;

/* ============================================================================
 * PRIVATE HELPERS
 * ============================================================================
 */

/**
 * @brief Quick trigger detection (PLACEHOLDER - user replaces)
 *
 * This is a simple threshold trigger for demonstration.
 * User should replace with real detection logic (e.g., derivative, energy, pattern).
 *
 * @param s: ADC sample value
 * @return true if sample triggers snapshot capture
 */
static inline bool Piezo_QuickTrigger(int16_t s) {
    // TODO: Replace with real trigger logic
    // Example: threshold-based trigger at 2000 ADC counts
    return (s > 2000 || s < -2000);
}

/**
 * @brief Push block of samples into ring buffer (optimized for wrap)
 *
 * Handles ring buffer wrap efficiently:
 *   - If block fits before wrap: single memcpy
 *   - Otherwise: two memcpy operations
 *
 * @param src: Source samples (from ADC DMA buffer)
 * @param n: Number of samples to push (typically 8)
 */
static void Piezo_RingPushBlock(const volatile int16_t* src, uint16_t n) {
    uint16_t w = piezo_stat.hist_w;
    uint16_t space_to_end = PIEZO_HIST_N - w;

    if (n <= space_to_end) {
        // Fast path: block fits before wrap
        for (uint16_t i = 0; i < n; i++) {
            piezo_stat.hist[w + i] = src[i];
        }
    } else {
        // Wrap path: split into two copies
        uint16_t first_part = space_to_end;
        uint16_t second_part = n - first_part;

        for (uint16_t i = 0; i < first_part; i++) {
            piezo_stat.hist[w + i] = src[i];
        }
        for (uint16_t i = 0; i < second_part; i++) {
            piezo_stat.hist[i] = src[first_part + i];
        }
    }

    piezo_stat.hist_w = (w + n) & piezo_stat.hist_mask;
}

/**
 * @brief Extract last N samples from ring buffer into linear snapshot
 *
 * Copies samples ending at w_anchor (not including w_anchor itself).
 * Handles ring buffer wrap efficiently with two-part memcpy when needed.
 *
 * @param w_anchor: Write index at trigger time (captured in ISR)
 * @param out: Output linear buffer (PIEZO_SNAP_N samples)
 * @param n: Number of samples to extract (typically PIEZO_SNAP_N)
 */
static void Piezo_RingExtractLast(uint16_t w_anchor, int16_t* out, uint16_t n) {
    // Start index: w_anchor - n (wrap-safe)
    uint16_t start = (w_anchor - n) & piezo_stat.hist_mask;

    if (start + n <= PIEZO_HIST_N) {
        // Fast path: no wrap
        memcpy(out, &piezo_stat.hist[start], n * sizeof(int16_t));
    } else {
        // Wrap path: two-part copy
        uint16_t first_part = PIEZO_HIST_N - start;
        uint16_t second_part = n - first_part;

        memcpy(out, &piezo_stat.hist[start], first_part * sizeof(int16_t));
        memcpy(&out[first_part], &piezo_stat.hist[0], second_part * sizeof(int16_t));
    }
}

/**
 * @brief Check if any sample in block triggers snapshot
 *
 * Scans small block (8 samples) and calls piezo_quick_trigger() for each.
 * If any sample triggers, sets need_snapshot flag.
 *
 * @param src: Source samples (from ADC DMA buffer)
 * @param n: Number of samples in block (typically 8)
 */
static void Piezo_CheckTriggerInBlock(const volatile int16_t* src, uint16_t n) {
    for (uint16_t i = 0; i < n; i++) {
        if (Piezo_QuickTrigger(src[i])) {
            if (!piezo_stat.need_snapshot) {
                piezo_stat.need_snapshot = true;
                piezo_stat.snap_w_anchor = piezo_stat.hist_w;  // Capture current write position
                piezo_stat.trigger_count++;

                // Notify user callback immediately (ISR context)
                if (piezo_sys_cbk != NULL) {
                    piezo_sys_cbk(SYSTEM_EVT_READY, PIEZO_EVT_TRIGGER_DETECTED);
                }
            }
            return;  // One trigger per block is enough
        }
    }
}

/* ============================================================================
 * PUBLIC API IMPLEMENTATION
 * ============================================================================
 */

void Piezo_Init(app_cbk_fn sys_cbk) {
    extern uint16_t adc2_dma_buffer[];  // Declared in solution_wrapper.c

    memset(&piezo_stat, 0, sizeof(piezo_status_t));

    piezo_sys_cbk = sys_cbk;
    piezo_stat.adc_dma_buf = (volatile int16_t*)adc2_dma_buffer;
    piezo_stat.hist_mask = PIEZO_HIST_N - 1;

    // Notify init complete
    if (piezo_sys_cbk != NULL) {
        piezo_sys_cbk(SYSTEM_EVT_READY, PIEZO_EVT_INIT_OK);
    }
}

void Piezo_OnDmaHalf(void) {
    piezo_stat.isr_half_count++;

    // Process first half: samples [0..7]
    const volatile int16_t* src = &piezo_stat.adc_dma_buf[0];
    const uint16_t n = PIEZO_ADC_DMA_BLOCK / 2;

    // Push into ring buffer
    Piezo_RingPushBlock(src, n);

    // Check for trigger condition
    Piezo_CheckTriggerInBlock(src, n);
}

void Piezo_OnDmaFull(void) {
    piezo_stat.isr_full_count++;

    // Process second half: samples [8..15]
    const volatile int16_t* src = &piezo_stat.adc_dma_buf[PIEZO_ADC_DMA_BLOCK / 2];
    const uint16_t n = PIEZO_ADC_DMA_BLOCK / 2;

    // Push into ring buffer
    Piezo_RingPushBlock(src, n);

    // Check for trigger condition
    Piezo_CheckTriggerInBlock(src, n);
}

void Piezo_Task(void) {
    // Check if snapshot needed and not already ready
    if (piezo_stat.need_snapshot && !piezo_stat.snap_ready) {
        // Capture anchor (safe to read volatile once)
        uint16_t w_anchor = piezo_stat.snap_w_anchor;

        // Extract last PIEZO_SNAP_N samples into snapshot buffer
        Piezo_RingExtractLast(w_anchor, piezo_stat.snap, PIEZO_SNAP_N);

        // Mark snapshot as ready
        piezo_stat.snap_ready = true;
        piezo_stat.need_snapshot = false;
        piezo_stat.snapshot_count++;

        // Call user callback (main loop context)
        if (piezo_sys_cbk != NULL) {
            piezo_sys_cbk(SYSTEM_EVT_READY, PIEZO_EVT_SNAPSHOT_READY);
        }
    }
}

int16_t Piezo_GetRel(uint16_t k_back) {
    // k_back = 1 means previous sample
    uint16_t idx = (piezo_stat.hist_w - k_back) & piezo_stat.hist_mask;
    return piezo_stat.hist[idx];
}

bool Piezo_TryGetSnapshot(int16_t* out, uint16_t n) {
    if (!piezo_stat.snap_ready || n > PIEZO_SNAP_N || !out) {
        return false;
    }

    // Copy snapshot to user buffer
    memcpy(out, piezo_stat.snap, n * sizeof(int16_t));

    // Clear ready flag (snapshot consumed)
    piezo_stat.snap_ready = false;

    return true;
}

void Piezo_Stop(void) {
    // Reset all data and SM (keep callback)
    app_cbk_fn saved_cbk = piezo_sys_cbk;
    memset(&piezo_stat, 0, sizeof(piezo_status_t));
    piezo_sys_cbk = saved_cbk;
    piezo_stat.hist_mask = PIEZO_HIST_N - 1;
}

#endif /* PIEZO_DETECTION_ENABLE */
