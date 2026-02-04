/**
 * @file logger_spi.c
 * @brief Frame Builder & SPI Protocol - Merges ADC + IMU data, manages SPI communication
 *
 * Role:
 *   1. Frame Assembly: Merge ADC samples (256 @ 100kHz) + IMU samples (0-50 @ ~104Hz) into
 *      fixed-size LogFrame_t structures with checksum validation
 *   2. Frame Queue: Maintain circular FIFO for frames awaiting SPI transmission
 *   3. SPI Slave: Handle CMD 42 (0x2A) to transmit logger_config_t to master (ESP32)
 *   4. Auto-Streaming: After config handshake, automatically stream queued frames via DMA
 *
 * Frame Assembly Path:
 *   - Wait for ADC block (256 samples @ 100kHz = ~2.56ms)
 *   - Collect available IMU samples (0-50) from ring buffer
 *   - Assemble frame with checksum (CRC8 or SUM8, selected at build time)
 *   - Queue for transmission (circular FIFO: up to LOGGER_FRAME_QUEUE_SIZE frames)
 *
 * SPI Protocol (Master: ESP32, Slave: STM32):
 *   - Master sends CMD 42 at startup to request config
 *   - Slave responds with logger_config_t (64B) via DMA
 *   - After config TX: Frames auto-stream from queue
 *   - GPIO_READY signals data availability during transmission
 *
 * Frame Layout (LOGGER_FRAME_SIZE_BYTES bytes):
 *   - magic (2B): 0x5A5A (frame marker)
 *   - n_imu (2B): 0-20 valid IMU samples in frame
 *   - adc[256] (512B): ADC int16_t samples
 *   - imu[20] (320B): IMU samples (16B each incl. timestamp, only n_imu valid)
 *   - mavlink_log (10B): MAVLink event flags + telemetry
 *   - checksum8 (1B): checksum (CRC8 or SUM8)
 *   - checksum_pad (1B): reserved (0)
 *   Total: 852 bytes
 */

#include "LSM6DS3.h"
#include "logger.h"
#include "mavlink_uart.h"
#include "prj_config.h"
#include "solution_wrapper.h"

#if (LOGGER_CHECKSUM_ALGO == LOGGER_CHECKSUM_ALGO_CRC8_HW)
#include "stm32g4xx.h"
#include "stm32g4xx_hal_rcc.h"
#endif

#if (LOGGER_PROFILING_ENABLE == 1u)
#include "stm32g4xx.h"
#include "stm32g4xx_hal_rcc.h"
#endif

#define LOGGER_FRAME_SIZE_BYTES sizeof(LogFrame_t)  // Auto-calculated from LogFrame_t structure

static logger_status_t loggerStat = {0};
static uint8_t s_logging_started = 0;
static volatile uint8_t s_start_pending = 0;

volatile logger_profile_t g_logger_profile = {0};

#if (LOGGER_PROFILING_ENABLE == 1u)
/**
 * @brief Initialize ARM DWT (Data Watchpoint and Trace) cycle counter
 *
 * Called once on first Logger_Task() invocation when profiling is enabled.
 * Configures the Cortex-M debug hardware for CPU cycle counting:
 *   1. Enables trace unit via CoreDebug->DEMCR (TRCENA bit)
 *   2. Resets cycle counter DWT->CYCCNT to zero
 *   3. Enables cycle counter via DWT->CTRL (CYCCNTENA bit)
 *
 * After init, DWT->CYCCNT increments by 1 each CPU clock cycle (e.g., 170 MHz).
 * Read via logger_profile_now_cycles() for timing measurements.
 *
 * Note: DWT is available on Cortex-M3/M4/M7. No additional HW cost - uses
 * existing debug registers. Counter wraps at ~25 seconds @ 170 MHz.
 */
static void logger_profile_dwt_enable_once(void) {
    static uint8_t inited = 0;
    if (inited) {
        return;
    }
    inited = 1;

    Logger_Profile_Reset();

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;  // Enable trace unit
    DWT->CYCCNT = 0;                                 // Reset cycle counter
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;             // Enable cycle counter

    g_logger_profile.enabled = 1u;
    g_logger_profile.cpu_hz = HAL_RCC_GetHCLKFreq();
}

static inline uint32_t logger_profile_now_cycles(void) { return DWT->CYCCNT; }

static inline void logger_profile_update_counter(volatile logger_prof_counter_t* counter, uint32_t delta_cycles) {
    counter->count++;
    counter->last_cycles = delta_cycles;
    counter->sum_cycles += (uint64_t)delta_cycles;
    if (delta_cycles > counter->max_cycles) {
        counter->max_cycles = delta_cycles;
    }
}
#endif

void Logger_Profile_Reset(void) { memset((void*)&g_logger_profile, 0, sizeof(g_logger_profile)); }

volatile uint32_t g_logger_frames_built = 0;
volatile uint32_t g_logger_frames_dropped = 0;

/* ADC buffer is initialized via static storage class (zero-initialized by default) */
static AdcBuffer_t g_adc_buffer = {0};

/* IMU buffer is initialized via static storage class (zero-initialized by default) */
static ImuBuffer_t g_imu_buffer = {0};

// Some builds can observe occasional duplicate IMU reads (same 12 raw bytes)
// within a few TIM1 ticks, typically caused by polling the DRDY/INT level
// and re-reading while the sensor data hasn't advanced yet.
// De-duplicate at the logger ingestion point to keep frame IMU blocks clean.
static uint8_t g_last_imu_raw[IMU_RAW_DATA_SIZE] = {0};
static uint32_t g_last_imu_ts = 0;
static uint8_t g_have_last_imu = 0;

/* ============================================================================
 * ADC BUFFER API
 * ============================================================================
 */

volatile uint32_t g_logger_adc_overruns = 0;

/**
 * @brief Get pointer to the oldest queued ADC block (LOGGER_ADC_BLOCK_SIZE samples)
 *
 * @param out_count: Pointer to uint32_t to receive sample count (always LOGGER_ADC_BLOCK_SIZE)
 *
 * Returns: Pointer to int16_t array (LOGGER_ADC_BLOCK_SIZE samples), or NULL if no block queued
 *
 * Note: The returned pointer stays valid until Logger_AdcBuffer_ClearReady() is called.
 *
 * Usage:
 *   uint32_t count = 0;
 *   const int16_t* samples = Logger_AdcBuffer_GetBuffer(&count);
 *   if (samples) { ...; Logger_AdcBuffer_ClearReady(); }
 */
const int16_t* Logger_AdcBuffer_GetBuffer(uint32_t* out_count) {
    if (g_adc_buffer.count == 0) {
        if (out_count) *out_count = 0;
        return NULL;
    }

    if (out_count) {
        *out_count = ADC_BUFFER_SIZE;
    }

    return g_adc_buffer.samples[g_adc_buffer.read_idx];
}

uint32_t Logger_AdcBuffer_GetFirstTimestamp(void) { return g_adc_buffer.block_timestamp; }

/**
 * @brief Advance ADC queue read pointer after processing current block (consumer side)
 *
 * Called from main loop (Logger_Task) after a frame is built from the ADC block.
 * Removes the oldest block from the FIFO queue, making room for new DMA blocks.
 *
 * Producer-Consumer Pattern:
 *   - Producer: Logger_AdcBuffer_OnComplete() called from DMA ISR (HAL_ADC_ConvCpltCallback)
 *   - Consumer: Logger_Task() in main loop reads block, then calls this to release it
 *
 * Why __disable_irq() is needed:
 *   The DMA complete ISR can fire at any time and modify queue state (write_idx, count).
 *   This function reads and modifies shared variables (read_idx, count, ready, timestamp).
 *   Without disabling interrupts, a race condition could occur:
 *     1. Main loop reads count=1
 *     2. ISR fires, increments count to 2
 *     3. Main loop decrements count to 0 (wrong! should be 1)
 *   The critical section ensures atomic read-modify-write of all queue state.
 *   Duration is very short (~10 cycles), so interrupt latency impact is minimal.
 */
void Logger_AdcBuffer_ClearReady(void) {
    __disable_irq();
    if (g_adc_buffer.count > 0) {
        g_adc_buffer.read_idx = (g_adc_buffer.read_idx + 1u) % ADC_BLOCK_QUEUE_DEPTH;
        g_adc_buffer.count--;
        g_adc_buffer.ready = (g_adc_buffer.count > 0u) ? ADC_FRAME_READY_FLAG : ADC_FRAME_EMPTY_FLAG;
        if (g_adc_buffer.count > 0u) {
            g_adc_buffer.block_timestamp = g_adc_buffer.block_timestamps[g_adc_buffer.read_idx];
        } else {
            g_adc_buffer.block_timestamp = 0;
        }
    }
    __enable_irq();
}

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
void Logger_AdcBuffer_OnComplete(const uint16_t* dma_buffer, uint32_t count) {
    if (!dma_buffer || count != LOGGER_ADC_BLOCK_SIZE) {
        return;
    }

    // Store reference timestamp for this ADC block as the timestamp of the *first* sample.
    // We capture the tick in the DMA callback (after the block completes) and back-calculate
    // the first-sample timestamp using the known block size.
    uint32_t ts_end = Logger_GetTimestamp();
    uint32_t ts_first = ts_end - LOGGER_ADC_BLOCK_SIZE;

    // Queue the block; if the queue is full we drop the newest block (and count an overrun).
    uint32_t w = g_adc_buffer.write_idx;
    if (g_adc_buffer.count >= ADC_BLOCK_QUEUE_DEPTH) {
        g_adc_buffer.overruns++;
        g_logger_adc_overruns = g_adc_buffer.overruns;
        return;
    }

    // Copy LOGGER_ADC_BLOCK_SIZE samples from DMA buffer to queued storage.
    memcpy(g_adc_buffer.samples[w], dma_buffer, LOGGER_ADC_BLOCK_SIZE * sizeof(int16_t));
    g_adc_buffer.block_timestamps[w] = ts_first;
    g_adc_buffer.write_idx = (w + 1u) % ADC_BLOCK_QUEUE_DEPTH;
    g_adc_buffer.count++;

    // Set ready flag
    g_adc_buffer.ready = ADC_FRAME_READY_FLAG;
    if (g_adc_buffer.count == 1u) {
        g_adc_buffer.read_idx = w;
        g_adc_buffer.block_timestamp = ts_first;
    }
}

/* ============================================================================
 * IMU RING BUFFER API (from logger_imu_time.c)
 * ============================================================================
 */

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
 *   - Captures and stores timestamp (100 kHz tick counter)
 *   - Increments count
 *   - If count > 50, discards oldest sample and keeps newest
 *
 * Returns: void
 */
void Logger_ImuOnNewSample(const uint8_t* raw_data) {
    // Test1Toggle();  // T1 DEBUG: Toggle on each IMU sample (should be ~6664 Hz)

    if (!raw_data) {
        return;
    }

    uint32_t timestamp = Logger_GetTimestamp();  // 100 kHz tick (10 us), shared timebase with ADC

    // Duplicate sample detection - complements the time-based rearm guard in LSM6DS3.c.
    //
    // Two-layer protection against IMU read issues at high ODR (e.g., 6.6 kHz):
    //
    //   Layer 1 - Rearm Guard (LSM6DS3.c):
    //     Prevents the polling state machine from getting stuck when the DRDY low phase
    //     is missed at high ODR. Uses time-based re-arm (expected_period - 1 ticks) to
    //     force a new read attempt even if DRDY stays high. This ensures reads continue.
    //
    //   Layer 2 - Duplicate Detection (here):
    //     The rearm guard intentionally triggers slightly early to avoid missed samples.
    //     This can cause the same IMU output registers to be read twice before the sensor
    //     updates them. This check filters out such duplicates by comparing:
    //       - Timestamp delta (dt <= 5 ticks = ~50µs, much faster than any valid ODR)
    //       - Raw data bytes (identical 12-byte payload = same sensor reading)
    //
    // Both layers are needed: rearm keeps reads flowing, duplicate detection keeps data clean.
    // At 3332 Hz ODR the expected delta is ~30 ticks, so dt<=5 strongly suggests a duplicate read.
    if (g_have_last_imu) {
        uint32_t dt = (uint32_t)(timestamp - g_last_imu_ts);
        if ((dt <= 5u) && (memcmp(raw_data, g_last_imu_raw, IMU_RAW_DATA_SIZE) == 0)) {
            return;
        }
    }
    memcpy(g_last_imu_raw, raw_data, IMU_RAW_DATA_SIZE);
    g_last_imu_ts = timestamp;
    g_have_last_imu = 1u;

    uint32_t head = g_imu_buffer.head;

    // Write sample to head position (O(1) operation)
    memcpy(&g_imu_buffer.samples[head].data[0], raw_data, IMU_RAW_DATA_SIZE);
    g_imu_buffer.samples[head].timestamp = timestamp;

    // Advance head with wrap-around
    g_imu_buffer.head = (head + 1) % IMU_BUFFER_MAX_SAMPLES;

    if (g_imu_buffer.count < IMU_BUFFER_MAX_SAMPLES) {
        // Buffer not full yet
        g_imu_buffer.count++;
    } else {
        // Buffer full - advance tail to discard oldest sample (no memmove!)
        g_imu_buffer.tail = (g_imu_buffer.tail + 1) % IMU_BUFFER_MAX_SAMPLES;
    }
}

/**
 * @brief Pop IMU samples from ring buffer (consumer function)
 *
 * Copies up to max_count most recent samples from ring buffer to output buffer.
 * Uses critical section for thread-safe access between ISR producer and main loop consumer.
 * Clears the ring buffer after copying.
 *
 * @param out_buf: Destination buffer for samples (must be at least max_count * sizeof(ImuRawSample_t))
 * @param max_count: Maximum number of samples to copy (typically LOGGER_IMU_BLOCK_SIZE = 20)
 *
 * @return Number of samples actually copied (0..max_count)
 */
uint32_t Logger_ImuRing_PopSamples(ImuRawSample_t* out_buf, uint32_t max_count) {
    if (!out_buf || max_count == 0) {
        return 0;
    }

    // Critical section to prevent race with Logger_ImuOnNewSample (ISR context)
    __disable_irq();

    uint32_t count = g_imu_buffer.count;
    if (count == 0) {
        __enable_irq();
        return 0;
    }

    // Calculate how many samples to copy (clamp to max_count, keep most recent)
    uint32_t to_copy = (count > max_count) ? max_count : count;
    uint32_t skip = count - to_copy;  // Skip oldest samples if buffer has more than max_count

    // Calculate starting read position (tail + skip, with wrap-around)
    uint32_t read_idx = (g_imu_buffer.tail + skip) % IMU_BUFFER_MAX_SAMPLES;

    // Copy samples to output buffer
    for (uint32_t i = 0; i < to_copy; i++) {
        out_buf[i] = g_imu_buffer.samples[read_idx];
        read_idx = (read_idx + 1) % IMU_BUFFER_MAX_SAMPLES;
    }

    // Clear ring buffer (reset to empty state)
    g_imu_buffer.head = 0;
    g_imu_buffer.tail = 0;
    g_imu_buffer.count = 0;

    __enable_irq();

    return to_copy;
}

/* IMU buffer clear via Logger_ImuRing_PopSamples or direct reset of head/tail/count */

/* ============================================================================
 * CRC8 CALCULATION (Table-based - Fast)
 * ============================================================================
 */

/* Pre-computed CRC8 lookup table (CRC8_TABLE_SIZE entries) */
/* Polynomial: CRC8_POLYNOMIAL (CRC-8-ATM / CRC-8-CCITT) */
/* Note: Placed in RAM (non-const) to avoid flash wait-states on each lookup in Debug builds. */
static uint8_t CRC8_TABLE[CRC8_TABLE_SIZE] = {0x00,
    0x07,
    0x0E,
    0x09,
    0x1C,
    0x1B,
    0x12,
    0x15,
    0x38,
    0x3F,
    0x36,
    0x31,
    0x24,
    0x23,
    0x2A,
    0x2D,
    0x70,
    0x77,
    0x7E,
    0x79,
    0x6C,
    0x6B,
    0x62,
    0x65,
    0x48,
    0x4F,
    0x46,
    0x41,
    0x54,
    0x53,
    0x5A,
    0x5D,
    0xE0,
    0xE7,
    0xEE,
    0xE9,
    0xFC,
    0xFB,
    0xF2,
    0xF5,
    0xD8,
    0xDF,
    0xD6,
    0xD1,
    0xC4,
    0xC3,
    0xCA,
    0xCD,
    0x90,
    0x97,
    0x9E,
    0x99,
    0x8C,
    0x8B,
    0x82,
    0x85,
    0xA8,
    0xAF,
    0xA6,
    0xA1,
    0xB4,
    0xB3,
    0xBA,
    0xBD,
    0xC7,
    0xC0,
    0xC9,
    0xCE,
    0xDB,
    0xDC,
    0xD5,
    0xD2,
    0xFF,
    0xF8,
    0xF1,
    0xF6,
    0xE3,
    0xE4,
    0xED,
    0xEA,
    0xB7,
    0xB0,
    0xB9,
    0xBE,
    0xAB,
    0xAC,
    0xA5,
    0xA2,
    0x8F,
    0x88,
    0x81,
    0x86,
    0x93,
    0x94,
    0x9D,
    0x9A,
    0x27,
    0x20,
    0x29,
    0x2E,
    0x3B,
    0x3C,
    0x35,
    0x32,
    0x1F,
    0x18,
    0x11,
    0x16,
    0x03,
    0x04,
    0x0D,
    0x0A,
    0x57,
    0x50,
    0x59,
    0x5E,
    0x4B,
    0x4C,
    0x45,
    0x42,
    0x6F,
    0x68,
    0x61,
    0x66,
    0x73,
    0x74,
    0x7D,
    0x7A,
    0x89,
    0x8E,
    0x87,
    0x80,
    0x95,
    0x92,
    0x9B,
    0x9C,
    0xB1,
    0xB6,
    0xBF,
    0xB8,
    0xAD,
    0xAA,
    0xA3,
    0xA4,
    0xF9,
    0xFE,
    0xF7,
    0xF0,
    0xE5,
    0xE2,
    0xEB,
    0xEC,
    0xC1,
    0xC6,
    0xCF,
    0xC8,
    0xDD,
    0xDA,
    0xD3,
    0xD4,
    0x69,
    0x6E,
    0x67,
    0x60,
    0x75,
    0x72,
    0x7B,
    0x7C,
    0x51,
    0x56,
    0x5F,
    0x58,
    0x4D,
    0x4A,
    0x43,
    0x44,
    0x1D,
    0x1A,
    0x13,
    0x14,
    0x01,
    0x06,
    0x0F,
    0x08,
    0x25,
    0x22,
    0x2B,
    0x2C,
    0x39,
    0x3E,
    0x37,
    0x30,
    0x4A,
    0x4D,
    0x44,
    0x43,
    0x56,
    0x51,
    0x58,
    0x5F,
    0x72,
    0x75,
    0x7C,
    0x7B,
    0x6E,
    0x69,
    0x60,
    0x67,
    0x3A,
    0x3D,
    0x34,
    0x33,
    0x26,
    0x21,
    0x28,
    0x2F,
    0x02,
    0x05,
    0x0C,
    0x0B,
    0x1E,
    0x19,
    0x10,
    0x17,
    0xAA,
    0xAD,
    0xA4,
    0xA3,
    0xB6,
    0xB1,
    0xB8,
    0xBF,
    0x92,
    0x95,
    0x9C,
    0x9B,
    0x8E,
    0x89,
    0x80,
    0x87,
    0xDA,
    0xDD,
    0xD4,
    0xD3,
    0xC6,
    0xC1,
    0xC8,
    0xCF,
    0xE2,
    0xE5,
    0xEC,
    0xEB,
    0xFE,
    0xF9,
    0xF0,
    0xF7};

/**
 * @brief Incremental CRC8 calculation (chunk-based for low-latency framing)
 *
 * Processes a chunk of bytes, updating CRC state. Designed to be called
 * repeatedly for large buffers without blocking for long periods.
 *
 * @param crc: Current CRC8 value
 * @param data: Pointer to data chunk
 * @param len: Length of chunk in bytes
 * @return Updated CRC8 value
 */
static inline uint8_t crc8_update_chunk(uint8_t crc, const uint8_t* data, uint32_t len) {
    const uint8_t* table = CRC8_TABLE;
    for (uint32_t i = 0; i < len; i++) {
        crc = table[crc ^ data[i]];
    }
    return crc;
}

static inline uint8_t sum8_update_chunk(uint8_t sum, const uint8_t* data, uint32_t len) {
    for (uint32_t i = 0; i < len; i++) {
        sum = (uint8_t)(sum + data[i]);
    }
    return sum;
}

#if (LOGGER_CHECKSUM_ALGO == LOGGER_CHECKSUM_ALGO_CRC8_HW)
static void crc8_hw_init_once(void) {
    static uint8_t inited = 0;
    if (inited) {
        return;
    }
    inited = 1;

    __HAL_RCC_CRC_CLK_ENABLE();

    // Configure CRC peripheral for CRC-8/ATM: poly=0x07, init=0x00, refin/refout=0, xorout=0.
    CRC->CR &= ~(CRC_CR_POLYSIZE | CRC_CR_REV_IN | CRC_CR_REV_OUT);
    CRC->CR |= CRC_CR_POLYSIZE_1;  // 0b10 => 8-bit polynomial
    CRC->POL = (uint32_t)CRC8_POLYNOMIAL;
}

static inline uint8_t crc8_hw_update_chunk(uint8_t state, const uint8_t* data, uint32_t len) {
    crc8_hw_init_once();

    // Load initial value into CRC->DR via INIT + RESET.
    CRC->INIT = (uint32_t)state;
    CRC->CR |= CRC_CR_RESET;

    // Feed bytes in order to match the existing software CRC8 (table) behavior.
    volatile uint8_t* dr8 = (volatile uint8_t*)&CRC->DR;
    for (uint32_t i = 0; i < len; i++) {
        dr8[0] = data[i];
    }

    return (uint8_t)(CRC->DR & 0xFFu);
}
#endif

static inline uint8_t checksum8_update_chunk(uint8_t state, const uint8_t* data, uint32_t len) {
#if (LOGGER_CHECKSUM_ALGO == LOGGER_CHECKSUM_ALGO_SUM8)
    return sum8_update_chunk(state, data, len);
#elif (LOGGER_CHECKSUM_ALGO == LOGGER_CHECKSUM_ALGO_CRC8_HW)
    return crc8_hw_update_chunk(state, data, len);
#else
    return crc8_update_chunk(state, data, len);
#endif
}

/**
 * @brief Fast table-based CRC8 calculation (full buffer)
 *
 * Uses pre-computed lookup table to compute CRC8 in O(n) time with minimal cycles.
 * Polynomial: 0x07 (CRC-8-ATM / CRC-8-CCITT)
 *
 * Performance:
 *   - Payload length is 850 bytes (frame excluding checksum8+pad).
 *   - Table-based CRC8 is O(n) with one lookup per byte.
 *
 * @param data: Pointer to data buffer
 * @param len: Length in bytes
 * @return CRC8 checksum (8-bit)
 */
static uint8_t crc8_compute(const uint8_t* data, uint32_t len) {
    uint8_t crc = CRC8_INIT; /* CRC8 init value */

    for (uint32_t i = 0; i < len; i++) {
        crc = CRC8_TABLE[crc ^ data[i]];
    }

    return crc;
}

/**
 * @brief Full CRC8 calculation (single-pass, non-incremental)
 *
 * Computes CRC8 over entire buffer in one call without state machine.
 * Used for validation and testing to verify incremental method produces same result.
 *
 * Performance:
 *   - 1116 bytes @ 168 MHz: ~25 μs (same as incremental, but no context switches)
 *   - Useful for comparing results with incremental 4-part method
 *
 * @param data: Pointer to data buffer
 * @param len: Length in bytes
 * @return CRC8 checksum (8-bit)
 */
static uint8_t crc8_compute_full(const uint8_t* data, uint32_t len) {
    uint8_t crc = CRC8_INIT; /* CRC8 init value */

    for (uint32_t i = 0; i < len; i++) {
        crc = CRC8_TABLE[crc ^ data[i]];
    }

    return crc;
}

/**
 * @brief Validate checksum computation: compare full vs incremental methods
 *
 * Computes the selected 8-bit checksum using both methods:
 *   1. Full single-pass method
 *   2. Incremental 4-part state machine method (simulated)
 *
 * Useful for unit testing and verifying that both approaches produce identical results.
 *
 * @param frame: Pointer to LogFrame_t structure to validate
 * @return 1 if both methods match, 0 if mismatch (indicates bug)
 *
 * Usage:
 *   LogFrame_t my_frame = {...};
 *   if (!checksum8_validate_methods(&my_frame)) {
 *       // ERROR: checksum methods diverged!
 *   }
 */
static uint32_t checksum8_validate_methods(const LogFrame_t* frame) {
    if (!frame) {
        return 0;
    }

    const uint8_t* frame_data = (const uint8_t*)frame;
    uint32_t data_len = sizeof(LogFrame_t) - sizeof(uint16_t);  // Exclude checksum field (checksum8 + pad = 2 bytes)

    // ===== METHOD 1: Full single-pass =====
    uint8_t checksum_full = checksum8_update_chunk(0, frame_data, data_len);

    // ===== METHOD 2: Incremental 4-part (simulate what Logger_Task does) =====
    uint8_t checksum_incremental = 0;

    // Part 1: header + first quarter of ADC
    checksum_incremental = checksum8_update_chunk(checksum_incremental, &frame_data[CRC_PART1_OFFSET], CRC_PART1_SIZE);

    // Part 2: second quarter of ADC
    checksum_incremental = checksum8_update_chunk(checksum_incremental, &frame_data[CRC_PART2_OFFSET], CRC_PART2_SIZE);

    // Part 3: third quarter of ADC + first quarter IMU
    checksum_incremental = checksum8_update_chunk(checksum_incremental, &frame_data[CRC_PART3_OFFSET], CRC_PART3_SIZE);

    // Part 4: fourth quarter ADC + remaining IMU
    uint32_t part4_size = data_len - CRC_PART4_OFFSET;
    checksum_incremental = checksum8_update_chunk(checksum_incremental, &frame_data[CRC_PART4_OFFSET], part4_size);

    // ===== COMPARE RESULTS =====
    if (checksum_full != checksum_incremental) {
        // Mismatch indicates a bug in either method.
        return 0;  // Validation FAILED
    } else {
        Test2Toggle();
        Test2Toggle();
    }

    return 1;  // Validation PASSED - both methods identical
}

/**
 * @brief Initialize CRC state machine for frame at given offset
 *
 * Sets up CRC_STATE to compute CRC over frame in 4 parts:
 *   Part 1: header + 1st quarter ADC (0..133 bytes)
 *   Part 2: 2nd quarter ADC (134..261 bytes)
 *   Part 3: 3rd quarter ADC + 1st quarter IMU (262..419 bytes)
 *   Part 4: remainder of payload (from CRC_PART4_OFFSET to end, excl. CRC field)
 *
 * @param crc_state: Pointer to CrcState_t to initialize
 * @param state_step: Which step (0=part1, 1=part2, 2=part3, 3=part4)
 */
static void crc_state_init_step(CrcState_t* crc_state, uint32_t state_step) {
    switch (state_step) {
        case 0: /* Part 1: header + 1st quarter ADC */
            crc_state->crc_offset = CRC_PART1_OFFSET;
            crc_state->crc_chunk_size = CRC_PART1_SIZE;
            break;
        case 1: /* Part 2: 2nd quarter ADC */
            crc_state->crc_offset = CRC_PART2_OFFSET;
            crc_state->crc_chunk_size = CRC_PART2_SIZE;
            break;
        case 2: /* Part 3: 3rd quarter ADC + 1st quarter IMU */
            crc_state->crc_offset = CRC_PART3_OFFSET;
            crc_state->crc_chunk_size = CRC_PART3_SIZE;
            break;
        case 3: /* Part 4: 4th quarter ADC + remaining IMU */
            crc_state->crc_offset = CRC_PART4_OFFSET;
            crc_state->crc_chunk_size = 0;
            break;
    }
}

/**
 * @brief Process one step of incremental CRC computation
 *
 * Computes CRC8 over a chunk of the frame, advancing the state machine.
 * Call repeatedly from Logger_Task() until all chunks are processed.
 *
 * @param builder: Pointer to FrameBuilder_t
 * @return 1 if CRC computation complete (all 4 parts done), 0 if still processing
 */
static uint32_t crc_state_process_step(FrameBuilder_t* builder) {
    if (!builder->current_frame) {
        return 0;
    }

    LogFrame_t* frame = builder->current_frame;
    uint8_t* frame_data = (uint8_t*)frame;
    uint32_t data_len = sizeof(LogFrame_t) - sizeof(uint16_t);  // Exclude checksum field (checksum8 + pad = 2 bytes)

    // Ensure we don't process past the end
    if (builder->crc_state.crc_offset + builder->crc_state.crc_chunk_size > data_len) {
        builder->crc_state.crc_chunk_size = data_len - builder->crc_state.crc_offset;
    }

    // Process this chunk
    uint8_t* chunk_ptr = &frame_data[builder->crc_state.crc_offset];
    builder->crc_state.crc_current = checksum8_update_chunk(builder->crc_state.crc_current, chunk_ptr, builder->crc_state.crc_chunk_size);

    return 1;  // Step complete
}
void Logger_FrameBuilder_Init(void) {
    memset(&loggerStat.frame_ctx, 0, sizeof(loggerStat.frame_ctx));

#if (LOGGER_PROFILING_ENABLE == 1u)
    // Read comment under method
    logger_profile_dwt_enable_once();
#endif

    /* Initialize logger ADC buffer (Phase 2) */
    memset(&g_adc_buffer, 0, sizeof(AdcBuffer_t));

    /* Initialize logger IMU ring buffer (Phase 3) */
    memset(&g_imu_buffer, 0, sizeof(ImuBuffer_t));
    memset(g_last_imu_raw, 0, sizeof(g_last_imu_raw));
    g_last_imu_ts = 0;
    g_have_last_imu = 0u;

    /* Initialize MAVLink fields */
    loggerStat.frame_ctx.builder.mavlink_events = 0;
    loggerStat.frame_ctx.builder.mavlink_speed = 0;
    loggerStat.frame_ctx.builder.mavlink_altitude = 0;

    loggerStat.frame_ctx.builder.state = FRAME_STATE_IDLE;
}

/**
 * @brief Initialize SPI slave protocol and logger configuration
 *
 * Called once from App_InitRun() to set up SPI slave communication:
 *   - Sets configuration (magic=LOGGER_CONFIG_MAGIC, version=1, ADC params)
 *   - Initializes SPI2 peripheral for interrupt-driven command reception
 *   - Starts listening for master (ESP32) command byte (CMD 42)
 *   - Initializes GPIO_READY signal (low = no data available)
 *   - Sets SPI state machine to IDLE
 *
 * Returns: void
 */
void Logger_Init(void) {
    // Initialize logger configuration
    loggerStat.config.magic = LOGGER_CONFIG_MAGIC;                  // 0xCAFE
    loggerStat.config.version_major = LOGGER_CONFIG_VERSION_MAJOR;  // 0
    loggerStat.config.version_minor = LOGGER_CONFIG_VERSION_MINOR;  // 1
    loggerStat.config.adc_sample_rate_khz = ADC_SAMPLING_FREQ_KHZ;  // 100 kHz sampling
    loggerStat.config.adc_block_size = LOGGER_ADC_BLOCK_SIZE;       // 256 samples per frame
    loggerStat.config.mavlink_logging_enabled = 1;                  // MAVLink logging enabled
    loggerStat.config.reserved[0] = (uint8_t)LOGGER_CHECKSUM_ALGO;  // checksum algorithm id (CRC8/SUM8)

#if (SPI_LOGGER_ENABLE == 1u)
    // TODO: when new sensorr  will be added make abstraction for it  and read settings from sensor object abstraction
    //  Populate IMU config early so the master can read a meaningful config immediately after reset.
    //  The sensor driver will overwrite this later with the actual hardware-readback values.
    imu_config_t imu_cfg = {0};
    imu_cfg.accel_odr_hz = (uint16_t)LSM6DS3_SAMPLING_FREQ_HZ;
    imu_cfg.gyro_odr_hz = (uint16_t)LSM6DS3_SAMPLING_FREQ_HZ;
    imu_cfg.accel_present = (imu_cfg.accel_odr_hz > 0) ? 1u : 0u;
    imu_cfg.gyro_present = (imu_cfg.gyro_odr_hz > 0) ? 1u : 0u;
    imu_cfg.chip_id = LSM6DS3_ID;
    imu_cfg.accel_range_g = 16u;
    imu_cfg.reserved_align = 0u;
    imu_cfg.gyro_range_dps = 2000u;
    imu_cfg.reserved0 = LSM6DS3_CTRL1_XL_HIT_VAL;
    imu_cfg.reserved1 = LSM6DS3_CTRL2_G_HIT_VAL;
    imu_cfg.reserved2 = LSM6DS3_CTRL3_C_VAL;
    Logger_OnAccelerometerReady(&imu_cfg);

    // Initialize SPI2 NVIC for interrupt-driven reception
    Logger_SPI_Init();

    // Start listening for incoming command bytes from ESP32 master
    // This arms the SPI slave RX interrupt to receive the first command
    Logger_SPI_StartListening();

    // Initialize GPIO to ready=false (no data available initially)
    Logger_GPIO_SetReady(false);
#endif
    // Initialize SPI state machine
    loggerStat.frame_ctx.spi_state = SPI_STATE_IDLE;
    s_logging_started = 0;

    g_logger_adc_overruns = 0;
    g_logger_frames_built = 0;
    g_logger_frames_dropped = 0;
}

/**
 * @brief Drain one frame from queue and initiate SPI DMA transmission
 *
 * High-level queue draining function that manages post-config frame streaming:
 *   1. Checks config_sent flag (set by Logger_SPI_RxCallback after config TX complete)
 *   2. If config_sent=1: Marks transition complete (config phase complete)
 *   3. Dequeues next frame from circular FIFO queue
 *   4. Copies frame to TX buffer (loggerStat.tx_frame)
 *   5. Initiates DMA transmission via Logger_SPI_Transmit()
 *   6. Raises GPIO_READY signal to indicate data available on MISO
 *
 * Note: DEBUG code fills first 50 bytes with pattern (1..50) for testing
 *
 * Returns: 1 if frame sent (or post-config transition), 0 if queue empty
 *
 * Called from:
 *   - Logger_Task() after new frame added to queue
 *   - Logger_Task() on subsequent invocations to continuously drain queue
 *   - Logger_SPI_TxCallback() indirectly (via Logger_Task state check)
 */
int Logger_DrainQueue(void) {
    // If we just finished sending config, reset flag (actual start handled by Logger_Task via s_start_pending)
    if (loggerStat.config_sent == 1) {
        loggerStat.config_sent = 0;  // Reset for next cycle
    }

    // Try to dequeue next frame
    if (!Logger_GetNextFrame(&loggerStat.tx_frame)) {
        // Queue is empty, no frame to send
        return 0;
    }

#if (SPI_LOGGER_ENABLE == 1u)
    // Frame available - signal and transmit
    Logger_SPI_Transmit((uint8_t*)&loggerStat.tx_frame, sizeof(LogFrame_t));
    Logger_GPIO_SetReady(true);
#endif
    return 1;  // Frame sent
}

/**
 * @brief Main periodic task: Assemble frames and drain SPI transmission queue
 *
 * Called from main loop (optimal: every ~2.56ms for 256 samples @ 100kHz):
 *   - Monitors ADC buffer ready flag
 *   - Collects 256 ADC samples when ready
 *   - Retrieves available IMU samples (0-50) from ring buffer
 *   - Assembles LogFrame_t with CRC8
 *   - Adds to queue (circular FIFO)
 *   - Auto-drains queued frames to SPI DMA when idle
 *
 * State Machine (Frame Builder):
 *   FRAME_STATE_IDLE:  Wait for ADC ready, collect data, transition to BUILD
 *   FRAME_STATE_BUILD: Assemble frame, queue, attempt drain, return to IDLE
 *
 * SPI Integration:
 *   - Auto-drains to SPI DMA when frames available and SPI idle
 *   - Continuous streaming: frames transmit back-to-back
 *   - Counts dropped frames if queue full
 *
 * Returns: Number of frames queued (0 or 1)
 */
uint32_t Logger_Task(void) {
    uint32_t frames_queued = 0;

#if (LOGGER_PROFILING_ENABLE == 1u)
    logger_profile_dwt_enable_once();
    uint32_t task_start = logger_profile_now_cycles();
#endif

    // Start acquisition once, after the config transfer completed.
    // The config TX completion is observed in Logger_DrainQueue() via loggerStat.config_sent.
    if (s_start_pending != 0u && s_logging_started == 0u) {
        s_start_pending = 0u;
        s_logging_started = 1u;
        Solution_LoggingStart();
    }

    // Check if previous SPI transmission completed and queue needs draining
    // This allows continuous streaming without waiting for ADC block
    if (loggerStat.frame_ctx.spi_state == SPI_STATE_PACKET_SENT_COMPLETE) {
        loggerStat.frame_ctx.spi_state = SPI_STATE_IDLE;
        // Try to drain next frame from queue
        if (Logger_DrainQueue()) {
            loggerStat.frame_ctx.spi_state = SPI_STATE_PACKET_SENT;
        }
    }

    switch (loggerStat.frame_ctx.builder.state) {
        case FRAME_STATE_IDLE: {
            // Check if ADC buffer is ready (LOGGER_ADC_BLOCK_SIZE samples accumulated)
            if (g_adc_buffer.ready) {
                // Get pointer to ADC buffer
                uint32_t adc_count = 0;
                const int16_t* adc_data = Logger_AdcBuffer_GetBuffer(&adc_count);

                if (adc_data && adc_count == LOGGER_ADC_BLOCK_SIZE) {
                    // Test2Toggle();  // T2 DEBUG: Toggle on each ADC block ready (should be ~390 Hz)
                    //  Copy ADC data to builder
#if (LOGGER_PROFILING_ENABLE == 1u)
                    uint32_t t0 = logger_profile_now_cycles();
#endif
                    memcpy(loggerStat.frame_ctx.builder.adc, adc_data, LOGGER_ADC_BLOCK_SIZE * sizeof(int16_t));
#if (LOGGER_PROFILING_ENABLE == 1u)
                    logger_profile_update_counter(&g_logger_profile.adc_builder_copy, logger_profile_now_cycles() - t0);
#endif
                    loggerStat.frame_ctx.builder.adc_count = LOGGER_ADC_BLOCK_SIZE;
                    loggerStat.frame_ctx.builder.adc_ts_first = Logger_AdcBuffer_GetFirstTimestamp();

                    // Clear ADC buffer for next block
                    Logger_AdcBuffer_ClearReady();

                    // Pop IMU samples from ring buffer (thread-safe, handles clamping)
#if (LOGGER_PROFILING_ENABLE == 1u)
                    t0 = logger_profile_now_cycles();
#endif
                    uint32_t imu_count = Logger_ImuRing_PopSamples(loggerStat.frame_ctx.builder.imu, LOGGER_IMU_BLOCK_SIZE);
#if (LOGGER_PROFILING_ENABLE == 1u)
                    logger_profile_update_counter(&g_logger_profile.imu_pop, logger_profile_now_cycles() - t0);
#endif
                    loggerStat.frame_ctx.builder.imu_count = imu_count;

                    // Now assemble and queue the frame
                    loggerStat.frame_ctx.builder.state = FRAME_STATE_BUILD;
                }
            }

        } break;

        case FRAME_STATE_BUILD: {
            // Check if frame queue has space
            // Pending frames in queue (do NOT use modulo here; it breaks full/empty detection when the
            // difference becomes a multiple of LOGGER_FRAME_QUEUE_SIZE).
            uint32_t queue_size = (uint32_t)(loggerStat.frame_ctx.queue_write_idx - loggerStat.frame_ctx.queue_read_idx);

            if (queue_size < (LOGGER_FRAME_QUEUE_SIZE - 1)) {
                // Get pointer to next frame slot in queue
                uint32_t write_idx = loggerStat.frame_ctx.queue_write_idx % LOGGER_FRAME_QUEUE_SIZE;
                LogFrame_t* frame = &loggerStat.frame_ctx.frame_queue[write_idx];

                // Assemble frame header
                frame->magic = LOGGER_FRAME_MAGIC;
                frame->n_imu = loggerStat.frame_ctx.builder.imu_count;
                frame->adc_timestamp = loggerStat.frame_ctx.builder.adc_ts_first;

                // Copy ADC block (LOGGER_ADC_BLOCK_SIZE samples, fixed)
#if (LOGGER_PROFILING_ENABLE == 1u)
                uint32_t t0 = logger_profile_now_cycles();
#endif
                memcpy(frame->adc, loggerStat.frame_ctx.builder.adc, LOGGER_ADC_BLOCK_SIZE * sizeof(int16_t));
#if (LOGGER_PROFILING_ENABLE == 1u)
                logger_profile_update_counter(&g_logger_profile.adc_frame_copy, logger_profile_now_cycles() - t0);
#endif

                // Copy IMU block (only first n_imu samples are valid)
                if (loggerStat.frame_ctx.builder.imu_count > 0) {
#if (LOGGER_PROFILING_ENABLE == 1u)
                    t0 = logger_profile_now_cycles();
#endif
                    memcpy(frame->imu, loggerStat.frame_ctx.builder.imu, loggerStat.frame_ctx.builder.imu_count * sizeof(ImuRawSample_t));
#if (LOGGER_PROFILING_ENABLE == 1u)
                    logger_profile_update_counter(&g_logger_profile.imu_frame_copy, logger_profile_now_cycles() - t0);
#endif

                    // Zero out remaining IMU slots
                    if (loggerStat.frame_ctx.builder.imu_count < LOGGER_IMU_BLOCK_SIZE) {
#if (LOGGER_PROFILING_ENABLE == 1u)
                        t0 = logger_profile_now_cycles();
#endif
                        memset(&frame->imu[loggerStat.frame_ctx.builder.imu_count],
                            0,
                            (LOGGER_IMU_BLOCK_SIZE - loggerStat.frame_ctx.builder.imu_count) * sizeof(ImuRawSample_t));
#if (LOGGER_PROFILING_ENABLE == 1u)
                        logger_profile_update_counter(&g_logger_profile.imu_frame_zero, logger_profile_now_cycles() - t0);
#endif
                    }
                } else {
                    // No IMU data, zero entire IMU block
#if (LOGGER_PROFILING_ENABLE == 1u)
                    t0 = logger_profile_now_cycles();
#endif
                    memset(frame->imu, 0, LOGGER_IMU_BLOCK_SIZE * sizeof(ImuRawSample_t));
#if (LOGGER_PROFILING_ENABLE == 1u)
                    logger_profile_update_counter(&g_logger_profile.imu_frame_zero, logger_profile_now_cycles() - t0);
#endif
                }

                // Copy MAVLink event data
                frame->mavlink_log.event_flags = loggerStat.frame_ctx.builder.mavlink_events;
                frame->mavlink_log.speed_ms = loggerStat.frame_ctx.builder.mavlink_speed;
                frame->mavlink_log.altitude_m = loggerStat.frame_ctx.builder.mavlink_altitude;

                if (loggerStat.frame_ctx.builder.mavlink_events != 0) {
                    Test1Toggle();
                    Test1Toggle();
                }

                // Reset MAVLink fields in builder for next frame
                loggerStat.frame_ctx.builder.mavlink_events = 0;
                loggerStat.frame_ctx.builder.mavlink_speed = 0;
                loggerStat.frame_ctx.builder.mavlink_altitude = 0;

                // Initialize CRC state machine (Part 1 of 4)
                loggerStat.frame_ctx.builder.current_frame = frame;
                loggerStat.frame_ctx.builder.crc_state.crc_current = 0x00;
                crc_state_init_step(&loggerStat.frame_ctx.builder.crc_state, 0);

                // Transition to CRC Part 1 processing
                loggerStat.frame_ctx.builder.state = FRAME_STATE_CRC_PART1;
            } else {
                loggerStat.frame_ctx.stats_frames_dropped++;
                g_logger_frames_dropped = loggerStat.frame_ctx.stats_frames_dropped;
                memset(&loggerStat.frame_ctx.builder, 0, sizeof(FrameBuilder_t));
                loggerStat.frame_ctx.builder.state = FRAME_STATE_IDLE;
            }
        } break;

        case FRAME_STATE_CRC_PART1: {
            // Process CRC Part 1 (~134 bytes)
#if (LOGGER_PROFILING_ENABLE == 1u)
            uint32_t t0 = logger_profile_now_cycles();
#endif
            crc_state_process_step(&loggerStat.frame_ctx.builder);
#if (LOGGER_PROFILING_ENABLE == 1u)
            logger_profile_update_counter(&g_logger_profile.crc_part1, logger_profile_now_cycles() - t0);
#endif
            loggerStat.frame_ctx.builder.state = FRAME_STATE_CRC_PART2;
        } break;

        case FRAME_STATE_CRC_PART2: {
            // Process CRC Part 2 (~128 bytes)
            crc_state_init_step(&loggerStat.frame_ctx.builder.crc_state, 1);
#if (LOGGER_PROFILING_ENABLE == 1u)
            uint32_t t0 = logger_profile_now_cycles();
#endif
            crc_state_process_step(&loggerStat.frame_ctx.builder);
#if (LOGGER_PROFILING_ENABLE == 1u)
            logger_profile_update_counter(&g_logger_profile.crc_part2, logger_profile_now_cycles() - t0);
#endif
            loggerStat.frame_ctx.builder.state = FRAME_STATE_CRC_PART3;
        } break;

        case FRAME_STATE_CRC_PART3: {
            // Process CRC Part 3 (~128 bytes)
            crc_state_init_step(&loggerStat.frame_ctx.builder.crc_state, 2);
#if (LOGGER_PROFILING_ENABLE == 1u)
            uint32_t t0 = logger_profile_now_cycles();
#endif
            crc_state_process_step(&loggerStat.frame_ctx.builder);
#if (LOGGER_PROFILING_ENABLE == 1u)
            logger_profile_update_counter(&g_logger_profile.crc_part3, logger_profile_now_cycles() - t0);
#endif
            loggerStat.frame_ctx.builder.state = FRAME_STATE_CRC_PART4;
        } break;

        case FRAME_STATE_CRC_PART4: {
            // Process CRC Part 4 (remaining ~248 bytes) and finalize
            crc_state_init_step(&loggerStat.frame_ctx.builder.crc_state, 3);
            uint32_t data_len = sizeof(LogFrame_t) - sizeof(uint16_t);
            uint32_t chunk_start = loggerStat.frame_ctx.builder.crc_state.crc_offset;
            uint32_t chunk_size = data_len - chunk_start;  // Remaining bytes

            LogFrame_t* frame = loggerStat.frame_ctx.builder.current_frame;
            uint8_t* frame_data = (uint8_t*)frame;
            uint8_t* chunk_ptr = &frame_data[chunk_start];

#if (LOGGER_PROFILING_ENABLE == 1u)
            uint32_t t0 = logger_profile_now_cycles();
#endif
            loggerStat.frame_ctx.builder.crc_state.crc_current =
                checksum8_update_chunk(loggerStat.frame_ctx.builder.crc_state.crc_current, chunk_ptr, chunk_size);
#if (LOGGER_PROFILING_ENABLE == 1u)
            logger_profile_update_counter(&g_logger_profile.crc_part4, logger_profile_now_cycles() - t0);
#endif

            // CRC computation complete - write result to frame
            frame->checksum8 = loggerStat.frame_ctx.builder.crc_state.crc_current;
            frame->checksum_pad = 0;

            // ===== VALIDATION: Compare full vs incremental checksum methods =====
#if (LOGGER_CRC_VALIDATION_ENABLE == 1u)
            if (!checksum8_validate_methods(frame)) {
                // Mismatch indicates a bug in either method.
                loggerStat.frame_ctx.stats_crc_mismatches++;
            }
#endif

            loggerStat.frame_ctx.queue_write_idx++;
            loggerStat.frame_ctx.stats_frames_built++;
            g_logger_frames_built = loggerStat.frame_ctx.stats_frames_built;

            // Drain queue: try to start transmitting frames via DMA
            // Only drain if SPI is currently IDLE (no transmission in progress)
            if (loggerStat.frame_ctx.spi_state == SPI_STATE_IDLE && Logger_DrainQueue()) {
                loggerStat.frame_ctx.spi_state = SPI_STATE_PACKET_SENT;
            }

            frames_queued = 1;

            // Reset builder for next frame
#if (LOGGER_PROFILING_ENABLE == 1u)
            t0 = logger_profile_now_cycles();
#endif
            memset(&loggerStat.frame_ctx.builder, 0, sizeof(FrameBuilder_t));
#if (LOGGER_PROFILING_ENABLE == 1u)
            logger_profile_update_counter(&g_logger_profile.builder_reset, logger_profile_now_cycles() - t0);
#endif
            loggerStat.frame_ctx.builder.state = FRAME_STATE_IDLE;
        } break;

        default:
            loggerStat.frame_ctx.builder.state = FRAME_STATE_IDLE;
            break;
    }

#if (LOGGER_PROFILING_ENABLE == 1u)
    logger_profile_update_counter(&g_logger_profile.task_total, logger_profile_now_cycles() - task_start);
#endif

    return frames_queued;
}

/**
 * @brief Get next complete frame from queue (Phase 5 SPI transmission)
 *
 * @param out_frame: Pointer to LogFrame_t to receive frame data
 *
 * Returns: 1 if frame available (copied to out_frame), 0 if queue empty
 *
 * Usage:
 *   LogFrame_t frame;
 *   if (Logger_GetNextFrame(&frame)) {
 *       // Transmit frame via SPI or process further
 *       // frame.magic == 0x5A5A
 *       // frame.adc[LOGGER_ADC_BLOCK_SIZE] has ADC samples
 *       // frame.imu[frame.n_imu] has IMU samples
 *       // frame.checksum8 has checksum value
 *   }
 */
int Logger_GetNextFrame(LogFrame_t* out_frame) {
    if (!out_frame) {
        return 0;
    }

    uint32_t queue_size = (uint32_t)(loggerStat.frame_ctx.queue_write_idx - loggerStat.frame_ctx.queue_read_idx);

    if (queue_size == 0) {
        return 0;  // Queue empty
    }

    uint32_t read_idx = loggerStat.frame_ctx.queue_read_idx % LOGGER_FRAME_QUEUE_SIZE;
    LogFrame_t* frame = &loggerStat.frame_ctx.frame_queue[read_idx];

    // Copy entire frame (LOGGER_FRAME_SIZE_BYTES bytes)
#if (LOGGER_PROFILING_ENABLE == 1u)
    uint32_t t0 = logger_profile_now_cycles();
#endif
    memcpy(out_frame, frame, LOGGER_FRAME_SIZE_BYTES);
#if (LOGGER_PROFILING_ENABLE == 1u)
    logger_profile_update_counter(&g_logger_profile.frame_dequeue_copy, logger_profile_now_cycles() - t0);
#endif
    loggerStat.frame_ctx.queue_read_idx++;

    return 1;  // Frame retrieved
}

/* Pending count via direct access: (queue_write_idx - queue_read_idx) % LOGGER_FRAME_QUEUE_SIZE */

/**
 * @brief SPI2 TX Complete callback - Frame/Config transmission finished
 *
 * Called when DMA transmission of data (config or frame) to master (ESP32) is complete.
 * Manages GPIO signaling and state machine transitions for continuous frame streaming.
 *
 * Sequence:
 *   1. Lower GPIO_READY to signal transmission complete (master stops reading MISO)
 *   2. Clear any SPI error flags (overflow, frame error) to prevent lockup
 *   3. Set state machine to PACKET_SENT_COMPLETE to trigger next drain cycle
 *
 * Actual queue draining happens in Logger_Task() when it detects PACKET_SENT_COMPLETE state:
 *   - Logger_Task() clears PACKET_SENT_COMPLETE and transitions to IDLE
 *   - Calls Logger_DrainQueue() to start next DMA transmission
 *   - Continuous streaming: frames transmit back-to-back as queue populates\n *
 * @note Called from SPI2 DMA interrupt context; keep execution time minimal
 */
void Logger_SPI_TxCallback(void) {
#if (SPI_LOGGER_ENABLE == 1u)
    // Signal ESP32 that DMA transmission finished by lowering GPIO
    Logger_GPIO_SetReady(false);
#endif

    // Clear any SPI error flags to prevent lockup
    __HAL_SPI_CLEAR_OVRFLAG(&LOGGER_SPI_HANDLE);
    __HAL_SPI_CLEAR_FREFLAG(&LOGGER_SPI_HANDLE);

    // Signal that transmission is complete
    // Logger_Task() will check this state and drain queue on next invocation
    loggerStat.frame_ctx.spi_state = SPI_STATE_PACKET_SENT_COMPLETE;

    // Config has been transmitted; start acquisition from task context.
    // (Solution_LoggingStart() is not ISR-safe.)
    if (loggerStat.config_sent == 1u) {
        s_start_pending = 1u;
    }

    // We only support SPI commands before streaming starts (CONFIG).
    // After acquisition begins, the master clocks raw frame reads and we must not arm RX for "command bytes",
    // otherwise the dummy MOSI bytes would be misinterpreted as commands.
    // Also avoid re-arming RX after config TX completes (s_start_pending set) to prevent BUSY_RX blocking TX DMA.
    if ((s_logging_started == 0u) && (s_start_pending == 0u)) {
        Logger_SPI_StartListening();
    }
}

/**
 * @brief SPI Slave RX Callback - Handle command from ESP32 master
 *
 * Called from HAL_SPI_RxCpltCallback in solution_wrapper.c when master
 * sends a command byte via dedicated CMD channel (CMD 42 only supported).
 *
 * SPI Protocol (Auto-Streaming):
 *   - Master initiates with CMD 42 (0x2A) at startup to read config
 *   - Slave responds by transmitting logger_config_t (64 bytes) via DMA
 *   - After config TX completes: STM32 starts acquisition and Logger_Task() streams queued frames
 *   - No manual READ commands needed - frames auto-send when available
 *
 * CMD 42 (LOGGER_SPI_CMD_CONFIG) Protocol Flow:
 *   1. Master sends command byte 42 on CMD channel
 *   2. RxCallback extracts command and copies logger_config_t to TX buffer
 *   3. Sets config_sent = 1 to mark one-time config phase
 *   4. Initiates DMA transmission of config data
 *   5. Raises GPIO_READY to signal data available
 *   6. After DMA completes: Logger_SPI_TxCallback() lowers GPIO
 *
 * Unknown Commands: If command != LOGGER_SPI_CMD_CONFIG, GPIO is lowered and no transmission occurs
 *
 * @note Called from SPI2 interrupt context; keep execution time minimal
 * @note Non-blocking; all transmission happens via DMA with callbacks
 * @param rx_buffer: Pointer to received command buffer (at least 5 bytes)
 */
void Logger_SPI_RxCallback(const uint8_t* rx_buffer) {
    if (!rx_buffer) {
        return;
    }

#if (SPI_LOGGER_ENABLE == 1u)
    uint8_t command = rx_buffer[0];  // Extract command byte

    if (command == LOGGER_SPI_CMD_CONFIG) {
        // ===== CONFIG COMMAND (0x2A) ====="
        // Single startup command: send logger configuration to master

        // Copy entire loggerStat.config to TX buffer (32 bytes)
        memcpy((uint8_t*)&loggerStat.tx_frame, (uint8_t*)&loggerStat.config, sizeof(logger_config_t));

        // Mark that config is being sent (one-time transition marker)
        // Next drain cycle will skip, then transition to frame mode
        loggerStat.config_sent = 1;

        // Test1Toggle();
        // Test1Toggle();

        // Start DMA transmission of config and mark state
        Logger_SPI_Transmit((uint8_t*)&loggerStat.tx_frame, sizeof(logger_config_t));

        loggerStat.frame_ctx.spi_state = SPI_STATE_PACKET_SENT;

        // Signal master that config data is ready on MISO
        Logger_GPIO_SetReady(true);

    } else {
        // ===== INVALID COMMAND =====
        // Unknown command: clear GPIO and skip DMA
        Logger_GPIO_SetReady(false);
    }
#endif
}

#if (COMP_HIT_DETECTION_ENABLE == 1u)
uint8_t Logger_PiezoCompCbk(system_evt_t evt, uint32_t usr_data, void* usr_ptr) {
    (void)usr_ptr; /* Unused parameter */

    /* Ignore non-ready events */
    if (evt != SYSTEM_EVT_READY) {
        return 0u;
    }

    /* TODO OSAV ADD piezo handling here */
    /* usr_data contains the threshold value in mV */

    return 0u;
}
#endif /* COMP_HIT_DETECTION_ENABLE */

/**
 * @brief MAVLink event callback for logger (replaces App_MavlinkCbk in logging mode)
 *
 * Captures all MAVLink events and telemetry data for inclusion in log frames.
 * Accumulates event flags and data during the current frame window.
 * Data is reset after Logger_Task() copies it to the assembled frame.
 *
 * Event Processing:
 *   - All events: Set corresponding bit in mavlink_events bitmask (bit N for event 0x0N)
 *   - MAVLINK_EVT_SPEED_RECEIVED (0x0A): Copy uint16_t groundspeed from usr_ptr
 *   - MAVLINK_EVT_ALTITUDE_RECEIVED (0x0B): Copy int32_t altitude from usr_ptr
 *
 * Integration:
 *   - Called from MAVLink UART handler when autopilot events occur
 *   - Registered via Mavlink_Init(Logger_MavlinkCbk, ...) in App_InitRun
 *   - Data merged into LogFrame_t.mavlink_log during FRAME_STATE_BUILD
 *
 * @param evt: System event type (SYSTEM_EVT_READY, SYSTEM_EVT_ERROR, etc.)
 * @param usr_data: MAVLink event type (mavlink_event_t enum cast to uint32_t)
 * @param usr_ptr: Pointer to event-specific data (uint16_t* for speed, int32_t* for altitude)
 *
 * Returns: 0 (standard callback return)
 */
uint8_t Logger_MavlinkCbk(system_evt_t evt, uint32_t usr_data, void* usr_ptr) {
    // Ignore non-MAVLink events
    if (evt != SYSTEM_EVT_READY) {
        return 0;
    }

    // Set event flag bit (bit N = event 0x0N)
    // usr_data contains mavlink_event_t enum value (0x00-0x0F)
    if (usr_data < 32) {  // Safety check: only set bits 0-31
        loggerStat.frame_ctx.builder.mavlink_events |= (1U << usr_data);
    }

    // Copy telemetry data for specific events
    switch ((mavlink_event_t)usr_data) {
        case MAVLINK_EVT_SPEED_RECEIVED:
            if (usr_ptr != NULL) {
                loggerStat.frame_ctx.builder.mavlink_speed = *(uint16_t*)usr_ptr;
            }
            break;

        case MAVLINK_EVT_ALTITUDE_RECEIVED:
            if (usr_ptr != NULL) {
                loggerStat.frame_ctx.builder.mavlink_altitude = *(int32_t*)usr_ptr;
            }
            break;

        default:
            // Other events: only flag is logged, no data copy needed
            break;
    }

    return 0;
}

/**
 * @brief Notify logger that accelerometer has been successfully initialized
 *
 * Called from LSM6DS3.c::Lsm6ds3_SetHitParams() when sensor configuration is complete.
 * Copies fully populated imu_config_t structure (with all values decoded from hardware
 * registers by the sensor driver) into logger configuration.
 *
 * The imu_config_t contains:
 *   - accel_present, gyro_present: Sensor availability flags
 *   - chip_id: LSM6DS3 device identifier
 *   - accel_odr_hz, gyro_odr_hz: Output data rates in Hz
 *   - accel_range_g, gyro_range_dps: Measurement ranges
 *   - reserved0-4: Register snapshots (CTRL1_XL, CTRL2_G, CTRL3_C, CTRL7_G, CTRL4_C)
 *   - reserved5-11: Future expansion
 *
 * After this function completes, Logger_Task() will include IMU config in frames
 * and the master (ESP32) can request config via CMD 42 to learn sensor parameters.
 *
 * @param imu_cfg: Pointer to imu_config_t with fully populated hardware register values
 *                 NULL if sensor initialization failed (function returns silently)
 */
void Logger_OnAccelerometerReady(const imu_config_t* imu_cfg) {
    if (imu_cfg == NULL) {
        return;  // Initialization failed
    }

    // Simply copy the entire populated structure to global config
    memcpy(&loggerStat.config.imu_config, imu_cfg, sizeof(imu_config_t));
}
