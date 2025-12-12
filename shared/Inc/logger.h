/**
 * @file logger.h
 * @brief Consolidated logger public API for impact data acquisition and SPI transmission
 *
 * Architecture:
 *   - Phase 1 (Timing):     Global timestamp synchronization via TIM3 slave
 *   - Phase 2 (ADC Ring):   Ring buffer capture for ADC2 100 kHz data
 *   - Phase 3 (IMU Time):   Timestamp correlation with LSM6DS3 samples
 *   - Phase 4 (Framing):    Frame builder with ADC + IMU payload
 *   - Phase 5 (SPI DMA):    DMA-driven SPI slave transmission pipeline
 *
 * Integration:
 *   1. Call Logger_Timing_Init() in Solution_HalInit() after TIM6 starts
 *   2. In LSM6DS3.c Lsm6ds3_GetDataCbk() callback (protected by SPI_LOGGER_ENABLE macro):
 *      Logger_ImuOnNewSample() with gyro/accel data and timestamp
 *   3. In main loop or timer callback: Logger_Task() processes queued samples
 *
 * Timing Model:
 *   TIM6 @ 100 kHz (ITR5) → TIM5 External Clock Mode 1
 *   Logger_GetTimestamp() returns TIM5->CNT (0..0xFFFFFFFF, increments every 10 µs)
 *   Rollover at ~42s (2^32 / 100kHz)
 */

#ifndef SHARED_INC_LOGGER_H_
#define SHARED_INC_LOGGER_H_

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "prj_config.h"

#ifdef __cplusplus
extern "C" {
#endif


#define LOGGER_FRAME_MAGIC              0x5A5A
#define LOGGER_FRAME_QUEUE_SIZE         10
#define LOGGER_IMU_BLOCK_SIZE           10
/* LOGGER_ADC_BLOCK_SIZE defined in prj_config.h - typically 256 samples */

/* CRC16 - Simple polynomial */
#define CRC16_POLY                      0xA001  // Reversed CRC16-CCITT
#define CRC16_INIT                      0xFFFF

/* SPI Protocol - Command definitions */
#define LOGGER_SPI_CMD_CONFIG           42      // Config command (0x2A) - retrieve logger_config_t
#define LOGGER_SPI_RX_COMMAND_SIZE      5       // RX buffer size: command byte + 4 padding bytes

#define LOGGER_CONFIG_MAGIC             0xCAFE
#define LOGGER_CONFIG_VERSION           1

/* ============================================================================
 * PHASE 1: TIMING FOUNDATION
 * ============================================================================
 */

/**
 * @brief Initialize timestamp system (deprecated, now a no-op)
 *
 * Timestamp initialization is now handled automatically in Solution_HalInit()
 * when TIM6 interrupt is set up. This function is retained for API compatibility.
 *
 * Returns: void
 */
void Logger_Timing_Init(void);

/**
 * @brief Get current global timestamp counter value
 *
 * Returns: uint32_t - Current 32-bit software counter
 *          Increments by 1 every 10 µs (100 kHz rate)
 *
 * Implementation: Software 32-bit counter incremented by TIM6 @ 100 kHz
 * Usage: uint32_t ts = Logger_GetTimestamp();  // Returns 32-bit counter
 */
uint32_t Logger_GetTimestamp(void);

/**
 * @brief Called from TIM6 interrupt handler to update timestamp counter
 *
 * Prerequisites:
 *   - Must be called from TIM6_DAC_IRQHandler in stm32g4xx_it.c
 *   - Increments g_timestamp by 1 (every 10 µs @ 100 kHz)
 *   - Maintains synchronization with ADC2 100 kHz sampling
 *
 * Returns: void
 */
void Logger_TIM6_UpdateCallback(void);

/**
 * @brief ADC sample structure (Phase 2)
 *
 * Represents a single ADC2 sample with synchronized timestamp.
 * Size: 6 bytes (uint16_t adc_value + uint32_t timestamp)
 */
typedef struct {
    uint16_t adc_value;    // 12-bit ADC reading (0-4095)
    uint32_t timestamp;    // TIM3->CNT @ 100 kHz (every 10 µs)
} AdcSample_t;

/* ============================================================================
 * PHASE 2: ADC LINEAR BUFFER (256-sample buffer + ready flag)
 * ============================================================================
 */

/**
 * @brief Initialize ADC2 linear buffer (256 samples, ready flag)
 *
 * Prerequisites:
 *   - TIM6 running @ 100 kHz
 *   - ADC2 DMA configured and running
 *
 * Effects:
 *   - Clears 256-sample buffer
 *   - Resets ready flag to 0
 *
 * Returns: void
 */
void Logger_AdcBuffer_Init(void);

/**
 * @brief Check if new ADC buffer is ready (256 samples accumulated)
 *
 * Returns: bool - true if ready flag set, false otherwise
 */
bool Logger_AdcBuffer_IsReady(void);

/**
 * @brief Get pointer to ready ADC buffer (256 samples, raw int16_t values)
 *
 * @param out_count: Pointer to uint32_t to receive sample count (always 256)
 *
 * Returns: Pointer to int16_t array (256 samples), or NULL if not ready
 *
 * Note: Buffer remains valid until next ADC_COMPLETE interrupt.
 *       Call after checking Logger_AdcBuffer_IsReady() == 1.
 */
const int16_t *Logger_AdcBuffer_GetBuffer(uint32_t *out_count);

/**
 * @brief Clear ready flag after processing current ADC buffer
 *
 * Call from main loop after Logger_Task() processes the buffer.
 * Next ADC_COMPLETE will set ready flag again.
 *
 * Returns: void
 */
void Logger_AdcBuffer_ClearReady(void);

/**
 * @brief DMA complete callback (called from HAL_ADC_ConvCpltCallback)
 *
 * Called when 256 samples accumulated in DMA circular buffer.
 * Sets ready flag and stores timestamp of first sample.
 *
 * Integration:
 *   In solution_wrapper.c HAL_ADC_ConvCpltCallback():
 *   if (hadc == &ADC_PIEZO_HANDLE) {
 *       extern uint16_t adc_dma_buffer[];
 *       Logger_AdcBuffer_OnComplete(adc_dma_buffer, 256);
 *   }
 *
 * @param dma_buffer: Pointer to DMA result buffer (256 samples)
 * @param count: Number of samples (256 expected)
 */
void Logger_AdcBuffer_OnComplete(const uint16_t *dma_buffer, uint32_t count);

/**
 * @brief Get reference timestamp for current ADC block (first sample)
 *
 * Returns: uint32_t - Timestamp of first sample in current buffer
 *          (100 kHz = 10 µs per tick)
 */
uint32_t Logger_AdcBuffer_GetFirstTimestamp(void);

/* ============================================================================
 * PHASE 3: IMU TIMESTAMPING (Placeholder - Phase 3 implementation)
 * ============================================================================
 */

/**
 * @brief Raw IMU sample (12 bytes from SPI buffer)
 *
 * Data format from LSM6DS3 register read:
 *   Bytes 0-1:  OUTX_L_G, OUTX_H_G → gx (int16_t)
 *   Bytes 2-3:  OUTY_L_G, OUTY_H_G → gy (int16_t)
 *   Bytes 4-5:  OUTZ_L_G, OUTZ_H_G → gz (int16_t)
 *   Bytes 6-7:  OUTX_L_XL, OUTX_H_XL → ax (int16_t)
 *   Bytes 8-9:  OUTY_L_XL, OUTY_H_XL → ay (int16_t)
 *   Bytes 10-11: OUTZ_L_XL, OUTZ_H_XL → az (int16_t)
 */
typedef struct {
    uint8_t data[12];  // Raw 12 bytes from SPI (gx, gy, gz, ax, ay, az as int16_t each)
} ImuRawSample_t;

/**
 * @brief Initialize IMU linear buffer for sample capture
 *
 * Prerequisites:
 *   - Logger_Timing_Init() completed (TIM6 running @ 100 kHz)
 *   - Logger_RingBuffer_Init() completed
 *   - LSM6DS3 running and polling for data
 *
 * Effects:
 *   - Clears 50-sample IMU buffer
 *   - Resets sample counter
 *   - Resets statistics
 *
 * Returns: void
 */
void Logger_ImuRing_Init(void);

/**
 * @brief Callback invoked when LSM6DS3 sample is ready (from Lsm6ds3_GetDataCbk)
 *
 * @param raw_data: Pointer to 12 raw bytes from SPI read [gx_lo, gx_hi, gy_lo, gy_hi, gz_lo, gz_hi,
 *                                                           ax_lo, ax_hi, ay_lo, ay_hi, az_lo, az_hi]
 *
 * Effects:
 *   - Copies 12 bytes directly into IMU buffer (zero parsing overhead in ISR)
 *   - If buffer full (50 samples), discards oldest and keeps newest
 *   - Increments sample counter
 *
 * Returns: void
 *
 * Note: This is a weak function - boards that don't use logger disable it
 *       via SPI_LOGGER_ENABLE macro gate in LSM6DS3.c
 *
 * Usage in LSM6DS3.c Lsm6ds3_GetDataCbk():
 *   #if (SPI_LOGGER_ENABLE == 1u)
 *       Logger_ImuOnNewSample(&lsm6ds3_Stat.rd_buff[1]);  // Skip dummy byte, pass 12 raw bytes
 *   #endif
 */
void Logger_ImuOnNewSample(const uint8_t *raw_data);

/**
 * @brief Get current IMU buffer fill level
 *
 * Returns: uint32_t - Number of IMU samples currently in buffer (0..50)
 */
uint32_t Logger_ImuRing_GetCount(void);

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
const ImuRawSample_t *Logger_ImuRing_GetBuffer(uint32_t *out_count);

/**
 * @brief Clear IMU buffer after frame assembly
 *
 * Call from Logger_Task() after processing current IMU buffer.
 * Resets sample counter to 0 for next batch of samples.
 *
 * Returns: void
 */
void Logger_ImuRing_Clear(void);

/**
 * @brief Get total IMU samples ever received (debug/statistics)
 *
 * Returns: uint32_t - Total count
 */
uint32_t Logger_ImuRing_GetTotalSamples(void);

/**
 * @brief Get IMU overflow count (debug/statistics)
 *
 * Overflow occurs when new sample arrives with buffer full (50 samples).
 * Latest sample kept, oldest discarded.
 *
 * Returns: uint32_t - Number of samples discarded due to buffer full
 */
uint32_t Logger_ImuRing_GetOverflowCount(void);

/* ============================================================================
 * PHASE 4: FRAME BUILDING (Implemented - logger_frame.c)
 * ============================================================================
 */

/**
 * @brief Frame format structure (fixed-size frame for efficient SPI transfer)
 *
 * Layout (little-endian, fixed 640 bytes):
 *   Bytes 0-1:       MAGIC_FRAME (0x5A5A)
 *   Bytes 2-3:       Frame counter (uint16_t, increments per frame)
 *   Bytes 4-5:       n_imu (uint16_t, number of IMU samples actually filled)
 *   Bytes 6-517:     adc[256] (256 × int16_t = 512 bytes)
 *   Bytes 518-637:   imu[LOGGER_IMU_BLOCK_SIZE] (10 × ImuRawSample_t = 10 × 12 = 120 bytes)
 *   Bytes 638-639:   crc16 (uint16_t over entire frame)
 *
 * Size: Fixed 640 bytes
 *   - Magic (2) + Counter (2) + n_imu (2) = 6 bytes header
 *   - ADC block (256 × 2) = 512 bytes
 *   - IMU block (10 × 12) = 120 bytes
 *   - CRC16 (2) = 2 bytes
 *   - Total = 640 bytes (fixed, DMA-friendly SPI transfer)
 *
 * Note: n_imu field indicates how many of the LOGGER_IMU_BLOCK_SIZE (10) slots are filled.
 *       Unused IMU slots are zeroed. LOGGER_IMU_BLOCK_SIZE = 10 per Phase 3 configuration.
 *
 * Queue: 10 frames max (6.4 KB total)
 */
typedef struct __attribute__((packed)) {
    uint16_t magic;           // 0x5A5A
    uint16_t frame_counter;   // Increments per frame
    uint16_t n_imu;           // Number of valid IMU samples (0-50)
    int16_t adc[256];         // Fixed 256 ADC samples
    ImuRawSample_t imu[LOGGER_IMU_BLOCK_SIZE];   // Fixed 50 IMU raw samples (only first n_imu are valid)
    uint16_t crc16;           // CRC16 over entire frame
} LogFrame_t;

/* ============================================================================
 * IMU CONFIGURATION STRUCTURE (Phase 5 SPI - Embedded in logger_config_t)
 * ============================================================================
 *
 * Extended IMU configuration with actual hardware register snapshots.
 * Embedded as nested structure within logger_config_t (offset 8-31).
 *
 * Layout (24 bytes total):
 *   Offset 0-3:   Sensor presence and identification (accel_present, gyro_present, chip_id, pad)
 *   Offset 4-7:   Output Data Rates (accel_odr_hz, gyro_odr_hz)
 *   Offset 8-11:  Full-scale ranges (accel_range_g, alignment byte, gyro_range_dps as uint16_t)
 *   Offset 12-16: Control register snapshots (CTRL1_XL, CTRL2_G, CTRL3_C, CTRL7_G, CTRL4_C)
 *   Offset 17-23: Reserved for future expansion (7 bytes)
 */
typedef struct __attribute__((packed)) {
    // Offset 0-3: Presence and identification (4 bytes)
    uint8_t accel_present;          // 1 if accelerometer enabled (ODR > 0), 0 otherwise
    uint8_t gyro_present;           // 1 if gyroscope enabled (ODR > 0), 0 otherwise
    uint8_t chip_id;                // WHO_AM_I register value (0x69 for LSM6DS3)
    uint8_t reserved_pad;           // Padding for alignment

    // Offset 4-7: Output Data Rates (4 bytes)
    uint16_t accel_odr_hz;          // Decoded from CTRL1_XL[7:4] (0-6664 Hz)
    uint16_t gyro_odr_hz;           // Decoded from CTRL2_G[7:4] (0-6664 Hz)

    // Offset 8-11: Full-scale ranges (4 bytes)
    uint8_t accel_range_g;          // Decoded from CTRL1_XL[3:2]: 2, 4, 8, or 16 G
    uint8_t reserved_align;         // Alignment byte for uint16_t gyro_range_dps
    uint16_t gyro_range_dps;        // Decoded from CTRL2_G[3:2]: 245, 500, 1000, or 2000 DPS

    // Offset 12-16: Control register snapshots (5 bytes)
    uint8_t reserved0;              // CTRL1_XL (0x10) snapshot - Accel ODR + FS
    uint8_t reserved1;              // CTRL2_G (0x11) snapshot - Gyro ODR + FS
    uint8_t reserved2;              // CTRL3_C (0x12) snapshot - Common control (BDU, IF_INC)
    uint8_t reserved3;              // CTRL7_G (0x16) snapshot - Gyro HP mode
    uint8_t reserved4;              // CTRL4_C (0x13) snapshot - DRDY control

    // Offset 17-23: Reserved for future expansion (7 bytes)
    uint8_t reserved5;              // Future: Additional sensor parameters
    uint8_t reserved6;              // Future: LIS2DH12 or other IMU support
    uint8_t reserved7;              // Future: Expansion slot
    uint8_t reserved8;              // Future: Expansion slot
    uint8_t reserved9;              // Future: Expansion slot
    uint8_t reserved10;             // Future: Expansion slot
    uint8_t reserved11;             // Future: Expansion slot
} imu_config_t;

_Static_assert(sizeof(imu_config_t) == 24, "imu_config_t must be exactly 24 bytes");

/* ============================================================================
 * LOGGER CONFIGURATION (Phase 5 SPI Protocol - Command 0x2A)
 * ============================================================================
 *
 * Device configuration structure sent to ESP32 master via command 42 (0x2A).
 * Defines ADC and IMU parameters for proper frame interpretation.
 *
 * Size: Fixed 32 bytes (padded to 640-byte DMA frame with zeros)
 * Magic: 0xCAFE (for validation)
 * Version: 1 (extensible for future protocol versions)
 * Layout: 4B header + 4B ADC params + 24B IMU config
 */

typedef struct __attribute__((packed)) {
    uint16_t magic;                 // 0xCAFE — validity check (2B)
    uint16_t version;               // Protocol version (2B)

    // --- Core ADC parameters (4B) ---
    uint16_t adc_sample_rate_khz;    // Sampling frequency kHz (2B)
    uint16_t adc_block_size;        // Samples per frame (2B)

    // --- Extended IMU configuration (24B) ---
    imu_config_t imu_config;        // Nested IMU config structure
} logger_config_t;

_Static_assert(sizeof(logger_config_t) == 32, "logger_config_t must be exactly 32 bytes");

/* ============================================================================
 * FRAME BUILDER STATE MACHINE (Phase 4 internal types)
 * ============================================================================
 */

typedef enum {
    FRAME_STATE_IDLE,    // Waiting for ADC block
    FRAME_STATE_BUILD,   // Frame assembled, ready to queue
} FrameState_t;

/**
 * @brief SPI transmission state machine for queue draining
 * 
 * IDLE:                  No transmission in progress, can start new transfers
 * PACKET_SENT:           DMA transmission initiated, waiting for TxCallback
 * PACKET_SENT_COMPLETE:  TxCallback completed, Logger_Task() should drain queue
 */
typedef enum {
    SPI_STATE_IDLE,                 // Initial state, ready for new transfers
    SPI_STATE_PACKET_SENT,          // Transmission started, waiting for DMA complete
    SPI_STATE_PACKET_SENT_COMPLETE, // TxCallback fired, ready to drain queue
} SpiState_t;

typedef struct {
    int16_t adc[LOGGER_ADC_BLOCK_SIZE];        // Dequeued ADC samples
    ImuRawSample_t imu[LOGGER_IMU_BLOCK_SIZE]; // Dequeued IMU samples
    uint16_t adc_count;
    uint16_t imu_count;
    uint32_t adc_ts_first;                      // Timestamp of first ADC sample
    FrameState_t state;
} FrameBuilder_t;

/* ============================================================================
 * FRAME CONTEXT (Phase 4+5 unified queue and state)
 * ============================================================================
 */

typedef struct {
    FrameBuilder_t builder;                              // Active frame builder
    LogFrame_t frame_queue[LOGGER_FRAME_QUEUE_SIZE];     // 10 complete frames (Phase 4+5 unified queue)
    volatile uint32_t queue_write_idx;
    volatile uint32_t queue_read_idx;
    uint16_t frame_counter;
    uint32_t stats_frames_built;
    uint32_t stats_frames_dropped;
    volatile uint32_t spi_pending_count;                 // Track frames pending SPI transmission
    volatile SpiState_t spi_state;                       // SPI transmission state machine
} FrameContext_t;

/* ============================================================================
 * LOGGER STATUS STRUCTURE
 * ============================================================================
 */

/**
 * @brief Logger module status and state container
 *
 * Aggregates all logger internal state into a single structure.
 * Declared as static in logger_frame.c to keep module-private.
 *
 * Members:
 *   - frame_ctx: Frame builder queue, state machine, statistics
 *   - config: Configuration sent to master (magic, version, ADC params, IMU params)
 *   - tx_frame: SPI TX buffer for DMA transmission (frame or config data)
 *   - rx_cmd: Extracted command byte for backward compatibility
 *   - config_sent: Flag indicating config was just sent (one-time transition marker)
 *
 * Note: RX command buffer is now in solution_wrapper.c (static logger_spi_rx_cmd_buffer)
 *       for direct SPI reception without unnecessary indirection.
 */
typedef struct {
    FrameContext_t frame_ctx;                      // Frame builder state + queue
    logger_config_t config;                        // Configuration structure
    LogFrame_t tx_frame;                           // TX buffer for SPI DMA
    uint8_t rx_cmd;                                // Extracted command byte
    volatile uint8_t config_sent;                  // 1 = config just sent, skip one drain cycle
} logger_status_t;

/* ============================================================================
 * LOGGER STATUS ACCESS (module-private in logger_frame.c)
 * ============================================================================
 */

/* Direct access to loggerStat via getter function is no longer available */
/* All functionality provided through specific API functions */

/* ============================================================================
 * LEGACY EXTERN DECLARATIONS (for backward compatibility)
 * ============================================================================
 */

/* Global frame context – accessible via frame builder API */
// extern FrameContext_t g_frame_ctx;

/* Global logger configuration – accessible via logger config API */
// extern logger_config_t g_logger_config;

/* Global SPI TX buffer – for use by Logger_SPI_RxCallback in logger_frame.c */
// extern LogFrame_t g_logger_tx_frame;

/* Global SPI RX command buffer (5 bytes) – now static in solution_wrapper.c */
// static uint8_t logger_spi_rx_cmd_buffer[5];  // In: solution_wrapper.c (direct SPI reception)

/* Global SPI RX command byte extracted from buffer[0] – for backward compatibility */
// extern uint8_t g_logger_spi_rx_cmd;

/**
 * @brief Initialize frame builder (call once at startup)
 *
 * Prerequisites:
 *   - Logger_Timing_Init() completed
 *   - Logger_RingBuffer_Init() completed
 *   - Logger_ImuRing_Init() completed
 *
 * Effects:
 *   - Clears frame builder state machine
 *   - Initializes frame queue
 *   - Resets frame counter and statistics
 */
void Logger_FrameBuilder_Init(void);

/**
 * @brief Initialize frame builder and start logging pipeline
 *
 * Initializes all logging subsystems:
 *   - ADC linear buffer (256-sample, ready flag)
 *   - IMU ring buffer (0-50 samples)
 *   - Frame builder state machine
 *   - Starts ADC2 DMA acquisition and sampling timer
 *
 * Must be called after Logger_Timing_Init() and Solution_HalInit().
 * Triggers Solution_LoggingStart() to calibrate ADC and start DMA.
 *
 * Returns: void
 */
void Logger_FrameBuilder_Init(void);

/**
 * @brief Start ADC data acquisition and logging
 *
 * Calibrates ADC2 and starts DMA-driven 100 kHz sampling.
 * Called from Logger_FrameBuilder_Init().
 * Integration point: solution_wrapper.c (hardware abstraction)
 *
 * Returns: void (calls Error_Handler on failure)
 */
void Solution_LoggingStart(void);

/**
 * @brief Main logger task: Poll buffers and assemble frames
 *
 * Call from main loop periodically (every 2.56 ms optimal):
 *   1. IDLE: Wait for ADC buffer ready
 *      - Get 256 ADC samples
 *      - Get 0..50 IMU samples
 *      - Assemble frame with both datasets
 *      - Compute CRC16, queue frame
 *   2. IMU_COLLECT: Queue the assembled frame, reset builder
 *
 * Single-pass design: All data collection happens in IDLE state,
 * no waiting between sub-states. Frame is queued immediately after assembly.
 *
 * Returns: uint32_t - Number of frames queued (0 or 1 per call)
 *
 * Frame Queue: 4 frames max
 *              If queue full, frame is dropped (stats tracked)
 */
uint32_t Logger_Task(void);

/**
 * @brief Get next complete frame from queue (for Phase 5 SPI DMA)
 *
 * Args: out_frame - Pointer to LogFrame_t to receive frame data
 *
 * Returns: int - 1 if frame available (copied to out_frame), 0 if queue empty
 *
 * Behavior:
 *   - Copies frame from internal queue to out_frame
 *   - Advances read pointer (frame removed from queue)
 *   - Called from Phase 5 SPI DMA transmission handler
 */
int Logger_GetNextFrame(LogFrame_t *out_frame);





/* ============================================================================
 * UTILITY: Compiler guard for SPI_LOGGER_ENABLE
 * ============================================================================
 */

/**
 * Integration checklist:
 *
 * [ ] 1. In hal_cfg.h: Add #define LOGGER_TIMESTAMP_TIMER_BASE TIM3
 * [ ] 2. In hal_cfg.h: Add extern TIM_HandleTypeDef htim3;
 * [ ] 3. In solution_wrapper.c: #include "logger.h"
 * [ ] 4. In solution_wrapper.c Solution_HalInit(): Add Logger_Timing_Init() call
 * [ ] 5. In LSM6DS3.c Lsm6ds3_GetDataCbk(): Add SPI_LOGGER_ENABLE gate before Logger_ImuOnNewSample()
 * [ ] 6. Compile and verify no errors
 * [ ] 7. Debug: Set breakpoint in Logger_Timing_Init(), verify TIM5->CNT increments @ 100 kHz
 * [ ] 8. Validate: Use Saleae to verify TIM6 TRGO matches TIM5 counter increments
 */

/* ============================================================================
 * PHASE 5: SPI SLAVE TRANSMISSION
 * ============================================================================
 *
 * Provides queue and transmission interface for Phase 4 LogFrame_t structures.
 * Queues complete frames (640 bytes) from logger_frame.c for DMA transmission.
 *
 * Architecture:
 *   - Phase 4 (logger_frame.c): Builds frames in g_frame_ctx.frame_queue via write_idx
 *   - Phase 5 (logger_spi.c): Reads frames via Logger_GetNextFrame(), transmits via DMA
 *   - Unified queue: Single 10-frame buffer eliminates duplication (6.4 KB, was 12.8 KB)
 *   - SPI interrupt: Logger_SPI_RxCallback() dequeues and transmits via DMA
 *
 * Integration:
 *   1. Call Logger_Init() once from App_InitRun
 *   2. Phase 4 automatically queues complete frames to g_frame_ctx.frame_queue
 *   3. Phase 5 reads frames via Logger_GetNextFrame() in Logger_SPI_RxCallback()
 *   3. Weak overrides in solution_wrapper.c:
 *      - Logger_GPIO_SetReady(bool): Drive LOGGER_SPI_DATA_RDY_PIN
 *      - Logger_SPI_TransmitFrame(LogFrame_t*): DMA transmission via HAL_SPI_Transmit_DMA
 *      - Logger_SPI_Init(): Enable SPI2 NVIC interrupt
 */

/**
 * @brief Get next complete frame from unified queue (Phase 4 → Phase 5)
 *
 * Retrieves frame from g_frame_ctx.frame_queue built by Phase 4 frame builder.
 * Copies entire LogFrame_t (640 bytes) to provided buffer and increments queue read index.
 *
 * @param out_frame: Pointer to LogFrame_t to receive frame data
 *
 * Returns: 1 if frame available (copied to out_frame), 0 if queue empty
 *
 * Usage in Phase 5 SPI callback:
 *   LogFrame_t frame;
 *   if (Logger_GetNextFrame(&frame)) {
 *       Logger_SPI_TransmitFrame(&frame);  // Transmit via DMA
 *       Logger_FrameTransmitComplete();    // Update state
 *   }
 *
 * @note Thread-safe when called from single SPI interrupt context
 */
/**
 * @brief Get number of frames pending SPI transmission
 * @return Count of frames queued in unified buffer awaiting transmission
 */
uint32_t Logger_GetPendingCount(void);

/**
 * @brief Initialize SPI slave transmission pipeline
 *
 * Called once from App_InitRun. Sets up GPIO ready signal and enables SPI2 interrupt.
 * Must be called before Logger_SPI_RxCallback() can be used.
 */
void Logger_Init(void);

/**
 * @brief SPI Slave RX callback - dequeue and transmit frame when master reads
 *
 * Called from HAL_SPI_RxCpltCallback in solution_wrapper.c when ESP32 master
 * asserts CS and initiates SPI transaction.
 *
 * Dequeues next frame and initiates DMA transmission via Logger_SPI_TransmitFrame().
 * @param rx_buffer: Pointer to 5-byte command buffer received from master
 *
 * Automatically manages GPIO ready signal.
 *
 * @note Called from SPI2 interrupt context; keep execution time minimal
 */
void Logger_SPI_RxCallback(const uint8_t *rx_buffer);

/**
 * @brief SPI Slave TX Complete callback – handle transmission completion
 *
 * Called when DMA transmission of frame/config data to master is complete.
 * Clears GPIO ready signal and re-arms RX for next command.
 *
 * @note Called from SPI2 interrupt context; keep execution time minimal
 */
void Logger_SPI_TxCallback(void);

/**
 * @brief Control SPI_DATA_RDY GPIO pin (ready signal to master)
 *
 * Override this in solution_wrapper.c to drive LOGGER_SPI_DATA_RDY_PIN.
 * Called automatically by Logger_PublishFrame() and Logger_FrameTransmitComplete().
 *
 * @param ready: true to set GPIO high (data available), false to set low (queue empty)
 */
void Logger_GPIO_SetReady(bool ready);

/**
 * @brief Transmit LogFrame_t via SPI2 slave DMA
 *
 * Override this in solution_wrapper.c to call HAL_SPI_Transmit() with blocking transfer.
 * Called automatically by Logger_SPI_RxCallback() when master initiates read.
 *
 * @param buffer: Pointer to data buffer to transmit
 * @param size: Number of bytes to transmit (e.g., 32 for config, 640 for frame)
 *
 * @note Blocking transfer; function returns when transmission completes
 * @note Automatically manages GPIO ready signal and re-arms RX
 */
void Logger_SPI_Transmit(const uint8_t *buffer, uint16_t size);

/**
 * @brief Initialize SPI2 slave and NVIC interrupt
 *
 * Override this in solution_wrapper.c to enable SPI2 interrupt handling.
 * Called once from Logger_Init().
 */
void Logger_SPI_Init(void);

/**
 * @brief Start listening for SPI master read transactions
 *
 * Called after Logger_Init() to arm SPI slave interrupt.
 * When ESP32 master initiates read, HAL_SPI_RxCpltCallback fires and calls Logger_SPI_RxCallback().
 */
void Logger_SPI_StartListening(void);

/**
 * @brief Notify logger that accelerometer has been successfully initialized
 *
 * Called from LSM6DS3.c::Lsm6ds3_SetHitParams() when sensor configuration is complete.
 * Decodes register values and populates g_logger_config.imu_config structure.
 * Automatically determines readiness: accel_present and gyro_present flags are set to 1
 * if their respective ODR values are > 0.
 *
 * @param imu_cfg: Pointer to imu_config_t structure with register values from hardware.
 *                 If NULL, initialization is skipped (returns early, readiness = 0).
 *                 If non-NULL, register bits are decoded and imu_config is populated.
 *
 * Register decoding (performed internally):
 *   - ODR extracted from bits [7:4] using switch statement
 *   - Accel range extracted from CTRL1_XL[3:2]: 0x0→2G, 0x1→16G, 0x2→4G, 0x3→8G
 *   - Gyro range extracted from CTRL2_G[3:2]: 0x0→245DPS, 0x1→500DPS, 0x2→1000DPS, 0x3→2000DPS
 *
 * Integration point:
 *   Called from LSM6DS3.c::Lsm6ds3_SetHitParams() (~line 338) after register writes verified
 */
void Logger_OnAccelerometerReady(const imu_config_t *imu_cfg);

#ifdef __cplusplus
}
#endif

#endif  // SHARED_INC_LOGGER_H_
