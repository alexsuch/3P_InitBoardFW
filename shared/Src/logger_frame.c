/**
 * @file logger_frame.c
 * @brief Frame Builder & SPI Protocol - Merges ADC + IMU data, manages SPI communication
 *
 * Role: 
 *   1. Frame Assembly: Merge ADC samples (256 @ 100kHz) + IMU samples (0-50 @ ~104Hz) into
 *      fixed-size LogFrame_t structures with CRC16 validation
 *   2. Frame Queue: Maintain circular FIFO for frames awaiting SPI transmission
 *   3. SPI Slave: Handle CMD 42 (0x2A) to transmit logger_config_t to master (ESP32)
 *   4. Auto-Streaming: After config handshake, automatically stream queued frames via DMA
 *
 * Frame Assembly Path:
 *   - Wait for ADC block (256 samples @ 100kHz = ~2.56ms)
 *   - Collect available IMU samples (0-50) from ring buffer
 *   - Assemble frame with CRC16 (reflected polynomial)
 *   - Queue for transmission (circular FIFO: up to LOGGER_FRAME_QUEUE_SIZE frames)
 *
 * SPI Protocol (Master: ESP32, Slave: STM32):
 *   - Master sends CMD 42 at startup to request config
 *   - Slave responds with logger_config_t (32B) via DMA
 *   - After config TX: Frames auto-stream from queue
 *   - GPIO_READY signals data availability during transmission
 *
 * Frame Layout (LOGGER_FRAME_SIZE_BYTES bytes):
 *   - magic (2B): 0x5A5A (frame marker)
 *   - n_imu (2B): 0-50 valid IMU samples in frame
 *   - adc[256] (512B): ADC int16_t samples
 *   - imu[50] (600B): IMU samples (12B each, only n_imu valid)
 *   - crc16 (2B): CRC16 over payload
 *   Total: ~1118 bytes
 */

#include "logger.h"
#include "prj_config.h"
#include "solution_wrapper.h"
#include <string.h>


#define LOGGER_FRAME_SIZE_BYTES         sizeof(LogFrame_t)  // Auto-calculated from LogFrame_t structure

static logger_status_t loggerStat = {0};

/* ============================================================================
 * CRC8 CALCULATION (Table-based - Fast)
 * ============================================================================
 */

// Pre-computed CRC8 lookup table (256 entries)
// Polynomial: 0x07 (CRC-8-ATM / CRC-8-CCITT)
static const uint8_t CRC8_TABLE[256] = {
    0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15,
    0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D,
    0x70, 0x77, 0x7E, 0x79, 0x6C, 0x6B, 0x62, 0x65,
    0x48, 0x4F, 0x46, 0x41, 0x54, 0x53, 0x5A, 0x5D,
    0xE0, 0xE7, 0xEE, 0xE9, 0xFC, 0xFB, 0xF2, 0xF5,
    0xD8, 0xDF, 0xD6, 0xD1, 0xC4, 0xC3, 0xCA, 0xCD,
    0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B, 0x82, 0x85,
    0xA8, 0xAF, 0xA6, 0xA1, 0xB4, 0xB3, 0xBA, 0xBD,
    0xC7, 0xC0, 0xC9, 0xCE, 0xDB, 0xDC, 0xD5, 0xD2,
    0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4, 0xED, 0xEA,
    0xB7, 0xB0, 0xB9, 0xBE, 0xAB, 0xAC, 0xA5, 0xA2,
    0x8F, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A,
    0x27, 0x20, 0x29, 0x2E, 0x3B, 0x3C, 0x35, 0x32,
    0x1F, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0D, 0x0A,
    0x57, 0x50, 0x59, 0x5E, 0x4B, 0x4C, 0x45, 0x42,
    0x6F, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7D, 0x7A,
    0x89, 0x8E, 0x87, 0x80, 0x95, 0x92, 0x9B, 0x9C,
    0xB1, 0xB6, 0xBF, 0xB8, 0xAD, 0xAA, 0xA3, 0xA4,
    0xF9, 0xFE, 0xF7, 0xF0, 0xE5, 0xE2, 0xEB, 0xEC,
    0xC1, 0xC6, 0xCF, 0xC8, 0xDD, 0xDA, 0xD3, 0xD4,
    0x69, 0x6E, 0x67, 0x60, 0x75, 0x72, 0x7B, 0x7C,
    0x51, 0x56, 0x5F, 0x58, 0x4D, 0x4A, 0x43, 0x44,
    0x1D, 0x1A, 0x13, 0x14, 0x01, 0x06, 0x0F, 0x08,
    0x25, 0x22, 0x2B, 0x2C, 0x39, 0x3E, 0x37, 0x30,
    0x4A, 0x4D, 0x44, 0x43, 0x56, 0x51, 0x58, 0x5F,
    0x72, 0x75, 0x7C, 0x7B, 0x6E, 0x69, 0x60, 0x67,
    0x3A, 0x3D, 0x34, 0x33, 0x26, 0x21, 0x28, 0x2F,
    0x02, 0x05, 0x0C, 0x0B, 0x1E, 0x19, 0x10, 0x17,
    0xAA, 0xAD, 0xA4, 0xA3, 0xB6, 0xB1, 0xB8, 0xBF,
    0x92, 0x95, 0x9C, 0x9B, 0x8E, 0x89, 0x80, 0x87,
    0xDA, 0xDD, 0xD4, 0xD3, 0xC6, 0xC1, 0xC8, 0xCF,
    0xE2, 0xE5, 0xEC, 0xEB, 0xFE, 0xF9, 0xF0, 0xF7
};

/**
 * @brief Fast table-based CRC8 calculation (full buffer)
 * 
 * Uses pre-computed lookup table to compute CRC8 in O(n) time with minimal cycles.
 * Polynomial: 0x07 (CRC-8-ATM / CRC-8-CCITT)
 * 
 * Performance:
 *   - 1116 bytes @ 168 MHz: ~25 μs (vs 50 μs for CRC16, 300+ μs for bit-by-bit)
 *   - ~2 cycles per byte (lookup + XOR)
 *   - 50% faster than CRC16, uses 4× less ROM (256B vs 1KB)
 * 
 * @param data: Pointer to data buffer
 * @param len: Length in bytes
 * @return CRC8 checksum (8-bit)
 */
static uint8_t crc8_compute(const uint8_t *data, uint32_t len)
{
    uint8_t crc = 0x00;  // CRC8 init value
    
    for (uint32_t i = 0; i < len; i++) {
        crc = CRC8_TABLE[crc ^ data[i]];
    }
    
    return crc;
}

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
static uint8_t crc8_update_chunk(uint8_t crc, const uint8_t *data, uint32_t len)
{
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
static uint8_t crc8_compute_full(const uint8_t *data, uint32_t len)
{
    uint8_t crc = 0x00;  // CRC8 init value
    
    for (uint32_t i = 0; i < len; i++) {
        crc = CRC8_TABLE[crc ^ data[i]];
    }
    
    return crc;
}

/**
 * @brief Validate CRC computation: compare full vs incremental methods
 * 
 * Computes CRC8 using both methods:
 *   1. Full single-pass method (crc8_compute_full)
 *   2. Incremental 4-part state machine method (simulated)
 * 
 * Useful for unit testing and verifying that both approaches produce identical results.
 * 
 * @param frame: Pointer to LogFrame_t structure to validate
 * @return 1 if both methods produce same CRC, 0 if mismatch (indicates bug)
 * 
 * Usage:
 *   LogFrame_t my_frame = {...};
 *   if (!crc8_validate_methods(&my_frame)) {
 *       // ERROR: CRC methods diverged!
 *   }
 */
static uint32_t crc8_validate_methods(const LogFrame_t *frame)
{
    if (!frame) {
        return 0;
    }
    
    const uint8_t *frame_data = (const uint8_t *)frame;
    uint32_t data_len = sizeof(LogFrame_t) - sizeof(uint16_t);  // Exclude crc16 field (2 bytes)
    
    // ===== METHOD 1: Full single-pass =====
    uint8_t crc_full = crc8_compute_full(frame_data, data_len);
    
    // ===== METHOD 2: Incremental 4-part (simulate what Logger_Task does) =====
    uint8_t crc_incremental = 0x00;
    
    // Part 1: bytes 0-133 (header + first quarter of ADC)
    crc_incremental = crc8_update_chunk(crc_incremental, &frame_data[0], 134);
    
    // Part 2: bytes 134-261 (second quarter of ADC)
    crc_incremental = crc8_update_chunk(crc_incremental, &frame_data[134], 128);
    
    // Part 3: bytes 262-389 (third quarter of ADC)
    crc_incremental = crc8_update_chunk(crc_incremental, &frame_data[262], 128);
    
    // Part 4: bytes 390 to end (remaining data)
    uint32_t part4_size = data_len - 390;
    crc_incremental = crc8_update_chunk(crc_incremental, &frame_data[390], part4_size);
    
    // ===== COMPARE RESULTS =====
    if (crc_full != crc_incremental) {

        // CRC MISMATCH - indicates bug in either method!
        return 0;  // Validation FAILED
    }
    else
    {
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
 *   Part 4: 2nd-4th quarter IMU (420..517 bytes, excl. crc16 field)
 *
 * @param crc_state: Pointer to CrcState_t to initialize
 * @param state_step: Which step (0=part1, 1=part2, 2=part3, 3=part4)
 */
static void crc_state_init_step(CrcState_t *crc_state, uint32_t state_step)
{
    switch (state_step) {
    case 0:  // FRAME_STATE_BUILD -> Part 1: header (6B) + ADC[0..63] (128B) = 134B
        crc_state->crc_offset = 0;
        crc_state->crc_chunk_size = 134;
        break;
    case 1:  // FRAME_STATE_CRC_PART2: ADC[64..127] (128B)
        crc_state->crc_offset = 134;
        crc_state->crc_chunk_size = 128;
        break;
    case 2:  // FRAME_STATE_CRC_PART3: ADC[128..191] (128B)
        crc_state->crc_offset = 262;
        crc_state->crc_chunk_size = 128;
        break;
    case 3:  // FRAME_STATE_CRC_PART4: ADC[192..255] (128B) + IMU (120B) - excl crc16 = 248B
        crc_state->crc_offset = 390;
        crc_state->crc_chunk_size = 128;  // Remaining data (will be computed in finalize)
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
static uint32_t crc_state_process_step(FrameBuilder_t *builder)
{
    if (!builder->current_frame) {
        return 0;
    }

    LogFrame_t *frame = builder->current_frame;
    uint8_t *frame_data = (uint8_t *)frame;
    uint32_t data_len = sizeof(LogFrame_t) - sizeof(uint8_t);  // Exclude crc16 field

    // Ensure we don't process past the end
    if (builder->crc_state.crc_offset + builder->crc_state.crc_chunk_size > data_len) {
        builder->crc_state.crc_chunk_size = data_len - builder->crc_state.crc_offset;
    }

    // Process this chunk
    uint8_t *chunk_ptr = &frame_data[builder->crc_state.crc_offset];
    builder->crc_state.crc_current = crc8_update_chunk(
        builder->crc_state.crc_current,
        chunk_ptr,
        builder->crc_state.crc_chunk_size
    );

    return 1;  // Step complete
}
void Logger_FrameBuilder_Init(void)
{
    memset(&loggerStat.frame_ctx, 0, sizeof(loggerStat.frame_ctx));
    loggerStat.frame_ctx.builder.state = FRAME_STATE_IDLE;
}

/**
 * @brief Initialize SPI slave protocol and logger configuration
 *
 * Called once from App_InitRun() to set up SPI slave communication:
 *   - Sets configuration (magic=LOGGER_CONFIG_MAGIC, version=1, ADC params)
 *   - Initializes SPI2 peripheral for interrupt-driven command reception
 *   - Starts listening for master (ESP32) command byte (CMD 42 only)
 *   - Initializes GPIO_READY signal (low = no data available)
 *   - Sets SPI state machine to IDLE
 *
 * Returns: void
 */
void Logger_Init(void)
{    
    // Initialize logger configuration
    loggerStat.config.magic = LOGGER_CONFIG_MAGIC;              // 0xCAFE
    loggerStat.config.version = LOGGER_CONFIG_VERSION;          // 1
    loggerStat.config.adc_sample_rate_khz = ADC_SAMPLING_FREQ_KHZ;  // 100 kHz sampling
    loggerStat.config.adc_block_size = LOGGER_ADC_BLOCK_SIZE;   // 256 samples per frame

    // Initialize SPI2 NVIC for interrupt-driven reception
    Logger_SPI_Init();
    
    // Start listening for incoming command bytes from ESP32 master
    // This arms the SPI slave RX interrupt to receive the first command
    Logger_SPI_StartListening();

    // Initialize GPIO to ready=false (no data available initially)
    Logger_GPIO_SetReady(false);
    
    // Initialize SPI state machine
    loggerStat.frame_ctx.spi_state = SPI_STATE_IDLE;
}


/**
 * @brief Drain one frame from queue and initiate SPI DMA transmission
 *
 * High-level queue draining function that manages post-config frame streaming:
 *   1. Checks config_sent flag (set by Logger_SPI_RxCallback after config TX complete)
 *   2. If config_sent=1: Marks transition complete, calls Solution_LoggingStart() to enable ADC/IMU
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
    // If we just finished sending config, skip this cycle and reset flag
    if (loggerStat.config_sent == 1) 
    {
        Solution_LoggingStart();  // Start logging after config sent

        loggerStat.config_sent = 0;  // Reset for next cycle
    }

    // Try to dequeue next frame
    if (!Logger_GetNextFrame(&loggerStat.tx_frame)) {
        // Queue is empty, no frame to send
        return 0;
    }
   
    // Frame available - signal and transmit
    Logger_SPI_Transmit((uint8_t*)&loggerStat.tx_frame, sizeof(LogFrame_t));
    Logger_GPIO_SetReady(true);
    
    return 1;  // Frame sent
}

/**
 * @brief Main periodic task: Assemble frames and drain SPI transmission queue
 *
 * Called from main loop (optimal: every ~2.56ms for 256 samples @ 100kHz):
 *   - Monitors ADC buffer ready flag
 *   - Collects 256 ADC samples when ready
 *   - Retrieves available IMU samples (0-50) from ring buffer
 *   - Assembles LogFrame_t with CRC16
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
uint32_t Logger_Task(void)
{
    uint32_t frames_queued = 0;
    
int j = 0;

    // Check if previous SPI transmission completed and queue needs draining
    // This allows continuous streaming without waiting for ADC block
    if (loggerStat.frame_ctx.spi_state == SPI_STATE_PACKET_SENT_COMPLETE) 
    {
        loggerStat.frame_ctx.spi_state = SPI_STATE_IDLE;
        // Try to drain next frame from queue
        if (Logger_DrainQueue()) {
            loggerStat.frame_ctx.spi_state = SPI_STATE_PACKET_SENT;
        }
    }



    switch (loggerStat.frame_ctx.builder.state) 
    {
    case FRAME_STATE_IDLE:
        {
            // Check if ADC buffer is ready (LOGGER_ADC_BLOCK_SIZE samples accumulated)
            if (Logger_AdcBuffer_IsReady()) 
            {
                // Get pointer to ADC buffer
                uint32_t adc_count = 0;
                const int16_t *adc_data = Logger_AdcBuffer_GetBuffer(&adc_count);
                
                if (adc_data && adc_count == LOGGER_ADC_BLOCK_SIZE) {
                    // Copy ADC data to builder
                    memcpy(loggerStat.frame_ctx.builder.adc, adc_data, LOGGER_ADC_BLOCK_SIZE * sizeof(int16_t));
                    loggerStat.frame_ctx.builder.adc_count = LOGGER_ADC_BLOCK_SIZE;
                    loggerStat.frame_ctx.builder.adc_ts_first = Logger_AdcBuffer_GetFirstTimestamp();
                    
                    // Clear ADC buffer for next block
                    Logger_AdcBuffer_ClearReady();
                    
                    // Get IMU samples from linear buffer (0..50 samples)
                    uint32_t imu_count = 0;
                    const ImuRawSample_t *imu_data = Logger_ImuRing_GetBuffer(&imu_count);
                    
                    if (imu_data && imu_count > 0) {
                        // Copy IMU data
                        memcpy(loggerStat.frame_ctx.builder.imu, imu_data, imu_count * sizeof(ImuRawSample_t));
                        loggerStat.frame_ctx.builder.imu_count = imu_count;
                        
                        // Clear IMU buffer for next batch
                        Logger_ImuRing_Clear();
                    } else {
                        // No IMU data, proceed with ADC-only frame
                        loggerStat.frame_ctx.builder.imu_count = 0;
                    }
                    
                    // Now assemble and queue the frame
                    loggerStat.frame_ctx.builder.state = FRAME_STATE_BUILD;
                }
            }



        }
        break;
        
    case FRAME_STATE_BUILD:
        {
            // Check if frame queue has space
            uint32_t queue_size = (loggerStat.frame_ctx.queue_write_idx - loggerStat.frame_ctx.queue_read_idx) 
                                 % LOGGER_FRAME_QUEUE_SIZE;
            
            if (queue_size < (LOGGER_FRAME_QUEUE_SIZE - 1)) {
                // Get pointer to next frame slot in queue
                uint32_t write_idx = loggerStat.frame_ctx.queue_write_idx % LOGGER_FRAME_QUEUE_SIZE;
                LogFrame_t *frame = &loggerStat.frame_ctx.frame_queue[write_idx];
                
                // Assemble frame header
                frame->magic = LOGGER_FRAME_MAGIC;
                frame->n_imu = loggerStat.frame_ctx.builder.imu_count;
                frame->adc_timestamp = loggerStat.frame_ctx.builder.adc_ts_first;
                
                // Copy ADC block (LOGGER_ADC_BLOCK_SIZE samples, fixed)
                memcpy(frame->adc, loggerStat.frame_ctx.builder.adc, LOGGER_ADC_BLOCK_SIZE * sizeof(int16_t));
                
                // Copy IMU block (only first n_imu samples are valid)
                if (loggerStat.frame_ctx.builder.imu_count > 0) {
                    memcpy(frame->imu, 
                           loggerStat.frame_ctx.builder.imu, 
                           loggerStat.frame_ctx.builder.imu_count * sizeof(ImuRawSample_t));
                    
                    // Zero out remaining IMU slots
                    if (loggerStat.frame_ctx.builder.imu_count < LOGGER_IMU_BLOCK_SIZE) {
                        memset(&frame->imu[loggerStat.frame_ctx.builder.imu_count],
                               0,
                               (LOGGER_IMU_BLOCK_SIZE - loggerStat.frame_ctx.builder.imu_count) * sizeof(ImuRawSample_t));
                    }
                } else {
                    // No IMU data, zero entire IMU block
                    memset(frame->imu, 0, LOGGER_IMU_BLOCK_SIZE * sizeof(ImuRawSample_t));
                }
                
                // Initialize CRC state machine (Part 1 of 4)
                loggerStat.frame_ctx.builder.current_frame = frame;
                loggerStat.frame_ctx.builder.crc_state.crc_current = 0x00;
                crc_state_init_step(&loggerStat.frame_ctx.builder.crc_state, 0);
                
                // Transition to CRC Part 1 processing
                loggerStat.frame_ctx.builder.state = FRAME_STATE_CRC_PART1;
            } else {
                loggerStat.frame_ctx.stats_frames_dropped++;
                memset(&loggerStat.frame_ctx.builder, 0, sizeof(FrameBuilder_t));
                loggerStat.frame_ctx.builder.state = FRAME_STATE_IDLE;
            }
        }
        break;
    
    case FRAME_STATE_CRC_PART1:
        {
            // Process CRC Part 1 (~134 bytes)
            crc_state_process_step(&loggerStat.frame_ctx.builder);
            loggerStat.frame_ctx.builder.state = FRAME_STATE_CRC_PART2;
        }
        break;
    
    case FRAME_STATE_CRC_PART2:
        {
            // Process CRC Part 2 (~128 bytes)
            crc_state_init_step(&loggerStat.frame_ctx.builder.crc_state, 1);
            crc_state_process_step(&loggerStat.frame_ctx.builder);
            loggerStat.frame_ctx.builder.state = FRAME_STATE_CRC_PART3;
        }
        break;
    
    case FRAME_STATE_CRC_PART3:
        {
            // Process CRC Part 3 (~128 bytes)
            crc_state_init_step(&loggerStat.frame_ctx.builder.crc_state, 2);
            crc_state_process_step(&loggerStat.frame_ctx.builder);
            loggerStat.frame_ctx.builder.state = FRAME_STATE_CRC_PART4;
        }
        break;
    
    case FRAME_STATE_CRC_PART4:
        {
            // Process CRC Part 4 (remaining ~248 bytes) and finalize
            crc_state_init_step(&loggerStat.frame_ctx.builder.crc_state, 3);
            uint32_t data_len = sizeof(LogFrame_t) - sizeof(uint8_t);
            uint32_t chunk_start = loggerStat.frame_ctx.builder.crc_state.crc_offset;
            uint32_t chunk_size = data_len - chunk_start;  // Remaining bytes
            
            LogFrame_t *frame = loggerStat.frame_ctx.builder.current_frame;
            uint8_t *frame_data = (uint8_t *)frame;
            uint8_t *chunk_ptr = &frame_data[chunk_start];
            
            loggerStat.frame_ctx.builder.crc_state.crc_current = crc8_update_chunk(
                loggerStat.frame_ctx.builder.crc_state.crc_current,
                chunk_ptr,
                chunk_size
            );
            
            // CRC computation complete - write result to frame
            frame->crc16 = loggerStat.frame_ctx.builder.crc_state.crc_current;
            
            // ===== VALIDATION: Compare full vs incremental CRC methods =====
            #if (LOGGER_CRC_VALIDATION_ENABLE == 1u)
            if (!crc8_validate_methods(frame)) {
                // CRC MISMATCH - indicates bug in either method!
                loggerStat.frame_ctx.stats_crc_mismatches++;
            }
            #endif
            
            loggerStat.frame_ctx.queue_write_idx++;
            loggerStat.frame_ctx.stats_frames_built++;
            
            // Drain queue: try to start transmitting frames via DMA
            // Only drain if SPI is currently IDLE (no transmission in progress)
            if (loggerStat.frame_ctx.spi_state == SPI_STATE_IDLE && Logger_DrainQueue()) {
                loggerStat.frame_ctx.spi_state = SPI_STATE_PACKET_SENT;
            }
            
            frames_queued = 1;

            //Test2Toggle();
            //Test2Toggle();

            
            // Reset builder for next frame
            memset(&loggerStat.frame_ctx.builder, 0, sizeof(FrameBuilder_t));
            loggerStat.frame_ctx.builder.state = FRAME_STATE_IDLE;
        }
        break;
        
    default:
        loggerStat.frame_ctx.builder.state = FRAME_STATE_IDLE;
        break;
    }
    


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
 *       // frame.crc16 has CRC value
 *   }
 */
int Logger_GetNextFrame(LogFrame_t *out_frame)
{
    if (!out_frame) {
        return 0;
    }
    
    uint32_t queue_size = (loggerStat.frame_ctx.queue_write_idx - loggerStat.frame_ctx.queue_read_idx) 
                         % LOGGER_FRAME_QUEUE_SIZE;
    
    if (queue_size == 0) {
        return 0;  // Queue empty
    }
    
    uint32_t read_idx = loggerStat.frame_ctx.queue_read_idx % LOGGER_FRAME_QUEUE_SIZE;
    LogFrame_t *frame = &loggerStat.frame_ctx.frame_queue[read_idx];
    
    // Copy entire frame (LOGGER_FRAME_SIZE_BYTES bytes)
    memcpy(out_frame, frame, LOGGER_FRAME_SIZE_BYTES);
    loggerStat.frame_ctx.queue_read_idx++;
    
    return 1;  // Frame retrieved
}

/**
 * @brief Get number of frames pending SPI transmission
 * @return Count of frames waiting to be transmitted
 */
uint32_t Logger_GetPendingCount(void)
{
    return loggerStat.frame_ctx.spi_pending_count;
}


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
void Logger_SPI_TxCallback(void) 
{
    // Transmission of one full frame/config is complete
    // Signal ESP32 that DMA transmission finished by lowering GPIO
    Logger_GPIO_SetReady(false);
    
    // Clear any SPI error flags to prevent lockup
    __HAL_SPI_CLEAR_OVRFLAG(&LOGGER_SPI_HANDLE);
    __HAL_SPI_CLEAR_FREFLAG(&LOGGER_SPI_HANDLE);
    
    // Signal that transmission is complete
    // Logger_Task() will check this state and drain queue on next invocation
    loggerStat.frame_ctx.spi_state = SPI_STATE_PACKET_SENT_COMPLETE;
}


/**
 * @brief SPI Slave RX Callback - Handle command from ESP32 master
 *
 * Called from HAL_SPI_RxCpltCallback in solution_wrapper.c when master
 * sends a command byte via dedicated CMD channel (CMD 42 only supported).
 *
 * SPI Protocol (Auto-Streaming):
 *   - Master initiates with single CMD 42 (0x2A) at startup
 *   - Slave responds by transmitting logger_config_t (32 bytes) via DMA
 *   - After config TX complete: Logger_Task() automatically streams queued frames
 *   - No manual READ commands needed - frames auto-send when available
 *
 * CMD 42 (LOGGER_SPI_CMD_CONFIG) Protocol Flow:
 *   1. Master sends command byte 42 on CMD channel
 *   2. RxCallback extracts command and copies logger_config_t to TX buffer
 *   3. Sets config_sent = 1 to mark one-time config phase
 *   4. Initiates DMA transmission of config data
 *   5. Raises GPIO_READY to signal data available
 *   6. After DMA completes: Logger_SPI_TxCallback() lowers GPIO
 *   7. Logger_Task() drain detects config_sent=1 and transitions to frame streaming mode
 *   8. ADC/IMU logging enabled via Solution_LoggingStart() (called from Logger_DrainQueue)\n *   9. Subsequent Logger_Task() cycles automatically drain frame queue
 *
 * Unknown Commands: If command != LOGGER_SPI_CMD_CONFIG, GPIO is lowered and no transmission occurs
 *
 * @note Called from SPI2 interrupt context; keep execution time minimal
 * @note Non-blocking; all transmission happens via DMA with callbacks
 * @param rx_buffer: Pointer to received command buffer (at least 5 bytes)
 */
void Logger_SPI_RxCallback(const uint8_t *rx_buffer)
{
    if (!rx_buffer) {
        return;
    }
        
    uint8_t command = rx_buffer[0];  // Extract command byte
    
    if (command == LOGGER_SPI_CMD_CONFIG) {
        // ===== CONFIG COMMAND (0x2A) ====="
        // Single startup command: send logger configuration to master
        
        // Copy entire loggerStat.config to TX buffer (32 bytes)
        memcpy((uint8_t*)&loggerStat.tx_frame, (uint8_t*)&loggerStat.config, sizeof(logger_config_t));
        
        // Mark that config is being sent (one-time transition marker)
        // Next drain cycle will skip, then transition to frame mode
        loggerStat.config_sent = 1;
        
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
void Logger_OnAccelerometerReady(const imu_config_t *imu_cfg)
{
    if (imu_cfg == NULL) {
        return;  // Initialization failed
    }
    
    // Simply copy the entire populated structure to global config
    memcpy(&loggerStat.config.imu_config, imu_cfg, sizeof(imu_config_t));
}