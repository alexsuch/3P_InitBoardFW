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
 *   - frame_counter (2B): Sequential frame number
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
 * CRC16 CALCULATION (Reflected polynomial)
 * ============================================================================
 */

static uint16_t crc16_compute(const uint8_t *data, uint32_t len)
{
    uint16_t crc = CRC16_INIT;
    
    for (uint32_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i];
        
        for (int j = 0; j < 8; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ CRC16_POLY;
            } else {
                crc = crc >> 1;
            }
        }
    }
    
    return crc;
}


/**
 * @brief Initialize frame builder state machine
 *
 * Called once at startup to initialize frame assembly subsystem:
 *   - Clears frame builder state (FRAME_STATE_IDLE)
 *   - Zeros ADC/IMU accumulator buffers
 *   - Prepares for first ADC block collection
 *
 * Returns: void
 */
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
    loggerStat.config.adc_sample_rate_hz = ADC_SAMPLING_FREQ_KHZ;  // 100 kHz sampling
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
   
#if 1
    // DEBUG: Fill first 50 bytes with pattern (1..50) for testing
    for (int i = 0; i < 50; i++) {
        ((uint8_t*)&loggerStat.tx_frame)[i] = (uint8_t)(i + 1);
    }
#endif
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
                frame->frame_counter = loggerStat.frame_ctx.frame_counter++;
                frame->n_imu = loggerStat.frame_ctx.builder.imu_count;
                
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
                
                // Compute CRC16 over entire frame (excluding crc16 field)
                uint16_t crc_data_len = sizeof(LogFrame_t) - sizeof(uint16_t);
                frame->crc16 = crc16_compute((const uint8_t *)frame, crc_data_len);
                
                loggerStat.frame_ctx.queue_write_idx++;
                loggerStat.frame_ctx.stats_frames_built++;
                
                // Drain queue: try to start transmitting frames via DMA
                // Only drain if SPI is currently IDLE (no transmission in progress)
                if (loggerStat.frame_ctx.spi_state == SPI_STATE_IDLE && Logger_DrainQueue()) {
                    loggerStat.frame_ctx.spi_state = SPI_STATE_PACKET_SENT;
                }
                
                frames_queued = 1;
            } else {
                loggerStat.frame_ctx.stats_frames_dropped++;
            }
            
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