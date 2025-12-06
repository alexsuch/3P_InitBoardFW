#include "spi_logger.h"
#include "solution_wrapper.h"
#include "timer.h"
#include "prj_config.h"
#include "init_brd.h"
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

// Forward declarations - these will be provided by board-specific code
extern bool Timer_Start(uint8_t timer_id, uint32_t timer_period_ms, void (*cbk)(uint8_t));
extern void Logger_SPI_StartListening(void);  // Start SPI slave listening for reads

// Simple LFSR-based pseudo-random number generator
static uint32_t Logger_rng_state = 0x12345678;

static uint32_t Logger_GetRandom(void) {
    Logger_rng_state = (Logger_rng_state << 1) ^ ((Logger_rng_state >> 31) ? 0xB8 : 0);
    return Logger_rng_state;
}

// Weak symbols for GPIO and SPI control - can be overridden by board-specific code
__attribute__((weak))
void Logger_GPIO_SetReady(bool ready) {
    // Default weak implementation - does nothing
    // Override this in solution_hal_cfg.c to control actual GPIO
    (void)ready;
}

__attribute__((weak))
void Logger_SPI_TransmitFrame(const Logger_frame_t* frame) {
    // Default weak implementation - does nothing
    // Override this in solution_hal_cfg.c to transmit via SPI2
    (void)frame;
}

__attribute__((weak))
void Logger_SPI_Init(void) {
    // Default weak implementation - does nothing
    // Override this in solution_hal_cfg.c to initialize SPI2 NVIC
}

// Circular buffer for frame queue
static volatile uint32_t Logger_head = 0;
static volatile uint32_t Logger_tail = 0;
static volatile uint32_t Logger_count = 0;
static Logger_frame_t Logger_queue[Logger_QUEUE_LENGTH];
static volatile uint32_t Logger_seq = 0;
static volatile uint32_t Logger_pending_samples = 0;
static volatile uint32_t Logger_dropped_samples = 0;

static void Logger_SetReady(bool ready) {
    Logger_GPIO_SetReady(ready);
}

// Increment pending samples counter and return true if it was 0 (ready signal transition)
static bool Logger_IncrementPending(void) {
    Logger_pending_samples++;
    return (Logger_pending_samples == 1);
}

// Decrement pending samples counter and return true if it becomes 0 (not ready signal transition)
static bool Logger_DecrementPending(void) {
    bool should_lower = false;
    if (Logger_pending_samples > 0) {
        Logger_pending_samples--;
        should_lower = (Logger_pending_samples == 0);
    }
    return should_lower;
}

// Drop oldest frame from queue
static void Logger_DropOldest(void) {
    if (Logger_count > 0) {
        Logger_tail = (Logger_tail + 1) % Logger_QUEUE_LENGTH;
        Logger_count--;
        if (Logger_DecrementPending()) {
            Logger_SetReady(false);
        }
    }
}

// Forward declaration for timer callback
static void Logger_TimerCallback(uint8_t tmr_id);

// Call once from App_InitRun
void Logger_Init(void) {
    Logger_head = Logger_tail = Logger_count = Logger_seq = 0;
    Logger_pending_samples = Logger_dropped_samples = 0;
    Logger_SetReady(false);
    
    // Initialize SPI2 NVIC for interrupt-driven transmission
    Logger_SPI_Init();
    
    // Start listening for master read transactions (this enables the interrupt)
    Logger_SPI_StartListening();
    
    // Start periodic timer for Logger packet transmission (every 1 second)
    Timer_Start(LOGGER_SEND_TMR, LOGGER_SEND_TMR_PERIOD_MS, Logger_TimerCallback);
}

// Timer callback - generates and publishes a new sample with random data
static void Logger_TimerCallback(uint8_t tmr_id) {
    (void)tmr_id;  // Unused parameter
    
    // Generate sample with random data
    Logger_sample_t sample = {
        .accel = {
            (int16_t)Logger_GetRandom(),
            (int16_t)(Logger_GetRandom() >> 8),
            (int16_t)(Logger_GetRandom() >> 16),
        },
        .gyro = {
            (int16_t)Logger_GetRandom(),
            (int16_t)(Logger_GetRandom() >> 8),
            (int16_t)(Logger_GetRandom() >> 16),
        },
        .temp = (int16_t)Logger_GetRandom(),
    };
    
    // Publish sample to queue for transmission
    Logger_PublishSample(&sample);
}

// Call to queue a new sample for transmission (like accel_link_publish_sample)
void Logger_PublishSample(const Logger_sample_t* sample) {
    if (!sample) {
        return;
    }

    Logger_frame_t frame = {
        .magic = Logger_FRAME_MAGIC,
        .seq = ++Logger_seq,
        .timestamp_us = 0,  // Fill with timestamp if available
    };
    
    // Copy sample data to frame
    memcpy(frame.accel, sample->accel, sizeof(frame.accel));
    memcpy(frame.gyro, sample->gyro, sizeof(frame.gyro));
    frame.temp = sample->temp;
    memset(frame.reserved, 0, sizeof(frame.reserved));

    // Try to add frame to queue
    if (Logger_count >= Logger_QUEUE_LENGTH) {
        // Queue is full - drop oldest and try again
        Logger_dropped_samples++;
        Logger_DropOldest();
    }

    if (Logger_count < Logger_QUEUE_LENGTH) {
        // Add new frame to queue head
        Logger_queue[Logger_head] = frame;
        Logger_head = (Logger_head + 1) % Logger_QUEUE_LENGTH;
        Logger_count++;

        if (Logger_IncrementPending()) {
            // First sample in queue - signal ready to master
            Logger_SetReady(true);
        }

        // Start periodic timer for Logger packet transmission (every 1 second)
        Timer_Start(LOGGER_SEND_TMR, LOGGER_SEND_TMR_PERIOD_MS, Logger_TimerCallback);
    }
}

// Call from App_Task (main loop) - NOW ONLY manages queue, doesn't transmit
void Logger_Task(void) {
    // App_Task no longer triggers transmission
    // Transmission is now driven by SPI slave interrupt when master reads
    // This function can be used for diagnostics or other monitoring
}

// SPI Slave Interrupt Handler - called by HAL when master initiates transaction
// This should be called from SPI2_IRQHandler in stm32g4xx_it.c
void Logger_SPI_RxCallback(void) {
    // DEBUG: Toggle TEST1 when interrupt fires
    Test1Toggle();
    
    // DEBUG: Also toggle TEST2 to confirm entry
    Test2Toggle();

    // If no pending samples, nothing to do
    if (Logger_pending_samples == 0 || Logger_count == 0) {
        Test2Toggle();  // Toggle back if returning early
        return;
    }

    // Get frame from queue tail and transmit via SPI
    Logger_frame_t* frame = &Logger_queue[Logger_tail];
    Logger_SPI_TransmitFrame(frame);

    // After transmission, remove frame from queue
    Logger_tail = (Logger_tail + 1) % Logger_QUEUE_LENGTH;
    Logger_count--;

    if (Logger_DecrementPending()) {
        // No more pending samples - clear ready signal
        Logger_SetReady(false);
    }
}

// Get number of pending frames in queue (for diagnostics)
uint32_t Logger_GetPendingCount(void) {
    return Logger_count;
}

// Get number of dropped samples (for diagnostics)
uint32_t Logger_GetDroppedCount(void) {
    return Logger_dropped_samples;
}
