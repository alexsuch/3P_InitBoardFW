#ifndef SHARED_SPI_LOGGER_H
#define SHARED_SPI_LOGGER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define Logger_FRAME_MAGIC         0xAACC5533U
#define Logger_FRAME_SIZE_BYTES    50U
#define Logger_QUEUE_LENGTH        8U

// Structure for a single logger frame (matches accel_link_frame_t)
typedef struct __attribute__((packed)) Logger_frame_t {
    uint32_t magic;
    uint32_t seq;
    uint64_t timestamp_us;
    int16_t accel[3];
    int16_t gyro[3];
    int16_t temp;
    uint8_t reserved[Logger_FRAME_SIZE_BYTES - 4 - 4 - 8 - 3*2 - 3*2 - 2];
} Logger_frame_t;

// Sample structure for publishing (user must fill fields)
typedef struct {
    int16_t accel[3];
    int16_t gyro[3];
    int16_t temp;
} Logger_sample_t;

// Core API (called from App_InitRun and App_Task)
// Call once from App_InitRun
void Logger_Init(void);
// Call from App_Task (main loop) - for diagnostics/monitoring
void Logger_Task(void);
// Call to queue a new sample for transmission
void Logger_PublishSample(const Logger_sample_t* sample);
// SPI slave RX callback - called by SPI interrupt when master reads
void Logger_SPI_RxCallback(void);

// Board-specific weak functions (override in solution_hal_cfg.c)
// Control SPI_DATA_RDY GPIO (ready signal to master)
void Logger_GPIO_SetReady(bool ready);
// Transmit frame via SPI2 slave (called from interrupt handler)
void Logger_SPI_TransmitFrame(const Logger_frame_t* frame);
// Initialize SPI2 NVIC for slave interrupt
void Logger_SPI_Init(void);

// Diagnostic functions
uint32_t Logger_GetPendingCount(void);
uint32_t Logger_GetDroppedCount(void);

#endif // SHARED_SPI_LOGGER_H
