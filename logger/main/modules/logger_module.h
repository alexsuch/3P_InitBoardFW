#pragma once
#include "app_state.h"
#include "config_manager.h"
#include "esp_attr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "hal/sdcard.h"
#include "logger_frame_link.h"
#include "target.h"

// ADD_DROP_COUNTERS is now a runtime configuration option

// #define LOGGER_BUFFER_SIZE 32768
#define LOGGER_BUFFER_SIZE 16384

typedef struct {
    uint8_t *data;
    size_t size;
} logger_chunk_t;

typedef struct logger_module_s {
    // Dynamic buffers (allocated on demand)
    uint8_t *ping_buffer;  // Dynamically allocated
    uint8_t *pong_buffer;  // Dynamically allocated
    uint8_t *active_buffer;
    volatile size_t active_buffer_idx;

    // Dynamic allocation tracking
    size_t buffer_size;      // Current buffer size
    bool buffers_allocated;  // Allocation status

    QueueHandle_t buffer_queue;   // Queue of chunks to write to SD
    TaskHandle_t writer_task_handle;  // SD card writer task
    sdcard_file_handle_t log_file;

    bool sd_card_ok;
    bool initialized;
    volatile bool shutdown_requested;  // Flag to signal tasks to exit gracefully

    uint64_t dropped_chunks;
    uint64_t dropped_queue_full;
    uint64_t last_drop_stats_time_us;

    // SD card diagnostics and recovery
    char log_file_path[96];
    uint64_t sd_write_failures;
    uint64_t sd_recoveries;

    // STM32 config received via SPI, written to log file header
    logger_config_t stm_config;
    bool stm_config_received;
} logger_module_t;

// Static logger module instance to avoid heap fragmentation
extern logger_module_t g_logger_module;

hal_err_t logger_module_init(logger_module_t *module, bool is_sd_card_ok);
esp_err_t logger_module_create_task(logger_module_t *module);

void logger_log_drop_statistics(logger_module_t *module);

// Force-flush active buffer to the writer queue, wait until it's drained, then fsync()
// max_wait – максимальний час очікування спорожнення черги запису.
void logger_module_flush_and_sync(logger_module_t *module, TickType_t max_wait);

// Dynamic allocation functions
hal_err_t logger_module_allocate_buffers(logger_module_t *module, size_t buffer_size);
hal_err_t logger_module_deallocate_buffers(logger_module_t *module);
// Reallocate buffers - REQUIRES tasks to be stopped first to prevent use-after-free
hal_err_t logger_module_reallocate_buffers(logger_module_t *module, size_t new_size);
hal_err_t logger_module_cleanup(logger_module_t *module);

// Convenience functions for static instance
hal_err_t logger_module_init_static(bool is_sd_card_ok);
esp_err_t logger_module_create_task_static(void);

hal_err_t logger_module_stop_and_free_memory_static(void);

// -- new: graceful stop for SD writes --
void logger_module_quiesce_and_fsync(logger_module_t *module);
void logger_module_set_sd_enabled(logger_module_t *module, bool enabled);

// Graceful stop: force-flush active buffer, wait queues drain, fsync/close file if треба
// timeout_ticks - скільки максимум чекати на спорожнення черг
void logger_module_quiesce_and_flush(logger_module_t *module, TickType_t timeout_ticks);

// Поновити роботу логера (створює таски, відкриває файл заново, тощо).
// Викликається після повернення з MSC.
hal_err_t logger_module_resume(logger_module_t *module);

// Stop logging, flush buffers, and free memory for WiFi mode
hal_err_t logger_module_stop_and_free_memory(logger_module_t *module);

// Set STM32 config and write 64-byte header to the beginning of the log file
void logger_module_set_stm_config(logger_module_t *module, const logger_config_t *config);

// Write a complete STM32 logger frame (852 bytes) to the log file
void logger_module_write_frame(logger_module_t *module, const logger_frame_t *frame);
