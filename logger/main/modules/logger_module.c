#include "logger_module.h"

#include <esp_heap_caps.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "app_state.h"
#include "build_info.h"
#include "app_commands.h"
#include "config_manager.h"
#include "log.h"
#include "platform/system.h"

#include "target.h"

static const char *TAG = "LOGR";

// Static assert to ensure stack size is properly aligned for FreeRTOS
_Static_assert((TASK_STACK_SIZE_LOGGER % sizeof(StackType_t)) == 0, "TASK_STACK_SIZE_LOGGER must be expressed in bytes and aligned to StackType_t");

// Static logger module instance to avoid heap fragmentation
logger_module_t g_logger_module;

extern const char *sdcard_get_mount_point(void);  // з вашого hal/sdcard.h

// Timeout for forced buffer flushing when no new data arrives (in milliseconds)
#define LOG_FLUSH_TIMEOUT_MS 500
// Interval for periodic diagnostic statistics written into the log file.
// If DIAGNOSTIC_IN_LOG_PERIOD_SEC is defined in target.h, use it; otherwise fall back to 30 seconds.
#ifdef DIAGNOSTIC_IN_LOG_PERIOD_SEC
#define DROPPED_CHUNKS_LOG_INTERVAL_MS (DIAGNOSTIC_IN_LOG_PERIOD_SEC * 1000U)
#else
#define DROPPED_CHUNKS_LOG_INTERVAL_MS 30000U
#endif
#define LOGGER_BUFFER_QUEUE_LENGTH 2U

// CSV/text helpers removed; binary logging only
static hal_err_t copy_config_file(const char *dest_path);
static void log_memory_status(const char *context);
static bool check_queue_creation_requirements(size_t queue_length, size_t item_size, const char *queue_name);
static void logger_log_drop_statistics_internal(logger_module_t *module);
static void logger_handle_sd_write_error(logger_module_t *module, int written, size_t expected);
static void logger_handle_sd_sync_error(logger_module_t *module);

// -------------------- helpers for buffering --------------------

static inline void IRAM_ATTR logger_flush_active_buffer(logger_module_t *m) {
    if (!m->sd_card_ok || m->active_buffer_idx == 0) return;
    logger_chunk_t chunk = {
        .data = m->active_buffer,
        .size = m->active_buffer_idx,
    };

    if (xQueueSend(m->buffer_queue, &chunk, pdMS_TO_TICKS(10)) != pdTRUE) {
        // drop: черга забита -> не блокуємось
        ++m->dropped_queue_full;
    }

    // xQueueSend(m->buffer_queue, &chunk, portMAX_DELAY);

    m->active_buffer = (m->active_buffer == m->ping_buffer) ? m->pong_buffer : m->ping_buffer;
    m->active_buffer_idx = 0;
}

static inline void IRAM_ATTR logger_append_bytes(logger_module_t *m, const uint8_t *data, size_t len) {
    if (!m->sd_card_ok || len == 0) return;
    if (len > m->buffer_size) {
        // дуже великий шматок — пишемо порціями
        size_t off = 0;
        while (off < len) {
            size_t chunk = len - off;
            if (chunk > m->buffer_size) chunk = m->buffer_size;
            memcpy(m->active_buffer, data + off, chunk);
            m->active_buffer_idx = chunk;
            logger_flush_active_buffer(m);
            off += chunk;
        }
        return;
    }
    if (m->active_buffer_idx + len > m->buffer_size) {
        logger_flush_active_buffer(m);
    }
    memcpy(m->active_buffer + m->active_buffer_idx, data, len);
    m->active_buffer_idx += len;
}

static inline void IRAM_ATTR logger_append_crlf(logger_module_t *m) {
    static const uint8_t crlf[2] = {'\r', '\n'};
    logger_append_bytes(m, crlf, 2);
}

static void logger_log_drop_statistics_internal(logger_module_t *module) {
    if (!module->sd_card_ok) return;

    // Create CSV comment line with diagnostic statistics
    char stats_line[128];
    int len = snprintf(stats_line, sizeof(stats_line), "# stats: dropped-chunks=%llu, dropped-queues=%llu, sd_failures=%llu, sd_recoveries=%llu\r\n",
                       (unsigned long long)module->dropped_chunks, (unsigned long long)module->dropped_queue_full,
                       (unsigned long long)module->sd_write_failures, (unsigned long long)module->sd_recoveries);

    if (len > 0 && len < (int)sizeof(stats_line)) {
        logger_append_bytes(module, (const uint8_t *)stats_line, len);
    }
}

void logger_log_drop_statistics(logger_module_t *module) { logger_log_drop_statistics_internal(module); }

void logger_module_reset_stats(logger_module_t *module) {
    if (!module) {
        return;
    }

    module->dropped_chunks = 0;
    module->dropped_queue_full = 0;
    module->sd_write_failures = 0;
    module->sd_recoveries = 0;
    module->last_drop_stats_time_us = esp_timer_get_time();
}

// -------------------- Task for writing to SD --------------------

static void IRAM_ATTR logger_sd_write_task(void *arg) {
    logger_module_t *module = (logger_module_t *)arg;

    if (module->buffer_queue == NULL) {
        LOG_E(TAG, "Logger task started with NULL buffer queue. Task exiting.");
        vTaskDelete(NULL);
        return;
    }

    logger_chunk_t chunk;
    int chunks_since_sync = 0;
    size_t bytes_since_sync = 0;
    while (!module->shutdown_requested) {
        // Use timeout instead of portMAX_DELAY to allow checking shutdown flag
        if (xQueueReceive(module->buffer_queue, &chunk, pdMS_TO_TICKS(100)) == pdTRUE) {
            if (module->sd_card_ok && chunk.size > 0) {
                uint64_t write_start_us = esp_timer_get_time();
                int written = sdcard_write(module->log_file, chunk.data, chunk.size);
                uint64_t write_dt_us = esp_timer_get_time() - write_start_us;

                if (written != (int)chunk.size) {
                    logger_handle_sd_write_error(module, written, chunk.size);
                    if (!module->sd_card_ok) {
                        // SD is in error state, skip sync for this chunk
                        continue;
                    }
                }
                if (written > 0) {
                    bytes_since_sync += (size_t)written;
                }
#ifdef DEBUG_LOGGER_WRITES
                // Comment it for production
                uint32_t write_kb_per_sec = 0;
                if (write_dt_us > 0 && written > 0) {
                    uint64_t bytes_per_sec = ((uint64_t)written * 1000000ULL) / write_dt_us;
                    write_kb_per_sec = (uint32_t)(bytes_per_sec / 1024ULL);
                }
                // LOG_I(TAG, "Write %d (%llu us, %u kB/s), dropped chunks %llu, dropped queue full %llu, sd_failures %llu", written,
                //       (unsigned long long)write_dt_us, write_kb_per_sec, (unsigned long long)module->dropped_chunks,
                //       (unsigned long long)module->dropped_queue_full, (unsigned long long)module->sd_write_failures);
#endif
                if (++chunks_since_sync >= 8) {
#ifdef DEBUG_LOGGER_WRITES
                    // Comment it for production
                    LOG_D(TAG, "Sync");
#endif
                    uint64_t sync_start_us = esp_timer_get_time();
                    fflush((FILE *)module->log_file);
                    if (sdcard_fsync(module->log_file) != HAL_ERR_NONE) {
                        logger_handle_sd_sync_error(module);
                        if (!module->sd_card_ok) {
                            // SD is in error state, writer will attempt recovery on next chunk
                            chunks_since_sync = 0;
                            continue;
                        }
                    }
#ifdef DEBUG_LOGGER_WRITES
                    uint64_t sync_dt_us = esp_timer_get_time() - sync_start_us;
                    uint32_t sync_kb_per_sec = 0;
                    if (sync_dt_us > 0 && bytes_since_sync > 0) {
                        uint64_t bytes_per_sec = ((uint64_t)bytes_since_sync * 1000000ULL) / sync_dt_us;
                        sync_kb_per_sec = (uint32_t)(bytes_per_sec / 1024ULL);
                    }
                    size_t free_heap = esp_get_free_heap_size();
                    LOG_I(TAG, "Sync done in %llu us, %u kB/s (total %u bytes, free_heap=%u bytes)", (unsigned long long)sync_dt_us, sync_kb_per_sec,
                          (unsigned)bytes_since_sync, (unsigned)free_heap);
                    LOG_I(TAG, "Dropped chunks %llu, dropped queue full %llu, sd_failures %llu", (unsigned long long)module->dropped_chunks,
                      (unsigned long long)module->dropped_queue_full, (unsigned long long)module->sd_write_failures);
#else
                    //TODO
#endif
                    chunks_since_sync = 0;
                    bytes_since_sync = 0;
                    vTaskDelay(1);  // yield
                }
            }
        }
    }

    // Task is exiting gracefully
    LOG_I(TAG, "Writer task exiting gracefully");
    module->writer_task_handle = NULL;
    vTaskDelete(NULL);
}

static void force_flush_active_buffer(logger_module_t *m) {
    // Мікро-репліка внутрішнього "flush" без зміни існуючих статичних функцій
    size_t sz = m->active_buffer_idx;
    if (sz == 0 || m->buffer_queue == NULL) return;
    logger_chunk_t chunk = {.data = m->active_buffer, .size = sz};
    if (xQueueSend(m->buffer_queue, &chunk, pdMS_TO_TICKS(100)) != pdTRUE) {
        ++m->dropped_queue_full;
        LOG_W(TAG, "buffer_queue full while force flush");
    }
    // переключаємо буфер (ping/pong)
    m->active_buffer = (m->active_buffer == m->ping_buffer) ? m->pong_buffer : m->ping_buffer;
    m->active_buffer_idx = 0;
}

void logger_module_quiesce_and_flush(logger_module_t *m, TickType_t timeout_ticks) {
    if (!m) return;
    // 1) форс-флаш активного буфера
    force_flush_active_buffer(m);
    // 2) дочекаємось спорожнення черг
    TickType_t start = xTaskGetTickCount();
    for (;;) {
        UBaseType_t q_buf = m->buffer_queue ? uxQueueMessagesWaiting(m->buffer_queue) : 0;
        if (q_buf == 0 && m->active_buffer_idx == 0) break;
        if ((xTaskGetTickCount() - start) > timeout_ticks) {
            LOG_W(TAG, "Timeout waiting for queues drain: buf=%u, idx=%u", (unsigned)q_buf, (unsigned)m->active_buffer_idx);
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    // 3) остаточний fflush/fsync — writer і так синкає кожні N шматків, але ми гарантуємо ще раз
    if (m->sd_card_ok && m->log_file) {
        fflush((FILE *)m->log_file);
        if (sdcard_fsync(m->log_file) != HAL_ERR_NONE) {
            logger_handle_sd_sync_error(m);
        }
    }
}

static void logger_handle_sd_write_error(logger_module_t *module, int written, size_t expected) {
    if (!module) {
        return;
    }

    module->sd_write_failures++;
    LOG_E(TAG, "SD write error: wrote %d of %d bytes (total failures=%llu)", written, (int)expected, (unsigned long long)module->sd_write_failures);

    if (!module->sd_card_ok) {
        // Recovery already attempted, keep card disabled.
        return;
    }

    module->sd_card_ok = false;

    if (module->log_file) {
        sdcard_close_file(module->log_file);
        module->log_file = NULL;
    }

    hal_err_t unmount_err = sdcard_unmount();
    if (unmount_err != HAL_ERR_NONE) {
        LOG_W(TAG, "SD unmount during recovery failed (err=%d)", unmount_err);
    }

    vTaskDelay(pdMS_TO_TICKS(50));

    hal_err_t mount_err = sdcard_mount();
    if (mount_err != HAL_ERR_NONE) {
        LOG_E(TAG, "SD remount failed after write error (err=%d)", mount_err);
        return;
    }

    if (module->log_file_path[0] != '\0') {
        hal_err_t open_err = sdcard_open_file(module->log_file_path, "a", &module->log_file);
        if (open_err != HAL_ERR_NONE) {
            LOG_E(TAG, "Failed to reopen log file %s after write error (err=%d)", module->log_file_path, open_err);
            return;
        }
        setvbuf((FILE *)module->log_file, NULL, _IONBF, 0);
    }

    module->sd_card_ok = true;
    module->sd_recoveries++;
    LOG_W(TAG, "SD card recovered, logging resumed (recoveries=%llu)", (unsigned long long)module->sd_recoveries);
}

static void logger_handle_sd_sync_error(logger_module_t *module) {
    LOG_E(TAG, "SD sync error (fsync failed), attempting recovery");
    logger_handle_sd_write_error(module, -1, 0);
}

// Common graceful shutdown logic for logger module cleanup
static hal_err_t logger_module_graceful_shutdown(logger_module_t *module, const char *context) {
    if (!module) {
        LOG_E(TAG, "Invalid module pointer");
        return HAL_ERR_FAILED;
    }

    // MEMORY DEBUG: Track total cleanup
    size_t heap_at_start = esp_get_free_heap_size();
    LOG_I(TAG, "======= LOGGER GRACEFUL SHUTDOWN =======");
    LOG_I(TAG, "Context: %s", context);
    LOG_I(TAG, "Free heap at START: %zu bytes (%.2f KB)", heap_at_start, heap_at_start / 1024.0f);
    LOG_I(TAG, "Logger state - initialized: %s, sd_ok: %s", module->initialized ? "YES" : "NO", module->sd_card_ok ? "YES" : "NO");
    LOG_I(TAG, "========================================");

    // 1. Signal tasks to exit gracefully
    module->shutdown_requested = true;
    LOG_I(TAG, "Shutdown signal sent to tasks");

    // 2. Stop logging by setting mode to IDLE
    app_state_t *state = app_state_get_instance();
    if (state) {
        app_state_begin_update();
        app_state_set_u32(APP_STATE_FIELD_CURRENT_MODE, (uint32_t *)&state->current_mode, APP_MODE_IDLE);
        app_state_end_update();
        LOG_I(TAG, "Logger mode set to IDLE");
    }

    // 3. Flush and sync all buffers to ensure data is written to SD card
    if (module->initialized && module->sd_card_ok) {
        LOG_I(TAG, "Flushing logger buffers to SD card...");
        logger_module_quiesce_and_flush(module, pdMS_TO_TICKS(3000));  // 3 second timeout
        LOG_I(TAG, "Logger buffers flushed successfully");
    }

    // 4. Sentinel not needed - no processing task

    // 5. Wait for writer task to exit naturally
    TickType_t wait_start = xTaskGetTickCount();
    int consecutive_null_checks = 0;
    const int REQUIRED_NULL_CHECKS = 10;  // Require 10 consecutive checks (100ms) with NULL handles

    while (module->writer_task_handle && (xTaskGetTickCount() - wait_start) < pdMS_TO_TICKS(5000)) {
        vTaskDelay(pdMS_TO_TICKS(10));
        // Verify handle is actually NULL (task has exited)
        if (!module->writer_task_handle) {
            consecutive_null_checks++;
            if (consecutive_null_checks >= REQUIRED_NULL_CHECKS) {
                LOG_I(TAG, "Task confirmed exited after %d checks", consecutive_null_checks);
                break;
            }
        } else {
            consecutive_null_checks = 0;  // Reset if handle is still non-NULL
        }
    }

    // Additional safety delay to ensure task has completely stopped accessing buffers
    LOG_I(TAG, "Waiting additional 200ms to ensure all buffer operations have completed");
    vTaskDelay(pdMS_TO_TICKS(200));

    // 6. Only delete writer task if it hasn't exited (last resort)

    if (module->writer_task_handle) {
        LOG_W(TAG, "Force deleting writer task");
        TaskHandle_t task_to_delete = module->writer_task_handle;
        module->writer_task_handle = NULL;  // Clear handle immediately
        vTaskDelete(task_to_delete);
        // Give idle task time to clean up TCB and stack memory with heap monitoring
        size_t heap_before = esp_get_free_heap_size();
        for (int i = 0; i < 20; i++) {
            vTaskDelay(pdMS_TO_TICKS(10));
            size_t heap_now = esp_get_free_heap_size();
            if (heap_now > heap_before + 1024) {
                break;  // Task cleanup appears complete
            }
        }
        LOG_I(TAG, "Logger writer task deleted");
    }

    // 7. Close log file
    if (module->log_file) {
        sdcard_close_file(module->log_file);
        module->log_file = NULL;
        LOG_I(TAG, "Log file closed");
    }

    // 8. Delete queues - MEMORY DEBUG
    // CRITICAL: Ensure queues are completely drained before deletion
    // The buffer_queue contains pointers to ping/pong buffers, so it must be empty
    LOG_I(TAG, "=== QUEUE CLEANUP DEBUG ===");
    size_t heap_before_queues = esp_get_free_heap_size();
    LOG_I(TAG, "Free heap before queue cleanup: %zu bytes (%.2f KB)", heap_before_queues, heap_before_queues / 1024.0f);

    // Verify queues are empty before deletion
    if (module->buffer_queue) {
        UBaseType_t buffer_queue_items = uxQueueMessagesWaiting(module->buffer_queue);
        if (buffer_queue_items > 0) {
            LOG_W(TAG, "WARNING: buffer_queue still has %u items! Draining...", buffer_queue_items);
            logger_chunk_t dummy_chunk;
            while (xQueueReceive(module->buffer_queue, &dummy_chunk, 0) == pdTRUE) {
                buffer_queue_items--;
            }
            LOG_I(TAG, "Drained %u remaining items from buffer_queue", buffer_queue_items);
            vTaskDelay(pdMS_TO_TICKS(50));  // Give writer task time to finish any in-progress operations
        }
    }

    // Data queue not used in raw frame mode

    if (module->buffer_queue) {
        LOG_I(TAG, "Deleting buffer_queue (2 items × 8 bytes = ~16 bytes)");
        size_t heap_before_buffer = esp_get_free_heap_size();
        vQueueDelete(module->buffer_queue);
        module->buffer_queue = NULL;
        size_t heap_after_buffer = esp_get_free_heap_size();
        LOG_I(TAG, "Heap after buffer queue delete: %zu bytes (+%zu freed)", heap_after_buffer, heap_after_buffer - heap_before_buffer);
    } else {
        LOG_W(TAG, "buffer_queue was NULL - already deleted?");
    }

    size_t heap_after_queues = esp_get_free_heap_size();
    size_t total_queue_freed = heap_after_queues - heap_before_queues;
    LOG_I(TAG, "Total queue memory freed: %zu bytes (%.2f KB)", total_queue_freed, total_queue_freed / 1024.0f);
    LOG_I(TAG, "===========================");

    // 9. Final safety check: ensure no tasks are running before freeing buffers
    if (module->writer_task_handle) {
        LOG_E(TAG, "CRITICAL: Attempting to free buffers while writer task is still running!");
        // Don't free buffers if task is still running - this would cause heap corruption
        return HAL_ERR_FAILED;
    }

    // Additional delay to ensure all memory operations have completed
    vTaskDelay(pdMS_TO_TICKS(100));

    // 10. Deallocate buffers (now safe - tasks are confirmed stopped)
    logger_module_deallocate_buffers(module);

    // 11. Reset module state
    module->initialized = false;
    module->sd_card_ok = false;
    module->active_buffer = NULL;
    module->active_buffer_idx = 0;
    module->shutdown_requested = false;

    // MEMORY DEBUG: Final summary
    size_t heap_at_end = esp_get_free_heap_size();
    size_t total_memory_freed = heap_at_end - heap_at_start;
    LOG_I(TAG, "======= SHUTDOWN SUMMARY =======");
    LOG_I(TAG, "Free heap at END: %zu bytes (%.2f KB)", heap_at_end, heap_at_end / 1024.0f);
    LOG_I(TAG, "TOTAL MEMORY FREED: %zu bytes (%.2f KB)", total_memory_freed, total_memory_freed / 1024.0f);
    LOG_I(TAG, "Expected to free: ~43KB (32KB buffers + 10KB queue + 1KB header)");
    if (total_memory_freed >= 30000) {
        LOG_I(TAG, "✅ SUCCESS: Significant memory freed (%zu KB)", total_memory_freed / 1024);
    } else if (total_memory_freed >= 10000) {
        LOG_W(TAG, "⚠️  PARTIAL: Some memory freed (%zu KB) but less than expected", total_memory_freed / 1024);
    } else {
        LOG_E(TAG, "❌ FAILED: Minimal memory freed (%zu KB) - buffers may not have been allocated", total_memory_freed / 1024);
    }
    LOG_I(TAG, "Graceful shutdown completed: %s", context);
    LOG_I(TAG, "===============================");
    return HAL_ERR_NONE;
}

hal_err_t logger_module_resume(logger_module_t *m) {
    if (!m) return HAL_ERR_FAILED;
    // якщо таска вже жива — нічого не робимо
    if (m->writer_task_handle) return HAL_ERR_NONE;
    // переконуємось, що файл відкритий (logger_module_init робить open+CSV header)
    if (!m->initialized) {
        LOG_W(TAG, "Logger not initialized, calling init_static again");
        if (logger_module_init_static(true) != HAL_ERR_NONE) return HAL_ERR_FAILED;
    }
    logger_module_create_task(m);
    return HAL_ERR_NONE;
}

hal_err_t logger_module_stop_and_free_memory(logger_module_t *module) { return logger_module_graceful_shutdown(module, "WiFi mode memory cleanup"); }

// -------------------- Initialization --------------------

static hal_err_t copy_config_file(const char *dest_path) {
    FILE *src = fopen(SD_MOUNT_PATH "/configuration.ini", "r");
    if (!src) {
        return HAL_ERR_FAILED;
    }
    FILE *dst = fopen(dest_path, "w");
    if (!dst) {
        fclose(src);
        return HAL_ERR_FAILED;
    }
    char buffer[128];
    size_t bytes;
    while ((bytes = fread(buffer, 1, sizeof(buffer), src)) > 0) {
        fwrite(buffer, 1, bytes, dst);
    }
    fclose(src);
    fclose(dst);
    return HAL_ERR_NONE;
}

static void log_memory_status(const char *context) {
    size_t free_heap = esp_get_free_heap_size();
    size_t min_free_heap = esp_get_minimum_free_heap_size();
    size_t total_heap = heap_caps_get_total_size(MALLOC_CAP_8BIT);
    size_t free_internal = heap_caps_get_free_size(MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
    size_t free_spiram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);

    LOG_W(TAG, "Memory status at %s:", context);
    LOG_W(TAG, "  Free heap: %zu bytes", free_heap);
    LOG_W(TAG, "  Min free heap: %zu bytes", min_free_heap);
    LOG_W(TAG, "  Total heap: %zu bytes", total_heap);
    LOG_W(TAG, "  Free internal: %zu bytes", free_internal);
    LOG_W(TAG, "  Free SPIRAM: %zu bytes", free_spiram);
    LOG_W(TAG, "  Heap fragmentation: %.1f%%", (float)(total_heap - free_heap) / total_heap * 100.0f);
}

// Memory allocation failure hook - called when malloc fails
void vApplicationMallocFailedHook(void) {
    LOG_E(TAG, "CRITICAL: Memory allocation failed! System may be unstable.");
    log_memory_status("malloc failure");

    // Log additional system information
    LOG_E(TAG, "System information:");
    LOG_E(TAG, "  Uptime: %llu ms", esp_timer_get_time() / 1000);
    LOG_E(TAG, "  Reset reason: %d", esp_reset_reason());

    // Try to continue but log the critical error
    // In production, you might want to restart the system here
}

static bool check_queue_creation_requirements(size_t queue_length, size_t item_size, const char *queue_name) {
    size_t required_memory = queue_length * item_size;
    size_t free_heap = esp_get_free_heap_size();
    size_t min_free_heap = esp_get_minimum_free_heap_size();

    LOG_I(TAG, "Queue creation requirements for %s:", queue_name);
    LOG_I(TAG, "  Queue length: %zu items", queue_length);
    LOG_I(TAG, "  Item size: %zu bytes", item_size);
    LOG_I(TAG, "  Required memory: %zu bytes", required_memory);
    LOG_I(TAG, "  Available heap: %zu bytes", free_heap);
    LOG_I(TAG, "  Min free heap: %zu bytes", min_free_heap);

    if (required_memory > free_heap) {
        LOG_E(TAG, "Insufficient memory for %s: need %zu bytes, have %zu bytes", queue_name, required_memory, free_heap);
        return false;
    }

    if (free_heap < required_memory * 2) {
        LOG_W(TAG, "Low memory warning for %s: free heap (%zu) is less than 2x minimum (%zu)", queue_name, free_heap, required_memory);
    }

    return true;
}

hal_err_t logger_module_init(logger_module_t *module, bool is_sd_card_ok) {
    memset(module, 0, sizeof(logger_module_t));

    module->sd_card_ok = is_sd_card_ok;
    module->shutdown_requested = false;
    if (!module->sd_card_ok) {
        LOG_W(TAG, "SD card not available. Logger writes disabled, but monitoring active.");
        // Do not fail here - allow module to initialize so STM32 config can be received
    } else {
        // Allocate buffers dynamically only if SD card is present
        hal_err_t buffer_err = logger_module_allocate_buffers(module, LOGGER_BUFFER_SIZE);
        if (buffer_err != HAL_ERR_NONE) {
            LOG_E(TAG, "Failed to allocate logger buffers");
            return buffer_err;
        }

        // Queue for chunks to write to SD - create now so tasks can be started
        log_memory_status("before buffer queue creation");
        if (!check_queue_creation_requirements(LOGGER_BUFFER_QUEUE_LENGTH, sizeof(logger_chunk_t), "buffer queue")) {
            LOG_E(TAG, "Buffer queue creation requirements not met. Logger disabled.");
            module->sd_card_ok = false;
            // Continue initialization without SD features
        } else {
            module->buffer_queue = xQueueCreate(LOGGER_BUFFER_QUEUE_LENGTH, sizeof(logger_chunk_t));
            if (module->buffer_queue == NULL) {
                LOG_E(TAG, "Failed to create buffer queue (%u items, %zu bytes each). Logger disabled.", (unsigned)LOGGER_BUFFER_QUEUE_LENGTH, sizeof(logger_chunk_t));
                log_memory_status("after buffer queue creation failure");
                module->sd_card_ok = false;
                // Continue initialization without SD features
            } else {
                LOG_I(TAG, "Buffer queue created successfully (%u items, %zu bytes each)", (unsigned)LOGGER_BUFFER_QUEUE_LENGTH, sizeof(logger_chunk_t));
            }
        }
    }

    if (module->sd_card_ok) {
        module->active_buffer = module->ping_buffer;
        module->active_buffer_idx = 0;
    }

    module->last_drop_stats_time_us = esp_timer_get_time();

    // NOTE: Folder and file are NOT created here.
    // Call logger_module_start_session() when logging actually starts.
    LOG_I(TAG, "Logger module initialized (buffers ready: %s, session: none).", module->sd_card_ok ? "YES" : "NO");
    module->initialized = true;
    return HAL_ERR_NONE;
}

hal_err_t logger_module_start_session(logger_module_t *module) {
    if (!module) {
        LOG_E(TAG, "Invalid module pointer");
        return HAL_ERR_FAILED;
    }
    if (!module->initialized) {
        LOG_E(TAG, "Logger module not initialized");
        return HAL_ERR_FAILED;
    }
    if (!module->sd_card_ok) {
        LOG_E(TAG, "SD card not available");
        return HAL_ERR_FAILED;
    }
    // If a session is already active (file open), do nothing
    if (module->log_file != NULL) {
        LOG_W(TAG, "Logging session already active, file: %s", module->log_file_path);
        return HAL_ERR_NONE;
    }

    // Temporary string buffers used during session start
    char log_dir_path[64];
    char config_path[96];
    char log_file_path[96];

    // Find first free folder index: /N
    int log_num = 0;
    struct stat st;
    while (1) {
        snprintf(log_dir_path, sizeof(log_dir_path), SD_MOUNT_PATH "/%d", log_num);
        if (stat(log_dir_path, &st) != 0) break;
        log_num++;
    }

    if (mkdir(log_dir_path, 0755) != 0) {
        LOG_E(TAG, "Failed to create log directory %s", log_dir_path);
        return HAL_ERR_FAILED;
    }

    // Copy config file to log folder
    snprintf(config_path, sizeof(config_path), "%s/configuration.ini", log_dir_path);
    copy_config_file(config_path);

    snprintf(log_file_path, sizeof(log_file_path), "%s/data_%d.dat", log_dir_path, log_num);
    strncpy(module->log_file_path, log_file_path, sizeof(module->log_file_path) - 1);
    module->log_file_path[sizeof(module->log_file_path) - 1] = '\0';

    hal_err_t err = sdcard_open_file(log_file_path, "wb", &module->log_file);
    if (err != HAL_ERR_NONE) {
        LOG_E(TAG, "Failed to create log file %s", log_file_path);
        return err;
    }

    setvbuf((FILE *)module->log_file, NULL, _IONBF, 0);

    LOG_I(TAG, "Logging session started. Binary data to %s", log_file_path);
    return HAL_ERR_NONE;
}

esp_err_t logger_module_create_task(logger_module_t *module) {
    module->writer_task_handle = NULL;

    // Only create the SD writer task - LOGGER role writes raw 852-byte frames directly
    BaseType_t status = xTaskCreatePinnedToCore(logger_sd_write_task, "LOG_WRITE", TASK_STACK_SIZE_LOGGER, module, TASK_PRIORITY_LOGGER - 1,
                                                 &module->writer_task_handle, CORE_LOGGER_WRITE);
    if (status != pdPASS) {
        LOG_E(TAG, "Failed to create LOG_WRITE task! Memory may be insufficient.");
        module->writer_task_handle = NULL;
        return ESP_FAIL;
    }

    LOG_I(TAG, "Logger writer task created successfully (raw frame mode).");
    return ESP_OK;
}

void logger_module_flush_and_sync(logger_module_t *module, TickType_t max_wait) {
    if (!module || !module->sd_card_ok) return;
    // 1) скинемо активний буфер у queue writer-а
    logger_flush_active_buffer(module);  // внутрішня inline-функція
    // 2) дочекаємось, поки writer-черга спорожніє (або тайм-аут)
    TickType_t start = xTaskGetTickCount();
    while (uxQueueMessagesWaiting(module->buffer_queue) > 0) {
        if ((xTaskGetTickCount() - start) >= max_wait) break;
        vTaskDelay(1);
    }
    // 3) остаточний sync на файл
    if (module->log_file) {
        fflush((FILE *)module->log_file);
        sdcard_fsync(module->log_file);
    }
}

// -------------------- Quiesce helpers (for MSC) --------------------
void logger_module_set_sd_enabled(logger_module_t *module, bool enabled) { module->sd_card_ok = enabled; }

void logger_module_quiesce_and_fsync(logger_module_t *m) {
    if (!m || !m->initialized) return;

    // Виштовхнути активний буфер, якщо є дані
    if (m->active_buffer_idx > 0) {
        logger_flush_active_buffer(m);  // внутрішня функція цього файлу
    }
    // Дочекатися, поки черга спорожниться
    for (;;) {
        UBaseType_t qb = m->buffer_queue ? uxQueueMessagesWaiting(m->buffer_queue) : 0;
        if (m->active_buffer_idx == 0 && qb == 0) break;
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    // Фінальний sync
    if (m->log_file) {
        fflush((FILE *)m->log_file);
        sdcard_fsync(m->log_file);
    }
}

// Dynamic allocation functions
hal_err_t logger_module_allocate_buffers(logger_module_t *module, size_t buffer_size) {
    if (!module) {
        LOG_E(TAG, "Invalid module pointer");
        return HAL_ERR_FAILED;
    }

    if (module->buffers_allocated) {
        LOG_W(TAG, "Buffers already allocated");
        return HAL_ERR_NONE;
    }

    LOG_I(TAG, "Allocating logger buffers (%zu bytes each)", buffer_size);

    // Allocate ping buffer
    module->ping_buffer = malloc(buffer_size);
    if (!module->ping_buffer) {
        LOG_E(TAG, "Failed to allocate ping buffer (%zu bytes)", buffer_size);
        return HAL_ERR_FAILED;
    }

    // Allocate pong buffer
    module->pong_buffer = malloc(buffer_size);
    if (!module->pong_buffer) {
        LOG_E(TAG, "Failed to allocate pong buffer (%zu bytes)", buffer_size);
        free(module->ping_buffer);
        module->ping_buffer = NULL;
        return HAL_ERR_FAILED;
    }

    module->buffer_size = buffer_size;
    module->buffers_allocated = true;
    module->active_buffer = module->ping_buffer;
    module->active_buffer_idx = 0;

    LOG_I(TAG, "Logger buffers allocated successfully (%zu bytes each)", buffer_size);
    // Memory allocation completed
    return HAL_ERR_NONE;
}

hal_err_t logger_module_deallocate_buffers(logger_module_t *module) {
    if (!module) {
        LOG_E(TAG, "Invalid module pointer");
        return HAL_ERR_FAILED;
    }

    // MEMORY DEBUG: Log heap before cleanup
    size_t heap_before = esp_get_free_heap_size();
    LOG_I(TAG, "=== LOGGER BUFFER CLEANUP DEBUG ===");
    LOG_I(TAG, "Free heap BEFORE cleanup: %zu bytes (%.2f KB)", heap_before, heap_before / 1024.0f);
    LOG_I(TAG, "Buffers allocated flag: %s", module->buffers_allocated ? "TRUE" : "FALSE");
    LOG_I(TAG, "Buffer size: %zu bytes", module->buffer_size);

    if (!module->buffers_allocated) {
        LOG_W(TAG, "Buffers not allocated - no cleanup needed");
        LOG_I(TAG, "===================================");
        return HAL_ERR_NONE;
    }

    LOG_I(TAG, "Deallocating logger buffers (2×%zu + 1KB = %zu KB total)", module->buffer_size, (module->buffer_size * 2 + 1024) / 1024);

    // Deallocate ping buffer
    if (module->ping_buffer) {
        LOG_I(TAG, "Freeing ping_buffer (%zu bytes)", module->buffer_size);
        free(module->ping_buffer);
        module->ping_buffer = NULL;
        size_t heap_after_ping = esp_get_free_heap_size();
        LOG_I(TAG, "Heap after ping free: %zu bytes (+%zu freed)", heap_after_ping, heap_after_ping - heap_before);
    } else {
        LOG_W(TAG, "ping_buffer was NULL - already freed?");
    }

    // Deallocate pong buffer
    if (module->pong_buffer) {
        LOG_I(TAG, "Freeing pong_buffer (%zu bytes)", module->buffer_size);
        size_t heap_before_pong = esp_get_free_heap_size();
        free(module->pong_buffer);
        module->pong_buffer = NULL;
        size_t heap_after_pong = esp_get_free_heap_size();
        LOG_I(TAG, "Heap after pong free: %zu bytes (+%zu freed)", heap_after_pong, heap_after_pong - heap_before_pong);
    } else {
        LOG_W(TAG, "pong_buffer was NULL - already freed?");
    }

    module->buffers_allocated = false;
    module->buffer_size = 0;
    module->active_buffer = NULL;
    module->active_buffer_idx = 0;

    // MEMORY DEBUG: Log final heap status
    size_t heap_after = esp_get_free_heap_size();
    size_t total_freed = heap_after - heap_before;
    LOG_I(TAG, "Free heap AFTER cleanup: %zu bytes (%.2f KB)", heap_after, heap_after / 1024.0f);
    LOG_I(TAG, "Total memory freed: %zu bytes (%.2f KB)", total_freed, total_freed / 1024.0f);
    LOG_I(TAG, "Logger buffers deallocated successfully");
    LOG_I(TAG, "===================================");

    return HAL_ERR_NONE;
}

/**
 * Reallocate logger buffers with new size.
 *
 * CRITICAL: This function must only be called when logger tasks are stopped
 * to prevent use-after-free errors. The tasks (logger_processing_task and
 * logger_sd_write_task) access ping_buffer, pong_buffer, and active_buffer
 * concurrently, so reallocation while they're running will cause undefined behavior.
 *
 * Proper usage pattern:
 * 1. Call logger_module_graceful_shutdown() or logger_module_cleanup() to stop tasks
 * 2. Call this function to reallocate buffers
 * 3. Call logger_module_resume() to restart tasks with new buffers
 */
hal_err_t logger_module_reallocate_buffers(logger_module_t *module, size_t new_size) {
    if (!module) {
        LOG_E(TAG, "Invalid module pointer");
        return HAL_ERR_FAILED;
    }

    // Ensure task is not running to prevent use-after-free
    if (module->writer_task_handle) {
        LOG_E(TAG, "Cannot reallocate buffers while writer task is running. Task must be stopped first.");
        return HAL_ERR_FAILED;
    }

    if (!module->buffers_allocated) {
        LOG_W(TAG, "Buffers not allocated, allocating new buffers");
        return logger_module_allocate_buffers(module, new_size);
    }

    if (new_size == module->buffer_size) {
        LOG_I(TAG, "Buffer size unchanged (%zu bytes)", new_size);
        return HAL_ERR_NONE;
    }

    LOG_I(TAG, "Reallocating logger buffers from %zu to %zu bytes", module->buffer_size, new_size);

    // Deallocate current buffers
    hal_err_t err = logger_module_deallocate_buffers(module);
    if (err != HAL_ERR_NONE) {
        LOG_E(TAG, "Failed to deallocate current buffers");
        return err;
    }

    // Allocate new buffers
    return logger_module_allocate_buffers(module, new_size);
}

hal_err_t logger_module_cleanup(logger_module_t *module) { return logger_module_graceful_shutdown(module, "module cleanup"); }

// Convenience functions for static instance
hal_err_t logger_module_init_static(bool is_sd_card_ok) { return logger_module_init(&g_logger_module, is_sd_card_ok); }

esp_err_t logger_module_create_task_static(void) { return logger_module_create_task(&g_logger_module); }

hal_err_t logger_module_stop_and_free_memory_static(void) { return logger_module_stop_and_free_memory(&g_logger_module); }

static void logger_module_write_config_header_internal(logger_module_t *module, const logger_config_t *config) {
    if (!module || !config) {
        return;
    }

    if (!(module->sd_card_ok && module->log_file)) {
        return;
    }

    // Avoid concurrent writes to the same FILE* from the SD writer task.
    // This is especially important during session start/config sync when frames may already be streaming.
    const bool had_writer_task = (module->writer_task_handle != NULL);
    if (had_writer_task) {
        vTaskSuspend(module->writer_task_handle);
    }

    // Seek to the beginning of the file
    FILE *fp = (FILE *)module->log_file;
    long current_pos = ftell(fp);

    if (fseek(fp, 0, SEEK_SET) != 0) {
        LOG_E(TAG, "Failed to seek to beginning of log file");
        if (had_writer_task) {
            vTaskResume(module->writer_task_handle);
        }
        return;
    }

    size_t written = fwrite(config, 1, sizeof(logger_config_t), fp);
    if (written != sizeof(logger_config_t)) {
        LOG_E(TAG, "Failed to write config header: wrote %zu of %zu bytes", written, sizeof(logger_config_t));
    } else {
        LOG_I(TAG, "Wrote %zu-byte config header to log file (magic=0x%04X, v%u.%u)", sizeof(logger_config_t), config->magic,
              config->version_major, config->version_minor);
    }
    fflush(fp);

    // Seek back to original position (or to end of header if file was empty)
    if (current_pos < (long)sizeof(logger_config_t)) {
        fseek(fp, sizeof(logger_config_t), SEEK_SET);
    } else {
        fseek(fp, current_pos, SEEK_SET);
    }

    if (had_writer_task) {
        vTaskResume(module->writer_task_handle);
    }
}

void logger_module_set_stm_config(logger_module_t *module, const logger_config_t *config) {
    if (!module || !config) {
        LOG_E(TAG, "Invalid arguments to logger_module_set_stm_config");
        return;
    }

    // Store the config in the module
    memcpy(&module->stm_config, config, sizeof(logger_config_t));
    module->stm_config_received = true;

    // Write the 64-byte config header to the beginning of the log file
    logger_module_write_config_header_internal(module, &module->stm_config);

    // Publish event that config has been received
    es_t *es = app_state_get_event_sender();
    if (es) {
        es_event_t ev = {.id = APP_EVENT_LOGGER_CONFIG_RECEIVED};
        es_emit(es, &ev);
    }
}

void logger_module_write_config_header(logger_module_t *module, const logger_config_t *config) {
    logger_module_write_config_header_internal(module, config);
}

void IRAM_ATTR logger_module_write_frame(logger_module_t *module, const logger_frame_t *frame) {
    if (!module || !frame) {
        return;
    }
    
    // Only write if SD card is ready and we're initialized
    if (!module->sd_card_ok || !module->initialized) {
        return;
    }
    
    // Validate frame magic before writing
    if (frame->magic != LOGGER_FRAME_MAGIC) {
        return;
    }
    
    // Write the entire 852-byte frame using the buffered write system
    logger_append_bytes(module, (const uint8_t *)frame, sizeof(logger_frame_t));
}
