#include "bootlog.h"

#if defined(LAST_SERIAL_LOG_NVS)

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "esp_attr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"

static const char *TAG = "bootlog";

typedef struct {
    uint32_t magic;
    uint32_t boot_count;
    int reset_reason;
    uint32_t write_index;
    uint32_t line_count;
    char lines[BOOTLOG_MAX_LINES][BOOTLOG_LINE_LEN];
} bootlog_rtc_t;

// Persist across warm resets (WDT/SW/PANIC). Power-cycle will clear.
RTC_NOINIT_ATTR static bootlog_rtc_t s_bootlog_rtc;

static portMUX_TYPE s_bootlog_lock = portMUX_INITIALIZER_UNLOCKED;
static bool s_hook_installed = false;
static vprintf_like_t s_prev_vprintf = NULL;

static const uint32_t BOOTLOG_MAGIC = 0x424C4F47U;  // "BLOG"

static void bootlog_raw_printf(const char *fmt, ...);

// ---------------- NVS persistence (survives power-cycle) ----------------

typedef struct {
    uint32_t magic;
    uint32_t version;
    bootlog_rtc_t rtc;
} bootlog_nvs_blob_t;

static const uint32_t BOOTLOG_NVS_MAGIC = 0x424C4C4EU;  // "BLLN"
static const uint32_t BOOTLOG_NVS_VERSION = 1;
static const char *BOOTLOG_NVS_NS = "bootlog";
static const char *BOOTLOG_NVS_KEY_LAST = "last";

static bool s_nvs_enabled = false;
static uint32_t s_nvs_flush_interval_ms = 30000;
static uint64_t s_nvs_last_flush_ms = 0;
static bool s_nvs_dirty = false;
static bool s_nvs_initialized = false;

static bootlog_nvs_blob_t s_nvs_blob_copy;
static bootlog_nvs_blob_t s_nvs_blob_read;
static bootlog_snapshot_t s_nvs_snapshot;

static inline void bootlog_lock(void) {
    if (xPortInIsrContext()) {
        portENTER_CRITICAL_ISR(&s_bootlog_lock);
    } else {
        portENTER_CRITICAL(&s_bootlog_lock);
    }
}

static inline void bootlog_unlock(void) {
    if (xPortInIsrContext()) {
        portEXIT_CRITICAL_ISR(&s_bootlog_lock);
    } else {
        portEXIT_CRITICAL(&s_bootlog_lock);
    }
}

static void bootlog_rtc_init_if_needed(void) {
    if (s_bootlog_rtc.magic == BOOTLOG_MAGIC) {
        return;
    }
    memset(&s_bootlog_rtc, 0, sizeof(s_bootlog_rtc));
    s_bootlog_rtc.magic = BOOTLOG_MAGIC;
}

static void bootlog_rtc_clear_lines(void) {
    bootlog_lock();
    s_bootlog_rtc.write_index = 0;
    s_bootlog_rtc.line_count = 0;
    memset(s_bootlog_rtc.lines, 0, sizeof(s_bootlog_rtc.lines));
    bootlog_unlock();
}

static void bootlog_rtc_push_line(const char *line) {
    if (line == NULL || line[0] == '\0') {
        return;
    }

    bootlog_lock();

    const uint32_t idx = s_bootlog_rtc.write_index % BOOTLOG_MAX_LINES;
    (void)snprintf(s_bootlog_rtc.lines[idx], BOOTLOG_LINE_LEN, "%s", line);

    s_bootlog_rtc.write_index = (idx + 1U) % BOOTLOG_MAX_LINES;
    if (s_bootlog_rtc.line_count < BOOTLOG_MAX_LINES) {
        s_bootlog_rtc.line_count++;
    }

    s_nvs_dirty = true;
    bootlog_unlock();
}

static int bootlog_vprintf(const char *fmt, va_list args) {
    char line[BOOTLOG_LINE_LEN];
    line[0] = '\0';

    va_list copy;
    va_copy(copy, args);
    (void)vsnprintf(line, sizeof(line), fmt, copy);
    va_end(copy);

    bootlog_rtc_push_line(line);

    if (s_prev_vprintf) {
        return s_prev_vprintf(fmt, args);
    }

    // Fallback: should not normally happen.
    return vprintf(fmt, args);
}

void bootlog_capture_snapshot(bootlog_snapshot_t *out) {
    if (!out) {
        return;
    }

    memset(out, 0, sizeof(*out));

    bootlog_rtc_init_if_needed();

    bootlog_lock();
    out->valid = (s_bootlog_rtc.magic == BOOTLOG_MAGIC);
    out->reset_reason = s_bootlog_rtc.reset_reason;
    out->boot_count = s_bootlog_rtc.boot_count;
    out->write_index = s_bootlog_rtc.write_index;
    out->line_count = s_bootlog_rtc.line_count;
    for (size_t i = 0; i < BOOTLOG_MAX_LINES; i++) {
        (void)snprintf(out->lines[i], BOOTLOG_LINE_LEN, "%s", s_bootlog_rtc.lines[i]);
    }
    bootlog_unlock();
}

void bootlog_init_for_new_boot(void) {
    bootlog_rtc_init_if_needed();

    if (!s_hook_installed) {
        s_prev_vprintf = esp_log_set_vprintf(&bootlog_vprintf);
        s_hook_installed = true;
    }

    bootlog_lock();
    s_bootlog_rtc.boot_count++;
    s_bootlog_rtc.reset_reason = (int)esp_reset_reason();
    bootlog_unlock();

    bootlog_rtc_clear_lines();

    bootlog_raw_printf("I (%lu) %s: Bootlog capture enabled (RTC), max_lines=%d, line_len=%d\n",
                       (unsigned long)(xTaskGetTickCount() * portTICK_PERIOD_MS),
                       TAG,
                       (int)BOOTLOG_MAX_LINES,
                       (int)BOOTLOG_LINE_LEN);
}

static void bootlog_raw_printf(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    if (s_prev_vprintf) {
        (void)s_prev_vprintf(fmt, args);
    } else {
        (void)vprintf(fmt, args);
    }
    va_end(args);
}

void bootlog_dump_snapshot(const bootlog_snapshot_t *snapshot) {
    if (!snapshot || !snapshot->valid) {
        return;
    }

    const unsigned int count = snapshot->line_count;
    if (count == 0) {
        return;
    }

    bootlog_raw_printf("\n=== PREVIOUS BOOT LOG (last %u lines, boot_count=%u, reset_reason=%d) ===\n",
                       count, snapshot->boot_count, snapshot->reset_reason);

    const unsigned int write_idx = snapshot->write_index % BOOTLOG_MAX_LINES;
    const unsigned int start_idx = (write_idx + BOOTLOG_MAX_LINES - count) % BOOTLOG_MAX_LINES;

    for (unsigned int n = 0; n < count; n++) {
        const unsigned int idx = (start_idx + n) % BOOTLOG_MAX_LINES;
        const char *line = snapshot->lines[idx];
        if (line[0] == '\0') {
            continue;
        }
        bootlog_raw_printf("%s", line);
        if (line[strlen(line) - 1] != '\n') {
            bootlog_raw_printf("\n");
        }
    }
    bootlog_raw_printf("=== END PREVIOUS BOOT LOG ===\n\n");
}

static uint64_t bootlog_now_ms(void) {
    return (uint64_t)(esp_timer_get_time() / 1000);
}

static bool bootlog_nvs_try_init(void) {
    if (s_nvs_initialized) {
        return true;
    }
    esp_err_t err = nvs_flash_init();
    if (err != ESP_OK) {
        // Do NOT erase NVS here (could wipe config). Just disable persistence.
        bootlog_raw_printf("E (%lu) %s: nvs_flash_init failed: %s\n",
                           (unsigned long)(xTaskGetTickCount() * portTICK_PERIOD_MS),
                           TAG,
                           esp_err_to_name(err));
        return false;
    }
    s_nvs_initialized = true;
    return true;
}

static esp_err_t bootlog_nvs_write_locked_copy(void) {
    if (!bootlog_nvs_try_init()) {
        return ESP_FAIL;
    }

    nvs_handle_t handle;
    esp_err_t err = nvs_open(BOOTLOG_NVS_NS, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return err;
    }

    err = nvs_set_blob(handle, BOOTLOG_NVS_KEY_LAST, &s_nvs_blob_copy, sizeof(s_nvs_blob_copy));
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    return err;
}

void bootlog_nvs_enable(unsigned int flush_interval_ms) {
    s_nvs_flush_interval_ms = (flush_interval_ms == 0) ? 30000U : (uint32_t)flush_interval_ms;
    s_nvs_enabled = true;
}

void bootlog_nvs_poll(void) {
    if (!s_nvs_enabled) {
        return;
    }

    if (!s_nvs_dirty) {
        return;
    }

    const uint64_t now_ms = bootlog_now_ms();
    if ((now_ms - s_nvs_last_flush_ms) < (uint64_t)s_nvs_flush_interval_ms) {
        return;
    }

    bootlog_nvs_flush_now();
}

void bootlog_nvs_flush_now(void) {
    if (!s_nvs_enabled) {
        return;
    }

    // Copy current RTC ring under lock (keep lock short).
    bootlog_lock();
    s_nvs_blob_copy.magic = BOOTLOG_NVS_MAGIC;
    s_nvs_blob_copy.version = BOOTLOG_NVS_VERSION;
    memcpy(&s_nvs_blob_copy.rtc, &s_bootlog_rtc, sizeof(s_nvs_blob_copy.rtc));
    s_nvs_dirty = false;
    bootlog_unlock();

    const esp_err_t err = bootlog_nvs_write_locked_copy();
    if (err != ESP_OK) {
        bootlog_raw_printf("W (%lu) %s: bootlog NVS flush failed: %s\n",
                           (unsigned long)(xTaskGetTickCount() * portTICK_PERIOD_MS),
                           TAG,
                           esp_err_to_name(err));
        // Mark dirty again so a later retry can happen.
        bootlog_lock();
        s_nvs_dirty = true;
        bootlog_unlock();
        return;
    }

    s_nvs_last_flush_ms = bootlog_now_ms();
}

void bootlog_nvs_dump_last(void) {
    if (!bootlog_nvs_try_init()) {
        return;
    }

    nvs_handle_t handle;
    esp_err_t err = nvs_open(BOOTLOG_NVS_NS, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        return;
    }

    size_t required = sizeof(s_nvs_blob_read);
    err = nvs_get_blob(handle, BOOTLOG_NVS_KEY_LAST, &s_nvs_blob_read, &required);
    nvs_close(handle);

    if (err != ESP_OK || required != sizeof(s_nvs_blob_read)) {
        return;
    }
    if (s_nvs_blob_read.magic != BOOTLOG_NVS_MAGIC || s_nvs_blob_read.version != BOOTLOG_NVS_VERSION) {
        return;
    }

    memset(&s_nvs_snapshot, 0, sizeof(s_nvs_snapshot));
    s_nvs_snapshot.valid = true;
    s_nvs_snapshot.reset_reason = s_nvs_blob_read.rtc.reset_reason;
    s_nvs_snapshot.boot_count = s_nvs_blob_read.rtc.boot_count;
    s_nvs_snapshot.write_index = s_nvs_blob_read.rtc.write_index;
    s_nvs_snapshot.line_count = s_nvs_blob_read.rtc.line_count;
    for (size_t i = 0; i < BOOTLOG_MAX_LINES; i++) {
        (void)snprintf(s_nvs_snapshot.lines[i], BOOTLOG_LINE_LEN, "%s", s_nvs_blob_read.rtc.lines[i]);
    }

    bootlog_raw_printf("\n=== NVS BOOT LOG (persisted) ===\n");
    bootlog_dump_snapshot(&s_nvs_snapshot);
}

#endif  // defined(LAST_SERIAL_LOG_NVS)
