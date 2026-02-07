#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <target.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined(LAST_SERIAL_LOG_NVS)

// Persisted ring buffer of the most recent log lines across soft resets using RTC memory.
// Intended for debugging cases where USB serial disappears (e.g., switching USB classes).

#ifndef BOOTLOG_MAX_LINES
#define BOOTLOG_MAX_LINES 10
#endif

#ifndef BOOTLOG_LINE_LEN
#define BOOTLOG_LINE_LEN 192
#endif

typedef struct {
    bool valid;
    int reset_reason;
    unsigned int boot_count;
    unsigned int write_index;
    unsigned int line_count;
    char lines[BOOTLOG_MAX_LINES][BOOTLOG_LINE_LEN];
} bootlog_snapshot_t;

// Capture the current persisted RTC bootlog into a snapshot (does not modify RTC state).
void bootlog_capture_snapshot(bootlog_snapshot_t *out);

// Initialize bootlog capture for the new boot:
// - installs an esp_log vprintf hook
// - clears the persisted ring buffer (so it captures this boot's logs)
// - records the current reset reason
void bootlog_init_for_new_boot(void);

// Dump a previously captured snapshot using the original vprintf (does not recurse into esp_log).
void bootlog_dump_snapshot(const bootlog_snapshot_t *snapshot);

// ---------------- NVS persistence (survives power-cycle) ----------------

// Enable periodic persistence of the current ring buffer into NVS.
// Call bootlog_nvs_poll() from a normal task context (e.g., main loop) to perform throttled commits.
void bootlog_nvs_enable(unsigned int flush_interval_ms);

// Poll function that writes to NVS when dirty + interval elapsed.
void bootlog_nvs_poll(void);

// Force an immediate flush to NVS (safe only from task context).
void bootlog_nvs_flush_now(void);

// Dump the last snapshot stored in NVS (if present) using the original vprintf.
void bootlog_nvs_dump_last(void);

#else

typedef struct {
    bool valid;
} bootlog_snapshot_t;

static inline void bootlog_capture_snapshot(bootlog_snapshot_t *out) {
    if (out) out->valid = false;
}
static inline void bootlog_init_for_new_boot(void) {}
static inline void bootlog_dump_snapshot(const bootlog_snapshot_t *snapshot) { (void)snapshot; }
static inline void bootlog_nvs_enable(unsigned int flush_interval_ms) { (void)flush_interval_ms; }
static inline void bootlog_nvs_poll(void) {}
static inline void bootlog_nvs_flush_now(void) {}
static inline void bootlog_nvs_dump_last(void) {}

#endif  // defined(LAST_SERIAL_LOG_NVS)

#ifdef __cplusplus
}
#endif
