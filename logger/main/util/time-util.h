#pragma once

#include <esp_timer.h>
#include <limits.h>
#include <stdbool.h>
#include <stdint.h>

#include "project-os.h"

#define MILLIS_PER_SEC (1000)
#define MICROS_PER_SEC (1000000)

typedef TickType_t time_ticks_t;
typedef uint32_t time_millis_t;
typedef uint64_t time_micros_t;

#define TIME_MICROS_MAX UINT64_MAX

#define MILLIS_TO_TICKS(ms) ((ms) / portTICK_PERIOD_MS)
#define SECS_TO_TICKS(s) MILLIS_TO_TICKS(1000 * s)
#define FREQ_TO_TICKS(hz) MILLIS_TO_TICKS(1000 / hz)
#define TICKS_TO_MILLIS(t) (t * portTICK_PERIOD_MS)

#define MILLIS_TO_MICROS(ms) (ms * 1000)
#define SECS_TO_MICROS(s) MILLIS_TO_MICROS(s * 1000)
#define FREQ_TO_MICROS(hz) MILLIS_TO_TICKS(1000000 / hz)

#define TIME_CYCLE_EVERY_MS(ms, n) (((time_ticks_now() * portTICK_PERIOD_MS) / ms) % n)

typedef TickType_t (*time_ticks_provider_t)(void);
typedef time_micros_t (*time_micros_provider_t)(void);
typedef void (*time_delay_ticks_provider_t)(TickType_t ticks);

typedef struct {
    time_ticks_provider_t get_ticks;
    time_micros_provider_t get_micros;
    time_delay_ticks_provider_t delay_ticks;
} time_source_hooks_t;

void time_install_hooks(const time_source_hooks_t *hooks);
void time_reset_hooks(void);

time_ticks_t time_source_get_ticks(void);
time_micros_t time_source_get_micros(void);
void time_source_delay_ticks(TickType_t ticks);

static inline uint64_t hal_time_micros_now(void) { return (uint64_t)time_source_get_micros(); }

static inline time_ticks_t time_ticks_now(void) { return time_source_get_ticks(); }

static inline time_millis_t time_millis_now(void) { return time_source_get_ticks() * portTICK_PERIOD_MS; }

static inline void time_millis_delay(unsigned ms) { time_source_delay_ticks(ms / portTICK_PERIOD_MS); }

static inline bool time_ticks_ellapsed(time_ticks_t since, time_ticks_t now, time_ticks_t duration) { return since == 0 || now - since >= duration; }

static inline time_micros_t time_micros_now(void) { return time_source_get_micros(); }

static inline void time_micros_delay(time_micros_t delay) {
    time_micros_t end = time_source_get_micros() + delay;
    while (time_source_get_micros() < end) {}
}
