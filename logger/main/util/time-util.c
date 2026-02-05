#include "util/time-util.h"

#include <esp_timer.h>

static TickType_t default_get_ticks(void) { return xTaskGetTickCount(); }
static time_micros_t default_get_micros(void) { return (time_micros_t)esp_timer_get_time(); }
static void default_delay_ticks(TickType_t ticks) { vTaskDelay(ticks); }

static time_ticks_provider_t s_get_ticks = default_get_ticks;
static time_micros_provider_t s_get_micros = default_get_micros;
static time_delay_ticks_provider_t s_delay_ticks = default_delay_ticks;

static inline time_ticks_provider_t ensure_ticks_provider(void) { return s_get_ticks ? s_get_ticks : default_get_ticks; }

static inline time_micros_provider_t ensure_micros_provider(void) { return s_get_micros ? s_get_micros : default_get_micros; }

static inline time_delay_ticks_provider_t ensure_delay_provider(void) { return s_delay_ticks ? s_delay_ticks : default_delay_ticks; }

void time_install_hooks(const time_source_hooks_t *hooks) {
    if (!hooks) {
        time_reset_hooks();
        return;
    }

    s_get_ticks = hooks->get_ticks ? hooks->get_ticks : default_get_ticks;
    s_get_micros = hooks->get_micros ? hooks->get_micros : default_get_micros;
    s_delay_ticks = hooks->delay_ticks ? hooks->delay_ticks : default_delay_ticks;
}

void time_reset_hooks(void) {
    s_get_ticks = default_get_ticks;
    s_get_micros = default_get_micros;
    s_delay_ticks = default_delay_ticks;
}

time_ticks_t time_source_get_ticks(void) { return ensure_ticks_provider()(); }

time_micros_t time_source_get_micros(void) { return ensure_micros_provider()(); }

void time_source_delay_ticks(TickType_t ticks) { ensure_delay_provider()(ticks); }
