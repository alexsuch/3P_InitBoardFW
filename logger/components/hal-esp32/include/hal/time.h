#pragma once

#include <esp_timer.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

inline uint64_t hal_time_micros_now(void) { return esp_timer_get_time(); }

#ifdef __cplusplus
}
#endif