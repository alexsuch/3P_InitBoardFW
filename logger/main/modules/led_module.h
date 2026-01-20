#pragma once

#include <log.h>
#include <target.h>

#include "app_state.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/gpio.h"
#include "util/event_sender.h"

#if defined(USE_RGB_LED)
#include "hal/ws2812.h"
#endif

typedef struct led_module_s {
    bool is_initialized;
    app_mode_t current_mode;
    es_token_t app_state_subscription;
#if defined(USE_RGB_LED)
    bool rgb_led_initialized;
    hal_ws2812_color_t rgb_color_buf[1];  // Single LED buffer
#endif
} led_module_t;

void led_module_init(led_module_t *led_module);
void led_blink_task(void *pvParameters);
void led_module_on_app_state_change(const es_event_t *ev, void *user);

#if defined(USE_RGB_LED)
// RGB LED color definitions for app modes
#define LED_COLOR_ERROR (hal_ws2812_color_t){.r = 255, .g = 0, .b = 0}
#define LED_COLOR_IDLE (hal_ws2812_color_t){.r = 0, .g = 255, .b = 0}       // Solid green in IDLE
#define LED_COLOR_LOGGING (hal_ws2812_color_t){.r = 255, .g = 128, .b = 0}  // Orange during logging
#define LED_COLOR_DOWNLOAD (hal_ws2812_color_t){.r = 0, .g = 0, .b = 255}  // Solid blue in download mode
#define LED_COLOR_ONLINE (hal_ws2812_color_t){.r = 0, .g = 0, .b = 255}

// Internal function to update RGB LED color based on app mode
void led_module_update_rgb_color(led_module_t *led_module, app_mode_t mode);
#endif
