#pragma once

#include <hal/err.h>
#include <hal/gpio.h>
#include <stddef.h>
#include <stdint.h>

#define WS2812_RMT_RESOLUTION_HZ 20000000  // 20MHz (50ns per tick)
#define HAL_WS2812_COLOR_LEVEL_MAX 255
#define HAL_WS2812_COLOR_STEP_MIN INT8_MIN
#define HAL_WS2812_COLOR_STEP_MAX INT8_MAX

typedef struct hal_ws2812_color_s {
    // Declared in the same order as they're sent out,
    // so we can avoid a copy to reorder the data in memory.
    uint8_t r;
    uint8_t g;
    uint8_t b;
} __attribute__((packed)) hal_ws2812_color_t;

typedef struct hal_ws2812_color_step_s {
    int8_t r;
    int8_t g;
    int8_t b;
} __attribute__((packed)) hal_ws2812_color_step_t;

typedef struct hal_ws2812_state_s {
    hal_ws2812_color_t color;         // Current color
    hal_ws2812_color_t target_color;  // Color we want to transition to
} hal_ws2812_state_t;

// Compile-time selectable color byte order for various WS2812-compatible LEDs
// Many variants expect GRB (classic WS2812B), while others expect RGB/BGR/etc.
typedef enum {
    HAL_WS2812_ORDER_GRB = 0,
    HAL_WS2812_ORDER_RGB = 1,
    HAL_WS2812_ORDER_BRG = 2,
    HAL_WS2812_ORDER_GBR = 3,
    HAL_WS2812_ORDER_RBG = 4,
    HAL_WS2812_ORDER_BGR = 5,
} hal_ws2812_color_order_e;

// Helper to construct a color that works in both C and C++ contexts
static inline hal_ws2812_color_t hal_ws2812_rgb(uint8_t red, uint8_t green, uint8_t blue) {
    hal_ws2812_color_t color;
    color.r = red;
    color.g = green;
    color.b = blue;
    return color;
}

#define HAL_WS2812_RGB(red, green, blue) hal_ws2812_rgb((uint8_t)(red), (uint8_t)(green), (uint8_t)(blue))

#define HAL_WS2812_TRACKING HAL_WS2812_RGB(HAL_WS2812_COLOR_LEVEL_MAX, 0, HAL_WS2812_COLOR_LEVEL_MAX)
#define HAL_WS2812_RED HAL_WS2812_RGB(HAL_WS2812_COLOR_LEVEL_MAX, 0, 0)
#define HAL_WS2812_GREEN HAL_WS2812_RGB(0, HAL_WS2812_COLOR_LEVEL_MAX, 0)
#define HAL_WS2812_BLUE HAL_WS2812_RGB(0, 0, HAL_WS2812_COLOR_LEVEL_MAX)
#define HAL_WS2812_PURPLE HAL_WS2812_RGB(HAL_WS2812_COLOR_LEVEL_MAX, 0, HAL_WS2812_COLOR_LEVEL_MAX)
#define HAL_WS2812_YELLOW HAL_WS2812_RGB(HAL_WS2812_COLOR_LEVEL_MAX, HAL_WS2812_COLOR_LEVEL_MAX, 0)
#define HAL_WS2812_WHITE HAL_WS2812_RGB(HAL_WS2812_COLOR_LEVEL_MAX, HAL_WS2812_COLOR_LEVEL_MAX, HAL_WS2812_COLOR_LEVEL_MAX)
#define HAL_WS2812_OFF HAL_WS2812_RGB(0, 0, 0)

hal_err_t hal_ws2812_open(hal_gpio_t gpio);
hal_err_t hal_ws2812_close(hal_gpio_t gpio);
hal_err_t hal_ws2812_set_colors(hal_gpio_t gpio, hal_ws2812_color_order_e color_order, const hal_ws2812_color_t *colors, size_t count);