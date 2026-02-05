#include "led_module.h"

#include "app_state.h"
#include "driver/gpio.h"
#include "util/event_sender.h"

#if defined(USE_RGB_LED)
#include "hal/err.h"
#include "hal/ws2812.h"
#endif

static const char* TAG = "LED";

#if defined(USE_RGB_LED)
#define LED_MODULE_DEFAULT_BRIGHTNESS 20
static uint8_t rgb_led_brightness = LED_MODULE_DEFAULT_BRIGHTNESS;
#endif

// Task to control both board LED and external LED based on app mode
void led_blink_task(void* pvParameters) {
    led_module_t* led_module = (led_module_t*)pvParameters;

    // Setup GPIO pins as output
    // GPIO 40 connected to JTAG so we need to reset it before use
#if defined(USE_LED_OUT)
    gpio_reset_pin(LED_OUT_GPIO);
    hal_gpio_setup(LED_OUT_GPIO, HAL_GPIO_DIR_OUTPUT, HAL_GPIO_PULL_NONE);
    hal_gpio_set_level(LED_OUT_GPIO, HAL_GPIO_HIGH);
#endif
#if BOARD_LED_ENABLED
    hal_gpio_setup(BOARDLED_PIN, HAL_GPIO_DIR_OUTPUT, HAL_GPIO_PULL_NONE);
    hal_gpio_set_level(BOARDLED_PIN, HAL_GPIO_HIGH);
#endif

    // MEMORY OPTIMIZATION: Removed expensive printf-style logging from task to save stack space
    // This single LOG_I call could consume 1-2KB of stack due to printf formatting
    // Simple logging alternative without format strings:
#if BOARD_LED_ENABLED
    LOG_I(TAG, "task started - onboard and external LEDs configured");
#else
    LOG_I(TAG, "task started - external LED configured (board LED disabled)");
#endif

    TickType_t last_toggle = xTaskGetTickCount();
    TickType_t pattern_deadline = last_toggle;
    uint8_t online_phase = 0;
    uint8_t download_phase = 0;
    bool blink_state = true;

    while (1) {
        app_state_t* state = app_state_get_instance();
        app_mode_t mode = state ? state->current_mode : led_module->current_mode;
        led_module->current_mode = mode;
#if defined(USE_RGB_LED)
        // Refresh every loop to ensure mode changes are reflected promptly.
        led_module_update_rgb_color(led_module, led_module->current_mode);
#endif

        switch (led_module->current_mode) {
            case APP_MODE_LOGGING: {
                if (xTaskGetTickCount() - last_toggle >= pdMS_TO_TICKS(500)) {
                    blink_state = !blink_state;
                    last_toggle = xTaskGetTickCount();
                }
#if BOARD_LED_ENABLED
                hal_gpio_set_level(BOARDLED_PIN, blink_state ? HAL_GPIO_HIGH : HAL_GPIO_LOW);
#endif
#if defined(USE_LED_OUT)
                hal_gpio_set_level(LED_OUT_GPIO, blink_state ? HAL_GPIO_HIGH : HAL_GPIO_LOW);
#endif
                vTaskDelay(pdMS_TO_TICKS(50));
                break;
            }
            case APP_MODE_ERROR: {
                if (xTaskGetTickCount() - last_toggle >= pdMS_TO_TICKS(50)) {
                    blink_state = !blink_state;
                    last_toggle = xTaskGetTickCount();
                }
#if BOARD_LED_ENABLED
                hal_gpio_set_level(BOARDLED_PIN, blink_state ? HAL_GPIO_HIGH : HAL_GPIO_LOW);
#endif
#if defined(USE_LED_OUT)
                hal_gpio_set_level(LED_OUT_GPIO, blink_state ? HAL_GPIO_HIGH : HAL_GPIO_LOW);
#endif
                vTaskDelay(pdMS_TO_TICKS(20));
                break;
            }
            case APP_MODE_DOWNLOAD: {
                TickType_t now = xTaskGetTickCount();
                if (now >= pattern_deadline) {
                    switch (download_phase) {
                        case 0:
                            // First short flash - LED ON
#if BOARD_LED_ENABLED
                            hal_gpio_set_level(BOARDLED_PIN, HAL_GPIO_HIGH);
#endif
#if defined(USE_LED_OUT)
                            hal_gpio_set_level(LED_OUT_GPIO, HAL_GPIO_HIGH);
#endif
                            pattern_deadline = now + pdMS_TO_TICKS(150);
                            download_phase = 1;
                            break;
                        case 1:
                            // Gap between flashes - LED OFF
#if BOARD_LED_ENABLED
                            hal_gpio_set_level(BOARDLED_PIN, HAL_GPIO_LOW);
#endif
#if defined(USE_LED_OUT)
                            hal_gpio_set_level(LED_OUT_GPIO, HAL_GPIO_LOW);
#endif
                            pattern_deadline = now + pdMS_TO_TICKS(100);
                            download_phase = 2;
                            break;
                        case 2:
                            // Second short flash - LED ON
#if BOARD_LED_ENABLED
                            hal_gpio_set_level(BOARDLED_PIN, HAL_GPIO_HIGH);
#endif
#if defined(USE_LED_OUT)
                            hal_gpio_set_level(LED_OUT_GPIO, HAL_GPIO_HIGH);
#endif
                            pattern_deadline = now + pdMS_TO_TICKS(150);
                            download_phase = 3;
                            break;
                        case 3:
                            // Gap after second flash - LED OFF
#if BOARD_LED_ENABLED
                            hal_gpio_set_level(BOARDLED_PIN, HAL_GPIO_LOW);
#endif
#if defined(USE_LED_OUT)
                            hal_gpio_set_level(LED_OUT_GPIO, HAL_GPIO_LOW);
#endif
                            pattern_deadline = now + pdMS_TO_TICKS(100);
                            download_phase = 4;
                            break;
                        case 4:
                        default:
                            // Longer pause before repeating - LED OFF
#if BOARD_LED_ENABLED
                            hal_gpio_set_level(BOARDLED_PIN, HAL_GPIO_LOW);
#endif
#if defined(USE_LED_OUT)
                            hal_gpio_set_level(LED_OUT_GPIO, HAL_GPIO_LOW);
#endif
                            pattern_deadline = now + pdMS_TO_TICKS(900);
                            download_phase = 0;
                            break;
                    }
                }
                vTaskDelay(pdMS_TO_TICKS(20));
                break;
            }
            case APP_MODE_ONLINE: {
                TickType_t now = xTaskGetTickCount();
                if (now >= pattern_deadline) {
                    switch (online_phase) {
                        case 0:
#if BOARD_LED_ENABLED
                            hal_gpio_set_level(BOARDLED_PIN, HAL_GPIO_HIGH);
#endif
#if defined(USE_LED_OUT)
                            hal_gpio_set_level(LED_OUT_GPIO, HAL_GPIO_HIGH);
#endif
                            pattern_deadline = now + pdMS_TO_TICKS(200);
                            online_phase = 1;
                            break;
                        case 1:
#if BOARD_LED_ENABLED
                            hal_gpio_set_level(BOARDLED_PIN, HAL_GPIO_LOW);
#endif
#if defined(USE_LED_OUT)
                            hal_gpio_set_level(LED_OUT_GPIO, HAL_GPIO_LOW);
#endif
                            pattern_deadline = now + pdMS_TO_TICKS(200);
                            online_phase = 2;
                            break;
                        case 2:
#if BOARD_LED_ENABLED
                            hal_gpio_set_level(BOARDLED_PIN, HAL_GPIO_HIGH);
#endif
#if defined(USE_LED_OUT)
                            hal_gpio_set_level(LED_OUT_GPIO, HAL_GPIO_HIGH);
#endif
                            pattern_deadline = now + pdMS_TO_TICKS(200);
                            online_phase = 3;
                            break;
                        case 3:
                        default:
#if BOARD_LED_ENABLED
                            hal_gpio_set_level(BOARDLED_PIN, HAL_GPIO_LOW);
#endif
#if defined(USE_LED_OUT)
                            hal_gpio_set_level(LED_OUT_GPIO, HAL_GPIO_LOW);
#endif
                            pattern_deadline = now + pdMS_TO_TICKS(1000);
                            online_phase = 0;
                            break;
                    }
                }
                vTaskDelay(pdMS_TO_TICKS(20));
                break;
            }
            case APP_MODE_IDLE:
            default: {
#if BOARD_LED_ENABLED
                hal_gpio_set_level(BOARDLED_PIN, HAL_GPIO_HIGH);
#endif
#if defined(USE_LED_OUT)
                hal_gpio_set_level(LED_OUT_GPIO, HAL_GPIO_HIGH);
#endif
                vTaskDelay(pdMS_TO_TICKS(200));
                break;
            }
        }
    }
}

#if defined(USE_RGB_LED)
void led_module_update_rgb_color(led_module_t* led_module, app_mode_t mode) {
    if (!led_module || !led_module->rgb_led_initialized) {
        return;
    }

    hal_ws2812_color_t color;
    switch (mode) {
        case APP_MODE_ERROR:
            color = LED_COLOR_ERROR;
            break;
        case APP_MODE_LOGGING:
            color = LED_COLOR_LOGGING;
            break;
        case APP_MODE_DOWNLOAD:
            color = LED_COLOR_DOWNLOAD;
            break;
        case APP_MODE_ONLINE:
            color = LED_COLOR_ONLINE;
            break;
        case APP_MODE_IDLE:
        default:
            color = LED_COLOR_IDLE;
            break;
    }

    // Apply brightness scaling
    hal_ws2812_color_t scaled = color;
    if (rgb_led_brightness == 0) {
        scaled.r = scaled.g = scaled.b = 0;
    } else if (rgb_led_brightness < 255) {
        scaled.r = ((uint16_t)color.r * rgb_led_brightness) / 255;
        scaled.g = ((uint16_t)color.g * rgb_led_brightness) / 255;
        scaled.b = ((uint16_t)color.b * rgb_led_brightness) / 255;
    }

    led_module->rgb_color_buf[0] = scaled;
    hal_ws2812_set_colors(RGB_LED_GPIO, LED_COLOR_ORDER, led_module->rgb_color_buf, 1);
}
#endif

void led_module_on_app_state_change(const es_event_t* ev, void* user) {
    led_module_t* led_module = (led_module_t*)user;
    if (!ev || ev->id != 1) return;  // Only handle app state change events

    uint64_t* changed_mask = (uint64_t*)ev->payload;

    // MEMORY OPTIMIZATION: Simplified logging to avoid expensive printf formatting
    // The original LOG_I with %llu formatting consumed significant stack space
    // Static strings use minimal stack compared to printf-style formatting
    if (led_module->current_mode == APP_MODE_LOGGING) {
        LOG_I(TAG, "mode: LOGGING");
    } else if (led_module->current_mode == APP_MODE_ERROR) {
        LOG_I(TAG, "mode: ERROR");
    } else if (led_module->current_mode == APP_MODE_DOWNLOAD) {
        LOG_I(TAG, "mode: DOWNLOAD");
    } else if (led_module->current_mode == APP_MODE_ONLINE) {
        LOG_I(TAG, "mode: ONLINE");
    } else {
        LOG_I(TAG, "mode: IDLE");
    }
    // Check if the mode field changed
    if (*changed_mask & APP_STATE_FIELD_CURRENT_MODE) {
        app_state_t* app_state = app_state_get_instance();
        led_module->current_mode = app_state->current_mode;
#if defined(USE_RGB_LED)
        led_module_update_rgb_color(led_module, led_module->current_mode);
#endif
    }
}

#if defined(USE_RGB_LED)
static void led_module_init_rgb(led_module_t* led_module) {
    if (!led_module) {
        return;
    }

    // Initialize RGB LED on RGB_LED_GPIO
    hal_err_t ret = hal_ws2812_open(RGB_LED_GPIO);
    if (ret != HAL_ERR_NONE) {
        LOG_E(TAG, "Failed to initialize RGB LED on GPIO %d", RGB_LED_GPIO);
        led_module->rgb_led_initialized = false;
        return;
    }

    // Initialize color buffer to off with brightness applied
    hal_ws2812_color_t initial_color = LED_COLOR_IDLE;
    hal_ws2812_color_t scaled_initial = initial_color;
    if (rgb_led_brightness == 0) {
        scaled_initial.r = scaled_initial.g = scaled_initial.b = 0;
    } else if (rgb_led_brightness < 255) {
        scaled_initial.r = ((uint16_t)initial_color.r * rgb_led_brightness) / 255;
        scaled_initial.g = ((uint16_t)initial_color.g * rgb_led_brightness) / 255;
        scaled_initial.b = ((uint16_t)initial_color.b * rgb_led_brightness) / 255;
    }
    led_module->rgb_color_buf[0] = scaled_initial;
    hal_ws2812_set_colors(RGB_LED_GPIO, LED_COLOR_ORDER, led_module->rgb_color_buf, 1);

    led_module->rgb_led_initialized = true;
    LOG_I(TAG, "RGB LED initialized on GPIO %d", RGB_LED_GPIO);
}
#endif

void led_module_init(led_module_t* led_module) {
    if (!led_module) {
        return;
    }

    led_module->is_initialized = true;
    led_module->current_mode = APP_MODE_IDLE;  // Default to idle mode

#if defined(USE_RGB_LED)
    led_module->rgb_led_initialized = false;
    led_module_init_rgb(led_module);
    // Set initial color based on initial mode
    if (led_module->rgb_led_initialized) {
        led_module_update_rgb_color(led_module, led_module->current_mode);
    }
#endif

    // Subscribe to app state changes using event sender
    es_t* event_sender = app_state_get_event_sender();
    if (event_sender) {
        es_subscribe(event_sender, led_module_on_app_state_change, led_module, 0, &led_module->app_state_subscription);
        LOG_I(TAG, "module subscribed to app state changes");
    }

    LOG_I(TAG, "module initialized");
}
