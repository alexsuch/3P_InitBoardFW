#include <target.h>
#include <esp_err.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#if defined(USE_FEATURE_BUTTON)

#include <log.h>

#include "button.h"
#include "util/time-util.h"

static const char *TAG = "button";

#define DOUBLE_PRESS_INTERVAL MILLIS_TO_TICKS(250)
#define LONG_PRESS_INTERVAL MILLIS_TO_TICKS(1000)
#define REALLY_LONG_PRESS_INTERVAL MILLIS_TO_TICKS(3000)
#define SUPER_LONG_PRESS_INTERVAL MILLIS_TO_TICKS(5000)

// Debounce/glitch filtering. Needed because EMI/noise can create phantom "clicks",
// especially when other peripherals are active (SD/USB).
#define PRESS_STABLE_INTERVAL MILLIS_TO_TICKS(30)
#define RELEASE_STABLE_INTERVAL MILLIS_TO_TICKS(30)
// After a long press is already detected, require a longer stable release to avoid
// treating short "up" glitches as a real release (which can start a new click sequence).
#define RELEASE_STABLE_AFTER_LONGPRESS_INTERVAL MILLIS_TO_TICKS(250)
// Cooldown after a handled long-press release to suppress spurious re-presses.
#define POST_HANDLED_COOLDOWN_INTERVAL MILLIS_TO_TICKS(300)
// Cooldown after multi-click events to avoid chaining into accidental higher-order clicks.
#define POST_CLICK_EVENT_COOLDOWN_INTERVAL MILLIS_TO_TICKS(200)

static bool button_is_down(button_t *button) {
    if (button->cfg.gpio == HAL_GPIO_NONE) {
        return false;
    }
    return hal_gpio_get_level(button->cfg.gpio) == HAL_GPIO_LOW;
}

static void button_gpio_init(button_t *button) {
    if (button->cfg.gpio == HAL_GPIO_NONE) {
        LOG_W(TAG, "Button %d disabled (gpio=%u)", (int)button->id, (unsigned)button->cfg.gpio);
        return;
    }
    HAL_ERR_ASSERT_OK(hal_gpio_setup(button->cfg.gpio, HAL_GPIO_DIR_INPUT, HAL_GPIO_PULL_UP));
}

static void button_send_event(button_t *button, button_event_type_e evt) {
    LOG_I(TAG, "Event %d in button %d", (int)evt, (int)button->id);
    button_callback_f callback = button->callback;
    if (callback) {
        button_event_t ev = {
            .button = button,
            .type = evt,
        };
        callback(&ev, button->user_data);
    }
}

void button_init(button_t *button) {
    LOG_I(TAG, "Initializing button %d", (int)button->id);
    button_gpio_init(button);

    time_ticks_t now = time_ticks_now();

    button->state.is_down = button_is_down(button);
    button->state.ignore = (button->cfg.gpio == HAL_GPIO_NONE) ? true : button->state.is_down;
    button->state.ignore_since = now;
    button->state.ignore_duration = 0;
    button->state.long_press_sent = false;
    button->state.really_long_press_sent = false;
    button->state.super_long_press_sent = false;
    button->state.down_since = 0;
    button->state.last_release_time = 0;
    button->state.click_num = 0;
    button->state.press_candidate_since = 0;
    button->state.release_candidate_since = 0;
}

void button_update(button_t *button) {
    time_ticks_t now = time_ticks_now();
    bool raw_is_down = button_is_down(button);

    if (button->state.ignore) {
        if (!raw_is_down && time_ticks_ellapsed(button->state.ignore_since, now, button->state.ignore_duration)) {
            button->state.ignore = false;
            button->state.ignore_since = 0;
            button->state.ignore_duration = 0;
        } else {
            return;
        }
    }

    if (raw_is_down) {
        button->state.release_candidate_since = 0;

        if (!button->state.is_down) {
            if (button->state.press_candidate_since == 0) {
                button->state.press_candidate_since = now;
                return;
            }
            if (!time_ticks_ellapsed(button->state.press_candidate_since, now, PRESS_STABLE_INTERVAL)) {
                return;
            }

            // Button press is stable
            button->state.is_down = true;
            button->state.press_candidate_since = 0;
            button->state.down_since = now;
            button->state.long_press_sent = false;
            button->state.really_long_press_sent = false;
            button->state.super_long_press_sent = false;
        } else {
            button->state.press_candidate_since = 0;
        }

        time_ticks_t pressed_duration = time_ticks_now() - button->state.down_since;

        if (!button->state.super_long_press_sent && pressed_duration > SUPER_LONG_PRESS_INTERVAL) {
            button_send_event(button, BUTTON_EVENT_TYPE_SUPER_LONG_PRESS);
            button->state.super_long_press_sent = true;
        } else if (!button->state.really_long_press_sent && pressed_duration > REALLY_LONG_PRESS_INTERVAL) {
            button_send_event(button, BUTTON_EVENT_TYPE_REALLY_LONG_PRESS);
            button->state.really_long_press_sent = true;
        } else if (!button->state.long_press_sent && pressed_duration > LONG_PRESS_INTERVAL) {
            button_send_event(button, BUTTON_EVENT_TYPE_LONG_PRESS);
            button->state.long_press_sent = true;
        }
    } else {
        button->state.press_candidate_since = 0;

        if (button->state.is_down) {
            // Candidate release: require stable high to avoid EMI glitches.
            if (button->state.release_candidate_since == 0) {
                button->state.release_candidate_since = now;
                return;
            }

            const bool long_handled = button->state.long_press_sent || button->state.really_long_press_sent || button->state.super_long_press_sent;
            const time_ticks_t required_release_stable = long_handled ? RELEASE_STABLE_AFTER_LONGPRESS_INTERVAL : RELEASE_STABLE_INTERVAL;
            if (!time_ticks_ellapsed(button->state.release_candidate_since, now, required_release_stable)) {
                return;
            }

            // Button release is stable
            button->state.is_down = false;
            button->state.release_candidate_since = 0;

            // Debounce: Ignore short glitches (increased to 80ms for heavy EMI environments)
            if ((now - button->state.down_since) < MILLIS_TO_TICKS(80)) {
                return;
            }

            button->state.last_release_time = now;

            if (!button->state.long_press_sent && !button->state.really_long_press_sent && !button->state.super_long_press_sent) {
                // This is a short press - increment click counter
                button->state.click_num++;

                // Check for quadruple press (immediate trigger)
                if (button->state.click_num == 4) {
                    // Safety: Only allow quadruple press on the primary ENTER button.
                    // External/BOOT button wiring is often noisy and can cause phantom clicks.
                    if (button->id == BUTTON_ID_ENTER) {
                        button_send_event(button, BUTTON_EVENT_TYPE_QUADRUPLE_PRESS);
                        button->state.ignore = true;
                        button->state.ignore_since = now;
                        button->state.ignore_duration = POST_CLICK_EVENT_COOLDOWN_INTERVAL;
                    }
                    button->state.click_num = 0;  // Reset counter
                } 
                // Triple/Double/Single handled in timeout
            } else {
                // Long press was handled. Reset click sequence and suppress any
                // spurious post-release pulses that could be misinterpreted as clicks.
                button->state.click_num = 0;
                button->state.ignore = true;
                button->state.ignore_since = now;
                button->state.ignore_duration = POST_HANDLED_COOLDOWN_INTERVAL;
            }
        } else {
            button->state.release_candidate_since = 0;

             // Button is not pressed - check for press timeout
             // Only evaluate click sequences if we actually have pending clicks.
             // Otherwise (click_num==0) we'd repeatedly trigger this block and keep the button in ignore/cooldown forever.
            if (button->state.click_num > 0 && (now - button->state.last_release_time) > DOUBLE_PRESS_INTERVAL) {
                bool emitted = false;
                if (button->state.click_num == 3) {
                    button_send_event(button, BUTTON_EVENT_TYPE_TRIPLE_PRESS);
                    emitted = true;
                } else if (button->state.click_num == 2) {
                    button_send_event(button, BUTTON_EVENT_TYPE_DOUBLE_PRESS);
                    emitted = true;
                } else if (button->state.click_num == 1) {
                    button_send_event(button, BUTTON_EVENT_TYPE_SHORT_PRESS);
                    emitted = true;
                }
                button->state.click_num = 0;  // Reset counter

                if (emitted) {
                    // Brief cooldown after emitting click event to avoid propagating
                    // noise into a new click sequence immediately.
                    button->state.ignore = true;
                    button->state.ignore_since = now;
                    button->state.ignore_duration = POST_CLICK_EVENT_COOLDOWN_INTERVAL;
                }
            }
        }
    }
}

#endif  // defined(USE_FEATURE_BUTTON)
