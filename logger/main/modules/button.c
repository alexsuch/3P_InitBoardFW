#include <target.h>

#if defined(USE_FEATURE_BUTTON)

#include <log.h>

#include "button.h"
#include "util/time-util.h"

static const char *TAG = "button";

#define DOUBLE_PRESS_INTERVAL MILLIS_TO_TICKS(250)
#define DOUBLE_PRESS_TIMEOUT MILLIS_TO_TICKS(500)
#define LONG_PRESS_INTERVAL MILLIS_TO_TICKS(1000)
#define REALLY_LONG_PRESS_INTERVAL MILLIS_TO_TICKS(3000)
#define SUPER_LONG_PRESS_INTERVAL MILLIS_TO_TICKS(5000)

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

    button->state.is_down = button_is_down(button);
    button->state.ignore = (button->cfg.gpio == HAL_GPIO_NONE) ? true : button->state.is_down;
    button->state.long_press_sent = false;
    button->state.really_long_press_sent = false;
    button->state.super_long_press_sent = false;
    button->state.down_since = 0;
    button->state.last_release_time = 0;
    button->state.click_num = 0;
}

void button_update(button_t *button) {
    bool is_down = button_is_down(button);
    time_ticks_t now = time_ticks_now();

    if (button->state.ignore) {
        if (!is_down) {
            button->state.ignore = false;
        }
        return;
    }

    if (is_down) {
        if (!button->state.is_down) {
            // Button was just pressed
            button->state.is_down = true;
            button->state.down_since = now;
            button->state.long_press_sent = false;
            button->state.really_long_press_sent = false;
            button->state.super_long_press_sent = false;
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
        if (button->state.is_down) {
            // Button was just released
            button->state.is_down = false;
            button->state.last_release_time = now;

            if (!button->state.long_press_sent && !button->state.really_long_press_sent && !button->state.super_long_press_sent) {
                // This is a short press - increment click counter
                button->state.click_num++;

                // Check if this is a triple press
                if (button->state.click_num == 3) {
                    button_send_event(button, BUTTON_EVENT_TYPE_TRIPLE_PRESS);
                    button->state.click_num = 0;  // Reset counter
                } else {
                    // Single or double press - send event after timeout if no additional press
                    // We'll handle this in the timeout logic below
                }
            }
        } else {
            // Button is not pressed - check for press timeout
            if ((now - button->state.last_release_time) > DOUBLE_PRESS_TIMEOUT) {
                if (button->state.click_num == 2) {
                    button_send_event(button, BUTTON_EVENT_TYPE_DOUBLE_PRESS);
                } else if (button->state.click_num == 1) {
                    button_send_event(button, BUTTON_EVENT_TYPE_SHORT_PRESS);
                }
                button->state.click_num = 0;  // Reset counter
            }
        }
    }
}

#endif  // defined(USE_FEATURE_BUTTON)
