#pragma once

#include <hal/gpio.h>
#include <stdbool.h>
#include <stdint.h>
#include <target.h>

#include "util/time-util.h"

typedef struct button_s button_t;

typedef enum {
    // Single button targets use only this button
    BUTTON_ID_ENTER = 0,
    BUTTON_ID_BOOT,
#if defined(USE_BUTTON_5WAY)
    BUTTON_ID_LEFT,
    BUTTON_ID_RIGHT,
    BUTTON_ID_UP,
    BUTTON_ID_DOWN,
#endif
} button_id_e;

typedef enum {
    BUTTON_EVENT_TYPE_SHORT_PRESS = 1,
    BUTTON_EVENT_TYPE_DOUBLE_PRESS,
    BUTTON_EVENT_TYPE_TRIPLE_PRESS,
    BUTTON_EVENT_TYPE_QUADRUPLE_PRESS,
    BUTTON_EVENT_TYPE_LONG_PRESS,
    BUTTON_EVENT_TYPE_REALLY_LONG_PRESS,
    BUTTON_EVENT_TYPE_SUPER_LONG_PRESS,
} button_event_type_e;

typedef struct button_event_s {
    const button_t *button;
    button_event_type_e type;
} button_event_t;

#define button_event_id(e) (e->button->id)

#define BUTTON_CONFIG_FROM_GPIO(io) ((button_config_t){.gpio = io})

typedef struct button_config_s {
    hal_gpio_t gpio;
} button_config_t;

typedef struct button_state_s {
    bool is_down;
    uint8_t click_num;
    bool long_press_sent;
    bool really_long_press_sent;
    bool super_long_press_sent;
    time_ticks_t down_since;
    time_ticks_t last_release_time;

    // Debounce / glitch filtering
    time_ticks_t press_candidate_since;
    time_ticks_t release_candidate_since;

    // Ignore window (used on startup and as a cooldown after "handled" events)
    bool ignore;
    time_ticks_t ignore_since;
    time_ticks_t ignore_duration;
} button_state_t;

typedef void (*button_callback_f)(const button_event_t *ev, void *user_data);

typedef struct button_s {
    button_config_t cfg;
    button_id_e id;
    button_callback_f callback;
    void *user_data;
    button_state_t state;
} button_t;

#if defined(USE_FEATURE_BUTTON)
void button_init(button_t *button);
void button_update(button_t *button);
#else
static inline void button_init(button_t *button) { (void)button; }
static inline void button_update(button_t *button) { (void)button; }
#endif
