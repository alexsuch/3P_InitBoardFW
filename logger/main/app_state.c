#include "app_state.h"

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <string.h>

#include "log.h"
#include "util/event_sender.h"

static const char *TAG = "STATE";

static app_state_t g_app_state;
static es_t g_event_sender;
static es_slot_t g_event_slots[16];
static uint16_t g_free_stack[16];

void app_state_init(void) {
    memset(&g_app_state, 0, sizeof(g_app_state));
    g_app_state.current_mode = APP_MODE_IDLE;  // Initialize with idle mode

    // Initialize event sender
    es_init(&g_event_sender, g_event_slots, 16, g_free_stack, NULL);
    g_app_state.event_sender = &g_event_sender;

    g_app_state.state_mutex = xSemaphoreCreateMutex();
}

app_state_t *app_state_get_instance(void) { return &g_app_state; }

es_t *app_state_get_event_sender(void) { return g_app_state.event_sender; }

void app_state_begin_update(void) {
    if (g_app_state.state_mutex) {
        xSemaphoreTake(g_app_state.state_mutex, portMAX_DELAY);
    }
    g_app_state.update_in_progress = true;
    g_app_state.changed_mask = 0;
}

void app_state_end_update(void) {
    g_app_state.update_in_progress = false;
    if (g_app_state.changed_mask != 0 && g_app_state.event_sender) {
        es_event_t event = {.id = 1,  // APP_STATE_CHANGED event
                            .payload = &g_app_state.changed_mask,
                            .size = sizeof(g_app_state.changed_mask),
                            .meta_flags = 0};
        es_emit(g_app_state.event_sender, &event);
    }
    if (g_app_state.state_mutex) {
        xSemaphoreGive(g_app_state.state_mutex);
    }
}

#define SET_FIELD(type, field_mask, ptr, value)       \
    do {                                              \
        if (*(ptr) != (type)(value)) {                \
            *(ptr) = (type)(value);                   \
            g_app_state.changed_mask |= (field_mask); \
        }                                             \
    } while (0)

void app_state_set_i32(app_state_field_mask_e field_mask, int32_t *field_ptr, int32_t value) { SET_FIELD(int32_t, field_mask, field_ptr, value); }

void app_state_set_u8(app_state_field_mask_e field_mask, uint8_t *field_ptr, uint8_t value) { SET_FIELD(uint8_t, field_mask, field_ptr, value); }

void app_state_set_u32(app_state_field_mask_e field_mask, uint32_t *field_ptr, uint32_t value) { SET_FIELD(uint32_t, field_mask, field_ptr, value); }

void app_state_set_i8(app_state_field_mask_e field_mask, int8_t *field_ptr, int8_t value) { SET_FIELD(int8_t, field_mask, field_ptr, value); }

void app_state_set_error(app_err_t error_code) {
    app_state_t *state = app_state_get_instance();
    app_state_begin_update();
    app_state_set_u8(APP_STATE_FIELD_CURRENT_MODE, (uint8_t *)&state->current_mode, APP_MODE_ERROR);
    app_state_set_i32(APP_STATE_FIELD_SYSTEM_ERROR_CODE, &state->system_error_code, error_code);
    app_state_end_update();
}
