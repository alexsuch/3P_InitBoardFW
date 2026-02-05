#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <stdbool.h>
#include <stdint.h>

#include "app_errors.h"
#include "util/event_sender.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    APP_STATE_FIELD_NONE = 0,
    APP_STATE_FIELD_CURRENT_MODE = (1ULL << 49),
    APP_STATE_FIELD_SYSTEM_ERROR_CODE = (1ULL << 50),

    // WiFi state fields
    APP_STATE_FIELD_WIFI_STA_ACTIVE = (1ULL << 54),
    APP_STATE_FIELD_WIFI_AP_ACTIVE = (1ULL << 55),
    APP_STATE_FIELD_WIFI_STA_IP = (1ULL << 56),
    APP_STATE_FIELD_WIFI_STA_RSSI = (1ULL << 57),
    APP_STATE_FIELD_WIFI_STA_RETRY_COUNT = (1ULL << 58),
    APP_STATE_FIELD_WIFI_STA_STATE = (1ULL << 59),
    APP_STATE_FIELD_WIFI_AP_STATE = (1ULL << 60),
    APP_STATE_FIELD_WIFI_TX_POWER_LEVEL = (1ULL << 61),
    APP_STATE_FIELD_WIFI_TX_POWER_DBM = (1ULL << 62),
    APP_STATE_FIELD_WIFI_CONNECTION_TIME = (1ULL << 63),

    // WiFi tracker fields (for tracking state changes) - removed due to 64-bit limit
    // APP_STATE_FIELD_TRACKER_WIFI_STA_ACTIVE = (1ULL << 64),  // Exceeds 64-bit limit
    // APP_STATE_FIELD_TRACKER_WIFI_AP_ACTIVE = (1ULL << 65),   // Exceeds 64-bit limit
    // APP_STATE_FIELD_TRACKER_WIFI_STA_IP = (1ULL << 66),      // Exceeds 64-bit limit

} app_state_field_mask_e;

typedef enum {
    APP_MODE_ERROR = 0,
    APP_MODE_IDLE = 1,
    APP_MODE_LOGGING = 2,
    APP_MODE_DOWNLOAD = 3,
    APP_MODE_ONLINE = 4,
} app_mode_t;

typedef struct app_state_s {
    app_err_t system_error_code;
    app_mode_t current_mode;  // Current application mode

    // WiFi state properties
    uint8_t wifi_sta_active;        // STA connection status (0/1) - Provided by: WiFi module
    uint8_t wifi_ap_active;         // AP status (0/1) - Provided by: WiFi module
    uint32_t wifi_sta_ip;           // STA IP address - Provided by: WiFi module
    int8_t wifi_sta_rssi;           // STA RSSI in dBm - Provided by: WiFi module
    uint8_t wifi_sta_retry_count;   // STA retry count - Provided by: WiFi module
    uint8_t wifi_sta_state;         // STA state (wifi_mode_state_e) - Provided by: WiFi module
    uint8_t wifi_ap_state;          // AP state (wifi_mode_state_e) - Provided by: WiFi module
    uint8_t wifi_tx_power_level;    // TX power level (wifi_tx_power_level_t) - Provided by: WiFi module
    int8_t wifi_tx_power_dbm;       // TX power in dBm - Provided by: WiFi module
    uint32_t wifi_connection_time;  // Connection start time - Provided by: WiFi module

    es_t *event_sender;
    uint64_t changed_mask;
    bool update_in_progress;
    SemaphoreHandle_t state_mutex;
} app_state_t;

void app_state_init(void);
app_state_t *app_state_get_instance(void);
es_t *app_state_get_event_sender(void);

void app_state_begin_update(void);
void app_state_end_update(void);

void app_state_set_i32(app_state_field_mask_e field_mask, int32_t *field_ptr, int32_t value);
void app_state_set_u8(app_state_field_mask_e field_mask, uint8_t *field_ptr, uint8_t value);
void app_state_set_u32(app_state_field_mask_e field_mask, uint32_t *field_ptr, uint32_t value);
void app_state_set_i8(app_state_field_mask_e field_mask, int8_t *field_ptr, int8_t value);

void app_state_set_error(app_err_t error_code);

#ifdef __cplusplus
}
#endif
