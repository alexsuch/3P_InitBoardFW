#include "wifi.h"

#include "target.h"  // Include target.h to get USE_WIFI definition

#if defined(USE_WIFI)

#include <limits.h>
#include <log.h>
#include <stdint.h>
#include <string.h>
#include <util/time-util.h>

#include "app_logic.h"
#include "app_state.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "io/io_manager.h"
#include "lwip/ip4_addr.h"
#include "lwip/ip_addr.h"
#include "nvs_flash.h"
#include "util/event_sender.h"
#include "wifi/captive_portal.h"

static const char *TAG = "WIFI";

// Static assert to ensure stack size is properly aligned for FreeRTOS
_Static_assert((TASK_STACK_SIZE_WIFI_EVENT % sizeof(StackType_t)) == 0, "TASK_STACK_SIZE_WIFI_EVENT must be expressed in bytes and aligned to StackType_t");

// BUG TODO: ESP-NOW does not work on STA mode or STA+AP mode

typedef struct {
    wifi_t *wifi;
    esp_event_base_t base;
    int32_t id;
    union {
        wifi_event_sta_disconnected_t sta_disconnected;
        ip_event_got_ip_t got_ip;
    } data;
} wifi_event_message_t;

static QueueHandle_t s_wifi_event_queue = NULL;
static TaskHandle_t s_wifi_event_task = NULL;

static void wifi_event_worker_task(void *param);
static void wifi_handle_event_message(const wifi_event_message_t *message);
static void wifi_handle_sta_start(wifi_t *wifi, app_state_t *state);
static void wifi_handle_sta_disconnected(wifi_t *wifi, app_state_t *state, const wifi_event_sta_disconnected_t *event);
static void wifi_handle_ap_start(wifi_t *wifi, app_state_t *state);
static void wifi_handle_ap_stop(wifi_t *wifi, app_state_t *state);
static void wifi_handle_sta_got_ip(wifi_t *wifi, app_state_t *state, const ip_event_got_ip_t *event);
static void wifi_update_sta_connection_state(app_state_t *state, uint8_t active, uint32_t ip);
static void wifi_update_ap_active_state(app_state_t *state, uint8_t active);
static void wifi_dispatch_event(wifi_t *wifi, app_event_t *event);
static void wifi_dispatch_simple_event(wifi_t *wifi, app_event_id_e id);
static void wifi_copy_string(char *dest, size_t dest_len, const char *src);
static void wifi_prepare_sta_config(const wifi_t *wifi, wifi_config_t *config);
static void wifi_prepare_ap_config(const wifi_t *wifi, wifi_config_t *config);
static esp_err_t wifi_reset_system_state(void);

static esp_err_t wifi_initialize_system_components(void);

static esp_err_t wifi_reset_system_state(void) {
    LOG_I(TAG, "Resetting WiFi system state for clean initialization...");

    // Stop WiFi if it's running
    esp_wifi_stop();

    // Deinitialize WiFi to reset the system state
    esp_wifi_deinit();

    // Also try to deinitialize the event loop if it exists
    esp_event_loop_delete_default();

    // Give the system a moment to clean up
    vTaskDelay(pdMS_TO_TICKS(200));

    return ESP_OK;
}

static esp_err_t wifi_initialize_system_components(void) {
    // Reset retry counter at function entry to allow fresh retries on each call
    static int event_loop_retry_count = 0;
    event_loop_retry_count = 0;

    // Initialize NVS flash storage first (required for WiFi)
    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES || nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        LOG_W(TAG, "NVS partition needs to be erased, retrying...");
        esp_err_t erase_err = nvs_flash_erase();
        if (erase_err != ESP_OK) {
            LOG_E(TAG, "Failed to erase NVS: %s", esp_err_to_name(erase_err));
            return erase_err;
        }
        nvs_err = nvs_flash_init();
    }
    if (nvs_err != ESP_OK) {
        LOG_E(TAG, "Failed to initialize NVS flash: %s", esp_err_to_name(nvs_err));
        return nvs_err;
    }
    LOG_I(TAG, "NVS flash initialized successfully");

    esp_err_t netif_err = esp_netif_init();
    if (netif_err == ESP_ERR_INVALID_STATE) {
        LOG_W(TAG, "esp_netif already initialized");
    } else if (netif_err != ESP_OK) {
        LOG_E(TAG, "Failed to initialize netif: %s", esp_err_to_name(netif_err));
        return netif_err;
    }

    const int MAX_EVENT_LOOP_RETRIES = 4;

    esp_err_t event_err = esp_event_loop_create_default();
    if (event_err == ESP_ERR_INVALID_STATE) {
        if (event_loop_retry_count >= MAX_EVENT_LOOP_RETRIES) {
            LOG_E(TAG, "Event loop creation failed after %d retries", MAX_EVENT_LOOP_RETRIES);
            return ESP_ERR_INVALID_STATE;
        }
        event_loop_retry_count++;
        LOG_W(TAG, "Event loop creation failed due to invalid state, resetting system and retrying...");
        esp_err_t reset_err = wifi_reset_system_state();
        if (reset_err != ESP_OK) {
            LOG_E(TAG, "Failed to reset WiFi system state: %s", esp_err_to_name(reset_err));
            return reset_err;
        }

        netif_err = esp_netif_init();
        if (netif_err == ESP_ERR_INVALID_STATE) {
            LOG_W(TAG, "esp_netif already initialized after reset");
        } else if (netif_err != ESP_OK) {
            LOG_E(TAG, "Failed to initialize netif after reset: %s", esp_err_to_name(netif_err));
            return netif_err;
        }

        event_err = esp_event_loop_create_default();
    }

    if (event_err != ESP_OK) {
        LOG_E(TAG, "Failed to create event loop: %s", esp_err_to_name(event_err));
        return event_err;
    }

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_err_t wifi_init_err = esp_wifi_init(&cfg);
    if (wifi_init_err != ESP_OK) {
        LOG_E(TAG, "Failed to initialize WiFi: %s", esp_err_to_name(wifi_init_err));
        return wifi_init_err;
    }

    return ESP_OK;
}

// WiFi TX Power Presets
static const struct {
    wifi_tx_power_level_t level;
    int8_t power_dbm;
    const char *description;
} wifi_tx_power_presets[] = {{WIFI_TX_POWER_MIN, 0, "Minimum (0 dBm)"},
                             {WIFI_TX_POWER_LOW, 5, "Low (5 dBm)"},
                             {WIFI_TX_POWER_MEDIUM, 10, "Medium (10 dBm)"},
                             {WIFI_TX_POWER_HIGH, 15, "High (15 dBm)"},
                             {WIFI_TX_POWER_MAX, 20, "Maximum (20 dBm)"}};

#define WIFI_TX_POWER_PRESET_COUNT (sizeof(wifi_tx_power_presets) / sizeof(wifi_tx_power_presets[0]))

// Event group to signal connection events from the ESP-IDF event handler
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1
static int s_retry_num = 0;

static esp_err_t _wifi_reconfigure(wifi_t *wifi);

static void wifi_copy_string(char *dest, size_t dest_len, const char *src) {
    if (dest == NULL || dest_len == 0) {
        return;
    }

    if (src == NULL) {
        dest[0] = '\0';
        return;
    }

    strncpy(dest, src, dest_len - 1);
    dest[dest_len - 1] = '\0';
}

static void wifi_prepare_sta_config(const wifi_t *wifi, wifi_config_t *config) {
    if (!wifi || !config) {
        return;
    }

    *config = (wifi_config_t){
        .sta =
            {
                .threshold.authmode = WIFI_AUTH_WPA2_PSK,
                .failure_retry_cnt = 3,
                .scan_method = WIFI_FAST_SCAN,
            },
    };

    wifi_copy_string((char *)config->sta.ssid, sizeof(config->sta.ssid), wifi->sta_ssid);
    wifi_copy_string((char *)config->sta.password, sizeof(config->sta.password), wifi->sta_pwd);
}

static void wifi_prepare_ap_config(const wifi_t *wifi, wifi_config_t *config) {
    if (!wifi || !config) {
        return;
    }

    *config = (wifi_config_t){
        .ap =
            {
                .ssid_len = strlen(wifi->ap_ssid),
                .max_connection = 4,
            },
    };

    wifi_copy_string((char *)config->ap.ssid, sizeof(config->ap.ssid), wifi->ap_ssid);
    wifi_copy_string((char *)config->ap.password, sizeof(config->ap.password), wifi->ap_pwd);
    config->ap.authmode = strlen(wifi->ap_pwd) == 0 ? WIFI_AUTH_OPEN : WIFI_AUTH_WPA_WPA2_PSK;
}

esp_err_t wifi_start_sta(wifi_t *wifi) {
    LOG_I(TAG, "Executing: Start STA Mode");
    wifi->sta_enabled = true;
    return _wifi_reconfigure(wifi);
}

esp_err_t wifi_start_ap(wifi_t *wifi) {
    LOG_I(TAG, "Executing: Start AP Mode");
    wifi->ap_enabled = true;
    return _wifi_reconfigure(wifi);
}

esp_err_t wifi_stop_sta(wifi_t *wifi) {
    LOG_I(TAG, "Executing: Stop STA Mode");
    wifi->sta_enabled = false;
    return _wifi_reconfigure(wifi);
}

esp_err_t wifi_stop_ap(wifi_t *wifi) {
    LOG_I(TAG, "Executing: Stop AP Mode");
    wifi->ap_enabled = false;
    return _wifi_reconfigure(wifi);
}

// Центральна функція реконфігурації
static esp_err_t _wifi_reconfigure(wifi_t *wifi) {
    wifi_mode_t current_mode;
    esp_err_t mode_err = esp_wifi_get_mode(&current_mode);

    // Handle the case when WiFi is not initialized (first time)
    if (mode_err == ESP_ERR_WIFI_NOT_INIT) {
        LOG_I(TAG, "WiFi not initialized yet, proceeding with first-time setup...");
        current_mode = WIFI_MODE_NULL;
    } else if (mode_err != ESP_OK) {
        LOG_E(TAG, "Failed to get WiFi mode: %s", esp_err_to_name(mode_err));
        return mode_err;
    }

    // Визначаємо новий режим на основі прапорів
    wifi_mode_t new_mode = 0;
    if (wifi->sta_enabled && wifi->ap_enabled)
        new_mode = WIFI_MODE_APSTA;
    else if (wifi->sta_enabled)
        new_mode = WIFI_MODE_STA;
    else if (wifi->ap_enabled)
        new_mode = WIFI_MODE_AP;
    else
        new_mode = WIFI_MODE_NULL;

    // Stop any captive portal services before reconfiguring the radios
    captive_portal_stop(&wifi->captive_portal);

    // Stop Wi-Fi before changing configuration (ignore benign states)
    esp_err_t stop_err = esp_wifi_stop();
    if (stop_err != ESP_OK && stop_err != ESP_ERR_WIFI_NOT_INIT && stop_err != ESP_ERR_WIFI_NOT_STARTED) {
        LOG_W(TAG, "Failed to stop WiFi: %s", esp_err_to_name(stop_err));
        // Don't abort on stop errors, just log and continue
    }

    if (new_mode == WIFI_MODE_NULL) {
        LOG_I(TAG, "All WiFi modes disabled. WiFi is now off.");
        return ESP_OK;
    }

    LOG_I(TAG, "Reconfiguring WiFi to mode: %d (STA:%d, AP:%d)", new_mode, wifi->sta_enabled, wifi->ap_enabled);
    mode_err = esp_wifi_set_mode(new_mode);
    if (mode_err != ESP_OK) {
        LOG_E(TAG, "Failed to set WiFi mode: %s", esp_err_to_name(mode_err));
        return mode_err;
    }

    // Переконфігуровуємо інтерфейси
    if (wifi->sta_enabled) {
        wifi_config_t sta_config;
        wifi_prepare_sta_config(wifi, &sta_config);
        esp_err_t sta_config_err = esp_wifi_set_config(WIFI_IF_STA, &sta_config);
        if (sta_config_err != ESP_OK) {
            LOG_E(TAG, "Failed to set STA config: %s", esp_err_to_name(sta_config_err));
            return sta_config_err;
        }
        LOG_I(TAG, "STA interface configured.");
    }

    if (wifi->ap_enabled) {
        wifi_config_t ap_config;
        wifi_prepare_ap_config(wifi, &ap_config);
        esp_err_t ap_config_err = esp_wifi_set_config(WIFI_IF_AP, &ap_config);
        if (ap_config_err != ESP_OK) {
            LOG_E(TAG, "Failed to set AP config: %s", esp_err_to_name(ap_config_err));
            return ap_config_err;
        }
        LOG_I(TAG, "AP interface configured.");
    }

    // Запускаємо Wi-Fi з новою конфігурацією
    return esp_wifi_start();
}

void wifi_load_config(wifi_t *wifi) {
    LOG_I(TAG, "Loading WiFi configuration from settings...");

    // Configure WiFi modes based on target.h defines
#ifdef WIFI_MODE_STA_ONLY
    wifi->sta_enabled = true;
    wifi->ap_enabled = false;
    wifi->start_ap_if_sta_failed = false;
    wifi->captive_portal_enabled = false;
#elif defined(WIFI_MODE_AP_ONLY)
    wifi->sta_enabled = false;
    wifi->ap_enabled = true;
    wifi->start_ap_if_sta_failed = false;
    wifi->captive_portal_enabled = true;
#elif defined(WIFI_MODE_STA_AP)
    wifi->sta_enabled = true;
    wifi->ap_enabled = true;
    wifi->start_ap_if_sta_failed = true;
    wifi->captive_portal_enabled = false;
#else
    // Default to STA+AP mode if no mode is defined
    wifi->sta_enabled = true;
    wifi->ap_enabled = true;
    wifi->start_ap_if_sta_failed = true;
    wifi->captive_portal_enabled = false;
#endif

    // Load WiFi credentials from target.h defines
    wifi_copy_string(wifi->sta_ssid, sizeof(wifi->sta_ssid), WIFI_STA_SSID);
    wifi_copy_string(wifi->sta_pwd, sizeof(wifi->sta_pwd), WIFI_STA_PASSWORD);
    wifi_copy_string(wifi->ap_ssid, sizeof(wifi->ap_ssid), WIFI_AP_SSID);
    wifi_copy_string(wifi->ap_pwd, sizeof(wifi->ap_pwd), WIFI_AP_PASSWORD);
    wifi->ap_channel = WIFI_AP_CHANNEL;

    // ESP-NOW MAC is now configured per logical port; no global setting
    wifi->espnow_enabled = false;

    // Load TX power level
    wifi->tx_power_level = WIFI_TX_POWER_MEDIUM;
    wifi->tx_power_dbm = wifi_tx_power_presets[wifi->tx_power_level].power_dbm;
}

// Historical note: the WiFi event callback previously caused a stack overflow in the
// sys_evt task when it performed logging, state updates, and queueing directly.
// The heavy lifting now happens in a dedicated worker task to keep the callback lean.

// WiFi events are routed through a worker task to keep the ESP-IDF callback lean
// and avoid stack pressure on the system event task.
static void event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    LOG_I(TAG, "[EVENT] Received event: base=%s, id=%ld", event_base, (long)event_id);
    
    wifi_event_message_t message = {0};
    message.wifi = (wifi_t *)arg;
    message.base = event_base;
    message.id = event_id;

    if (event_data != NULL) {
        if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
            memcpy(&message.data.sta_disconnected, event_data, sizeof(message.data.sta_disconnected));
        } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
            memcpy(&message.data.got_ip, event_data, sizeof(message.data.got_ip));
        }
    }

    if (s_wifi_event_queue == NULL) {
        LOG_E(TAG, "[EVENT] Queue is NULL! Events will be dropped.");
        return;
    }

    if (xQueueSend(s_wifi_event_queue, &message, 0) != pdTRUE) {
        LOG_W(TAG, "WiFi event queue full. Dropping event %ld.", (long)event_id);
    } else {
        LOG_I(TAG, "[EVENT] Event %ld queued successfully", (long)event_id);
    }
}

static void wifi_event_worker_task(void *param) {
    (void)param;

    wifi_event_message_t message;
    for (;;) {
        if (xQueueReceive(s_wifi_event_queue, &message, portMAX_DELAY) == pdTRUE) {
            wifi_handle_event_message(&message);
        }
    }
}

static void wifi_worker_delay_if_idle(uint32_t delay_ms) {
    if (delay_ms == 0) {
        return;
    }

    const TickType_t check_interval = pdMS_TO_TICKS(50);
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(delay_ms);

    for (;;) {
        if (s_wifi_event_queue != NULL && uxQueueMessagesWaiting(s_wifi_event_queue) > 0) {
            break;
        }

        TickType_t now = xTaskGetTickCount();
        if ((int32_t)(deadline - now) <= 0) {
            break;
        }

        TickType_t remaining = deadline - now;
        TickType_t sleep_ticks = (remaining < check_interval) ? remaining : check_interval;
        if (sleep_ticks == 0) {
            break;
        }

        vTaskDelay(sleep_ticks);
    }
}

static void wifi_update_sta_connection_state(app_state_t *state, uint8_t active, uint32_t ip) {
    if (state == NULL) {
        return;
    }

    app_state_begin_update();
    app_state_set_u8(APP_STATE_FIELD_WIFI_STA_ACTIVE, &state->wifi_sta_active, active);
    app_state_set_u32(APP_STATE_FIELD_WIFI_STA_IP, &state->wifi_sta_ip, ip);
    app_state_end_update();
}

static void wifi_update_sta_state(app_state_t *state, uint8_t sta_state, uint8_t retry_count, int8_t rssi) {
    if (state == NULL) {
        return;
    }

    app_state_begin_update();
    app_state_set_u8(APP_STATE_FIELD_WIFI_STA_STATE, &state->wifi_sta_state, sta_state);
    app_state_set_u8(APP_STATE_FIELD_WIFI_STA_RETRY_COUNT, &state->wifi_sta_retry_count, retry_count);
    app_state_set_i8(APP_STATE_FIELD_WIFI_STA_RSSI, &state->wifi_sta_rssi, rssi);
    app_state_end_update();
}

static void wifi_update_ap_state(app_state_t *state, uint8_t ap_state) {
    if (state == NULL) {
        return;
    }

    app_state_begin_update();
    app_state_set_u8(APP_STATE_FIELD_WIFI_AP_STATE, &state->wifi_ap_state, ap_state);
    app_state_end_update();
}

static void wifi_update_tx_power(app_state_t *state, uint8_t power_level, int8_t power_dbm) {
    if (state == NULL) {
        return;
    }

    app_state_begin_update();
    app_state_set_u8(APP_STATE_FIELD_WIFI_TX_POWER_LEVEL, &state->wifi_tx_power_level, power_level);
    app_state_set_i8(APP_STATE_FIELD_WIFI_TX_POWER_DBM, &state->wifi_tx_power_dbm, power_dbm);
    app_state_end_update();
}

static void wifi_update_connection_time(app_state_t *state, uint32_t connection_time) {
    if (state == NULL) {
        return;
    }

    app_state_begin_update();
    app_state_set_u32(APP_STATE_FIELD_WIFI_CONNECTION_TIME, &state->wifi_connection_time, connection_time);
    app_state_end_update();
}

static void wifi_update_ap_active_state(app_state_t *state, uint8_t active) {
    if (state == NULL) {
        return;
    }

    app_state_begin_update();
    app_state_set_u8(APP_STATE_FIELD_WIFI_AP_ACTIVE, &state->wifi_ap_active, active);
    app_state_end_update();
}

static void wifi_dispatch_event(wifi_t *wifi, app_event_t *event) {
    if (wifi == NULL || event == NULL) {
        return;
    }

    // Convert app_event_t to es_event_t and emit
    es_event_t es_event = {.id = event->id, .payload = &event->payload, .size = sizeof(event->payload), .meta_flags = 0};

    // Get the global event sender from app state
    es_t *event_sender = app_state_get_event_sender();
    if (event_sender) {
        es_emit(event_sender, &es_event);
    }
}

static void wifi_dispatch_simple_event(wifi_t *wifi, app_event_id_e id) {
    app_event_t event = {0};
    event.id = id;
    wifi_dispatch_event(wifi, &event);
}

static void wifi_handle_sta_start(wifi_t *wifi, app_state_t *state) {
    if (wifi->sta_state == WIFI_MODE_STATE_CONNECTING) {
        return;
    }

    esp_wifi_set_ps(WIFI_PS_NONE);
    wifi->sta_retry_count = 0;
    wifi->connection_start_time = time_micros_now() / 1000000;
    wifi->sta_state = WIFI_MODE_STATE_CONNECTING;

    // Update app state

    wifi_update_sta_state(state, wifi->sta_state, wifi->sta_retry_count, wifi->last_rssi);
    wifi_update_connection_time(state, wifi->connection_start_time);

    wifi_dispatch_simple_event(wifi, APP_EVENT_WIFI_STA_CONNECTING);
    esp_wifi_connect();
}

static void wifi_handle_sta_disconnected(wifi_t *wifi, app_state_t *state, const wifi_event_sta_disconnected_t *disconnect_event) {
    const uint8_t reason = disconnect_event->reason;
    const char *reason_str = WIFI_DISCONNECT_REASON_TO_STRING(reason);

    wifi->sta_state = WIFI_MODE_STATE_DISCONNECTED;
    wifi_update_sta_connection_state(state, 0, 0);
    wifi_update_sta_state(state, wifi->sta_state, wifi->sta_retry_count, wifi->last_rssi);
    wifi_dispatch_simple_event(wifi, APP_EVENT_WIFI_STA_DISCONNECTED);

    if (!wifi->sta_enabled) {
        LOG_I(TAG, "STA mode is disabled in settings. Skipping reconnection attempts.");
        wifi->sta_state = WIFI_MODE_STATE_STOPPED;
        return;
    }

    if (wifi->sta_retry_count < WIFI_RETRY_CONNECT_COUNT) {
        wifi->sta_retry_count++;

        uint32_t delay_ms;
        if (reason == WIFI_DISCONNECT_REASON_4WAY_HANDSHAKE_TIMEOUT) {
            delay_ms = 10000 + (wifi->sta_retry_count * 5000);
            LOG_W(TAG, "STA disconnected due to wrong password. Using longer delay.");
        } else if (reason == WIFI_DISCONNECT_REASON_AUTH_EXPIRE || reason == WIFI_DISCONNECT_REASON_CONNECTION_FAIL) {
            delay_ms = (wifi->sta_retry_count <= 4) ? (3000 << (wifi->sta_retry_count - 1)) : 30000;
        } else {
            delay_ms = 5000 + (wifi->sta_retry_count * 2000);
        }

        LOG_W(TAG, "STA disconnected. Reason: %s (code %d). Retrying to connect (attempt %d/%d)...", reason_str, reason, wifi->sta_retry_count,
              WIFI_RETRY_CONNECT_COUNT);
        LOG_I(TAG, "Waiting %lu ms before retry attempt %d/%d", delay_ms, wifi->sta_retry_count, WIFI_RETRY_CONNECT_COUNT);

        // Update app state with retry count
        wifi_update_sta_state(state, wifi->sta_state, wifi->sta_retry_count, wifi->last_rssi);

        wifi_worker_delay_if_idle(delay_ms);
        esp_wifi_connect();
        return;
    }

    LOG_E(TAG, "STA disconnected. Reason: %s (code %d). Failed to connect after %d attempts.", reason_str, reason, WIFI_RETRY_CONNECT_COUNT);

    if (wifi->reset_attempts < WIFI_MAX_RESET_ATTEMPTS) {
        wifi->reset_attempts++;
        LOG_W(TAG, "Attempting WiFi stack reset (attempt %d/%d) to recover from persistent connection issues", wifi->reset_attempts, WIFI_MAX_RESET_ATTEMPTS);

        esp_err_t reset_err = wifi_reset_stack(wifi);
        if (reset_err == ESP_OK) {
            LOG_I(TAG, "WiFi stack reset successful, connection should be re-established");
            return;
        }

        LOG_E(TAG, "WiFi stack reset failed: %s", esp_err_to_name(reset_err));
    } else {
        LOG_E(TAG, "WiFi stack reset attempts exhausted (%d/%d). Giving up on connection recovery.", wifi->reset_attempts, WIFI_MAX_RESET_ATTEMPTS);
        wifi->reset_attempts = 0;
    }

    wifi->sta_state = WIFI_MODE_STATE_FAILED;
    wifi_dispatch_simple_event(wifi, APP_EVENT_WIFI_STA_CONNECT_FAILED);

    if (wifi->sta_enabled && wifi->start_ap_if_sta_failed) {
        LOG_I(TAG, "STA connection failed and STA is still enabled. Scheduling AP mode as a fallback.");
        wifi->ap_enabled = true;
        wifi->restart_needed = true;

        LOG_I(TAG, "WiFi restart scheduled via app logic to avoid race conditions.");
        app_logic_t *app_logic = get_global_app_logic_instance();
        if (app_logic) {
            app_logic_send_command(app_logic, APP_CMD_RECONFIGURE_WIFI);
        }
    }
}

static void wifi_handle_ap_start(wifi_t *wifi, app_state_t *state) {
    wifi->ap_state = WIFI_MODE_STATE_CONNECTED;
    LOG_I(TAG, "AP mode started successfully");

    uint8_t primary_channel;
    wifi_second_chan_t second_channel;
    if (esp_wifi_get_channel(&primary_channel, &second_channel) == ESP_OK) {
        LOG_I(TAG, "AP started on channel %d, second channel %d", primary_channel, second_channel);
    }

    wifi_update_ap_active_state(state, 1);
    wifi_update_ap_state(state, wifi->ap_state);
    wifi_dispatch_simple_event(wifi, APP_EVENT_WIFI_AP_STARTED);

    if (wifi->captive_portal_enabled) {
        // Only start captive portal if WiFi is in AP-only mode (not STA or STA_AP)
        wifi_mode_t current_mode;
        esp_err_t mode_err = esp_wifi_get_mode(&current_mode);

        if (mode_err == ESP_OK && current_mode == WIFI_MODE_AP) {
            esp_netif_t *ap_netif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
            if (ap_netif) {
                esp_netif_ip_info_t ip_info = {0};
                if (esp_netif_get_ip_info(ap_netif, &ip_info) == ESP_OK) {
                    ip4_addr_t lwip_ip;
                    IP4_ADDR(&lwip_ip, esp_ip4_addr1(&ip_info.ip), esp_ip4_addr2(&ip_info.ip), esp_ip4_addr3(&ip_info.ip), esp_ip4_addr4(&ip_info.ip));
                    // COMMENTED OUT: Captive portal start on AP mode
                    // esp_err_t dns_err = captive_portal_start(&wifi->captive_portal, &lwip_ip, wifi->captive_portal_http_server);
                    // if (dns_err != ESP_OK) {
                    //     LOG_W(TAG, "Failed to start captive portal DNS: %s", esp_err_to_name(dns_err));
                    // } else {
                    //     LOG_I(TAG, "Captive portal started for AP-only mode");
                    // }
                    LOG_I(TAG, "Captive portal disabled for AP mode");
                } else {
                    LOG_W(TAG, "Failed to obtain AP interface IP information for captive portal");
                }
            } else {
                LOG_W(TAG, "AP network interface not available for captive portal setup");
            }
        } else {
            LOG_I(TAG, "Captive portal disabled: WiFi is not in AP-only mode (current mode: %d)", current_mode);
        }
    }
}

static void wifi_handle_ap_stop(wifi_t *wifi, app_state_t *state) {
    wifi->ap_state = WIFI_MODE_STATE_STOPPED;
    LOG_I(TAG, "AP mode stopped");

    wifi_update_ap_active_state(state, 0);
    wifi_update_ap_state(state, wifi->ap_state);
    wifi_dispatch_simple_event(wifi, APP_EVENT_WIFI_AP_STOPPED);

    captive_portal_stop(&wifi->captive_portal);
}

static void wifi_handle_sta_got_ip(wifi_t *wifi, app_state_t *state, const ip_event_got_ip_t *event) {
    LOG_I(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));

    LOG_I(TAG, "STA connection successful. Resetting retry counter.");
    wifi->sta_retry_count = 0;
    wifi->sta_state = WIFI_MODE_STATE_CONNECTED;

    if (wifi->reset_attempts > 0) {
        LOG_I(TAG, "Resetting WiFi stack reset attempts counter (was %d)", wifi->reset_attempts);
        wifi->reset_attempts = 0;
    }

    wifi->ip = event->ip_info.ip.addr;
    wifi->connection_start_time = time_micros_now() / 1000000;

    uint8_t primary_channel;
    wifi_second_chan_t second_channel;
    if (esp_wifi_get_channel(&primary_channel, &second_channel) == ESP_OK) {
        LOG_I(TAG, "STA connected on channel %d, second channel %d", primary_channel, second_channel);
    }

    wifi_update_sta_connection_state(state, 1, wifi->ip);
    wifi_update_sta_state(state, wifi->sta_state, wifi->sta_retry_count, wifi->last_rssi);
    wifi_update_connection_time(state, wifi->connection_start_time);

    app_event_t app_event = {0};
    app_event.id = APP_EVENT_WIFI_STA_CONNECTED;
    app_event.payload.wifi_connected.ip = wifi->ip;
    wifi_dispatch_event(wifi, &app_event);
}

static void wifi_handle_event_message(const wifi_event_message_t *message) {
    wifi_t *wifi = message->wifi;
    app_state_t *state = app_state_get_instance();
    if (message->base == WIFI_EVENT) {
        switch (message->id) {
            case WIFI_EVENT_STA_START:
                wifi_handle_sta_start(wifi, state);
                break;
            case WIFI_EVENT_STA_DISCONNECTED:
                wifi_handle_sta_disconnected(wifi, state, &message->data.sta_disconnected);
                break;
            case WIFI_EVENT_AP_START:
                wifi_handle_ap_start(wifi, state);
                break;
            case WIFI_EVENT_AP_STOP:
                wifi_handle_ap_stop(wifi, state);
                break;
            default:
                break;
        }
    } else if (message->base == IP_EVENT) {
        switch (message->id) {
            case IP_EVENT_STA_GOT_IP:
                wifi_handle_sta_got_ip(wifi, state, &message->data.got_ip);
                break;
            default:
                break;
        }
    }
}

esp_err_t wifi_init(wifi_t *wifi) {
    LOG_I(TAG, "Initializing WiFi Module...");
    static bool is_initialized = false;

    // Print heap size and free size and minimum free size
    LOG_I(TAG, "Free heap: %zu bytes", esp_get_free_heap_size());
    LOG_I(TAG, "Minimum free heap: %zu bytes", esp_get_minimum_free_heap_size());

    if (!is_initialized) {
        // Check if WiFi is already initialized by ESP-IDF
        wifi_mode_t current_mode;
        esp_err_t mode_err = esp_wifi_get_mode(&current_mode);

        if (mode_err == ESP_ERR_WIFI_NOT_INIT) {
            // WiFi is not initialized, proceed with initialization
            LOG_I(TAG, "WiFi not initialized, performing first-time setup...");
            esp_err_t init_err = wifi_initialize_system_components();
            if (init_err != ESP_OK) {
                return init_err;
            }
        } else if (mode_err == ESP_OK) {
            // WiFi is already initialized by ESP-IDF, just mark our flag
            LOG_I(TAG, "WiFi already initialized by ESP-IDF, skipping initialization...");
            is_initialized = true;
        } else {
            // WiFi is in an invalid state (like ESP_ERR_INVALID_STATE), reset and try again
            LOG_W(TAG, "WiFi is in invalid state (%s), resetting system state...", esp_err_to_name(mode_err));
            esp_err_t reset_err = wifi_reset_system_state();
            if (reset_err != ESP_OK) {
                LOG_E(TAG, "Failed to reset WiFi system state: %s", esp_err_to_name(reset_err));
                return reset_err;
            }

            // Now try initialization again
            LOG_I(TAG, "Retrying WiFi initialization after system reset...");
            esp_err_t init_err = wifi_initialize_system_components();
            if (init_err != ESP_OK) {
                return init_err;
            }
        }

        if (s_wifi_event_queue == NULL) {
            s_wifi_event_queue = xQueueCreate(4, sizeof(wifi_event_message_t));
            if (s_wifi_event_queue == NULL) {
                LOG_E(TAG, "Failed to create WiFi event queue");
                return ESP_ERR_NO_MEM;
            }
        }

        if (s_wifi_event_task == NULL) {
            BaseType_t task_created = xTaskCreate(wifi_event_worker_task, "wifi_evt", TASK_STACK_SIZE_WIFI_EVENT / sizeof(StackType_t), NULL,
                                                  TASK_PRIORITY_WIFI_EVENT, &s_wifi_event_task);
            if (task_created != pdPASS) {
                LOG_E(TAG, "Failed to create WiFi event task");
                return ESP_FAIL;
            }
        }

        esp_err_t wifi_event_err = esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, wifi, NULL);
        if (wifi_event_err != ESP_OK) {
            LOG_E(TAG, "Failed to register WiFi event handler: %s", esp_err_to_name(wifi_event_err));
            return wifi_event_err;
        }

        esp_err_t ip_event_err = esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, wifi, NULL);
        if (ip_event_err != ESP_OK) {
            LOG_E(TAG, "Failed to register IP event handler: %s", esp_err_to_name(ip_event_err));
            return ip_event_err;
        }

        // Only mark as initialized if all steps succeeded
        is_initialized = true;
    }

    memset(wifi, 0, sizeof(wifi_t));
    // Notifier removed: WiFi publishes to AppBus only

    // Initialize new fields
    wifi->last_rssi = 0;
    wifi->connection_start_time = 0;
    wifi->sta_retry_count = 0;
    wifi->reset_attempts = 0;

    captive_portal_init(&wifi->captive_portal);

    // Load configuration from settings for the first time
    wifi_load_config(wifi);

    LOG_I(TAG, "WiFi Module Initialized.");
    return ESP_OK;
}

// ESP-NOW callbacks (can be left empty for now if only sending)
static void espnow_send_cb(const wifi_tx_info_t *wifi_tx_info, esp_now_send_status_t status) {
    const uint8_t *mac_addr = wifi_tx_info ? wifi_tx_info->des_addr : NULL;
    if (mac_addr) {
        LOG_I(TAG, "ESPNOW send callback: MAC: %02x:%02x:%02x:%02x:%02x:%02x, status: %s", mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4],
              mac_addr[5], status == ESP_NOW_SEND_SUCCESS ? "SUCCESS" : "FAIL");
    } else {
        LOG_I(TAG, "ESPNOW send callback: status: %s", status == ESP_NOW_SEND_SUCCESS ? "SUCCESS" : "FAIL");
    }
    // Optional: handle send status
    (void)wifi_tx_info;
    LOG_I(TAG, "ESPNOW send callback: status: %s", status == ESP_NOW_SEND_SUCCESS ? "SUCCESS" : "FAIL");
}

static void espnow_recv_cb(const esp_now_recv_info_t *esp_now_info, const uint8_t *data, int data_len) {
    if (!data || data_len <= 0) {
        LOG_E(TAG, "ESPNOW receive callback: data is NULL or data_len is <= 0");
        return;
    }

    const uint8_t *src_mac = esp_now_info ? esp_now_info->src_addr : NULL;
    int rssi = INT_MIN;

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 4, 0)
    if (esp_now_info && esp_now_info->rx_ctrl) {
        rssi = esp_now_info->rx_ctrl->rssi;
    }
#endif

    LOG_D(TAG, "ESPNOW received %d bytes from %02x:%02x:%02x:%02x:%02x:%02x, RSSI: %d", data_len, src_mac ? src_mac[0] : 0, src_mac ? src_mac[1] : 0,
          src_mac ? src_mac[2] : 0, src_mac ? src_mac[3] : 0, src_mac ? src_mac[4] : 0, src_mac ? src_mac[5] : 0, rssi);
}

esp_err_t wifi_start(wifi_t *wifi) {
    wifi_mode_t current_mode = WIFI_MODE_NULL;
    esp_err_t mode_err = esp_wifi_get_mode(&current_mode);

    // Handle the case when WiFi is not initialized (first time)
    if (mode_err == ESP_ERR_WIFI_NOT_INIT) {
        LOG_I(TAG, "WiFi not initialized yet, proceeding with first-time setup...");
        current_mode = WIFI_MODE_NULL;
    } else if (mode_err != ESP_OK) {
        LOG_E(TAG, "Failed to get WiFi mode: %s", esp_err_to_name(mode_err));
        return mode_err;
    }

    // If WiFi is already active, we need to stop it before reconfiguring.
    // This makes this function safe to call for mode changes.
    // NOTE: This will stop ESP-NOW if it is running, and it will NOT be
    // automatically restarted by this function. The caller must handle ESP-NOW restart.
    if (current_mode != WIFI_MODE_NULL) {
        LOG_I(TAG, "WiFi is already running. Stopping to reconfigure...");
        esp_err_t stop_err = esp_wifi_stop();
        if (stop_err != ESP_OK && stop_err != ESP_ERR_WIFI_NOT_INIT && stop_err != ESP_ERR_WIFI_NOT_STARTED) {
            LOG_W(TAG, "Failed to stop WiFi: %s", esp_err_to_name(stop_err));
            // Don't abort on stop errors, just log and continue
        }

        // Destroy the netifs to allow for clean re-creation
        esp_netif_t *sta_netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
        if (sta_netif) {
            esp_netif_destroy(sta_netif);
        }
        esp_netif_t *ap_netif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
        if (ap_netif) {
            esp_netif_destroy(ap_netif);
        }
        vTaskDelay(pdMS_TO_TICKS(100));  // Give it a moment to tear down.
    }
    // WiFi Initialization Sequence:
    // 1. Set WiFi mode (esp_wifi_set_mode)
    // 2. Configure interfaces (esp_wifi_set_config)
    // 3. Start WiFi (esp_wifi_start)
    // 4. Set TX power (esp_wifi_set_max_tx_power) - MUST be before connections
    // 5. Connect to AP (esp_wifi_connect) - triggered by WIFI_EVENT_STA_START

    wifi_mode_t mode = WIFI_MODE_STA;

    if (wifi->ap_enabled && wifi->sta_enabled) {
        mode = WIFI_MODE_APSTA;
    } else if (wifi->ap_enabled) {
        mode = WIFI_MODE_AP;
    } else if (wifi->sta_enabled) {
        mode = WIFI_MODE_STA;
    } else {
        LOG_I(TAG, "WiFi is disabled in settings.");
        return ESP_OK;
    }

    LOG_I(TAG, "Setting WiFi mode to: %d", mode);
    esp_err_t mode_set_err = esp_wifi_set_mode(mode);
    if (mode_set_err != ESP_OK) {
        LOG_E(TAG, "Failed to set WiFi mode: %s", esp_err_to_name(mode_set_err));
        return mode_set_err;
    }

    // ADDED: Channel coordination for APSTA mode
    uint8_t ap_channel = 0;  // Default to auto channel
    if (wifi->ap_enabled) {
        // AP can use either channel 0 (auto) or defined channel
        ap_channel = wifi->espnow_enabled ? wifi->ap_channel : 0;
        LOG_I(TAG, "AP will use channel %d (%s)", ap_channel, ap_channel == 0 ? "auto" : "fixed");
    }

    if (wifi->sta_enabled) {
        esp_netif_create_default_wifi_sta();
        wifi_config_t sta_config;
        wifi_prepare_sta_config(wifi, &sta_config);

        // ADDED: Channel coordination - STA should avoid AP channel when in APSTA mode
        if (wifi->ap_enabled && ap_channel > 0) {
            // Configure STA to avoid the AP channel
            sta_config.sta.channel = 0;  // Let STA auto-select, but we'll configure scan to avoid AP channel
            sta_config.sta.scan_method = WIFI_FAST_SCAN;
            // Note: We can't directly exclude channels in ESP-IDF, but we can optimize scanning
            LOG_I(TAG, "STA configured to auto-select channel (avoiding AP channel %d)", ap_channel);
        } else {
            // Normal STA configuration
            sta_config.sta.scan_method = WIFI_FAST_SCAN;
            LOG_I(TAG, "STA configured for normal channel selection");
        }

        esp_err_t sta_config_err = esp_wifi_set_config(WIFI_IF_STA, &sta_config);
        if (sta_config_err != ESP_OK) {
            LOG_E(TAG, "Failed to set STA config: %s", esp_err_to_name(sta_config_err));
            return sta_config_err;
        }

        // ESP-NOW MAC address will be set in wifi_start_espnow() using logical port configuration

        LOG_I(TAG, "STA interface configured for SSID: %s", wifi->sta_ssid);
    }

    if (wifi->ap_enabled) {
        esp_netif_t *ap_netif = esp_netif_create_default_wifi_ap();
        wifi_config_t ap_config;
        wifi_prepare_ap_config(wifi, &ap_config);

        // ADDED: Set AP channel based on coordination logic
        ap_config.ap.channel = ap_channel;
        if (ap_channel == 0) {
            LOG_I(TAG, "AP interface configured for SSID: %s on auto channel", wifi->ap_ssid);
        } else {
            LOG_I(TAG, "AP interface configured for SSID: %s on fixed channel %d", wifi->ap_ssid, ap_channel);
        }

        // Configure AP with static IP address 10.0.0.1
        // First disable DHCP server on AP interface
        esp_err_t dhcp_err = esp_netif_dhcps_stop(ap_netif);
        if (dhcp_err != ESP_OK) {
            LOG_W(TAG, "Failed to stop DHCP server: %s", esp_err_to_name(dhcp_err));
        }

        esp_netif_ip_info_t ip_info;
        IP4_ADDR(&ip_info.ip, 10, 0, 0, 1);            // AP IP: 10.0.0.1
        IP4_ADDR(&ip_info.gw, 10, 0, 0, 1);            // Gateway: 10.0.0.1
        IP4_ADDR(&ip_info.netmask, 255, 255, 255, 0);  // Netmask: 255.255.255.0

        esp_err_t ip_err = esp_netif_set_ip_info(ap_netif, &ip_info);
        if (ip_err != ESP_OK) {
            LOG_E(TAG, "Failed to set AP IP address: %s", esp_err_to_name(ip_err));
        } else {
            LOG_I(TAG, "AP IP address set to 10.0.0.1");
        }

        // Restart DHCP server after setting static IP
        dhcp_err = esp_netif_dhcps_start(ap_netif);
        if (dhcp_err != ESP_OK) {
            LOG_W(TAG, "Failed to start DHCP server: %s", esp_err_to_name(dhcp_err));
        } else {
            LOG_I(TAG, "DHCP server restarted successfully");
        }

        if (wifi->espnow_enabled) {
            // ESP-NOW requires fixed channel, so use the configured channel
            ap_config.ap.channel = wifi->ap_channel;
            LOG_I(TAG, "AP interface configured for SSID: %s on fixed channel %d (for ESP-NOW)", wifi->ap_ssid, ap_config.ap.channel);
            // ESP-NOW MAC address will be set in wifi_start_espnow() using logical port configuration
        }

        esp_err_t ap_config_err = esp_wifi_set_config(WIFI_IF_AP, &ap_config);
        if (ap_config_err != ESP_OK) {
            LOG_E(TAG, "Failed to set AP config: %s", esp_err_to_name(ap_config_err));
            return ap_config_err;
        }
        // AP state will be set to CONNECTED when WIFI_EVENT_AP_START is received
        wifi->ap_state = WIFI_MODE_STATE_STARTED;
    }

    // MODIFIED: Replaced direct call with a retry loop for robustness
    esp_err_t start_err;
    int retry_count = 0;
    while (retry_count < WIFI_RETRY_CONNECT_COUNT) {
        start_err = esp_wifi_start();
        if (start_err == ESP_OK) {
            LOG_I(TAG, "esp_wifi_start() call was successful.");

            // Apply TX power setting immediately after WiFi starts, before any connections
            wifi_set_tx_power_level(wifi, wifi->tx_power_level);

            // Configure WiFi connection optimizations
            if (wifi->sta_enabled) {
                // Set longer beacon timeout to reduce disconnections due to temporary signal issues
                esp_wifi_set_inactive_time(WIFI_IF_STA, 10);  // 10 seconds instead of default 6s

                // Disable power saving to improve connection stability
                esp_wifi_set_ps(WIFI_PS_NONE);

                LOG_I(TAG, "WiFi STA optimizations applied: 10s beacon timeout, power saving disabled");
            }

            return ESP_OK;
        }

        retry_count++;
        LOG_E(TAG, "Failed to start WiFi with error: %s. Retrying in 2s... (attempt %d/%d)", esp_err_to_name(start_err), retry_count, WIFI_RETRY_CONNECT_COUNT);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    LOG_E(TAG, "Failed to start WiFi after %d attempts. Error: %s", WIFI_RETRY_CONNECT_COUNT, esp_err_to_name(start_err));
    if (wifi->ap_enabled) wifi->ap_state = WIFI_MODE_STATE_FAILED;
    if (wifi->sta_enabled) wifi->sta_state = WIFI_MODE_STATE_FAILED;

    return start_err;
}

esp_err_t wifi_stop(wifi_t *wifi) {
    LOG_I(TAG, "Stopping WiFi STA and AP services...");
    time_micros_t now = time_micros_now();

    captive_portal_stop(&wifi->captive_portal);

    esp_err_t err = esp_wifi_stop();

    app_state_t *state = app_state_get_instance();
    app_state_begin_update();
    app_state_set_u8(APP_STATE_FIELD_WIFI_STA_ACTIVE, &state->wifi_sta_active, 0);
    app_state_set_u8(APP_STATE_FIELD_WIFI_AP_ACTIVE, &state->wifi_ap_active, 0);
    app_state_set_u32(APP_STATE_FIELD_WIFI_STA_IP, &state->wifi_sta_ip, 0);
    app_state_end_update();

    wifi->sta_state = WIFI_MODE_STATE_STOPPED;
    wifi->ap_state = WIFI_MODE_STATE_STOPPED;

    // Destroy default netif to allow re-creation on next start
    esp_netif_t *sta_netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (sta_netif) esp_netif_destroy(sta_netif);
    esp_netif_t *ap_netif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
    if (ap_netif) esp_netif_destroy(ap_netif);

    wifi_dispatch_simple_event(wifi, APP_EVENT_WIFI_DISCONNECTED);

    return err;
}

esp_err_t wifi_get_device_mac_current(wifi_interface_t ifx, uint8_t out[6]) { return esp_wifi_get_mac(ifx, out); }

// WiFi TX Power Control Implementation
esp_err_t wifi_set_tx_power_level(wifi_t *wifi, wifi_tx_power_level_t level) {
    if (level >= WIFI_TX_POWER_PRESET_COUNT) {
        LOG_E(TAG, "Invalid TX power level: %d", level);
        return ESP_ERR_INVALID_ARG;
    }

    // Check if WiFi is started before setting TX power
    wifi_mode_t current_mode;
    esp_err_t mode_err = esp_wifi_get_mode(&current_mode);
    if (mode_err != ESP_OK) {
        LOG_E(TAG, "Failed to get WiFi mode: %s", esp_err_to_name(mode_err));
        return mode_err;
    }

    int8_t power_dbm = wifi_tx_power_presets[level].power_dbm;
    int8_t power_internal = power_dbm * 4;  // Convert to 0.25 dBm units

    esp_err_t err = esp_wifi_set_max_tx_power(power_internal);
    if (err == ESP_OK) {
        LOG_I(TAG, "TX power set to %s (%d dBm)", wifi_tx_power_presets[level].description, power_dbm);
        wifi->tx_power_dbm = power_dbm;
        wifi->tx_power_level = level;

        // Update app state
        app_state_t *state = app_state_get_instance();
        wifi_update_tx_power(state, wifi->tx_power_level, wifi->tx_power_dbm);

        // TODO: Add settings persistence when settings system is implemented
        // settings_begin_update();
        // setting_set_u8(settings_get_key(SETTING_KEY_WIFI_TX_POWER_LEVEL), level);
        // settings_end_update();
    } else {
        LOG_E(TAG, "Failed to set TX power: %s", esp_err_to_name(err));
    }

    return err;
}

wifi_tx_power_level_t wifi_get_tx_power_level(wifi_t *wifi) {
    int8_t power_internal;
    esp_err_t err = esp_wifi_get_max_tx_power(&power_internal);
    if (err == ESP_OK) {
        int8_t power_dbm = power_internal / 4;  // Convert from internal units

        // Find matching preset
        for (int i = 0; i < WIFI_TX_POWER_PRESET_COUNT; i++) {
            if (wifi_tx_power_presets[i].power_dbm == power_dbm) {
                wifi->tx_power_level = wifi_tx_power_presets[i].level;
                wifi->tx_power_dbm = power_dbm;
                return wifi_tx_power_presets[i].level;
            }
        }
    }

    // Default fallback
    wifi->tx_power_level = WIFI_TX_POWER_MEDIUM;
    wifi->tx_power_dbm = wifi_tx_power_presets[WIFI_TX_POWER_MEDIUM].power_dbm;
    return WIFI_TX_POWER_MEDIUM;
}

// Function to get current RSSI and monitor connection quality
int8_t wifi_get_rssi(wifi_t *wifi) {
    if (wifi->sta_state != WIFI_MODE_STATE_CONNECTED) {
        return 0;  // Not connected
    }

    wifi_ap_record_t ap_info;
    esp_err_t err = esp_wifi_sta_get_ap_info(&ap_info);
    if (err == ESP_OK) {
        wifi->last_rssi = ap_info.rssi;

        // Update app state with RSSI
        app_state_t *state = app_state_get_instance();
        wifi_update_sta_state(state, wifi->sta_state, wifi->sta_retry_count, wifi->last_rssi);

        // Log warning if signal is weak
        if (ap_info.rssi < WIFI_RSSI_THRESHOLD) {
            LOG_W(TAG, "Weak WiFi signal detected: %d dBm (threshold: %d dBm)", ap_info.rssi, WIFI_RSSI_THRESHOLD);
        }

        return ap_info.rssi;
    } else {
        LOG_W(TAG, "Failed to get RSSI: %s", esp_err_to_name(err));
        return wifi->last_rssi;  // Return last known value
    }
}

esp_err_t wifi_reset_stack(wifi_t *wifi) {
    LOG_W(TAG, "Performing WiFi stack reset to recover from connection issues...");

    // Store current configuration
    bool sta_enabled = wifi->sta_enabled;
    bool ap_enabled = wifi->ap_enabled;
    bool espnow_enabled = wifi->espnow_enabled;
    wifi_tx_power_level_t tx_power = wifi->tx_power_level;

    // Stop ESP-NOW if running
    if (wifi->espnow_enabled) {
        wifi_stop_espnow(wifi);
    }

    // Stop WiFi completely
    esp_err_t stop_err = wifi_stop(wifi);
    if (stop_err != ESP_OK) {
        LOG_W(TAG, "WiFi stop returned error: %s, continuing with reset", esp_err_to_name(stop_err));
    }

    // Clear any pending events and wait for stack to settle
    vTaskDelay(pdMS_TO_TICKS(1000));

    // Reset internal state counters
    wifi->sta_retry_count = 0;
    wifi->connection_start_time = 0;
    wifi->last_rssi = 0;
    wifi->reset_attempts = 0;  // Reset reset attempts counter after successful reset

    // Restore configuration
    wifi->sta_enabled = sta_enabled;
    wifi->ap_enabled = ap_enabled;
    wifi->espnow_enabled = espnow_enabled;
    wifi->tx_power_level = tx_power;

    // Restart WiFi with fresh configuration
    esp_err_t start_err = wifi_start(wifi);
    if (start_err != ESP_OK) {
        LOG_E(TAG, "Failed to restart WiFi after reset: %s", esp_err_to_name(start_err));
        return start_err;
    }

    // Restart ESP-NOW if it was enabled
    if (espnow_enabled) {
        esp_err_t espnow_err = wifi_start_espnow(wifi);
        if (espnow_err != ESP_OK) {
            LOG_W(TAG, "Failed to restart ESP-NOW after WiFi reset: %s", esp_err_to_name(espnow_err));
        }
    }

    LOG_I(TAG, "WiFi stack reset completed successfully");
    return ESP_OK;
}

esp_err_t wifi_force_reconnect(wifi_t *wifi) {
    if (!wifi->sta_enabled) {
        LOG_W(TAG, "STA mode not enabled, cannot force reconnect");
        return ESP_ERR_INVALID_STATE;
    }

    LOG_I(TAG, "Forcing WiFi STA reconnection...");

    // Disconnect current connection if any
    esp_err_t disconnect_err = esp_wifi_disconnect();
    if (disconnect_err != ESP_OK && disconnect_err != ESP_ERR_WIFI_NOT_STARTED) {
        LOG_W(TAG, "Failed to disconnect: %s", esp_err_to_name(disconnect_err));
    }

    // Reset retry counter for fresh attempt
    wifi->sta_retry_count = 0;
    wifi->sta_state = WIFI_MODE_STATE_DISCONNECTED;

    // Wait a moment for disconnect to complete
    vTaskDelay(pdMS_TO_TICKS(500));

    // Attempt to connect
    esp_err_t connect_err = esp_wifi_connect();
    if (connect_err != ESP_OK) {
        LOG_E(TAG, "Failed to initiate reconnection: %s", esp_err_to_name(connect_err));
        return connect_err;
    }

    wifi->sta_state = WIFI_MODE_STATE_CONNECTING;
    LOG_I(TAG, "WiFi reconnection initiated");
    return ESP_OK;
}

bool wifi_should_reset_on_timeout(wifi_t *wifi, uint32_t timeout_seconds) {
    if (wifi->sta_state != WIFI_MODE_STATE_CONNECTING) {
        return false;  // Not in connecting state
    }

    if (wifi->connection_start_time == 0) {
        return false;  // No connection attempt started
    }

    uint32_t current_time = time_micros_now() / 1000000;  // Convert to seconds
    uint32_t elapsed_time = current_time - wifi->connection_start_time;

    if (elapsed_time >= timeout_seconds) {
        LOG_W(TAG, "WiFi connection timeout: %lu seconds elapsed (threshold: %lu seconds)", elapsed_time, timeout_seconds);
        return true;
    }

    return false;
}

uint8_t wifi_get_optimal_ap_channel(wifi_t *wifi) {
    // If ESP-NOW is enabled, we need a fixed channel
    if (wifi->espnow_enabled) {
        LOG_I(TAG, "ESP-NOW enabled: AP will use fixed channel %d", wifi->ap_channel);
        return wifi->ap_channel;
    }

    // If only AP mode (no STA), we can use auto channel
    if (wifi->ap_enabled && !wifi->sta_enabled) {
        LOG_I(TAG, "AP-only mode: using auto channel selection");
        return 0;  // Auto channel
    }

    // If APSTA mode, prefer auto channel to avoid conflicts
    if (wifi->ap_enabled && wifi->sta_enabled) {
        LOG_I(TAG, "APSTA mode: using auto channel to avoid STA conflicts");
        return 0;  // Auto channel
    }

    // Default fallback
    return wifi->ap_channel;
}

bool wifi_is_channel_suitable_for_sta(wifi_t *wifi, uint8_t channel) {
    // If AP is not enabled, any channel is suitable
    if (!wifi->ap_enabled) {
        return true;
    }

    // Get the AP channel
    uint8_t ap_channel = wifi_get_optimal_ap_channel(wifi);

    // If AP uses auto channel, any channel is suitable
    if (ap_channel == 0) {
        return true;
    }

    // Check if channels conflict (same channel or adjacent channels)
    if (channel == ap_channel) {
        LOG_W(TAG, "Channel %d conflicts with AP channel %d", channel, ap_channel);
        return false;
    }

    // Check for adjacent channel interference (channels 1, 6, 11 are non-overlapping)
    uint8_t ap_channel_group = (ap_channel - 1) / 5;  // 0, 1, or 2
    uint8_t sta_channel_group = (channel - 1) / 5;

    if (ap_channel_group == sta_channel_group) {
        LOG_W(TAG, "Channel %d is adjacent to AP channel %d (potential interference)", channel, ap_channel);
        return false;
    }

    LOG_I(TAG, "Channel %d is suitable for STA (AP on channel %d)", channel, ap_channel);
    return true;
}

// ESP-NOW function implementations (stubs for now)
esp_err_t wifi_start_espnow(wifi_t *wifi) {
    if (!wifi) {
        return ESP_ERR_INVALID_ARG;
    }

    LOG_I(TAG, "ESP-NOW start requested (not implemented)");
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t wifi_stop_espnow(wifi_t *wifi) {
    if (!wifi) {
        return ESP_ERR_INVALID_ARG;
    }

    LOG_I(TAG, "ESP-NOW stop requested (not implemented)");
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t wifi_send_espnow(wifi_t *wifi, const uint8_t broadcast_addr[6], const void *data, size_t len) {
    if (!wifi || !data || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    LOG_I(TAG, "ESP-NOW send requested (not implemented)");
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t wifi_ensure_espnow_peer(wifi_t *wifi, const uint8_t peer_mac[6]) {
    if (!wifi || !peer_mac) {
        return ESP_ERR_INVALID_ARG;
    }

    LOG_I(TAG, "ESP-NOW peer ensure requested (not implemented)");
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t wifi_remove_espnow_peer(wifi_t *wifi, const uint8_t peer_mac[6]) {
    if (!wifi || !peer_mac) {
        return ESP_ERR_INVALID_ARG;
    }

    LOG_I(TAG, "ESP-NOW peer remove requested (not implemented)");
    return ESP_ERR_NOT_SUPPORTED;
}

void wifi_set_captive_portal_http_server(wifi_t *wifi, void *http_server) {
    if (!wifi) {
        return;
    }

    LOG_I(TAG, "Setting captive portal HTTP server: %p", http_server);

    // Store the HTTP server instance
    wifi->captive_portal_http_server = http_server;

    // Update the captive portal with the HTTP server instance if it's running
    if (wifi->captive_portal_enabled && wifi->captive_portal.running) {
        void *old_server = wifi->captive_portal.http_server;

        // If we're switching to a different server and endpoints were registered on the old server
        if (old_server && old_server != http_server && wifi->captive_portal.http_endpoints_registered) {
            LOG_I(TAG, "Switching captive portal HTTP server from %p to %p", old_server, http_server);
            // Unregister endpoints from the old server
            web_server_configure_captive_portal(old_server, false);
            wifi->captive_portal.http_endpoints_registered = false;
        }

        // Update HTTP server reference
        wifi->captive_portal.http_server = http_server;

        // Register endpoints with the new server if not already registered
        if (http_server && !wifi->captive_portal.http_endpoints_registered) {
            web_server_configure_captive_portal(http_server, true);
            wifi->captive_portal.http_endpoints_registered = true;
        }
    }
}

// =====================================================================
// NVS Credential Storage Functions
// =====================================================================

#define WIFI_CREDS_NVS_NAMESPACE "wifi_creds"
#define WIFI_CREDS_LAST_SSID_KEY "last_ssid"
#define WIFI_CREDS_MAX_STORED 5

esp_err_t wifi_nvs_save_credentials(const char *ssid, const char *password) {
    if (!ssid || !password) {
        return ESP_ERR_INVALID_ARG;
    }
    
    nvs_handle_t handle;
    esp_err_t err = nvs_open(WIFI_CREDS_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        LOG_E(TAG, "Failed to open NVS: %s", esp_err_to_name(err));
        return err;
    }
    
    // Save password with SSID as key
    err = nvs_set_str(handle, ssid, password);
    if (err != ESP_OK) {
        LOG_E(TAG, "Failed to save password for SSID %s: %s", ssid, esp_err_to_name(err));
        nvs_close(handle);
        return err;
    }
    
    // Save as last connected SSID
    err = nvs_set_str(handle, WIFI_CREDS_LAST_SSID_KEY, ssid);
    if (err != ESP_OK) {
        LOG_E(TAG, "Failed to save last SSID: %s", esp_err_to_name(err));
    }
    
    err = nvs_commit(handle);
    if (err != ESP_OK) {
        LOG_E(TAG, "Failed to commit NVS: %s", esp_err_to_name(err));
    }
    
    nvs_close(handle);
    LOG_I(TAG, "Saved credentials for SSID: %s", ssid);
    return ESP_OK;
}

esp_err_t wifi_nvs_load_credentials(const char *ssid, char *password, size_t pwd_len) {
    if (!ssid || !password || pwd_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    nvs_handle_t handle;
    esp_err_t err = nvs_open(WIFI_CREDS_NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        LOG_W(TAG, "Failed to open NVS for reading: %s", esp_err_to_name(err));
        return err;
    }
    
    size_t required_size = pwd_len;
    err = nvs_get_str(handle, ssid, password, &required_size);
    if (err != ESP_OK) {
        LOG_W(TAG, "No credentials found for SSID %s: %s", ssid, esp_err_to_name(err));
    }
    
    nvs_close(handle);
    return err;
}

esp_err_t wifi_nvs_forget_credentials(const char *ssid) {
    if (!ssid) {
        return ESP_ERR_INVALID_ARG;
    }
    
    nvs_handle_t handle;
    esp_err_t err = nvs_open(WIFI_CREDS_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        LOG_E(TAG, "Failed to open NVS: %s", esp_err_to_name(err));
        return err;
    }
    
    err = nvs_erase_key(handle, ssid);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
        LOG_E(TAG, "Failed to erase SSID %s: %s", ssid, esp_err_to_name(err));
        nvs_close(handle);
        return err;
    }
    
    // Check if this was the last connected SSID
    char last_ssid[WIFI_SSID_MAX_LEN];
    size_t len = sizeof(last_ssid);
    if (nvs_get_str(handle, WIFI_CREDS_LAST_SSID_KEY, last_ssid, &len) == ESP_OK) {
        if (strcmp(last_ssid, ssid) == 0) {
            nvs_erase_key(handle, WIFI_CREDS_LAST_SSID_KEY);
        }
    }
    
    err = nvs_commit(handle);
    nvs_close(handle);
    
    LOG_I(TAG, "Forgot credentials for SSID: %s", ssid);
    return ESP_OK;
}

esp_err_t wifi_nvs_get_last_ssid(char *ssid, size_t len) {
    if (!ssid || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    nvs_handle_t handle;
    esp_err_t err = nvs_open(WIFI_CREDS_NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        return err;
    }
    
    size_t required_size = len;
    err = nvs_get_str(handle, WIFI_CREDS_LAST_SSID_KEY, ssid, &required_size);
    
    nvs_close(handle);
    return err;
}

#endif  // defined(USE_WIFI)