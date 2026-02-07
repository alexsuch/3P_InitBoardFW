#include "target.h"

#if defined(USE_WIFI)

#include "rpc_server.h"
#include "wifi/wifi.h"
#include "log.h"
#include "generated_assets.h"
#include "app_logic.h"
#include "logger_module.h"
#include "app_state.h"
#include "config_manager.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <esp_err.h>
#include <esp_http_server.h>
#include <esp_wifi.h>
#include <cJSON.h>
#include <freertos/timers.h>

#define TAG "RPC_SERVER"

// Maximum scan results
#define MAX_SCAN_RESULTS 20

// WebSocket status push interval
#define WS_STATUS_PUSH_INTERVAL_MS 500

// Maximum connected WebSocket clients
#define MAX_WS_CLIENTS 4

// WebSocket client tracking
static int ws_client_fds[MAX_WS_CLIENTS] = {-1, -1, -1, -1};
static httpd_handle_t ws_server_handle = NULL;
static rpc_server_t *ws_server_ctx = NULL;
static TimerHandle_t ws_status_timer = NULL;

// Forward declarations
static esp_err_t ws_handler(httpd_req_t *req);
static esp_err_t static_file_handler(httpd_req_t *req);

// Add WebSocket client to tracking
static void ws_add_client(int fd) {
    for (int i = 0; i < MAX_WS_CLIENTS; i++) {
        if (ws_client_fds[i] == -1) {
            ws_client_fds[i] = fd;
            LOG_I(TAG, "WebSocket client connected: fd=%d (slot %d)", fd, i);
            return;
        }
    }
    LOG_W(TAG, "No space for new WebSocket client: fd=%d", fd);
}

// Remove WebSocket client from tracking
static void ws_remove_client(int fd) {
    for (int i = 0; i < MAX_WS_CLIENTS; i++) {
        if (ws_client_fds[i] == fd) {
            ws_client_fds[i] = -1;
            LOG_I(TAG, "WebSocket client disconnected: fd=%d (slot %d)", fd, i);
            return;
        }
    }
}

// Send text to all connected WebSocket clients
static void ws_broadcast(const char *data, size_t len) {
    if (!ws_server_handle) return;
    
    httpd_ws_frame_t ws_pkt = {
        .type = HTTPD_WS_TYPE_TEXT,
        .payload = (uint8_t *)data,
        .len = len,
        .final = true
    };
    
    for (int i = 0; i < MAX_WS_CLIENTS; i++) {
        int fd = ws_client_fds[i];
        if (fd >= 0) {
            esp_err_t err = httpd_ws_send_frame_async(ws_server_handle, fd, &ws_pkt);
            if (err != ESP_OK) {
                LOG_W(TAG, "Failed to send to client fd=%d: %s", fd, esp_err_to_name(err));
                ws_remove_client(fd);
            }
        }
    }
}

// Build status JSON response
static cJSON* build_status_json(void) {
    cJSON *json = cJSON_CreateObject();
    cJSON_AddStringToObject(json, "event", "status");
    
    cJSON *data = cJSON_CreateObject();
    
    // WiFi status
    wifi_ap_record_t ap_info;
    if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
        cJSON_AddBoolToObject(data, "connected", true);
        cJSON_AddStringToObject(data, "ssid", (char *)ap_info.ssid);
        cJSON_AddNumberToObject(data, "rssi", ap_info.rssi);
        
        esp_netif_ip_info_t ip_info;
        esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
        if (netif && esp_netif_get_ip_info(netif, &ip_info) == ESP_OK) {
            char ip_str[16];
            snprintf(ip_str, sizeof(ip_str), IPSTR, IP2STR(&ip_info.ip));
            cJSON_AddStringToObject(data, "ip", ip_str);
        }
    } else {
        cJSON_AddBoolToObject(data, "connected", false);
    }
    
    // Logging status
    const app_state_t *state = app_state_get_instance();
    bool is_logging = state && (state->current_mode == APP_MODE_LOGGING);
    bool is_download = state && (state->current_mode == APP_MODE_DOWNLOAD);
    
    cJSON_AddBoolToObject(data, "logging_active", is_logging);
    cJSON_AddBoolToObject(data, "download_mode", is_download);
    
    cJSON_AddItemToObject(json, "data", data);
    return json;
}




static cJSON* build_config_json(void) {
    cJSON *json = cJSON_CreateObject();
    
    const app_state_t *state = app_state_get_instance();
    bool is_logging = state && (state->current_mode == APP_MODE_LOGGING);
    
    if (!g_logger_module.stm_config_received) {
        cJSON_AddBoolToObject(json, "config_valid", false);
        cJSON_AddBoolToObject(json, "logging_active", is_logging);
        return json;
    }
    
    const logger_config_t *cfg = &g_logger_module.stm_config;
    
    cJSON_AddBoolToObject(json, "config_valid", true);
    cJSON_AddBoolToObject(json, "logging_active", is_logging);
    cJSON_AddNumberToObject(json, "version_major", cfg->version_major);
    cJSON_AddNumberToObject(json, "version_minor", cfg->version_minor);
    
    // ADC params
    cJSON_AddNumberToObject(json, "adc_sample_rate_khz", cfg->adc_sample_rate_khz);
    cJSON_AddNumberToObject(json, "adc_block_size", cfg->adc_block_size);
    
    // Accelerometer params
    cJSON_AddBoolToObject(json, "accel_enable", cfg->accel_enable != 0);
    cJSON_AddNumberToObject(json, "accel_range_g", cfg->accel_range_g);
    cJSON_AddNumberToObject(json, "accel_odr_hz", cfg->accel_odr_hz);
    cJSON_AddNumberToObject(json, "accel_bw_hz", cfg->accel_bw_hz);
    cJSON_AddBoolToObject(json, "accel_lpf2_en", cfg->accel_lpf2_en != 0);
    cJSON_AddBoolToObject(json, "accel_hp_en", cfg->accel_hp_en != 0);
    cJSON_AddNumberToObject(json, "accel_hp_cutoff", cfg->accel_hp_cutoff);
    cJSON_AddBoolToObject(json, "accel_hm_mode", cfg->accel_hm_mode != 0);

    // Gyroscope params
    cJSON_AddBoolToObject(json, "gyro_enable", cfg->gyro_enable != 0);
    cJSON_AddNumberToObject(json, "gyro_range_dps", cfg->gyro_range_dps);
    cJSON_AddNumberToObject(json, "gyro_odr_hz", cfg->gyro_odr_hz);
    cJSON_AddBoolToObject(json, "gyro_lpf1_en", cfg->gyro_lpf1_en != 0);
    cJSON_AddNumberToObject(json, "gyro_lpf1_bw", cfg->gyro_lpf1_bw);
    cJSON_AddBoolToObject(json, "gyro_hp_en", cfg->gyro_hp_en != 0);
    cJSON_AddNumberToObject(json, "gyro_hp_cutoff", cfg->gyro_hp_cutoff);
    cJSON_AddBoolToObject(json, "gyro_hm_mode", cfg->gyro_hm_mode != 0);
    
    // System info
    cJSON_AddNumberToObject(json, "chip_id", cfg->chip_id);
    cJSON_AddNumberToObject(json, "checksum_algo", cfg->checksum_algo);
    cJSON_AddBoolToObject(json, "mavlink_logging_enabled", cfg->mavlink_logging_enabled != 0);
    
    return json;
}

static bool json_get_int(const cJSON *obj, const char *key, int *out) {
    if (!obj || !key || !out) return false;

    const cJSON *item = cJSON_GetObjectItem((cJSON *)obj, key);
    if (!item) return false;

    if (cJSON_IsNumber(item)) {
        *out = item->valueint;
        return true;
    }

    if (cJSON_IsString(item) && item->valuestring) {
        char *end = NULL;
        long v = strtol(item->valuestring, &end, 10);
        if (end && *end == '\0') {
            *out = (int)v;
            return true;
        }
    }

    return false;
}

static bool json_get_bool(const cJSON *obj, const char *key, bool *out) {
    if (!obj || !key || !out) return false;

    const cJSON *item = cJSON_GetObjectItem((cJSON *)obj, key);
    if (!item) return false;

    if (cJSON_IsBool(item)) {
        *out = cJSON_IsTrue(item);
        return true;
    }

    if (cJSON_IsNumber(item)) {
        *out = (item->valueint != 0);
        return true;
    }

    if (cJSON_IsString(item) && item->valuestring) {
        if (strcmp(item->valuestring, "true") == 0) {
            *out = true;
            return true;
        }
        if (strcmp(item->valuestring, "false") == 0) {
            *out = false;
            return true;
        }

        int v = 0;
        if (json_get_int(obj, key, &v)) {
            *out = (v != 0);
            return true;
        }
    }

    return false;
}

static bool int_in_set(int v, const int *allowed, size_t allowed_len) {
    for (size_t i = 0; i < allowed_len; i++) {
        if (allowed[i] == v) return true;
    }
    return false;
}

static bool apply_config_patch(logger_config_t *cfg, const cJSON *patch, char *err, size_t err_len) {
    if (!cfg || !patch) return false;

    // Allowed option sets (keep in sync with Web UI options).
    static const int k_odr_allowed[] = {0, 13, 26, 52, 104, 208, 416, 833, 1660, 3330, 6660};
    static const int k_accel_range_allowed[] = {2, 4, 8, 16};
    static const int k_accel_bw_allowed[] = {50, 100, 200, 400};
    static const int k_gyro_range_allowed[] = {125, 245, 500, 1000, 2000};

    bool b = false;
    int v = 0;

    if (json_get_bool(patch, "accel_enable", &b)) cfg->accel_enable = b ? 1 : 0;
    if (json_get_int(patch, "accel_range_g", &v)) {
        if (!int_in_set(v, k_accel_range_allowed, sizeof(k_accel_range_allowed) / sizeof(k_accel_range_allowed[0]))) {
            snprintf(err, err_len, "Invalid accel_range_g");
            return false;
        }
        cfg->accel_range_g = (uint8_t)v;
    }
    if (json_get_int(patch, "accel_odr_hz", &v)) {
        if (!int_in_set(v, k_odr_allowed, sizeof(k_odr_allowed) / sizeof(k_odr_allowed[0]))) {
            snprintf(err, err_len, "Invalid accel_odr_hz");
            return false;
        }
        cfg->accel_odr_hz = (uint16_t)v;
    }
    if (json_get_int(patch, "accel_bw_hz", &v)) {
        if (!int_in_set(v, k_accel_bw_allowed, sizeof(k_accel_bw_allowed) / sizeof(k_accel_bw_allowed[0]))) {
            snprintf(err, err_len, "Invalid accel_bw_hz");
            return false;
        }
        cfg->accel_bw_hz = (uint16_t)v;
    }
    if (json_get_bool(patch, "accel_lpf2_en", &b)) cfg->accel_lpf2_en = b ? 1 : 0;
    if (json_get_bool(patch, "accel_hp_en", &b)) cfg->accel_hp_en = b ? 1 : 0;
    if (json_get_bool(patch, "accel_hm_mode", &b)) cfg->accel_hm_mode = b ? 1 : 0;
    if (json_get_int(patch, "accel_hp_cutoff", &v)) {
        if (v < 0 || v > 3) {
            snprintf(err, err_len, "Invalid accel_hp_cutoff");
            return false;
        }
        cfg->accel_hp_cutoff = (uint8_t)v;
    }

    if (json_get_bool(patch, "gyro_enable", &b)) cfg->gyro_enable = b ? 1 : 0;
    if (json_get_int(patch, "gyro_range_dps", &v)) {
        if (!int_in_set(v, k_gyro_range_allowed, sizeof(k_gyro_range_allowed) / sizeof(k_gyro_range_allowed[0]))) {
            snprintf(err, err_len, "Invalid gyro_range_dps");
            return false;
        }
        cfg->gyro_range_dps = (uint16_t)v;
    }
    if (json_get_int(patch, "gyro_odr_hz", &v)) {
        if (!int_in_set(v, k_odr_allowed, sizeof(k_odr_allowed) / sizeof(k_odr_allowed[0]))) {
            snprintf(err, err_len, "Invalid gyro_odr_hz");
            return false;
        }
        cfg->gyro_odr_hz = (uint16_t)v;
    }
    if (json_get_bool(patch, "gyro_lpf1_en", &b)) cfg->gyro_lpf1_en = b ? 1 : 0;
    if (json_get_int(patch, "gyro_lpf1_bw", &v)) {
        if (v < 0 || v > 3) {
            snprintf(err, err_len, "Invalid gyro_lpf1_bw");
            return false;
        }
        cfg->gyro_lpf1_bw = (uint8_t)v;
    }
    if (json_get_bool(patch, "gyro_hp_en", &b)) cfg->gyro_hp_en = b ? 1 : 0;
    if (json_get_int(patch, "gyro_hp_cutoff", &v)) {
        if (v < 0 || v > 3) {
            snprintf(err, err_len, "Invalid gyro_hp_cutoff");
            return false;
        }
        cfg->gyro_hp_cutoff = (uint8_t)v;
    }
    if (json_get_bool(patch, "gyro_hm_mode", &b)) cfg->gyro_hm_mode = b ? 1 : 0;

    if (json_get_bool(patch, "mavlink_logging_enabled", &b)) cfg->mavlink_logging_enabled = b ? 1 : 0;

    if (err && err_len) {
        err[0] = '\0';
    }
    return true;
}

// Timer callback - push status to all clients
static void ws_status_push_callback(TimerHandle_t timer) {
    (void)timer;
    
    cJSON *json = build_status_json();
    char *response = cJSON_PrintUnformatted(json);
    cJSON_Delete(json);
    
    if (response) {
        ws_broadcast(response, strlen(response));
        free(response);
    }
}

// Handle WebSocket command
static void ws_handle_command(httpd_req_t *req, const char *cmd, int msg_id, cJSON *params) {
    cJSON *response = cJSON_CreateObject();
    cJSON_AddNumberToObject(response, "id", msg_id);
    
    if (strcmp(cmd, "status") == 0) {
        cJSON *result = build_status_json();
        cJSON *data = cJSON_DetachItemFromObject(result, "data");
        cJSON_Delete(result);
        cJSON_AddItemToObject(response, "result", data);
        
    } else if (strcmp(cmd, "scan") == 0) {
        LOG_I(TAG, "WS: scan");
        
        wifi_scan_config_t scan_config = {
            .ssid = NULL, .bssid = NULL, .channel = 0, .show_hidden = false,
            .scan_type = WIFI_SCAN_TYPE_ACTIVE,
            .scan_time = { .active = { .min = 100, .max = 300 } }
        };
        
        cJSON *result = cJSON_CreateObject();
        
        if (esp_wifi_scan_start(&scan_config, true) == ESP_OK) {
            uint16_t ap_count = 0;
            esp_wifi_scan_get_ap_num(&ap_count);
            if (ap_count > MAX_SCAN_RESULTS) ap_count = MAX_SCAN_RESULTS;
            
            wifi_ap_record_t *ap_records = malloc(ap_count * sizeof(wifi_ap_record_t));
            if (ap_records) {
                esp_wifi_scan_get_ap_records(&ap_count, ap_records);
                
                cJSON_AddBoolToObject(result, "success", true);
                cJSON *networks = cJSON_CreateArray();
                
                for (int i = 0; i < ap_count; i++) {
                    cJSON *network = cJSON_CreateObject();
                    cJSON_AddNumberToObject(network, "id", i);
                    cJSON_AddStringToObject(network, "ssid", (char *)ap_records[i].ssid);
                    cJSON_AddNumberToObject(network, "rssi", ap_records[i].rssi);
                    
                    const char *auth;
                    switch (ap_records[i].authmode) {
                        case WIFI_AUTH_OPEN: auth = "OPEN"; break;
                        case WIFI_AUTH_WEP: auth = "WEP"; break;
                        case WIFI_AUTH_WPA_PSK: auth = "WPA"; break;
                        case WIFI_AUTH_WPA2_PSK: auth = "WPA2"; break;
                        case WIFI_AUTH_WPA_WPA2_PSK: auth = "WPA/WPA2"; break;
                        case WIFI_AUTH_WPA3_PSK: auth = "WPA3"; break;
                        default: auth = "UNKNOWN"; break;
                    }
                    cJSON_AddStringToObject(network, "auth", auth);
                    cJSON_AddItemToArray(networks, network);
                }
                cJSON_AddItemToObject(result, "networks", networks);
                free(ap_records);
            } else {
                cJSON_AddBoolToObject(result, "success", false);
                cJSON_AddStringToObject(result, "error", "Memory allocation failed");
            }
        } else {
            cJSON_AddBoolToObject(result, "success", false);
            cJSON_AddStringToObject(result, "error", "Scan failed");
        }
        cJSON_AddItemToObject(response, "result", result);
        
    } else if (strcmp(cmd, "connect") == 0) {
        LOG_I(TAG, "WS: connect");
        cJSON *result = cJSON_CreateObject();
        
        cJSON *ssid_json = cJSON_GetObjectItem(params, "ssid");
        cJSON *password_json = cJSON_GetObjectItem(params, "password");
        
        if (!ssid_json || !cJSON_IsString(ssid_json) || !password_json || !cJSON_IsString(password_json)) {
            cJSON_AddBoolToObject(result, "success", false);
            cJSON_AddStringToObject(result, "error", "Missing ssid/password");
        } else {
            const char *ssid = ssid_json->valuestring;
            const char *password = password_json->valuestring;
            
            wifi_config_t wifi_config = {0};
            strncpy((char *)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid) - 1);
            strncpy((char *)wifi_config.sta.password, password, sizeof(wifi_config.sta.password) - 1);
            wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
            
            esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
            esp_wifi_disconnect();
            esp_wifi_connect();
            
            // Wait for connection
            int timeout_ms = 10000, elapsed = 0;
            wifi_ap_record_t ap_info;
            bool connected = false;
            
            while (elapsed < timeout_ms) {
                vTaskDelay(pdMS_TO_TICKS(500));
                elapsed += 500;
                if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
                    connected = true;
                    wifi_nvs_save_credentials(ssid, password);
                    break;
                }
            }
            
            if (connected) {
                cJSON_AddBoolToObject(result, "success", true);
                esp_netif_ip_info_t ip_info;
                esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
                if (netif && esp_netif_get_ip_info(netif, &ip_info) == ESP_OK) {
                    char ip_str[16];
                    snprintf(ip_str, sizeof(ip_str), IPSTR, IP2STR(&ip_info.ip));
                    cJSON_AddStringToObject(result, "ip", ip_str);
                }
            } else {
                cJSON_AddBoolToObject(result, "success", false);
                cJSON_AddStringToObject(result, "error", "Connection timeout");
            }
        }
        cJSON_AddItemToObject(response, "result", result);
        
    } else if (strcmp(cmd, "forget") == 0) {
        LOG_I(TAG, "WS: forget");
        cJSON *result = cJSON_CreateObject();
        
        cJSON *ssid_json = cJSON_GetObjectItem(params, "ssid");
        if (!ssid_json || !cJSON_IsString(ssid_json)) {
            cJSON_AddBoolToObject(result, "success", false);
            cJSON_AddStringToObject(result, "error", "Missing ssid");
        } else {
            esp_err_t err = wifi_nvs_forget_credentials(ssid_json->valuestring);
            cJSON_AddBoolToObject(result, "success", err == ESP_OK);
            if (err != ESP_OK) cJSON_AddStringToObject(result, "error", "SSID not found");
        }
        cJSON_AddItemToObject(response, "result", result);
        
    } else if (strcmp(cmd, "get_config") == 0) {
        LOG_I(TAG, "WS: get_config");
        cJSON *result = build_config_json();
        cJSON_AddItemToObject(response, "result", result);

    } else if (strcmp(cmd, "set_config") == 0) {
        LOG_I(TAG, "WS: set_config");

        cJSON *result = cJSON_CreateObject();

        const app_state_t *state = app_state_get_instance();
        const bool is_logging = state && (state->current_mode == APP_MODE_LOGGING);
        const bool is_download = state && (state->current_mode == APP_MODE_DOWNLOAD);
        if (is_logging || is_download) {
            cJSON_AddBoolToObject(result, "success", false);
            cJSON_AddStringToObject(result, "error", "Config locked (logging/download active)");
            cJSON_AddItemToObject(response, "result", result);
        } else {
            const cJSON *patch = cJSON_GetObjectItem(params, "config");
            if (patch && !cJSON_IsObject(patch)) {
                patch = NULL;
            }
            if (!patch) {
                patch = params;
            }

            logger_config_t cfg;
            bool have_cfg = false;
            if (g_logger_module.stm_config_received) {
                memcpy(&cfg, &g_logger_module.stm_config, sizeof(cfg));
                have_cfg = true;
            } else if (config_manager_load_stm_config(&cfg)) {
                have_cfg = true;
            }

            if (!have_cfg) {
                cJSON_AddBoolToObject(result, "success", false);
                cJSON_AddStringToObject(result, "error", "No base config available");
                cJSON_AddItemToObject(response, "result", result);
            } else {
                char err_buf[64] = {0};
                if (!apply_config_patch(&cfg, patch, err_buf, sizeof(err_buf))) {
                    cJSON_AddBoolToObject(result, "success", false);
                    cJSON_AddStringToObject(result, "error", err_buf[0] ? err_buf : "Invalid config");
                    cJSON_AddItemToObject(response, "result", result);
                } else {
                    // Mark as received from ESP32 side (informational).
                    cfg.config_source = LOGGER_CONFIG_SOURCE_RECEIVED;

                    // Persist first.
                    if (!config_manager_save_stm_config(&cfg)) {
                        cJSON_AddBoolToObject(result, "success", false);
                        cJSON_AddStringToObject(result, "error", "Failed to save config to NVS");
                        cJSON_AddItemToObject(response, "result", result);
                    } else {
                        // Update in-memory copy without emitting events / touching SD/log file.
                        memcpy(&g_logger_module.stm_config, &cfg, sizeof(cfg));
                        g_logger_module.stm_config_received = true;

                        // Apply to STM32 asynchronously via app logic task (loads config from NVS).
                        bool queued = false;
                        app_logic_t *app = get_global_app_logic_instance();
                        if (app && app_logic_send_command(app, APP_CMD_SEND_CONFIG_TO_STM) == ESP_OK) {
                            queued = true;
                        }

                        cJSON_AddBoolToObject(result, "success", true);
                        cJSON_AddBoolToObject(result, "queued", queued);
                        cJSON *cfg_json = build_config_json();
                        cJSON_AddItemToObject(result, "config", cfg_json);
                        cJSON_AddItemToObject(response, "result", result);
                    }
                }
            }
        }
        
    } else if (strcmp(cmd, "start_logging") == 0) {
        LOG_I(TAG, "WS: start_logging");
        app_logic_t *app = get_global_app_logic_instance();
        if (app) app_logic_send_command(app, APP_CMD_SET_MODE_LOGGING);
        
        cJSON *result = cJSON_CreateObject();
        cJSON_AddBoolToObject(result, "success", true);
        cJSON_AddItemToObject(response, "result", result);
        
    } else if (strcmp(cmd, "stop_logging") == 0) {
        LOG_I(TAG, "WS: stop_logging");
        app_logic_t *app = get_global_app_logic_instance();
        if (app) app_logic_send_command(app, APP_CMD_SET_MODE_IDLE);
        
        cJSON *result = cJSON_CreateObject();
        cJSON_AddBoolToObject(result, "success", true);
        cJSON_AddItemToObject(response, "result", result);
        
    } else if (strcmp(cmd, "mount_storage") == 0) {
        LOG_I(TAG, "WS: mount_storage");
        app_logic_t *app = get_global_app_logic_instance();
        // Trigger USB MSC (File Server)
        if (app) app_logic_send_command(app, APP_CMD_START_USB_FILE_SERVER);
        
        cJSON *result = cJSON_CreateObject();
        cJSON_AddBoolToObject(result, "success", true);
        cJSON_AddItemToObject(response, "result", result);
        
    } else if (strcmp(cmd, "unmount_storage") == 0) {
        LOG_I(TAG, "WS: unmount_storage");
        app_logic_t *app = get_global_app_logic_instance();
        // Stop USB MSC
        if (app) app_logic_send_command(app, APP_CMD_STOP_USB_FILE_SERVER);
        
        cJSON *result = cJSON_CreateObject();
        cJSON_AddBoolToObject(result, "success", true);
        cJSON_AddItemToObject(response, "result", result);
        
    } else {
        cJSON *result = cJSON_CreateObject();
        cJSON_AddBoolToObject(result, "success", false);
        cJSON_AddStringToObject(result, "error", "Unknown command");
        cJSON_AddItemToObject(response, "result", result);
    }
    
    // Send response
    char *resp_str = cJSON_PrintUnformatted(response);
    cJSON_Delete(response);
    
    if (resp_str) {
        httpd_ws_frame_t ws_pkt = {
            .type = HTTPD_WS_TYPE_TEXT,
            .payload = (uint8_t *)resp_str,
            .len = strlen(resp_str),
            .final = true
        };
        esp_err_t send_err = httpd_ws_send_frame(req, &ws_pkt);
        if (send_err != ESP_OK) {
            LOG_W(TAG, "Failed to send WS response (cmd=%s,id=%d): %s", cmd, msg_id, esp_err_to_name(send_err));
        }
        free(resp_str);
    }
}

// WebSocket handler
static esp_err_t ws_handler(httpd_req_t *req) {
    if (req->method == HTTP_GET) {
        // WebSocket handshake
        LOG_I(TAG, "WebSocket handshake");
        return ESP_OK;
    }
    
    // Receive frame
    httpd_ws_frame_t ws_pkt = {0};
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;
    
    // Get frame length
    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
    if (ret != ESP_OK) {
        LOG_E(TAG, "httpd_ws_recv_frame failed to get frame len: %s", esp_err_to_name(ret));
        return ret;
    }
    
    if (ws_pkt.len == 0) {
        return ESP_OK;
    }
    
    // Allocate buffer
    ws_pkt.payload = malloc(ws_pkt.len + 1);
    if (!ws_pkt.payload) {
        LOG_E(TAG, "Failed to allocate WS buffer");
        return ESP_ERR_NO_MEM;
    }
    
    // Receive payload
    ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
    if (ret != ESP_OK) {
        free(ws_pkt.payload);
        return ret;
    }
    ws_pkt.payload[ws_pkt.len] = '\0';
    
    // Track client on first message
    int sock_fd = httpd_req_to_sockfd(req);
    bool found = false;
    for (int i = 0; i < MAX_WS_CLIENTS; i++) {
        if (ws_client_fds[i] == sock_fd) { found = true; break; }
    }
    if (!found) ws_add_client(sock_fd);
    
    // Parse JSON
    cJSON *json = cJSON_Parse((char *)ws_pkt.payload);
    free(ws_pkt.payload);
    
    if (!json) {
        LOG_W(TAG, "Invalid JSON from WebSocket");
        return ESP_OK;
    }
    
    // Get command and ID
    cJSON *cmd = cJSON_GetObjectItem(json, "cmd");
    cJSON *id = cJSON_GetObjectItem(json, "id");
    
    if (cmd && cJSON_IsString(cmd)) {
        int msg_id = (id && cJSON_IsNumber(id)) ? id->valueint : 0;
        ws_handle_command(req, cmd->valuestring, msg_id, json);
    }
    
    cJSON_Delete(json);
    return ESP_OK;
}

// Static file handler - serves embedded web UI
static esp_err_t static_file_handler(httpd_req_t *req) {
    const char *file_path = "index.html.gz";
    
    const embedded_asset_t *asset = embedded_asset_find(file_path);
    if (!asset) {
        httpd_resp_send_404(req);
        return ESP_OK;
    }
    
    httpd_resp_set_type(req, "text/html");
    if (asset->is_gzipped) {
        httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
    }
    
    size_t len = asset->end - asset->start;
    httpd_resp_send(req, (const char *)asset->start, len);
    return ESP_OK;
}

// Initialize RPC server
esp_err_t rpc_server_init(rpc_server_t *server, const rpc_server_config_t *config) {
    if (!server || !config) return ESP_ERR_INVALID_ARG;
    
    memset(server, 0, sizeof(rpc_server_t));
    server->wifi = config->wifi;
    server->running = false;
    
    LOG_I(TAG, "RPC server initialized (WebSocket mode)");
    return ESP_OK;
}

// Start RPC server
esp_err_t rpc_server_start(rpc_server_t *server) {
    if (!server) return ESP_ERR_INVALID_ARG;
    if (server->running) return ESP_OK;
    
    // Configure HTTP server
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.stack_size = 8192;
    config.max_uri_handlers = 4;
    config.lru_purge_enable = true;
    
    esp_err_t err = httpd_start(&server->server, &config);
    if (err != ESP_OK) {
        LOG_E(TAG, "Failed to start HTTP server: %s", esp_err_to_name(err));
        return err;
    }
    
    ws_server_handle = server->server;
    ws_server_ctx = server;
    
    // Register WebSocket endpoint
    httpd_uri_t ws_uri = {
        .uri = "/ws",
        .method = HTTP_GET,
        .handler = ws_handler,
        .user_ctx = server,
        .is_websocket = true,
        .handle_ws_control_frames = true
    };
    httpd_register_uri_handler(server->server, &ws_uri);
    
    // Register static file handlers
    httpd_uri_t root_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = static_file_handler,
        .user_ctx = server
    };
    httpd_register_uri_handler(server->server, &root_uri);
    
    httpd_uri_t index_uri = {
        .uri = "/index.html",
        .method = HTTP_GET,
        .handler = static_file_handler,
        .user_ctx = server
    };
    httpd_register_uri_handler(server->server, &index_uri);
    
    // Start status push timer
    ws_status_timer = xTimerCreate("ws_status", pdMS_TO_TICKS(WS_STATUS_PUSH_INTERVAL_MS),
                                   pdTRUE, NULL, ws_status_push_callback);
    if (ws_status_timer) {
        xTimerStart(ws_status_timer, 0);
    }
    
    server->running = true;
    LOG_I(TAG, "WebSocket server started on port 80 (/ws endpoint)");
    
    return ESP_OK;
}

// Stop RPC server
esp_err_t rpc_server_stop(rpc_server_t *server) {
    if (!server) return ESP_ERR_INVALID_ARG;
    if (!server->running) return ESP_OK;
    
    // Stop status timer
    if (ws_status_timer) {
        xTimerStop(ws_status_timer, 0);
        xTimerDelete(ws_status_timer, 0);
        ws_status_timer = NULL;
    }
    
    // Clear client tracking
    for (int i = 0; i < MAX_WS_CLIENTS; i++) {
        ws_client_fds[i] = -1;
    }
    
    if (server->server) {
        httpd_stop(server->server);
        server->server = NULL;
    }
    
    ws_server_handle = NULL;
    ws_server_ctx = NULL;
    server->running = false;
    
    LOG_I(TAG, "WebSocket server stopped");
    return ESP_OK;
}

// Check if server is running
bool rpc_server_is_running(rpc_server_t *server) {
    return server && server->running;
}

#endif  // defined(USE_WIFI)
