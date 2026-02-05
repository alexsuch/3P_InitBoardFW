#include "target.h"

#if defined(USE_WIFI)

#include "rpc_server.h"
#include "wifi/wifi.h"
#include "log.h"
#include "generated_assets.h"

#include <string.h>
#include <esp_http_server.h>
#include <esp_wifi.h>
#include <cJSON.h>

#define TAG "RPC_SERVER"

// AP Credentials (hardcoded)
#define RPC_AP_SSID "Logger_AP"
#define RPC_AP_PASS "logger123"

// Maximum scan results
#define MAX_SCAN_RESULTS 20

// NVS namespace for WiFi credentials
#define WIFI_CREDS_NVS_NAMESPACE "wifi_creds"
#define WIFI_CREDS_LAST_SSID_KEY "last_ssid"

// Forward declarations
static esp_err_t rpc_handler(httpd_req_t *req);
static esp_err_t static_file_handler(httpd_req_t *req);
static esp_err_t handle_scan(rpc_server_t *server, httpd_req_t *req);
static esp_err_t handle_connect(rpc_server_t *server, httpd_req_t *req, cJSON *params);
static esp_err_t handle_forget(rpc_server_t *server, httpd_req_t *req, cJSON *params);
static esp_err_t handle_status(rpc_server_t *server, httpd_req_t *req);

// Send JSON response helper
static esp_err_t send_json_response(httpd_req_t *req, cJSON *json) {
    char *response = cJSON_PrintUnformatted(json);
    if (response == NULL) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_send(req, response, strlen(response));
    
    free(response);
    return ESP_OK;
}

// Send error response
static esp_err_t send_error_response(httpd_req_t *req, const char *error_msg) {
    cJSON *json = cJSON_CreateObject();
    cJSON_AddBoolToObject(json, "success", false);
    cJSON_AddStringToObject(json, "error", error_msg);
    
    esp_err_t ret = send_json_response(req, json);
    cJSON_Delete(json);
    return ret;
}

// RPC endpoint handler
static esp_err_t rpc_handler(httpd_req_t *req) {
    rpc_server_t *server = (rpc_server_t *)req->user_ctx;
    
    // Handle CORS preflight
    if (req->method == HTTP_OPTIONS) {
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
        httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "POST, OPTIONS");
        httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");
        httpd_resp_send(req, NULL, 0);
        return ESP_OK;
    }
    
    // Read request body
    int total_len = req->content_len;
    if (total_len <= 0 || total_len > 1024) {
        return send_error_response(req, "Invalid content length");
    }
    
    char *buf = malloc(total_len + 1);
    if (!buf) {
        return send_error_response(req, "Memory allocation failed");
    }
    
    int received = 0;
    while (received < total_len) {
        int ret = httpd_req_recv(req, buf + received, total_len - received);
        if (ret <= 0) {
            free(buf);
            return send_error_response(req, "Failed to receive request body");
        }
        received += ret;
    }
    buf[total_len] = '\0';
    
    // Parse JSON
    cJSON *json = cJSON_Parse(buf);
    free(buf);
    
    if (!json) {
        return send_error_response(req, "Invalid JSON");
    }
    
    // Get command
    cJSON *cmd = cJSON_GetObjectItem(json, "cmd");
    if (!cmd || !cJSON_IsString(cmd)) {
        cJSON_Delete(json);
        return send_error_response(req, "Missing 'cmd' field");
    }
    
    const char *cmd_str = cmd->valuestring;
    esp_err_t ret = ESP_FAIL;
    
    if (strcmp(cmd_str, "scan") == 0) {
        ret = handle_scan(server, req);
    } else if (strcmp(cmd_str, "connect") == 0) {
        ret = handle_connect(server, req, json);
    } else if (strcmp(cmd_str, "forget") == 0) {
        ret = handle_forget(server, req, json);
    } else if (strcmp(cmd_str, "status") == 0) {
        ret = handle_status(server, req);
    } else {
        ret = send_error_response(req, "Unknown command");
    }
    
    cJSON_Delete(json);
    return ret;
}

// Handle scan command
static esp_err_t handle_scan(rpc_server_t *server, httpd_req_t *req) {
    LOG_I(TAG, "Handling scan command");
    
    // Start WiFi scan
    wifi_scan_config_t scan_config = {
        .ssid = NULL,
        .bssid = NULL,
        .channel = 0,
        .show_hidden = false,
        .scan_type = WIFI_SCAN_TYPE_ACTIVE,
        .scan_time = {
            .active = {
                .min = 100,
                .max = 300
            }
        }
    };
    
    esp_err_t err = esp_wifi_scan_start(&scan_config, true);
    if (err != ESP_OK) {
        LOG_E(TAG, "WiFi scan start failed: %s", esp_err_to_name(err));
        return send_error_response(req, "Scan failed");
    }
    
    // Get scan results
    uint16_t ap_count = 0;
    esp_wifi_scan_get_ap_num(&ap_count);
    
    if (ap_count > MAX_SCAN_RESULTS) {
        ap_count = MAX_SCAN_RESULTS;
    }
    
    wifi_ap_record_t *ap_records = malloc(ap_count * sizeof(wifi_ap_record_t));
    if (!ap_records) {
        return send_error_response(req, "Memory allocation failed");
    }
    
    esp_wifi_scan_get_ap_records(&ap_count, ap_records);
    
    // Build JSON response
    cJSON *json = cJSON_CreateObject();
    cJSON_AddBoolToObject(json, "success", true);
    
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
    cJSON_AddItemToObject(json, "networks", networks);
    
    free(ap_records);
    
    esp_err_t ret = send_json_response(req, json);
    cJSON_Delete(json);
    return ret;
}

// Handle connect command
static esp_err_t handle_connect(rpc_server_t *server, httpd_req_t *req, cJSON *params) {
    LOG_I(TAG, "Handling connect command");
    
    cJSON *ssid_json = cJSON_GetObjectItem(params, "ssid");
    cJSON *password_json = cJSON_GetObjectItem(params, "password");
    
    if (!ssid_json || !cJSON_IsString(ssid_json)) {
        return send_error_response(req, "Missing 'ssid' parameter");
    }
    if (!password_json || !cJSON_IsString(password_json)) {
        return send_error_response(req, "Missing 'password' parameter");
    }
    
    const char *ssid = ssid_json->valuestring;
    const char *password = password_json->valuestring;
    
    LOG_I(TAG, "Connecting to SSID: %s", ssid);
    
    // Configure STA with new credentials
    wifi_config_t wifi_config = {0};
    strncpy((char *)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char *)wifi_config.sta.password, password, sizeof(wifi_config.sta.password) - 1);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    
    esp_err_t err = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    if (err != ESP_OK) {
        LOG_E(TAG, "Failed to set WiFi config: %s", esp_err_to_name(err));
        return send_error_response(req, "Failed to configure WiFi");
    }
    
    // Disconnect if currently connected
    esp_wifi_disconnect();
    
    // Connect
    err = esp_wifi_connect();
    if (err != ESP_OK) {
        LOG_E(TAG, "Failed to connect: %s", esp_err_to_name(err));
        return send_error_response(req, "Connection failed");
    }
    
    // Wait for connection (with timeout)
    int timeout_ms = 10000;
    int elapsed = 0;
    wifi_ap_record_t ap_info;
    
    while (elapsed < timeout_ms) {
        vTaskDelay(pdMS_TO_TICKS(500));
        elapsed += 500;
        
        if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
            // Connected successfully - save credentials to NVS
            wifi_nvs_save_credentials(ssid, password);
            
            // Get IP address
            esp_netif_ip_info_t ip_info;
            esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
            if (netif && esp_netif_get_ip_info(netif, &ip_info) == ESP_OK) {
                char ip_str[16];
                snprintf(ip_str, sizeof(ip_str), IPSTR, IP2STR(&ip_info.ip));
                
                cJSON *json = cJSON_CreateObject();
                cJSON_AddBoolToObject(json, "success", true);
                cJSON_AddStringToObject(json, "ip", ip_str);
                
                esp_err_t ret = send_json_response(req, json);
                cJSON_Delete(json);
                return ret;
            }
        }
    }
    
    return send_error_response(req, "Connection timeout");
}

// Handle forget command
static esp_err_t handle_forget(rpc_server_t *server, httpd_req_t *req, cJSON *params) {
    LOG_I(TAG, "Handling forget command");
    
    cJSON *ssid_json = cJSON_GetObjectItem(params, "ssid");
    if (!ssid_json || !cJSON_IsString(ssid_json)) {
        return send_error_response(req, "Missing 'ssid' parameter");
    }
    
    const char *ssid = ssid_json->valuestring;
    
    esp_err_t err = wifi_nvs_forget_credentials(ssid);
    if (err != ESP_OK) {
        return send_error_response(req, "SSID not found");
    }
    
    cJSON *json = cJSON_CreateObject();
    cJSON_AddBoolToObject(json, "success", true);
    
    esp_err_t ret = send_json_response(req, json);
    cJSON_Delete(json);
    return ret;
}

// Handle status command
static esp_err_t handle_status(rpc_server_t *server, httpd_req_t *req) {
    LOG_I(TAG, "Handling status command");
    
    cJSON *json = cJSON_CreateObject();
    
    wifi_ap_record_t ap_info;
    if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
        cJSON_AddBoolToObject(json, "connected", true);
        cJSON_AddStringToObject(json, "ssid", (char *)ap_info.ssid);
        cJSON_AddNumberToObject(json, "rssi", ap_info.rssi);
        
        // Get IP address
        esp_netif_ip_info_t ip_info;
        esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
        if (netif && esp_netif_get_ip_info(netif, &ip_info) == ESP_OK) {
            char ip_str[16];
            snprintf(ip_str, sizeof(ip_str), IPSTR, IP2STR(&ip_info.ip));
            cJSON_AddStringToObject(json, "ip", ip_str);
        }
    } else {
        cJSON_AddBoolToObject(json, "connected", false);
    }
    
    esp_err_t ret = send_json_response(req, json);
    cJSON_Delete(json);
    return ret;
}

// Static file handler - serves embedded web UI
static esp_err_t static_file_handler(httpd_req_t *req) {
    const char *uri = req->uri;
    
    // Handle root path - serve index.html
    const char *file_path = "index.html.gz";
    if (strcmp(uri, "/") != 0 && strcmp(uri, "/index.html") != 0) {
        // For now, just serve index.html for all paths (SPA routing)
        file_path = "index.html.gz";
    }
    
    // Find asset in embedded assets
    const embedded_asset_t *asset = embedded_asset_find(file_path);
    if (!asset) {
        LOG_W(TAG, "Asset not found: %s", file_path);
        httpd_resp_send_404(req);
        return ESP_OK;
    }
    
    // Set content type
    httpd_resp_set_type(req, "text/html");
    
    // Set gzip encoding if the asset is compressed
    if (asset->is_gzipped) {
        httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
    }
    
    // Send response
    size_t len = asset->end - asset->start;
    httpd_resp_send(req, (const char *)asset->start, len);
    
    return ESP_OK;
}

// Initialize RPC server
esp_err_t rpc_server_init(rpc_server_t *server, const rpc_server_config_t *config) {
    if (!server || !config) {
        return ESP_ERR_INVALID_ARG;
    }
    
    memset(server, 0, sizeof(rpc_server_t));
    server->wifi = config->wifi;
    server->running = false;
    
    LOG_I(TAG, "RPC server initialized");
    return ESP_OK;
}

// Start RPC server
esp_err_t rpc_server_start(rpc_server_t *server) {
    if (!server) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (server->running) {
        LOG_W(TAG, "RPC server already running");
        return ESP_OK;
    }
    
    // Configure HTTP server
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.stack_size = 8192;
    config.max_uri_handlers = 8;
    config.lru_purge_enable = true;
    
    esp_err_t err = httpd_start(&server->server, &config);
    if (err != ESP_OK) {
        LOG_E(TAG, "Failed to start HTTP server: %s", esp_err_to_name(err));
        return err;
    }
    
    // Register RPC endpoint
    httpd_uri_t rpc_uri = {
        .uri = "/rpc",
        .method = HTTP_POST,
        .handler = rpc_handler,
        .user_ctx = server
    };
    httpd_register_uri_handler(server->server, &rpc_uri);
    
    // Register OPTIONS handler for CORS
    httpd_uri_t options_uri = {
        .uri = "/rpc",
        .method = HTTP_OPTIONS,
        .handler = rpc_handler,
        .user_ctx = server
    };
    httpd_register_uri_handler(server->server, &options_uri);
    
    // Register root path handler for web UI
    httpd_uri_t root_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = static_file_handler,
        .user_ctx = server
    };
    httpd_register_uri_handler(server->server, &root_uri);
    
    // Register index.html handler
    httpd_uri_t index_uri = {
        .uri = "/index.html",
        .method = HTTP_GET,
        .handler = static_file_handler,
        .user_ctx = server
    };
    httpd_register_uri_handler(server->server, &index_uri);
    
    server->running = true;
    LOG_I(TAG, "RPC server started on port 80");
    
    return ESP_OK;
}

// Stop RPC server
esp_err_t rpc_server_stop(rpc_server_t *server) {
    if (!server) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!server->running) {
        return ESP_OK;
    }
    
    if (server->server) {
        httpd_stop(server->server);
        server->server = NULL;
    }
    
    server->running = false;
    LOG_I(TAG, "RPC server stopped");
    
    return ESP_OK;
}

// Check if server is running
bool rpc_server_is_running(rpc_server_t *server) {
    return server && server->running;
}

#endif  // defined(USE_WIFI)
