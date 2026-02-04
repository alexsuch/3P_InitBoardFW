#pragma once

#include "target.h"

#if defined(USE_WIFI)

#include <stdbool.h>
#include <esp_err.h>
#include <esp_http_server.h>

#ifdef __cplusplus
extern "C" {
#endif

// Forward declaration
typedef struct wifi_s wifi_t;

/**
 * @brief RPC Server configuration
 */
typedef struct {
    uint16_t port;          // HTTP port (default 80)
    wifi_t *wifi;           // Pointer to WiFi module for commands
} rpc_server_config_t;

/**
 * @brief RPC Server state
 */
typedef struct {
    httpd_handle_t server;  // HTTP server handle
    wifi_t *wifi;           // WiFi module reference
    bool running;           // Server running state
} rpc_server_t;

/**
 * @brief Initialize the RPC server
 * @param server Pointer to RPC server instance
 * @param config Configuration structure
 * @return ESP_OK on success
 */
esp_err_t rpc_server_init(rpc_server_t *server, const rpc_server_config_t *config);

/**
 * @brief Start the RPC server
 * @param server Pointer to RPC server instance
 * @return ESP_OK on success
 */
esp_err_t rpc_server_start(rpc_server_t *server);

/**
 * @brief Stop the RPC server
 * @param server Pointer to RPC server instance
 * @return ESP_OK on success
 */
esp_err_t rpc_server_stop(rpc_server_t *server);

/**
 * @brief Check if RPC server is running
 * @param server Pointer to RPC server instance
 * @return true if running
 */
bool rpc_server_is_running(rpc_server_t *server);

#ifdef __cplusplus
}
#endif

#endif  // defined(USE_WIFI)
