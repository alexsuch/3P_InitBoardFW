#pragma once

#include "target.h"  // Include target.h to get USE_WEB_FILE_SEREVER definition

#if defined(USE_WEB_FILE_SEREVER)

#include <stddef.h>

#include "app_errors.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/sdcard.h"

// Forward declarations for C++ classes
#ifdef __cplusplus
extern "C" {
#endif

// Forward declare C++ classes to avoid including headers in C code
typedef struct PsychicHttpServer PsychicHttpServer;
typedef struct PsychicRequest PsychicRequest;
typedef struct PsychicResponse PsychicResponse;
typedef struct PsychicStaticFileHandler PsychicStaticFileHandler;
// Note: PsychicHttpRequestCallback is defined in PsychicCore.h

#ifdef __cplusplus
}
#endif

// Web server configuration
#define WEB_SERVER_PORT 80
#define WEB_SERVER_MAX_CONNECTIONS 5
// Priority moved to include/target.h as TASK_PRIORITY_WEB_SERVER

// SD card file serving configuration
#define WEB_SERVER_SD_ROOT_PATH "/"
#define WEB_SERVER_DEFAULT_INDEX "index.html"
#define WEB_SERVER_CACHE_CONTROL "max-age=3600"
#define WEB_SERVER_DOWNLOAD_CHUNK_DEFAULT 2880  // 1440 safe default for MTU 1500 or 2x1440=2880
#define WEB_SERVER_DOWNLOAD_CHUNK_MAX 65536

typedef enum {
    WEB_SERVER_STATE_STOPPED = 0,
    WEB_SERVER_STATE_STARTING,
    WEB_SERVER_STATE_RUNNING,
    WEB_SERVER_STATE_STOPPING,
    WEB_SERVER_STATE_ERROR
} web_server_state_t;

typedef struct {
    // Server instance (void pointer to avoid C++ includes in C code)
    void *server;

    // State management
    web_server_state_t state;
    bool initialized;
    bool sd_card_available;
    bool use_native_upload;  // Upload mode: true = native ESP, false = PsychicHttp

    // Task management
    TaskHandle_t task_handle;

    // Configuration
    uint16_t port;
    char sd_root_path[64];
    char default_file[32];
    char cache_control[32];

    // Statistics
    uint32_t total_requests;
    uint32_t successful_requests;
    uint32_t failed_requests;
    uint64_t bytes_served;

} web_server_module_t;

// Static web server module instance to avoid heap fragmentation
extern web_server_module_t g_web_server_module;

// Initialization and cleanup
app_err_t web_server_init(web_server_module_t *module);
app_err_t web_server_deinit(web_server_module_t *module);

// Start/Stop functionality
app_err_t web_server_start(web_server_module_t *module);
app_err_t web_server_stop(web_server_module_t *module);
bool web_server_is_running(web_server_module_t *module);

// Configuration
app_err_t web_server_set_port(web_server_module_t *module, uint16_t port);
app_err_t web_server_set_sd_root(web_server_module_t *module, const char *root_path);
app_err_t web_server_set_default_file(web_server_module_t *module, const char *default_file);
void web_server_set_upload_mode(web_server_module_t *module, bool use_native);

// SD card file serving
app_err_t web_server_setup_sd_file_serving(web_server_module_t *module);
app_err_t web_server_add_custom_endpoint(web_server_module_t *module, const char *uri, esp_err_t (*callback)(void *));

// Task management
app_err_t web_server_create_task(web_server_module_t *module);
void web_server_destroy_task(web_server_module_t *module);

// Download tuning
#ifdef __cplusplus
extern "C" {
#endif
void web_server_set_download_chunk_size(size_t bytes);
size_t web_server_get_download_chunk_size(void);
size_t web_server_get_default_download_chunk_size(void);
void tune_download_chunk_on_boot(void);
#ifdef __cplusplus
}
#endif

// Statistics
void web_server_get_stats(web_server_module_t *module, uint32_t *total_requests, uint32_t *successful_requests, uint32_t *failed_requests,
                          uint64_t *bytes_served);

// Convenience functions for static instance
app_err_t web_server_init_static(void);
app_err_t web_server_start_static(void);
app_err_t web_server_stop_static(void);
app_err_t web_server_restart_static(void);
bool web_server_is_running_static(void);
void web_server_get_stats_static(uint32_t *total_requests, uint32_t *successful_requests, uint32_t *failed_requests, uint64_t *bytes_served);

// Internal task function
void web_server_task(void *arg);

#endif  // defined(USE_WEB_FILE_SEREVER)
