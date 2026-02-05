#include "web_server.h"

#include "target.h"  // Include target.h to get USE_WEB_FILE_SEREVER definition

#if defined(USE_WEB_FILE_SEREVER)

#include <esp_err.h>
#include <esp_heap_caps.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>
#include <string.h>

#include "app_state.h"
#include "config_manager.h"
#include "hal/sdcard.h"
#include "io/io_manager.h"

// Use the global io manager instance
extern io_manager_t g_io_manager;
#include "log.h"
#include "wifi/wifi.h"

// Declare C++ wrapper functions
void *web_server_create_server(void);
void web_server_destroy_server(void *server);
esp_err_t web_server_start_server(void *server, uint16_t port);
void web_server_stop_server(void *server);
esp_err_t web_server_add_endpoint(void *server, const char *uri, esp_err_t (*callback)(void *));
esp_err_t web_server_add_post_endpoint(void *server, const char *uri, esp_err_t (*callback)(void *));
void *web_server_setup_static_files(void *server, const char *uri, const char *path, const char *cache_control);

static const char *TAG = "WEB_SERVER";

// Track which server instance has static file serving configured
static void *s_static_registered_on = NULL;

// Static assert to ensure stack size is properly aligned for FreeRTOS
_Static_assert((TASK_STACK_SIZE_WEB_SERVER % sizeof(StackType_t)) == 0, "TASK_STACK_SIZE_WEB_SERVER must be expressed in bytes and aligned to StackType_t");

// Static web server module instance
web_server_module_t g_web_server_module = {0};

// Forward declarations
void web_server_task(void *arg);
extern esp_err_t web_server_file_manager_handler(void *request);
extern esp_err_t web_server_download_handler(void *request);
extern esp_err_t web_server_upload_handler(void *request);
extern esp_err_t web_server_delete_handler(void *request);
extern esp_err_t web_server_delete_all_handler(void *request);
extern esp_err_t web_server_test_handler(void *request);
static portMUX_TYPE s_stats_lock = portMUX_INITIALIZER_UNLOCKED;
static void web_server_update_stats(web_server_module_t *module, bool success, size_t bytes_served);
void web_server_notify_request_result(bool success, size_t bytes_served);

app_err_t web_server_init(web_server_module_t *module) {
    if (!module) {
        LOG_E(TAG, "Module pointer is NULL");
        return APP_ERR_INVALID_ARG;
    }

    LOG_I(TAG, "Initializing Web Server module...");

    // Initialize module structure
    memset(module, 0, sizeof(web_server_module_t));

    // Set default configuration
    module->port = WEB_SERVER_PORT;
    strncpy(module->sd_root_path, WEB_SERVER_SD_ROOT_PATH, sizeof(module->sd_root_path) - 1);
    strncpy(module->default_file, WEB_SERVER_DEFAULT_INDEX, sizeof(module->default_file) - 1);
    strncpy(module->cache_control, WEB_SERVER_CACHE_CONTROL, sizeof(module->cache_control) - 1);
    module->use_native_upload = true;  // Default to native ESP upload
    // Captive portal is now automatically managed based on WiFi AP state

    module->state = WEB_SERVER_STATE_STOPPED;
    module->initialized = true;

    // Check SD card availability
    if (g_io_manager.sd_card_ok) {
        module->sd_card_available = true;
        LOG_I(TAG, "SD card is available for file serving");
    } else {
        module->sd_card_available = false;
        LOG_W(TAG, "SD card is not available - file serving will be limited");
    }

    // Tune download chunk size based on available heap
    tune_download_chunk_on_boot();

    LOG_I(TAG, "Web Server module initialized successfully");
    return APP_OK;
}

app_err_t web_server_deinit(web_server_module_t *module) {
    if (!module) {
        LOG_E(TAG, "Module pointer is NULL");
        return APP_ERR_INVALID_ARG;
    }

    LOG_I(TAG, "Deinitializing Web Server module...");

    // Stop server if running
    if (module->state == WEB_SERVER_STATE_RUNNING) {
        web_server_stop(module);
    }

    // Clean up server instance
    if (module->server) {
        web_server_destroy_server(module->server);
        module->server = NULL;
    }

    module->initialized = false;
    module->state = WEB_SERVER_STATE_STOPPED;

    LOG_I(TAG, "Web Server module deinitialized");
    return APP_OK;
}

app_err_t web_server_start(web_server_module_t *module) {
    if (!module || !module->initialized) {
        LOG_E(TAG, "Module not initialized");
        return APP_ERR_INVALID_ARG;
    }

    if (module->state == WEB_SERVER_STATE_RUNNING) {
        LOG_W(TAG, "Web server is already running");
        return APP_OK;
    }

    if (module->state == WEB_SERVER_STATE_STARTING) {
        LOG_W(TAG, "Web server is already starting");
        return APP_OK;
    }

    LOG_I(TAG, "Starting Web Server on port %d...", module->port);

    module->state = WEB_SERVER_STATE_STARTING;

    // Create server instance using C++ wrapper
    module->server = web_server_create_server();
    if (!module->server) {
        LOG_E(TAG, "Failed to create HTTP server instance");
        module->state = WEB_SERVER_STATE_ERROR;
        return APP_ERR_MEMORY_ALLOCATION;
    }

    // Start the server first using wrapper
    esp_err_t ret = web_server_start_server(module->server, module->port);
    if (ret != ESP_OK) {
        LOG_E(TAG, "Failed to start HTTP server: %s", esp_err_to_name(ret));
        web_server_destroy_server(module->server);
        module->server = NULL;
        module->state = WEB_SERVER_STATE_ERROR;
        return APP_ERR_GENERIC;
    }

    // Add custom endpoints after server is started
    LOG_I(TAG, "Registering web server endpoints...");

    esp_err_t register_err = web_server_add_endpoint(module->server, "/", web_server_file_manager_handler);
    if (register_err != ESP_OK) {
        LOG_E(TAG, "Failed to register '/' handler: %s", esp_err_to_name(register_err));
        goto fail_register;
    }
    LOG_I(TAG, "Root endpoint '/' registered successfully");

    register_err = web_server_add_endpoint(module->server, "/download", web_server_download_handler);
    if (register_err != ESP_OK) {
        LOG_E(TAG, "Failed to register '/download' handler: %s", esp_err_to_name(register_err));
        goto fail_register;
    }
    LOG_I(TAG, "Download endpoint '/download' registered successfully");

    register_err = web_server_add_post_endpoint(module->server, "/upload", web_server_upload_handler);
    if (register_err != ESP_OK) {
        LOG_E(TAG, "Failed to register '/upload' handler: %s", esp_err_to_name(register_err));
        goto fail_register;
    }
    LOG_I(TAG, "Upload endpoint '/upload' registered successfully");

    register_err = web_server_add_endpoint(module->server, "/delete", web_server_delete_handler);
    if (register_err != ESP_OK) {
        LOG_E(TAG, "Failed to register '/delete' handler: %s", esp_err_to_name(register_err));
        goto fail_register;
    }
    LOG_I(TAG, "Delete endpoint '/delete' registered successfully");

    register_err = web_server_add_endpoint(module->server, "/delete-all", web_server_delete_all_handler);
    if (register_err != ESP_OK) {
        LOG_E(TAG, "Failed to register '/delete-all' handler: %s", esp_err_to_name(register_err));
        goto fail_register;
    }
    LOG_I(TAG, "Delete All endpoint '/delete-all' registered successfully");

    // Add a simple test endpoint for debugging
    register_err = web_server_add_endpoint(module->server, "/test", web_server_test_handler);
    if (register_err != ESP_OK) {
        LOG_E(TAG, "Failed to register '/test' handler: %s", esp_err_to_name(register_err));
        goto fail_register;
    }
    LOG_I(TAG, "Test endpoint '/test' registered successfully");

    // Set up SD card file serving if available
    if (module->sd_card_available && module->server != s_static_registered_on) {
        LOG_I(TAG, "Setting up SD card file serving from %s", module->sd_root_path);
        app_err_t sd_err = web_server_setup_sd_file_serving(module);
        if (sd_err != APP_OK) {
            LOG_E(TAG, "Failed to configure SD card file serving");
            goto fail_register;
        }
        LOG_I(TAG, "SD card file serving configured successfully");
        s_static_registered_on = module->server;
    } else if (!module->sd_card_available) {
        LOG_W(TAG, "SD card not available, skipping file serving setup");
    } else {
        LOG_D(TAG, "SD card file serving already configured for this server instance");
    }

    // Captive portal is now managed by the captive_portal module
    // The web server just provides the HTTP server instance
    LOG_I(TAG, "Web server ready for captive portal integration");

    module->state = WEB_SERVER_STATE_RUNNING;
    LOG_I(TAG, "Web Server started successfully on port %d", module->port);

    return APP_OK;

fail_register:
    if (module->server) {
        web_server_stop_server(module->server);
        web_server_destroy_server(module->server);
        module->server = NULL;
    }
    module->state = WEB_SERVER_STATE_ERROR;
    return APP_ERR_GENERIC;
}

app_err_t web_server_stop(web_server_module_t *module) {
    if (!module || !module->initialized) {
        LOG_E(TAG, "Module not initialized");
        return APP_ERR_INVALID_ARG;
    }

    if (module->state != WEB_SERVER_STATE_RUNNING) {
        LOG_W(TAG, "Web server is not running");
        return APP_OK;
    }

    LOG_I(TAG, "Stopping Web Server...");

    module->state = WEB_SERVER_STATE_STOPPING;

    if (module->server) {
        web_server_stop_server(module->server);
        web_server_destroy_server(module->server);
        module->server = NULL;
    }

    module->state = WEB_SERVER_STATE_STOPPED;
    LOG_I(TAG, "Web Server stopped successfully");

    return APP_OK;
}

bool web_server_is_running(web_server_module_t *module) {
    if (!module) {
        return false;
    }
    return module->state == WEB_SERVER_STATE_RUNNING;
}

app_err_t web_server_setup_sd_file_serving(web_server_module_t *module) {
    if (!module || !module->server) {
        LOG_E(TAG, "Module or server not initialized");
        return APP_ERR_INVALID_ARG;
    }

    if (!module->sd_card_available) {
        LOG_W(TAG, "SD card not available for file serving");
        return APP_ERR_GENERIC;
    }

    LOG_I(TAG, "Setting up SD card file serving from %s", module->sd_root_path);

    // Serve static files from SD card using wrapper
    void *staticHandler = web_server_setup_static_files(module->server, "/static", module->sd_root_path, module->cache_control);

    if (staticHandler) {
        web_server_set_default_file(staticHandler, module->default_file);
        LOG_I(TAG, "SD card file serving configured successfully");
        return APP_OK;
    } else {
        LOG_E(TAG, "Failed to configure SD card file serving");
        return APP_ERR_GENERIC;
    }
}

app_err_t web_server_add_custom_endpoint(web_server_module_t *module, const char *uri, esp_err_t (*callback)(void *)) {
    if (!module || !module->server || !uri || !callback) {
        LOG_E(TAG, "Invalid parameters for custom endpoint");
        return APP_ERR_INVALID_ARG;
    }

    if (module->state != WEB_SERVER_STATE_RUNNING) {
        LOG_E(TAG, "Cannot add endpoint - server not running");
        return APP_ERR_INVALID_STATE;
    }

    esp_err_t err = web_server_add_endpoint(module->server, uri, callback);
    if (err != ESP_OK) {
        LOG_E(TAG, "Failed to add custom endpoint %s: %s", uri, esp_err_to_name(err));
        return APP_ERR_GENERIC;
    }

    LOG_I(TAG, "Added custom endpoint: %s", uri);

    return APP_OK;
}

app_err_t web_server_create_task(web_server_module_t *module) {
    if (!module || !module->initialized) {
        LOG_E(TAG, "Module not initialized");
        return APP_ERR_INVALID_ARG;
    }

    if (module->task_handle) {
        LOG_W(TAG, "Web server task already exists");
        return APP_OK;
    }

    BaseType_t ret = xTaskCreatePinnedToCore(web_server_task, "WEB_SERVER", TASK_STACK_SIZE_WEB_SERVER / sizeof(StackType_t), module, TASK_PRIORITY_WEB_SERVER,
                                             &module->task_handle, 0);

    if (ret != pdPASS) {
        LOG_E(TAG, "Failed to create web server task");
        return APP_ERR_TASK_CREATION;
    }

    LOG_I(TAG, "Web server task created successfully");
    return APP_OK;
}

void web_server_destroy_task(web_server_module_t *module) {
    if (!module) {
        return;
    }

    if (module->task_handle) {
        TaskHandle_t task_to_delete = module->task_handle;
        module->task_handle = NULL;  // Clear handle immediately
        vTaskDelete(task_to_delete);
        // Give idle task time to clean up TCB and stack memory
        for (int i = 0; i < 10; i++) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        LOG_I(TAG, "Web server task destroyed");
    }
}

void web_server_get_stats(web_server_module_t *module, uint32_t *total_requests, uint32_t *successful_requests, uint32_t *failed_requests,
                          uint64_t *bytes_served) {
    if (!module) {
        return;
    }

    portENTER_CRITICAL(&s_stats_lock);
    uint32_t total = module->total_requests;
    uint32_t success = module->successful_requests;
    uint32_t failed = module->failed_requests;
    uint64_t bytes = module->bytes_served;
    portEXIT_CRITICAL(&s_stats_lock);

    if (total_requests) *total_requests = total;
    if (successful_requests) *successful_requests = success;
    if (failed_requests) *failed_requests = failed;
    if (bytes_served) *bytes_served = bytes;
}

// Download chunk size tuning based on available heap
void tune_download_chunk_on_boot(void) {
    size_t free_heap = heap_caps_get_free_size(MALLOC_CAP_8BIT);

    if (free_heap > 80000) {
        web_server_set_download_chunk_size(8 * 1024);
    } else if (free_heap > 40000) {
        web_server_set_download_chunk_size(4 * 1024);
    } else {
        web_server_set_download_chunk_size(2 * 1024);
    }
}

// Convenience functions for static instance
app_err_t web_server_init_static(void) { return web_server_init(&g_web_server_module); }

app_err_t web_server_start_static(void) { return web_server_start(&g_web_server_module); }

app_err_t web_server_stop_static(void) { return web_server_stop(&g_web_server_module); }

app_err_t web_server_restart_static(void) {
    LOG_I(TAG, "Restarting web server to recover from memory issues...");

    // Stop the server first
    app_err_t stop_result = web_server_stop(&g_web_server_module);
    if (stop_result != APP_OK) {
        LOG_W(TAG, "Warning: Stop returned %d during restart", stop_result);
    }

    // Small delay to allow cleanup
    vTaskDelay(pdMS_TO_TICKS(1000));

    // Start the server again
    app_err_t start_result = web_server_start(&g_web_server_module);
    if (start_result == APP_OK) {
        LOG_I(TAG, "Web server restart successful");
    } else {
        LOG_E(TAG, "Web server restart failed: %d", start_result);
    }

    return start_result;
}

bool web_server_is_running_static(void) { return web_server_is_running(&g_web_server_module); }

void web_server_get_stats_static(uint32_t *total_requests, uint32_t *successful_requests, uint32_t *failed_requests, uint64_t *bytes_served) {
    web_server_get_stats(&g_web_server_module, total_requests, successful_requests, failed_requests, bytes_served);
}

// Internal task function
void web_server_task(void *arg) {
    web_server_module_t *module = (web_server_module_t *)arg;

    LOG_I(TAG, "Web server task started");

    while (1) {
        // Monitor server state and handle any maintenance tasks
        if (module->state == WEB_SERVER_STATE_RUNNING) {
            // Server is running normally
            vTaskDelay(pdMS_TO_TICKS(1000));
        } else if (module->state == WEB_SERVER_STATE_ERROR) {
            // Handle error state - could attempt restart
            LOG_E(TAG, "Web server in error state");
            vTaskDelay(pdMS_TO_TICKS(5000));
        } else {
            // Server is stopped or stopping
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}

static void web_server_update_stats(web_server_module_t *module, bool success, size_t bytes_served) {
    if (!module) {
        return;
    }

    portENTER_CRITICAL(&s_stats_lock);
    module->total_requests++;
    if (success) {
        module->successful_requests++;
        module->bytes_served += bytes_served;
    } else {
        module->failed_requests++;
    }
    portEXIT_CRITICAL(&s_stats_lock);
}

void web_server_notify_request_result(bool success, size_t bytes_served) { web_server_update_stats(&g_web_server_module, success, bytes_served); }

void web_server_set_upload_mode(web_server_module_t *module, bool use_native) {
    if (!module) {
        LOG_E(TAG, "Module pointer is NULL");
        return;
    }

    module->use_native_upload = use_native;
    LOG_I(TAG, "Upload mode set to: %s", use_native ? "Native ESP" : "PsychicHttp");
}

#endif  // defined(USE_WEB_FILE_SEREVER)
