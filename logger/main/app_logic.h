#pragma once

#include <stdbool.h>

#include "app_commands.h"
#include "app_errors.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "io/io_manager.h"
#include "modules/button.h"
#include "modules/led_module.h"
#include "modules/logger_module.h"
#include "target.h"

#if defined(USE_WIFI)
#include "wifi/wifi.h"
#endif
#if defined(USE_WEB_FILE_SEREVER)
#include "modules/web_server.h"
#endif
#include "util/event_sender.h"

typedef struct {
    TaskHandle_t app_logic_handle;
    TaskHandle_t led_task_handle;
    TaskHandle_t logger_task_handle;
} task_management_t;

// Static module instances to avoid heap fragmentation
extern io_manager_t g_io_manager;
extern led_module_t g_led_module;
extern led_module_t g_led_module;

extern button_t g_button;
extern button_t g_button_external;
#if defined(USE_WEB_FILE_SEREVER)
extern web_server_module_t g_web_server_module;
#endif

typedef struct app_logic_s {
    // Modules (now pointing to static instances)
    io_manager_t *io_manager;
    led_module_t *led_module;
    logger_module_t *logger_module;

    button_t *button;
    button_t *button_external;
#if defined(USE_WIFI)
    wifi_t *wifi;
    bool wifi_initialized;
    es_token_t wifi_event_subscription;
#endif
#if defined(USE_WEB_FILE_SEREVER)
    web_server_module_t *web_server;
    bool web_server_initialized;
#endif

    // Tasks handles
    TaskHandle_t app_logic_handle;
    TaskHandle_t led_task_handle;
    TaskHandle_t logger_task_handle;

#if defined(USE_WEB_FILE_SEREVER)
    TaskHandle_t web_server_task_handle;
#endif

} app_logic_t;

void app_logic_init(app_logic_t *app);
void app_logic_task(void *arg);
void app_logic_on_event(const es_event_t *ev, void *user);
app_err_t app_logic_init_modules(app_logic_t *app);
esp_err_t app_logic_start_all_tasks(app_logic_t *app);
esp_err_t app_logic_start_minimal_tasks(app_logic_t *app);
esp_err_t app_logic_send_command(app_logic_t *app, app_command_id_e cmd_id);
esp_err_t app_logic_send_command_full(app_logic_t *app, const app_command_t *cmd);
app_logic_t *get_global_app_logic_instance(void);
