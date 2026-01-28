#include "app_logic.h"

#include <esp_err.h>
#include <esp_heap_caps.h>
#include <esp_system.h>
#include <esp_task_wdt.h>
#include <esp_timer.h>
#include <driver/gpio.h>
#include <hal/gpio.h>
#include <log.h>
#include <stdlib.h>
#include <string.h>
#include <target.h>

#include "app_commands.h"
#include "app_state.h"
#include "config_manager.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "hal/sdcard.h"
#include "io/io_manager.h"
#include "modules/button.h"
#include "modules/led_module.h"
#include "modules/logger_module.h"
#include "modules/remote_accel_reader.h"

#if defined(USE_WIFI)
#include "wifi/wifi.h"
#endif
#if defined(USE_WEB_FILE_SEREVER)
#include "modules/web_server.h"
#endif
#if defined(USE_USB_FILE_SERVER)
#include "tinyusb.h"
// clang-format off

// clang-format on
#include "tinyusb_msc.h"
#include "usb_msc.h"

#endif
#include "platform/system.h"

static const char *TAG = "APP";

// Static asserts to ensure stack sizes are properly aligned for FreeRTOS
_Static_assert((TASK_STACK_SIZE_LED_BLINK % sizeof(StackType_t)) == 0, "TASK_STACK_SIZE_LED_BLINK must be expressed in bytes and aligned to StackType_t");
_Static_assert((TASK_STACK_SIZE_IO_MANAGER % sizeof(StackType_t)) == 0, "TASK_STACK_SIZE_IO_MANAGER must be expressed in bytes and aligned to StackType_t");
_Static_assert((TASK_STACK_SIZE_APP_LOGIC % sizeof(StackType_t)) == 0, "TASK_STACK_SIZE_APP_LOGIC must be expressed in bytes and aligned to StackType_t");

// Static module instances to avoid heap fragmentation
io_manager_t g_io_manager;
led_module_t g_led_module;
button_t g_button;
button_t g_button_external;
#if defined(USE_WIFI)
wifi_t g_wifi_module;
#endif

static QueueHandle_t g_command_queue;
static app_logic_t *g_app_logic_instance = NULL;

static void app_logic_turn_off_leds_for_reboot(void) {
#if defined(USE_LED_OUT)
    gpio_reset_pin(LED_OUT_GPIO);
    (void)hal_gpio_setup(LED_OUT_GPIO, HAL_GPIO_DIR_OUTPUT, HAL_GPIO_PULL_NONE);
    (void)hal_gpio_set_level(LED_OUT_GPIO, HAL_GPIO_LOW);
#endif

#if BOARD_LED_ENABLED
    if (BOARDLED_PIN >= 0) {
        gpio_reset_pin(BOARDLED_PIN);
        (void)hal_gpio_setup(BOARDLED_PIN, HAL_GPIO_DIR_OUTPUT, HAL_GPIO_PULL_NONE);
        (void)hal_gpio_set_level(BOARDLED_PIN, HAL_GPIO_LOW);
    }
#endif

#if defined(USE_RGB_LED)
    (void)hal_ws2812_open(RGB_LED_GPIO);
    const hal_ws2812_color_t off = HAL_WS2812_OFF;
    (void)hal_ws2812_set_colors(RGB_LED_GPIO, (hal_ws2812_color_order_e)LED_COLOR_ORDER, &off, 1);
#endif
}

static void app_logic_reboot_with_leds_off(app_logic_t *app) {
    if (app && app->led_task_handle) {
        vTaskSuspend(app->led_task_handle);
    }
    app_logic_turn_off_leds_for_reboot();
    vTaskDelay(pdMS_TO_TICKS(50));
    system_reboot();
}

static void app_logic_reboot_to_boot_mode_with_leds_off(app_logic_t *app) {
    if (app && app->led_task_handle) {
        vTaskSuspend(app->led_task_handle);
    }
    app_logic_turn_off_leds_for_reboot();
    vTaskDelay(pdMS_TO_TICKS(50));
    system_reboot_to_boot_mode();
}

static void app_logic_delete_task_handle(TaskHandle_t *handle) {
    if (handle && *handle) {
        // Get task name for logging before deletion
        const char *task_name = pcTaskGetName(*handle);
        LOG_I(TAG, "Deleting task: %s", task_name ? task_name : "Unknown");

        // Store handle locally before deletion
        TaskHandle_t task_to_delete = *handle;
        *handle = NULL;  // Clear handle immediately to prevent reuse

        // Delete the task
        vTaskDelete(task_to_delete);

        // CRITICAL: Give the idle task sufficient time to clean up the TCB and stack memory
        // The idle task runs at lowest priority, so we need to yield multiple times
        // to ensure it gets CPU time to process the deletion. Also check heap integrity.
        size_t heap_before = esp_get_free_heap_size();
        for (int i = 0; i < 20; i++) {
            vTaskDelay(pdMS_TO_TICKS(10));  // Yield 20 times, 10ms each = 200ms total
            size_t heap_now = esp_get_free_heap_size();
            // If heap increased significantly, the task cleanup likely completed
            if (heap_now > heap_before + 1024) {
                break;  // Task cleanup appears complete
            }
        }

        LOG_I(TAG, "Task deleted successfully");
    }
}

static void app_logic_prepare_download_mode(app_logic_t *app);
static app_err_t app_logic_restore_logging_resources(app_logic_t *app);
static void app_logic_reconfigure_io_for_logging(app_logic_t *app);

// USB file server functions
#if defined(USE_USB_FILE_SERVER)
static bool g_usb_file_server_initialized = false;
static bool g_usb_file_server_running = false;
static tinyusb_msc_storage_handle_t s_msc_storage_hdl = NULL;

// ---- TinyUSB build-time config dump ----
static void tinyusb_log_compiled_config(void) {
    LOG_I(TAG, "TinyUSB compiled class config:");
    /* NOLINTBEGIN */
#if defined(CFG_TUD_MSC)
    LOG_I(TAG, "CFG_TUD_MSC=%d", (int)(CFG_TUD_MSC));
#else
    LOG_I(TAG, "CFG_TUD_MSC is NOT defined");
#endif
#if defined(CFG_TUD_CDC)
    LOG_I(TAG, "CFG_TUD_CDC=%d", (int)(CFG_TUD_CDC));
#else
    LOG_I(TAG, "CFG_TUD_CDC is NOT defined");
#endif
#if defined(CFG_TUD_HID)
    LOG_I(TAG, "CFG_TUD_HID=%d", (int)(CFG_TUD_HID));
#else
    LOG_I(TAG, "CFG_TUD_HID is NOT defined");
#endif
#if defined(CFG_TUD_MIDI)
    LOG_I(TAG, "CFG_TUD_MIDI=%d", (int)(CFG_TUD_MIDI));
#else
    LOG_I(TAG, "CFG_TUD_MIDI is NOT defined");
#endif
#if defined(CFG_TUD_VENDOR)
    LOG_I(TAG, "CFG_TUD_VENDOR=%d", (int)(CFG_TUD_VENDOR));
#else
    LOG_I(TAG, "CFG_TUD_VENDOR is NOT defined");
#endif
#if defined(CFG_TUD_DFU_RUNTIME)
    LOG_I(TAG, "CFG_TUD_DFU_RUNTIME=%d", (int)(CFG_TUD_DFU_RUNTIME));
#else
    LOG_I(TAG, "CFG_TUD_DFU_RUNTIME is NOT defined");
#endif
#if defined(CFG_TUD_NET)
    LOG_I(TAG, "CFG_TUD_NET=%d", (int)(CFG_TUD_NET));
#else
    LOG_I(TAG, "CFG_TUD_NET is NOT defined");
#endif
#if defined(CFG_TUD_AUDIO)
    LOG_I(TAG, "CFG_TUD_AUDIO=%d", (int)(CFG_TUD_AUDIO));
#else
    LOG_I(TAG, "CFG_TUD_AUDIO is NOT defined");
#endif
#if defined(CFG_TUD_VIDEO)
    LOG_I(TAG, "CFG_TUD_VIDEO=%d", (int)(CFG_TUD_VIDEO));
#else
    LOG_I(TAG, "CFG_TUD_VIDEO is NOT defined");
#endif
#if defined(CFG_TUD_ECM)
    LOG_I(TAG, "CFG_TUD_ECM=%d", (int)(CFG_TUD_ECM));
#else
    LOG_I(TAG, "CFG_TUD_ECM is NOT defined");
#endif
#if defined(CFG_TUD_NCM)
    LOG_I(TAG, "CFG_TUD_NCM=%d", (int)(CFG_TUD_NCM));
#else
    LOG_I(TAG, "CFG_TUD_NCM is NOT defined");
#endif
#if defined(CFG_TUD_RNDIS)
    LOG_I(TAG, "CFG_TUD_RNDIS=%d", (int)(CFG_TUD_RNDIS));
#else
    LOG_I(TAG, "CFG_TUD_RNDIS is NOT defined");
#endif
#if defined(CFG_TUD_ENDPOINT0_SIZE)
    LOG_I(TAG, "CFG_TUD_ENDPOINT0_SIZE=%d", (int)(CFG_TUD_ENDPOINT0_SIZE));
#else
    LOG_I(TAG, "CFG_TUD_ENDPOINT0_SIZE is NOT defined");
#endif
    /* NOLINTEND */
}

static bool usb_file_server_is_running(void) { return g_usb_file_server_running; }

extern const tusb_desc_device_t g_usb_desc_device;
extern const uint8_t g_usb_desc_fs_config[];
extern const uint8_t g_usb_desc_hs_config[];
extern const char *g_usb_string_desc_arr[];
extern const size_t g_usb_string_desc_count;

static app_err_t usb_file_server_start(void) {
    LOG_I(TAG, "USB file server start requested");

    // Check if already running
    if (g_usb_file_server_running) {
        LOG_W(TAG, "USB file server already running");
        return APP_OK;
    }

    // 1. Switch SD card to raw mode
    LOG_I(TAG, "Switching SD card to raw mode...");
    hal_err_t err = sdcard_mount_raw();
    if (err != HAL_ERR_NONE) {
        LOG_E(TAG, "Failed to mount SD card in raw mode: %d", err);
        return APP_ERR_GENERIC;
    }

    // 2. Initialize TinyUSB stack (if not already)
    if (!g_usb_file_server_initialized) {
        LOG_I(TAG, "Initializing TinyUSB stack...");

        // Install MSC driver first (required before creating storage)
        const tinyusb_msc_driver_config_t msc_driver_cfg = {
            .callback = NULL,
            .callback_arg = NULL,
        };
        ESP_ERROR_CHECK(tinyusb_msc_install_driver(&msc_driver_cfg));

        tinyusb_log_compiled_config();

        const tinyusb_config_t tusb_cfg = {
            .port = 0,
            .descriptor =
                {
                    .device = &g_usb_desc_device,
                    .string = g_usb_string_desc_arr,
                    .string_count = g_usb_string_desc_count,
                    .full_speed_config = g_usb_desc_fs_config,
                    .high_speed_config = g_usb_desc_hs_config,
                },
            .task =
                {
                    .priority = TASK_PRIORITY_LOGGER,  // Favor MSC over lower-priority tasks
                    .size = 12288,                     // Larger stack for USB + MSC buffering
                },
        };

        esp_err_t err = tinyusb_driver_install(&tusb_cfg);
        if (err != ESP_OK) {
            LOG_E(TAG, "Failed to install TinyUSB driver: %s", esp_err_to_name(err));
            tinyusb_msc_uninstall_driver();
            sdcard_unmount_raw();
            sdcard_mount();
            return APP_ERR_GENERIC;
        }

        // Register SD card as MSC LUN (required by esp_tinyusb MSC layer)
        if (s_msc_storage_hdl == NULL) {
            sdmmc_card_t *card = sdcard_get_card();
            if (!card) {
                LOG_E(TAG, "SD card handle is NULL, can't expose MSC");
                tinyusb_driver_uninstall();
                tinyusb_msc_uninstall_driver();
                sdcard_unmount_raw();
                sdcard_mount();
                return APP_ERR_GENERIC;
            }
            const tinyusb_msc_storage_config_t cfg = {
                .medium.card = card,
            };
            err = tinyusb_msc_new_storage_sdmmc(&cfg, &s_msc_storage_hdl);
            if (err != ESP_OK) {
                LOG_E(TAG, "Failed to create MSC storage: %s", esp_err_to_name(err));
                tinyusb_driver_uninstall();
                tinyusb_msc_uninstall_driver();
                sdcard_unmount_raw();
                sdcard_mount();
                return APP_ERR_GENERIC;
            }
        }

        g_usb_file_server_initialized = true;
        LOG_I(TAG, "TinyUSB stack initialized successfully");
    }

    // Enable USB connection
    tud_connect();

    // Mark as running
    g_usb_file_server_running = true;
    LOG_I(TAG, "USB file server started successfully");

    return APP_OK;
}

static app_err_t usb_file_server_stop(void) {
    LOG_I(TAG, "USB file server stop requested");

    if (!g_usb_file_server_running) {
        LOG_W(TAG, "USB file server is not running");
        return APP_OK;
    }

    // 0. Disconnect USB (forces host to drop MSC; COM will return after reboot/USB re-init)
    tud_disconnect();
    // Give host time to process disconnect and close handles (Windows can be slow)
    vTaskDelay(pdMS_TO_TICKS(500));

    // 1. Unmount raw SD card
    sdcard_unmount_raw();

    // 2. Remount VFS for app usage
    LOG_I(TAG, "Restoring SD card VFS mount...");
    if (sdcard_mount() != HAL_ERR_NONE) {
        LOG_E(TAG, "Failed to restore SD card VFS mount");
        // This is bad, but we can't do much else
    }

    // 3. Tear down MSC + TinyUSB stack so the USB peripheral is released cleanly.
    // Without this, the device stays in MSC-only mode and CDC/COM won't come back until reboot.
    if (s_msc_storage_hdl) {
        esp_err_t del_err = tinyusb_msc_delete_storage(s_msc_storage_hdl);
        if (del_err != ESP_OK) {
            LOG_W(TAG, "tinyusb_msc_delete_storage failed: %s", esp_err_to_name(del_err));
        }
        s_msc_storage_hdl = NULL;
    }

    esp_err_t tusb_uninstall_err = tinyusb_driver_uninstall();
    if (tusb_uninstall_err != ESP_OK) {
        LOG_W(TAG, "tinyusb_driver_uninstall failed: %s", esp_err_to_name(tusb_uninstall_err));
    }

    esp_err_t msc_uninstall_err = tinyusb_msc_uninstall_driver();
    if (msc_uninstall_err != ESP_OK) {
        LOG_W(TAG, "tinyusb_msc_uninstall_driver failed: %s", esp_err_to_name(msc_uninstall_err));
    }

    g_usb_file_server_initialized = false;

    // Mark as stopped
    g_usb_file_server_running = false;
    LOG_I(TAG, "USB file server stopped successfully");

    return APP_OK;
}
#endif

// Button event handlers
static void button_double_press_handler(const button_event_t *ev, void *user_data) {
    LOG_I(TAG, "Button double press detected - toggling logging mode");
    app_logic_t *app = (app_logic_t *)user_data;
    app_state_t *state = app_state_get_instance();

    // Toggle between logging and idle modes
    if (state->current_mode == APP_MODE_LOGGING) {
        app_logic_send_command(app, APP_CMD_SET_MODE_IDLE);
    } else {
        // Ensure logging-related tasks are running when switching into LOGGING
        if (app_logic_start_all_tasks(app) != ESP_OK) {
            LOG_E(TAG, "Failed to start logging tasks");
            return;
        }
        app_logic_send_command(app, APP_CMD_SET_MODE_LOGGING);
    }
}

static void button_long_press_handler(const button_event_t *ev, void *user_data) {
    LOG_I(TAG, "Button long press detected -> toggling online mode");
    app_logic_t *app = (app_logic_t *)user_data;
    app_state_t *state = app_state_get_instance();

#if defined(USE_WEB_FILE_SEREVER)
    const bool web_server_active = app->web_server && web_server_is_running(app->web_server);
    const bool wifi_up = state->wifi_sta_active || state->wifi_ap_active;

    if (web_server_active || wifi_up) {
        LOG_I(TAG, "Online mode is active. Sending command to stop.");
        app_logic_send_command(app, APP_CMD_ONLINE_MODE_STOP);
    } else {
        LOG_I(TAG, "Online mode is inactive. Sending command to start.");
        // First stop logging only if active, then start WiFi
        if (state->current_mode == APP_MODE_LOGGING) {
            app_logic_send_command(app, APP_CMD_STOP_LOGGING_AND_FREE_MEMORY);
        }
        app_logic_send_command(app, APP_CMD_ONLINE_MODE_START);
    }
#elif defined(USE_WIFI)
    const bool wifi_up = state->wifi_sta_active || state->wifi_ap_active;

    if (wifi_up) {
        LOG_I(TAG, "WiFi is active. Sending command to stop.");
        app_logic_send_command(app, APP_CMD_ONLINE_MODE_STOP);
    } else {
        LOG_I(TAG, "WiFi is inactive. Sending command to start.");
        // First stop logging only if active, then start WiFi
        if (state->current_mode == APP_MODE_LOGGING) {
            app_logic_send_command(app, APP_CMD_STOP_LOGGING_AND_FREE_MEMORY);
        }
        app_logic_send_command(app, APP_CMD_ONLINE_MODE_START);
    }
#elif defined(USE_USB_FILE_SERVER)
    if (usb_file_server_is_running()) {
        LOG_I(TAG, "USB file server is active. Sending command to stop.");
        app_logic_send_command(app, APP_CMD_STOP_USB_FILE_SERVER);
    } else {
        LOG_I(TAG, "USB file server is inactive. Sending command to start.");
        // First stop logging only if active, then start USB file server
        if (state->current_mode == APP_MODE_LOGGING) {
            app_logic_send_command(app, APP_CMD_STOP_LOGGING_AND_FREE_MEMORY);
        }
        app_logic_send_command(app, APP_CMD_START_USB_FILE_SERVER);
    }
#endif
}

static void button_event_handler(const button_event_t *ev, void *user_data) {
    LOG_I(TAG, "Button event detected: type=%d, button=%d", ev->type, ev->button->id);
    switch (ev->type) {
        case BUTTON_EVENT_TYPE_DOUBLE_PRESS:
            button_double_press_handler(ev, user_data);
            break;
        case BUTTON_EVENT_TYPE_SUPER_LONG_PRESS:
            button_long_press_handler(ev, user_data);
            break;
        case BUTTON_EVENT_TYPE_TRIPLE_PRESS:
            // LOG_I(TAG, "Button %d triple press detected - rebooting to boot mode", ev->button->id);
            app_logic_t *app = (app_logic_t *)user_data;
            // app_logic_send_command(app, APP_CMD_REBOOT_TO_BOOT);
            app_logic_send_command(app, APP_CMD_REBOOT);
            break;
        default:
            // Handle other button events if needed
            break;
    }
}

void app_logic_init(app_logic_t *app) {
    LOG_I(TAG, "Initializing Application Logic...");

    // Store the global instance
    g_app_logic_instance = app;

#if defined(USE_WIFI)
    app->wifi = &g_wifi_module;
    app->wifi_initialized = false;
    app->wifi_event_subscription.idx = 0;
    app->wifi_event_subscription.gen = 0;
#endif

    // Initialize LED Module (using static instance to avoid heap fragmentation)
    app->led_module = &g_led_module;
    led_module_init(app->led_module);
    if (app->led_module->is_initialized) {
        BaseType_t led_ret = xTaskCreatePinnedToCore(led_blink_task, "LED_BLINK", TASK_STACK_SIZE_LED_BLINK / sizeof(StackType_t), app->led_module,
                                                     TASK_PRIORITY_LED_BLINK, &app->led_task_handle, 0);
        if (led_ret != pdPASS) {
            LOG_E(TAG, "Failed to create LED task");
            return;
        } else {
            LOG_I(TAG, "LED task created successfully");
        }
    } else {
        LOG_W(TAG, "Skipping LED task creation, module not initialized.");
    }

    // Initialize IO Manager (common for all roles; SD may be absent on some boards)
    app->io_manager = &g_io_manager;

    if (io_manager_init(app->io_manager) != HAL_ERR_NONE) {
        LOG_E(TAG, "Failed to initialize IO Manager. Halting.");
        app_state_set_error(APP_ERR_IO_INIT_FAILED);
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    // Load configuration now that IO is ready
    config_manager_load();

    // Initialize Logger Module (using static instance to avoid heap fragmentation)
    app->logger_module = &g_logger_module;

    // Initialize Button Module (using static instance to avoid heap fragmentation)
    app->button = &g_button;
    button_config_t button_cfg = {.gpio = BUTTON_ENTER_GPIO};
    app->button->cfg = button_cfg;
    app->button->id = BUTTON_ID_ENTER;
    app->button->callback = button_event_handler;
    app->button->user_data = app;
    button_init(app->button);

    // Initialize External Button Module (using static instance to avoid heap fragmentation)
    app->button_external = &g_button_external;
    button_config_t button_external_cfg = {.gpio = BUTTON_EXTERNAL_GPIO};
    app->button_external->cfg = button_external_cfg;
    app->button_external->id = BUTTON_ID_BOOT;
    app->button_external->callback = button_event_handler;
    app->button_external->user_data = app;
    button_init(app->button_external);

#if defined(USE_WEB_FILE_SEREVER)
    // Initialize Web Server Module (using static instance to avoid heap fragmentation)
    app->web_server = &g_web_server_module;
    app->web_server_initialized = false;
#endif

    g_command_queue = xQueueCreate(10, sizeof(app_command_t));
    if (!g_command_queue) {
        LOG_E(TAG, "Failed to create app command queue");
        app_state_set_error(APP_ERR_MEMORY_ALLOCATION_FAILED);
        return;
    }

#if defined(USE_WIFI)
    es_t *event_sender = app_state_get_event_sender();
    if (event_sender) {
        if (es_subscribe(event_sender, app_logic_on_event, app, 0, &app->wifi_event_subscription) == 0) {
            LOG_I(TAG, "App logic subscribed to application events");
        } else {
            LOG_W(TAG, "Failed to subscribe app logic to event sender");
        }
    } else {
        LOG_W(TAG, "Event sender not available for app logic subscription");
    }
#endif

    if (app_state_get_instance()->system_error_code != APP_OK) {
        LOG_E(TAG, "Application Logic Initialization failed. Halting.");
    } else {
        LOG_I(TAG, "Application Logic Initialized successfully.");
    }
}

app_err_t app_logic_init_modules(app_logic_t *app) {
    LOG_I(TAG, "Initializing application modules...");
    app_state_t *state = app_state_get_instance();

    // Reset STM32
    LOG_I(TAG, "Resetting STM32 on GPIO %d", STM_RESET_GPIO);
    gpio_reset_pin(STM_RESET_GPIO);
    gpio_set_direction(STM_RESET_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(STM_RESET_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(STM_RESET_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(500));
    // Set back to Hi-Z (input) so ST-LINK can control STM32 reset for debugging
    gpio_set_direction(STM_RESET_GPIO, GPIO_MODE_INPUT);
    LOG_I(TAG, "STM32 reset complete, GPIO %d set to Hi-Z for ST-LINK access", STM_RESET_GPIO);

    // All sensing and frame generation is handled by STM32.

    // Initialize Logger module
    if (logger_module_init_static(app->io_manager->sd_card_ok) != HAL_ERR_NONE) {
        LOG_E(TAG, "Failed to initialize logger module. Entering error state.");
        app_state_set_error(APP_ERR_LOGGER_INIT_FAILED);
        return APP_ERR_LOGGER_INIT_FAILED;
    }

    // Set up SPI slave link to receive data from STM32
    remote_accel_reader_init();


    LOG_I(TAG, "All application modules initialized successfully.");
    return APP_OK;
}

static void app_logic_prepare_download_mode(app_logic_t *app) {
    if (!app) {
        return;
    }

    // MEMORY DEBUG: Track task cleanup
    size_t heap_before_tasks = esp_get_free_heap_size();
    LOG_I(TAG, "=== TASK CLEANUP FOR DOWNLOAD MODE ===");
    LOG_I(TAG, "Free heap BEFORE task cleanup: %zu bytes (%.2f KB)", heap_before_tasks, heap_before_tasks / 1024.0f);
    LOG_I(TAG, "Quiescing peripheral tasks for download mode");

    // IO Manager task removed (UART functionality removed)

    if (app->io_manager) {
        LOG_I(TAG, "Shutting down IO Manager resources...");
        io_manager_shutdown(app->io_manager);
        LOG_I(TAG, "IO Manager shutdown complete");
    }

    // No ESP-side sensor tasks to stop; acquisition is handled by STM32.

    // OPTIONAL: Stop LED task during download mode to save additional ~1.5KB
    // LED provides visual feedback but isn't essential for file downloads
    // Comment out these lines if you want LED indication during download
    /*
    if (app->led_task_handle) {
        LOG_I(TAG, "Stopping LED task (stack: 1.5KB) - optional optimization...");
        size_t heap_before_led = esp_get_free_heap_size();
        app_logic_delete_task_handle(&app->led_task_handle);
        size_t heap_after_led = esp_get_free_heap_size();
        LOG_I(TAG, "LED task stopped. Heap: %zu bytes (+%zu freed)",
              heap_after_led, heap_after_led - heap_before_led);
    }
    */

    // Logger task is deleted inside logger_module_stop_and_free_memory; clear mirror
    if (app->logger_module) {
        app->logger_module->writer_task_handle = NULL;
    }
    app->logger_task_handle = NULL;

    // MEMORY DEBUG: Summary
    size_t heap_after_tasks = esp_get_free_heap_size();
    size_t total_task_memory_freed = heap_after_tasks - heap_before_tasks;
    LOG_I(TAG, "=== TASK CLEANUP SUMMARY ===");
    LOG_I(TAG, "Free heap AFTER task cleanup: %zu bytes (%.2f KB)", heap_after_tasks, heap_after_tasks / 1024.0f);
    LOG_I(TAG, "Total task memory freed: %zu bytes (%.2f KB)", total_task_memory_freed, total_task_memory_freed / 1024.0f);
    LOG_I(TAG, "Expected task stack savings depend on the active feature set.");
    if (total_task_memory_freed >= 5000) {
        LOG_I(TAG, "SUCCESS: Significant task memory freed (%zu KB)", total_task_memory_freed / 1024);
    } else if (total_task_memory_freed >= 2000) {
        LOG_W(TAG, "PARTIAL: Some task memory freed (%zu KB) but less than expected", total_task_memory_freed / 1024);
    } else {
        LOG_E(TAG, "FAILED: Minimal task memory freed (%zu KB) - tasks may not have been running", total_task_memory_freed / 1024);
    }
    LOG_I(TAG, "============================");
}

static void app_logic_reconfigure_io_for_logging(app_logic_t *app) {
    if (!app || !app->io_manager) {
        return;
    }
}

static app_err_t app_logic_restore_logging_resources(app_logic_t *app) {
    if (!app) {
        return APP_ERR_INVALID_ARG;
    }

    app_logic_reconfigure_io_for_logging(app);

    // Re-initialize Logger if needed
    if (app->logger_module) {
        if (!app->logger_module->initialized) {
            bool sd_ok = app->io_manager ? app->io_manager->sd_card_ok : false;
            if (logger_module_init_static(sd_ok) != HAL_ERR_NONE) {
                LOG_E(TAG, "Failed to reinitialize logger module. Entering error state.");
                app_state_set_error(APP_ERR_LOGGER_INIT_FAILED);
                return APP_ERR_LOGGER_INIT_FAILED;
            }
        }
        // Create logger task
        if (app->logger_module->initialized && app->logger_module->sd_card_ok) {
            if (app->logger_module->writer_task_handle == NULL) {
                esp_err_t logger_result = logger_module_create_task_static();
                if (logger_result != ESP_OK) {
                    LOG_E(TAG, "Failed to create logger tasks - insufficient memory or other error. Entering error state.");
                    app_state_set_error(APP_ERR_LOGGER_INIT_FAILED);
                    return APP_ERR_GENERIC; // Changed from result = ESP_FAIL; to return APP_ERR_GENERIC; for consistency
                } else {
                    app->logger_task_handle = app->logger_module->writer_task_handle;
                    LOG_I(TAG, "Logger tasks created successfully");
                }
            } else {
                LOG_I(TAG, "Logger tasks already running");
            }
        } else {
            LOG_W(TAG, "Skipping logger task creation, module not initialized or SD card issue.");
        }
    }


    return APP_OK;
}

void app_logic_on_event(const es_event_t *ev, void *user) {
    if (!ev) return;

    app_logic_t *app = (app_logic_t *)user;
    app_state_t *state = app_state_get_instance();

#if defined(USE_WIFI)
    // Handle events from worker modules.
    switch (ev->id) {
        case APP_EVENT_WIFI_STA_CONNECTED:
            LOG_I(TAG, "Event received: WiFi connected.");
            LOG_I(TAG, "=== HEAP AFTER WIFI CONNECT ===");
            LOG_I(TAG, "Free heap: %zu bytes (%.1f KB)", esp_get_free_heap_size(), esp_get_free_heap_size() / 1024.0);
            LOG_I(TAG, "Minimum free heap: %zu bytes", esp_get_minimum_free_heap_size());
#if defined(USE_WEB_FILE_SEREVER)
            app_logic_send_command(app, APP_CMD_WEB_SERVER_START);
            // Reflect download mode while WiFi/web server active
            state = app_state_get_instance();
            app_state_begin_update();
            app_state_set_u8(APP_STATE_FIELD_CURRENT_MODE, (uint8_t *)&state->current_mode, APP_MODE_DOWNLOAD);
            app_state_end_update();
#endif
            break;
        case APP_EVENT_WIFI_AP_STARTED:
            LOG_I(TAG, "Event received: WiFi AP started.");
#if defined(USE_WEB_FILE_SEREVER)
            app_logic_send_command(app, APP_CMD_WEB_SERVER_START);
            // Reflect download mode while WiFi/web server active
            state = app_state_get_instance();
            app_state_begin_update();
            app_state_set_u8(APP_STATE_FIELD_CURRENT_MODE, (uint8_t *)&state->current_mode, APP_MODE_DOWNLOAD);
            app_state_end_update();
#endif
            break;
        case APP_EVENT_WIFI_STA_CONNECT_FAILED:
            LOG_E(TAG, "Event received: WiFi connection failed. Stopping online mode.");
            app_logic_send_command(app, APP_CMD_ONLINE_MODE_STOP);
            break;
        case APP_EVENT_WIFI_STA_DISCONNECTED:
            LOG_W(TAG, "Event received: WiFi STA disconnected.");
#if defined(USE_WEB_FILE_SEREVER)
            app_logic_send_command(app, APP_CMD_WEB_SERVER_STOP);
            // Return to IDLE mode when WiFi disconnects
            state = app_state_get_instance();
            app_state_begin_update();
            app_state_set_u8(APP_STATE_FIELD_CURRENT_MODE, (uint8_t *)&state->current_mode, APP_MODE_IDLE);
            app_state_end_update();
#endif
            break;
        case APP_EVENT_WIFI_DISCONNECTED:
            LOG_W(TAG, "Event received: WiFi fully disconnected.");
#if defined(USE_WEB_FILE_SEREVER)
            if (app->web_server && web_server_is_running(app->web_server)) {
                LOG_I(TAG, "Web server is still running, stopping it after WiFi disconnect.");
                app_logic_send_command(app, APP_CMD_WEB_SERVER_STOP);
            }
            // Return to IDLE mode when WiFi fully disconnects
            state = app_state_get_instance();
            app_state_begin_update();
            app_state_set_u8(APP_STATE_FIELD_CURRENT_MODE, (uint8_t *)&state->current_mode, APP_MODE_IDLE);
            app_state_end_update();
#endif
            break;
        case APP_EVENT_WIFI_AP_STOPPED:
            LOG_I(TAG, "Event received: WiFi AP stopped");
            break;
        default:
            // This is not a warning, as app_logic may not need to react to every event.
            // LOG_D(TAG, "Received unhandled event: %d", ev->id);
            break;
    }
#endif
}

// Forward declaration of the private command handler
static app_err_t _app_logic_handle_command(app_logic_t *app, app_command_t *cmd);

void app_logic_task(void *arg) {
    app_logic_t *app = (app_logic_t *)arg;
    app_command_t received_cmd;
    app_state_t *state = app_state_get_instance();
    uint32_t last_error_log_time = 0;
    const uint32_t ERROR_LOG_INTERVAL_MS = 2000;  // 2 seconds

    LOG_I(TAG, "App Logic Task started.");

    while (1) {
        // Update button state
        button_update(app->button);
        button_update(app->button_external);

        // Check for error state and log periodically
        uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
        if (state->current_mode == APP_MODE_ERROR || state->system_error_code != APP_OK) {
            if (current_time - last_error_log_time >= ERROR_LOG_INTERVAL_MS) {
                LOG_E(TAG, "App is in ERROR state - Error code: %ld, Mode: %d", state->system_error_code, state->current_mode);
                last_error_log_time = current_time;
            }
        }

        // Wait for a command to arrive in the queue with a short timeout
        if (xQueueReceive(g_command_queue, &received_cmd, pdMS_TO_TICKS(10))) {
            LOG_D(TAG, "Processing command ID: %d", received_cmd.id);
            // Delegate the actual work to a handler function.
            _app_logic_handle_command(app, &received_cmd);
        }
    }
}

// Private function containing the command execution logic (the switch statement).
static app_err_t _app_logic_handle_command(app_logic_t *app, app_command_t *cmd) {
    app_err_t result = APP_OK;
    app_state_t *state = app_state_get_instance();
    switch (cmd->id) {
        case APP_CMD_SET_MODE_IDLE:
            // 1) Switch app state to IDLE (logger stops accepting new snapshots)
            app_state_begin_update();
            app_state_set_u8(APP_STATE_FIELD_CURRENT_MODE, (uint8_t *)&state->current_mode, APP_MODE_IDLE);
            app_state_end_update();
            LOG_I(TAG, "Mode changed to IDLE - quiescing tasks");
            // 2) Flush logger buffers to SD and fsync
            if (app->logger_module && app->logger_module->initialized && app->logger_module->sd_card_ok) {
                logger_module_flush_and_sync(app->logger_module, pdMS_TO_TICKS(2000));
            }
            // 3) Suspend worker tasks
            // logger task - can suspend after queue drain
            if (app->logger_module) {
                if (app->logger_module->writer_task_handle) vTaskSuspend(app->logger_module->writer_task_handle);
            }
            break;
        case APP_CMD_SET_MODE_LOGGING: {
            // Do not allow starting logging if the app is already in ERROR state.
            // Otherwise UI may show "LOGGING" (green LED) while nothing is actually recorded.
            if (state->current_mode == APP_MODE_ERROR || state->system_error_code != APP_OK) {
                LOG_W(TAG, "Ignoring LOGGING start request: app is in ERROR state (error=%ld, mode=%d)",
                      state->system_error_code, state->current_mode);
                result = APP_ERR_INVALID_STATE;
                break;
            }

            app_err_t restore_err = app_logic_restore_logging_resources(app);
            if (restore_err != APP_OK) {
                LOG_E(TAG, "Failed to restore logging resources: %d", restore_err);
                result = restore_err;
                break;
            }

            if (app->logger_module) {
                logger_module_reset_stats(app->logger_module);
            }

            // Start a new logging session by restarting STM32 and re-reading the logger config.
            remote_accel_reader_restart_session();

#if defined(USE_WEB_FILE_SEREVER)
            web_server_set_download_chunk_size(web_server_get_default_download_chunk_size());
#endif


            if (app->logger_module) {
                if (app->logger_module->writer_task_handle && eTaskGetState(app->logger_module->writer_task_handle) == eSuspended) {
                    vTaskResume(app->logger_module->writer_task_handle);
                }
            }

            app_state_begin_update();
            app_state_set_u8(APP_STATE_FIELD_CURRENT_MODE, (uint8_t *)&state->current_mode, APP_MODE_LOGGING);
            app_state_end_update();
            LOG_I(TAG, "Mode changed to LOGGING - logging tasks active");
            break;
        }
        case APP_CMD_SET_MODE_ERROR:
            app_state_begin_update();
            app_state_set_u8(APP_STATE_FIELD_CURRENT_MODE, (uint8_t *)&state->current_mode, APP_MODE_ERROR);
            app_state_end_update();
            LOG_I(TAG, "Mode changed to ERROR");
            break;
#if defined(USE_WIFI)
        case APP_CMD_RECONFIGURE_WIFI: {
            LOG_I(TAG, "WiFi reconfiguration requested");
            if (!app->wifi) {
                LOG_W(TAG, "WiFi module not initialized");
                result = APP_ERR_INVALID_STATE;
                break;
            }

            esp_err_t start_err = wifi_start(app->wifi);
            if (start_err != ESP_OK) {
                LOG_E(TAG, "WiFi reconfigure failed: %s", esp_err_to_name(start_err));
                result = APP_ERR_GENERIC;
            }
            break;
        }
        case APP_CMD_ONLINE_MODE_START: {
            LOG_I(TAG, "Starting online mode sequence: Starting WiFi...");

            // Print heap memory info before WiFi start to detect memory leaks
            size_t free_heap = esp_get_free_heap_size();
            size_t min_heap = esp_get_minimum_free_heap_size();
            size_t total_heap = heap_caps_get_total_size(MALLOC_CAP_DEFAULT);
            LOG_I(TAG, "=== PRE-WIFI HEAP CHECK ===");
            LOG_I(TAG, "Free heap: %zu bytes (%.2f KB)", free_heap, free_heap / 1024.0f);
            LOG_I(TAG, "Min free heap: %zu bytes (%.2f KB)", min_heap, min_heap / 1024.0f);
            LOG_I(TAG, "Total heap: %zu bytes (%.2f KB)", total_heap, total_heap / 1024.0f);
            LOG_I(TAG, "Heap usage: %.1f%%", ((total_heap - free_heap) * 100.0f) / total_heap);
            LOG_I(TAG, "===========================");

            if (!app->wifi) {
                LOG_W(TAG, "WiFi module not initialized");
                result = APP_ERR_INVALID_STATE;
                break;
            }

            if (!app->wifi_initialized) {
                LOG_I(TAG, "Initializing WiFi module on demand...");
                esp_err_t init_err = wifi_init(app->wifi);
                if (init_err != ESP_OK) {
                    LOG_E(TAG, "Failed to initialize WiFi: %s", esp_err_to_name(init_err));
                    result = APP_ERR_GENERIC;
                    break;
                }
                app->wifi_initialized = true;
            }

            esp_err_t start_err = wifi_start(app->wifi);
            if (start_err != ESP_OK) {
                LOG_E(TAG, "Failed to start WiFi: %s", esp_err_to_name(start_err));
                result = APP_ERR_GENERIC;
            }
            break;
        }
        case APP_CMD_ONLINE_MODE_STOP:
            LOG_I(TAG, "Stopping online mode sequence...");
#if defined(USE_WEB_FILE_SEREVER)
            LOG_I(TAG, "Stopping Web Server...");
            if (app->web_server && app->web_server->initialized) {
                if (web_server_is_running(app->web_server)) {
                    if (web_server_stop(app->web_server) != APP_OK) {
                        LOG_E(TAG, "Failed to stop web server");
                        result = APP_ERR_GENERIC;
                    } else {
                        LOG_I(TAG, "Web server stopped successfully");
                    }
                } else {
                    LOG_I(TAG, "Web server already stopped");
                }

                web_server_destroy_task(app->web_server);
                app->web_server_task_handle = NULL;
                LOG_I(TAG, "Web server task destroyed");
            } else {
                LOG_W(TAG, "Web server module not initialized");
            }

            web_server_set_download_chunk_size(web_server_get_default_download_chunk_size());
#endif
            if (app->wifi) {
                LOG_I(TAG, "Stopping WiFi...");
                esp_err_t stop_err = wifi_stop(app->wifi);
                if (stop_err != ESP_OK) {
                    LOG_E(TAG, "Failed to stop WiFi: %s", esp_err_to_name(stop_err));
                    result = APP_ERR_GENERIC;
                } else {
                    // Print heap memory info after WiFi stop to verify cleanup
                    size_t free_heap = esp_get_free_heap_size();
                    size_t min_heap = esp_get_minimum_free_heap_size();
                    size_t total_heap = heap_caps_get_total_size(MALLOC_CAP_DEFAULT);
                    LOG_I(TAG, "=== POST-WIFI HEAP CHECK ===");
                    LOG_I(TAG, "Free heap: %zu bytes (%.2f KB)", free_heap, free_heap / 1024.0f);
                    LOG_I(TAG, "Min free heap: %zu bytes (%.2f KB)", min_heap, min_heap / 1024.0f);
                    LOG_I(TAG, "Total heap: %zu bytes (%.2f KB)", total_heap, total_heap / 1024.0f);
                    LOG_I(TAG, "Heap usage: %.1f%%", ((total_heap - free_heap) * 100.0f) / total_heap);
                    LOG_I(TAG, "============================");
                }
            } else {
                LOG_W(TAG, "WiFi module not initialized");
                result = APP_ERR_INVALID_STATE;
            }

            // Return to IDLE mode when online mode is stopped
            state = app_state_get_instance();
            app_state_begin_update();
            app_state_set_u8(APP_STATE_FIELD_CURRENT_MODE, (uint8_t *)&state->current_mode, APP_MODE_IDLE);
            app_state_end_update();

            // Reboot after exiting download mode to ensure clean state
            LOG_I(TAG, "Exiting download mode - rebooting system for clean state");
            vTaskDelay(pdMS_TO_TICKS(1000));  // Give time for logs to flush
            app_logic_reboot_with_leds_off(app);
            break;
#endif
#if defined(USE_WEB_FILE_SEREVER)
        case APP_CMD_WEB_SERVER_START:
            LOG_I(TAG, "Web server start requested");
            LOG_I(TAG, "=== HEAP BEFORE WEB SERVER INIT ===");
            LOG_I(TAG, "Free heap: %zu bytes (%.1f KB)", esp_get_free_heap_size(), esp_get_free_heap_size() / 1024.0);
            LOG_I(TAG, "Minimum free heap: %zu bytes", esp_get_minimum_free_heap_size());

            if (!app->web_server) {
                LOG_W(TAG, "Web server module not available");
                result = APP_ERR_INVALID_STATE;
                break;
            }

            if (!app->web_server_initialized) {
                LOG_I(TAG, "Initializing web server module now that WiFi is ready...");
                LOG_I(TAG, "Heap before web_server_init: %zu bytes", esp_get_free_heap_size());

                if (web_server_init(app->web_server) != APP_OK) {
                    LOG_E(TAG, "Failed to initialize web server module");
                    result = APP_ERR_GENERIC;
                    break;
                }
                app->web_server_initialized = true;

                LOG_I(TAG, "Heap after web_server_init: %zu bytes", esp_get_free_heap_size());
            }

            if (web_server_is_running(app->web_server)) {
                LOG_W(TAG, "Web server is already running");
                break;
            }

            // Check if WiFi is connected or AP is active first
            if (!state->wifi_sta_active && !state->wifi_ap_active) {
                LOG_W(TAG, "WiFi not connected. Web server requires WiFi connection.");
                LOG_I(TAG, "Please ensure WiFi is configured and connected before starting web server.");
                result = APP_ERR_INVALID_STATE;
                break;
            }

            // Create web server task first
            LOG_I(TAG, "Heap before web_server_create_task: %zu bytes", esp_get_free_heap_size());
            if (web_server_create_task(app->web_server) != APP_OK) {
                LOG_E(TAG, "Failed to create web server task");
                result = APP_ERR_GENERIC;
                break;
            }

            app->web_server_task_handle = app->web_server->task_handle;
            LOG_I(TAG, "Web server task created successfully");
            LOG_I(TAG, "Heap after web_server_create_task: %zu bytes", esp_get_free_heap_size());

            // Now start the web server
            LOG_I(TAG, "Heap before web_server_start: %zu bytes", esp_get_free_heap_size());
            if (web_server_start(app->web_server) != APP_OK) {
                LOG_E(TAG, "Failed to start web server");
                result = APP_ERR_GENERIC;
            } else {
                LOG_I(TAG, "Web server started successfully");
                LOG_I(TAG, "Heap after web_server_start: %zu bytes", esp_get_free_heap_size());

                // Set the HTTP server instance for captive portal
                if (app->wifi) {
                    wifi_set_captive_portal_http_server(app->wifi, app->web_server->server);
                }
            }
            break;
        case APP_CMD_WEB_SERVER_STOP:
            LOG_I(TAG, "Web server stop requested");
            if (app->web_server && app->web_server->initialized) {
                if (web_server_is_running(app->web_server)) {
                    if (web_server_stop(app->web_server) != APP_OK) {
                        LOG_E(TAG, "Failed to stop web server");
                        result = APP_ERR_GENERIC;
                    } else {
                        LOG_I(TAG, "Web server stopped successfully");
                    }
                } else {
                    LOG_I(TAG, "Web server already stopped");
                }

                web_server_destroy_task(app->web_server);
                app->web_server_task_handle = NULL;
                LOG_I(TAG, "Web server task destroyed");
            } else {
                LOG_W(TAG, "Web server module not initialized");
            }
            break;
#endif
        case APP_CMD_STOP_LOGGING_AND_FREE_MEMORY: {
            // TOTAL MEMORY TRACKING: Track entire cleanup process
            size_t heap_at_very_start = esp_get_free_heap_size();
            LOG_I(TAG, "======= TOTAL CLEANUP PROCESS START =======");
            LOG_I(TAG, "Starting logging shutdown and memory cleanup for WiFi mode");
            LOG_I(TAG, "Free heap at VERY START: %zu bytes (%.2f KB)", heap_at_very_start, heap_at_very_start / 1024.0f);
            LOG_I(TAG, "===========================================");

            bool logger_stopped = false;
            bool can_prepare_download = true;
            size_t heap_after_logger = heap_at_very_start;

            if (app->logger_module && app->logger_module->initialized && state->current_mode == APP_MODE_LOGGING) {
                hal_err_t logger_err = logger_module_stop_and_free_memory(app->logger_module);
                if (logger_err != HAL_ERR_NONE) {
                    LOG_E(TAG, "Failed to stop logger and free memory: %d", logger_err);
                    result = APP_ERR_GENERIC;
                    can_prepare_download = false;
                } else {
                    logger_stopped = true;
                    heap_after_logger = esp_get_free_heap_size();

                    // Print heap memory info after logger shutdown to verify memory cleanup
                    size_t min_heap = esp_get_minimum_free_heap_size();
                    size_t total_heap = heap_caps_get_total_size(MALLOC_CAP_DEFAULT);
                    LOG_I(TAG, "=== POST-LOGGER HEAP CHECK ===");
                    LOG_I(TAG, "Free heap: %zu bytes (%.2f KB)", heap_after_logger, heap_after_logger / 1024.0f);
                    LOG_I(TAG, "Min free heap: %zu bytes (%.2f KB)", min_heap, min_heap / 1024.0f);
                    LOG_I(TAG, "Total heap: %zu bytes (%.2f KB)", total_heap, total_heap / 1024.0f);
                    LOG_I(TAG, "Heap usage: %.1f%%", ((total_heap - heap_after_logger) * 100.0f) / total_heap);
                    LOG_I(TAG, "==============================");
                }
            } else {
                LOG_I(TAG, "Logger not logging; skipping logger shutdown");
            }

            if (can_prepare_download) {
                app_logic_prepare_download_mode(app);

#if defined(USE_WEB_FILE_SEREVER)
                // Use default (smaller) chunk size to avoid persistent EAGAIN stalls on constrained links
                web_server_set_download_chunk_size(web_server_get_default_download_chunk_size());
#endif

                state = app_state_get_instance();
                app_state_begin_update();
                app_state_set_u8(APP_STATE_FIELD_CURRENT_MODE, (uint8_t *)&state->current_mode, APP_MODE_IDLE);
                app_state_end_update();

                // TOTAL MEMORY TRACKING: Final summary
                size_t heap_at_very_end = esp_get_free_heap_size();
                size_t total_memory_recovered = heap_at_very_end - heap_at_very_start;
                size_t logger_memory_freed = heap_after_logger - heap_at_very_start;
                size_t task_memory_freed = heap_at_very_end - heap_after_logger;

                LOG_I(TAG, "======= TOTAL CLEANUP SUMMARY =======");
                LOG_I(TAG, "Free heap at VERY END: %zu bytes (%.2f KB)", heap_at_very_end, heap_at_very_end / 1024.0f);
                LOG_I(TAG, "Logger memory freed: %zu bytes (%.2f KB)", logger_memory_freed, logger_memory_freed / 1024.0f);
                LOG_I(TAG, "Task memory freed: %zu bytes (%.2f KB)", task_memory_freed, task_memory_freed / 1024.0f);
                LOG_I(TAG, "TOTAL MEMORY RECOVERED: %zu bytes (%.2f KB)", total_memory_recovered, total_memory_recovered / 1024.0f);
                LOG_I(TAG, "Expected total recovery: ~60KB (52KB logger + 7KB tasks)");

                if (total_memory_recovered >= 50000) {
                    LOG_I(TAG, "EXCELLENT: Major memory recovery (%zu KB) - system ready for WiFi/download", total_memory_recovered / 1024);
                } else if (total_memory_recovered >= 30000) {
                    LOG_I(TAG, "GOOD: Significant memory recovery (%zu KB) - should work for WiFi/download", total_memory_recovered / 1024);
                } else if (total_memory_recovered >= 10000) {
                    LOG_W(TAG, "PARTIAL: Some memory recovery (%zu KB) - might be tight for WiFi/download", total_memory_recovered / 1024);
                } else {
                    LOG_E(TAG, "FAILED: Minimal memory recovery (%zu KB) - likely insufficient for WiFi/download", total_memory_recovered / 1024);
                }
                LOG_I(TAG, "====================================");

                if (logger_stopped) {
                    LOG_I(TAG, "Logger stopped and system prepared for download mode");
                }
            }
            break;
        }
        case APP_CMD_REBOOT_TO_BOOT:
            LOG_I(TAG, "Rebooting to boot mode...");
            app_logic_reboot_to_boot_mode_with_leds_off(app);
            break;
        case APP_CMD_REBOOT:
            LOG_I(TAG, "Rebooting device...");
            vTaskDelay(pdMS_TO_TICKS(100));  // Give time for logs to flush
            app_logic_reboot_with_leds_off(app);
            break;
#if defined(USE_USB_FILE_SERVER)
        case APP_CMD_START_USB_FILE_SERVER:
            LOG_I(TAG, "USB file server start requested");
            result = usb_file_server_start();
            if (result != APP_OK) {
                LOG_E(TAG, "Failed to start USB file server: %d", result);
            } else {
                app_state_begin_update();
                app_state_set_u8(APP_STATE_FIELD_CURRENT_MODE, (uint8_t *)&state->current_mode, APP_MODE_DOWNLOAD);
                app_state_end_update();
            }
            break;
        case APP_CMD_STOP_USB_FILE_SERVER:
            LOG_I(TAG, "USB file server stop requested");
            result = usb_file_server_stop();
            if (result != APP_OK) {
                LOG_E(TAG, "Failed to stop USB file server: %d", result);
            } else {
                app_state_begin_update();
                app_state_set_u8(APP_STATE_FIELD_CURRENT_MODE, (uint8_t *)&state->current_mode, APP_MODE_IDLE);
                app_state_end_update();
                LOG_I(TAG, "Exiting download mode - rebooting system for clean state");
                app_logic_reboot_with_leds_off(app);
            }
            break;
#endif
        default:
            LOG_W(TAG, "Received unhandled application command: %d", cmd->id);
            result = APP_ERR_UNKNOWN_COMMAND;
            break;
    }

    return result;
}

// ============================================================================
// Task Management Functions
// ============================================================================

esp_err_t app_logic_start_all_tasks(app_logic_t *app) {
    LOG_I(TAG, "Starting all application tasks...");

    uint32_t cpu_cores = system_get_cpu_cores();
    esp_err_t result = ESP_OK;

    // Start app_logic_task (if not already running)
    if (app->app_logic_handle == NULL) {
        BaseType_t app_logic_ret = xTaskCreatePinnedToCore(app_logic_task, "APP_LOGIC", TASK_STACK_SIZE_APP_LOGIC / sizeof(StackType_t), app,
                                                           TASK_PRIORITY_APP_LOGIC, &app->app_logic_handle, CORE_APP_LOGIC);
        if (app_logic_ret != pdPASS) {
            LOG_E(TAG, "Failed to create app_logic_task");
            result = ESP_FAIL;
        } else {
            LOG_I(TAG, "app_logic_task created successfully");
        }
    } else {
        LOG_I(TAG, "app_logic_task already running");
    }



    // Web server will be started/stopped via button long press
    // No automatic startup - user controls it manually

    return result;
}

esp_err_t app_logic_start_minimal_tasks(app_logic_t *app) {
    LOG_I(TAG, "Starting minimal application tasks...");

    // Check if app_logic_task already exists to prevent duplicate creation
    if (app->app_logic_handle != NULL) {
        LOG_W(TAG, "app_logic_task already exists, skipping creation");
    } else {
        // Start only app_logic_task (essential for button handling and commands)
        BaseType_t app_logic_ret = xTaskCreatePinnedToCore(app_logic_task, "APP_LOGIC", TASK_STACK_SIZE_APP_LOGIC / sizeof(StackType_t), app,
                                                           TASK_PRIORITY_APP_LOGIC, &app->app_logic_handle, 0);
        if (app_logic_ret != pdPASS) {
            LOG_E(TAG, "Failed to create app_logic_task");
            return ESP_FAIL;
        } else {
            LOG_I(TAG, "app_logic_task created successfully (minimal mode)");
        }
    }

    // Give the system time to stabilize after task creation
    // This helps prevent heap corruption issues during error state initialization
    vTaskDelay(pdMS_TO_TICKS(100));


    return ESP_OK;
}

esp_err_t app_logic_send_command_full(app_logic_t *app, const app_command_t *cmd) {
    if (!app || !cmd) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!g_command_queue) {
        LOG_E(TAG, "Command queue not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (xQueueSend(g_command_queue, cmd, pdMS_TO_TICKS(10)) != pdPASS) {
        LOG_E(TAG, "Failed to send command %d to app logic queue", cmd->id);
        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
}

esp_err_t app_logic_send_command(app_logic_t *app, app_command_id_e cmd_id) {
    app_command_t cmd = {.id = cmd_id};
    return app_logic_send_command_full(app, &cmd);
}

app_logic_t *get_global_app_logic_instance(void) { return g_app_logic_instance; }
