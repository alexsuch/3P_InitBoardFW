#include <driver/gpio.h>
#include <esp_task_wdt.h>
#include <log.h>
#include <target.h>
#include <version.h>

#include "app_logic.h"
#include "app_state.h"
#include "build_info.h"
#include "esp_attr.h"
#include "esp_heap_caps.h"
#include "esp_log_level.h"
#include "esp_rom_sys.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/gpio.h"
#include "hal/init.h"
#include "io/io_manager.h"
#include "platform/system.h"
#include "esp_core_dump.h"
#include "util/bootlog.h"

// Static assert to ensure stack size is properly aligned for FreeRTOS
_Static_assert((TASK_STACK_SIZE_USB_BOOT_CMD % sizeof(StackType_t)) == 0, "TASK_STACK_SIZE_USB_BOOT_CMD must be expressed in bytes and aligned to StackType_t");
#include "soc/soc.h"

#if defined(CONFIG_IDF_TARGET_ESP32S3)
// Boot mode handling is ESP32-S3 specific in this project.
static void usb_boot_cmd_task(void *arg) {
    // vTaskDelay(pdMS_TO_TICKS(3000));

    const char *MAGIC = "BOOT\n";  // what PlatformIO will send
    char line[16];
    int idx = 0;

    while (1) {
        int c = fgetc(stdin);  // console is already on USB-CDC (CONFIG_ESP_CONSOLE_USB_CDC=y)
        if (c < 0) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        if (c == '\r') continue;
        if (idx < (int)sizeof(line) - 1) line[idx++] = (char)c;
        if (c == '\n') {
            line[idx] = 0;
            idx = 0;
            if (strcmp(line, MAGIC) == 0) {
                fflush(stdout);
                vTaskDelay(pdMS_TO_TICKS(50));
                system_reboot_to_boot_mode();  // transition to ROM USB bootloader
            }
        }
    }
}

void usb_boot_cmd_start(void) {
    xTaskCreatePinnedToCore(usb_boot_cmd_task, "usb_boot_cmd", TASK_STACK_SIZE_USB_BOOT_CMD / sizeof(StackType_t), NULL, TASK_PRIORITY_USB_BOOT_CMD, NULL,
                            tskNO_AFFINITY);
}
#endif  // CONFIG_IDF_TARGET_ESP32S3

static const char *TAG = "Main";

void app_main() {
    // CRITICAL FIX: Clear the force download boot bit.
    // This must be the first action in app_main to prevent getting stuck in bootloader mode.
    system_clear_force_download_boot_bit();

    static bootlog_snapshot_t prev_bootlog;
    bootlog_capture_snapshot(&prev_bootlog);
    bootlog_init_for_new_boot();

    // Give user time to attach monitor before printing key debug info.
    vTaskDelay(pdMS_TO_TICKS(3000));

    LOG_I(TAG, "Reset reason: %d", esp_reset_reason());
    bootlog_nvs_dump_last();
    bootlog_dump_snapshot(&prev_bootlog);

    // Print heap memory information
    size_t free_heap = esp_get_free_heap_size();
    size_t min_heap = esp_get_minimum_free_heap_size();
    size_t total_heap = heap_caps_get_total_size(MALLOC_CAP_DEFAULT);

    LOG_I(TAG, "=== HEAP MEMORY INFO ===");
    LOG_I(TAG, "Free heap: %zu bytes (%.2f KB)", free_heap, free_heap / 1024.0f);
    LOG_I(TAG, "Min free heap: %zu bytes (%.2f KB)", min_heap, min_heap / 1024.0f);
    LOG_I(TAG, "Total heap: %zu bytes (%.2f KB)", total_heap, total_heap / 1024.0f);
    LOG_I(TAG, "Heap usage: %.1f%%", ((total_heap - free_heap) * 100.0f) / total_heap);
    LOG_I(TAG, "========================");

    // usb_cdc_hooks_init();
#if defined(CONFIG_IDF_TARGET_ESP32S3)
    usb_boot_cmd_start();
#endif  // CONFIG_IDF_TARGET_ESP32S3

    // vTaskDelay(2000 / portTICK_PERIOD_MS);
    // hal_platform_init();

    ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_IRAM));

    // esp_log_set_level_master(ESP_LOG_VERBOSE);
    // esp_log_level_set("*", ESP_LOG_DEBUG);

#if defined(CONFIG_IDF_TARGET_ESP32C3)
    // esp_log_set_level_master is not available on some targets; use per-tag levels.
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("LOGR", ESP_LOG_DEBUG);
    esp_log_level_set("task_wdt", ESP_LOG_NONE);
#else
    esp_log_set_level_master(ESP_LOG_INFO);
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("LOGR", ESP_LOG_DEBUG);
    // Silence noisy Task Watchdog logs during high-frequency logging tests.
    esp_log_level_set("task_wdt", ESP_LOG_NONE);
#endif

    // Chip info
    LOG_I(TAG, "3P Logger %s", FIRMWARE_VERSION);
    LOG_I(TAG, "Build timestamp: %s", BUILD_TIMESTAMP);
    LOG_I(TAG, "Git hash: %s", GIT_HASH);
    LOG_I(TAG, "ESP32 Model: %s", system_get_esp32_model());
    LOG_I(TAG, "ESP32 Chip Revision: %d", system_get_chip_revision());
    LOG_I(TAG, "ESP32 CPU Frequency: %lu MHz", system_get_cpu_frequency_mhz());
    LOG_I(TAG, "ESP32 CPU Cores: %lu", system_get_cpu_cores());
    LOG_I(TAG, "ESP32 Flash Size: %lu", system_get_flash_size_bytes());

    // Check for saved core dump
    esp_core_dump_summary_t summary;
    if (esp_core_dump_get_summary(&summary) == ESP_OK) {
        LOG_I(TAG, "Creates Core Dump found!");
        LOG_I(TAG, "Core Dump Summary:");
        LOG_I(TAG, "  PC: 0x%lx", summary.exc_pc);
        LOG_I(TAG, "  TCB: 0x%lx", summary.exc_tcb);
    } else {
        LOG_I(TAG, "No Core Dump found.");
    }

    app_state_init();

    // Disable Task Watchdog Timer during high-frequency logging performance tests.
    // This avoids spurious resets when stressing SPI, SD card, and logging pipelines.
    (void)esp_task_wdt_deinit();

    // Memory manager removed - using standard malloc/free

    app_logic_t *app_logic = calloc(1, sizeof(app_logic_t));
    if (app_logic == NULL) {
        LOG_E(TAG, "Failed to allocate memory for app_logic. Halting.");
        while (1) {
            vTaskDelay(portMAX_DELAY);
        }
    }

    app_logic_init(app_logic);

    if (app_logic_init_modules(app_logic) == APP_OK) {
        // Start only essential tasks at boot; start heavy tasks on-demand (button events)
        app_logic_start_minimal_tasks(app_logic);

        // DISABLED: Auto-start logging removed - use web UI start button instead.
        // LOG_I(TAG, "Auto-starting logging mode (temporary)...");
        // if (app_logic_start_all_tasks(app_logic) == ESP_OK) {
        //     app_logic_send_command(app_logic, APP_CMD_SET_MODE_LOGGING);
        // } else {
        //     LOG_E(TAG, "Failed to start logging tasks for auto-start");
        // }
    } else {
        LOG_E(TAG, "Module initialization failed. Check logs for details. Starting minimal tasks only.");
        app_state_t *state = app_state_get_instance();
        app_state_begin_update();
        app_state_set_u8(APP_STATE_FIELD_CURRENT_MODE, (uint8_t *)&state->current_mode, APP_MODE_ERROR);
        app_state_end_update();
        app_logic_start_minimal_tasks(app_logic);
    }
}
