#pragma once

#include <esp_err.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "esp_app_format.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_flash_encrypt.h"
#include "esp_heap_caps.h"
#include "esp_mac.h"
#include "esp_pm.h"
#include "esp_secure_boot.h"
#include "esp_sleep.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "target.h"  // Include target.h to get USE_WIFI definition
#if defined(USE_WIFI)
#include "esp_wifi.h"
#endif
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#if defined(USE_WIFI)
typedef struct {
    wifi_mode_t mode;
    int8_t tx_power_dbm;
    char sta_connected_ssid[33];
    int8_t sta_rssi;
} wifi_info_t;
#else
typedef struct {
    int mode;  // Placeholder, not used when WiFi is disabled
    int8_t tx_power_dbm;
    char sta_connected_ssid[33];
    int8_t sta_rssi;
} wifi_info_t;
#endif

typedef struct {
    esp_chip_model_t model;
    uint8_t revision;
    uint8_t cores;
    uint32_t features;
    char unique_id[18];

    uint32_t flash_size_bytes;
    size_t total_heap_size;
    size_t free_heap_size;
    size_t largest_free_block;
    size_t min_free_heap_ever;

    char idf_version[32];
    char app_version[32];
    char project_name[32];
    int64_t uptime_seconds;

    esp_reset_reason_t reset_reason;
    esp_sleep_wakeup_cause_t wakeup_cause;
    uint32_t cpu_freq_mhz;
    bool is_secure_boot_enabled;
    bool is_flash_encryption_enabled;

    wifi_info_t wifi;

} device_info_t;

typedef enum {
    SYSTEM_FLAG_SCREEN = 1 << 0,   // Screen support is available and screen is detected
    SYSTEM_FLAG_BUTTON = 1 << 1,   // Button is available,
    SYSTEM_FLAG_BATTERY = 1 << 2,  // Battery circuitry is available
} system_flag_e;

system_flag_e system_get_flags(void);
system_flag_e system_add_flag(system_flag_e flag);
system_flag_e system_remove_flag(system_flag_e flag);
bool system_has_flag(system_flag_e flag);
float system_temperature(void);

bool system_awake_from_deep_sleep(void);
void system_reboot(void);
void system_reboot_to_boot_mode(void);
void system_clear_force_download_boot_bit(void);
void system_shutdown(void);
const char *system_get_esp32_model(void);
uint16_t system_get_chip_revision(void);
uint32_t system_get_cpu_frequency_mhz(void);
uint32_t system_get_cpu_cores(void);
uint32_t system_get_flash_size_bytes(void);
void system_get_device_info(device_info_t *info);