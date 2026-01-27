#include "system.h"

#include <driver/gpio.h>
#include <driver/rtc_io.h>
#include <esp_app_format.h>
#include <esp_chip_info.h>
#include <esp_efuse.h>
#include <esp_err.h>
#include <esp_event.h>
#include <esp_http_client.h>
#include <esp_https_ota.h>
#include <esp_idf_version.h>
#include <esp_log.h>
#include <esp_ota_ops.h>
#include <esp_partition.h>
#include <esp_rom_sys.h>
#include <esp_sleep.h>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/task.h>
#include <soc/rtc.h>
#include <soc/rtc_cntl_reg.h>
#include <soc/soc.h>
#include <stdio.h>
#include <string.h>
#include <target.h>

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
#include "esp_wifi_types.h"
#if defined(USE_WIFI)
#include "esp_wifi.h"
#endif
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "util/time-util.h"

static const char *TAG = "SYS";

static system_flag_e system_flags = 0;

system_flag_e system_get_flags(void) { return system_flags; }

system_flag_e system_add_flag(system_flag_e flag) {
    system_flag_e old_flags = system_flags;
    system_flags |= flag;
    return old_flags;
}

system_flag_e system_remove_flag(system_flag_e flag) {
    system_flag_e old_flags = system_flags;
    system_flags &= ~flag;
    return old_flags;
}

bool system_has_flag(system_flag_e flag) { return system_flags & flag; }

const char *system_get_esp32_model() {
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);

    switch (chip_info.model) {
        case CHIP_ESP32:
            return "ESP32";
        case CHIP_ESP32S3:
            return "ESP32-S3";
        case CHIP_ESP32C3:
            return "ESP32-C3";
        case CHIP_ESP32C2:
            return "ESP32-C2";
        case CHIP_ESP32C6:
            return "ESP32-C6";
        case CHIP_ESP32H2:
            return "ESP32-H2";
        default:
            return "Unknown ESP32";
    }
}

uint16_t system_get_chip_revision(void) {
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    return chip_info.revision;
}

uint32_t system_get_cpu_frequency_mhz(void) {
    rtc_cpu_freq_config_t conf;
    rtc_clk_cpu_freq_get_config(&conf);
    return conf.freq_mhz;
}

uint32_t system_get_cpu_cores(void) {
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    return chip_info.cores;
}

uint32_t system_get_flash_size_bytes(void) {
    uint32_t flash_size;
    esp_err_t ret = esp_flash_get_size(NULL, &flash_size);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get flash size: %s", esp_err_to_name(ret));
        return 0;
    }
    return flash_size;
}

#define BUTTON_SYSTEM_GPIO BUTTON_ENTER_GPIO

float system_temperature(void) {
    extern uint8_t temprature_sens_read();
    return (temprature_sens_read() - 32) / 1.8;
}

bool system_awake_from_deep_sleep(void) { return esp_sleep_get_wakeup_cause() != ESP_SLEEP_WAKEUP_UNDEFINED; }

void system_reboot(void) { esp_restart(); }

void system_clear_force_download_boot_bit(void) {
    /*
     * CRITICAL FIX: Clear the force download boot bit.
     * This bit is "sticky" and is not cleared by some types of reboots.
     * If it is not cleared, the device will get stuck in bootloader mode.
     * This action must be the first in app_main to guarantee breaking the cycle.
     *
     * Note: This project targets ESP32-S3 and uses RTC_CNTL_OPTION1_REG.
     */
    REG_CLR_BIT(RTC_CNTL_OPTION1_REG, RTC_CNTL_FORCE_DOWNLOAD_BOOT);
}

IRAM_ATTR void system_reboot_to_boot_mode(void) {
    // give the console time to flush the buffer so "BOOT\n" is guaranteed to go
    fflush(stdout);
    vTaskDelay(pdMS_TO_TICKS(100));

    // 1) Add the bit (don't overwrite the entire register!)
    REG_SET_BIT(RTC_CNTL_OPTION1_REG, RTC_CNTL_FORCE_DOWNLOAD_BOOT);

    // 2) Short pause to ensure the bit is guaranteed to set
    vTaskDelay(pdMS_TO_TICKS(100));

    // 3) Clean ROM reset -> ROM sees FORCE_DOWNLOAD_BOOT
    esp_rom_software_reset_system();

    while (true) {
    }  // in case the compiler tries to "go further"
}

void system_shutdown(void) {
    // Wait until the button is released if it's now low
    while (gpio_get_level(BUTTON_SYSTEM_GPIO) == 0) {
    };

    // Wake up if the button is pressed
    // TODO: This might increase sleep current consumption
    // by a lot, measure it.

    // ESP32-S3 sleep configuration
    ESP_ERROR_CHECK(esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON));
    ESP_ERROR_CHECK(gpio_pulldown_dis(BUTTON_SYSTEM_GPIO));
    ESP_ERROR_CHECK(gpio_pullup_en(BUTTON_SYSTEM_GPIO));
    ESP_ERROR_CHECK(esp_sleep_enable_ext1_wakeup(1ULL << BUTTON_SYSTEM_GPIO, ESP_EXT1_WAKEUP_ANY_LOW));

    // TODO: Wake up if the 5V power is connected.
    // If battery power doesn't enable the 5V line
    // we can use a voltage divider from there to
    // a GPIO that supports wakeup by going high.
    esp_deep_sleep_start();
}

void system_get_device_info(device_info_t *info) {
    memset(info, 0, sizeof(device_info_t));

    // --- Information about the chip ---
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    info->model = chip_info.model;
    info->revision = chip_info.revision;
    info->cores = chip_info.cores;
    info->features = chip_info.features;

    // --- Unique ID (MAC address) ---
    uint8_t mac[6];
#if defined(USE_WIFI)
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
#else
    esp_read_mac(mac, ESP_MAC_ETH);
#endif
    snprintf(info->unique_id, sizeof(info->unique_id), "%02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    // --- Information about memory ---
    esp_flash_get_size(NULL, &info->flash_size_bytes);
    info->total_heap_size = heap_caps_get_total_size(MALLOC_CAP_DEFAULT);
    info->free_heap_size = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);
    info->largest_free_block = heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT);
    info->min_free_heap_ever = heap_caps_get_minimum_free_size(MALLOC_CAP_DEFAULT);

    // --- Information about the system and software ---
    strncpy(info->idf_version, esp_get_idf_version(), sizeof(info->idf_version) - 1);
    const esp_app_desc_t *app_desc = esp_app_get_description();
    strncpy(info->app_version, app_desc->version, sizeof(info->app_version) - 1);
    strncpy(info->project_name, app_desc->project_name, sizeof(info->project_name) - 1);
    info->uptime_seconds = esp_timer_get_time() / 1000000;

    // --- State and security ---
    info->reset_reason = esp_reset_reason();
    info->wakeup_cause = esp_sleep_get_wakeup_cause();
    info->cpu_freq_mhz = system_get_cpu_frequency_mhz();
    info->is_secure_boot_enabled = esp_secure_boot_enabled();
    info->is_flash_encryption_enabled = esp_flash_encryption_enabled();

#if defined(USE_WIFI)
    // --- Information about Wi-Fi ---
    // Check if Wi-Fi is initialized.
    // If esp_wifi_get_mode returns an error, then Wi-Fi is disabled.
    wifi_mode_t current_mode;
    esp_err_t err = esp_wifi_get_mode(&current_mode);
    if (err != ESP_OK) {
        // Wi-Fi is not initialized, set default values
        info->wifi.mode = WIFI_MODE_NULL;
        info->wifi.tx_power_dbm = 0;
        info->wifi.sta_rssi = 0;
        // SSID field is already empty after memset
        return;  // Exit, since further reading of Wi-Fi data is not possible
    }
    info->wifi.mode = current_mode;

    // If Wi-Fi is initialized, get the rest of the parameters
    int8_t power;
    esp_wifi_get_max_tx_power(&power);
    info->wifi.tx_power_dbm = power / 4;

    if (info->wifi.mode == WIFI_MODE_STA || info->wifi.mode == WIFI_MODE_APSTA) {
        wifi_ap_record_t ap_info;
        if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
            strncpy(info->wifi.sta_connected_ssid, (char *)ap_info.ssid, 32);
            info->wifi.sta_rssi = ap_info.rssi;
        } else {
            // Wi-Fi in STA mode, but not connected
            info->wifi.sta_rssi = -127;
        }
    }
#else
    // Wi-Fi not available, set default values
    info->wifi.mode = WIFI_MODE_NULL;
    info->wifi.tx_power_dbm = 0;
    info->wifi.sta_rssi = 0;
#endif
}
