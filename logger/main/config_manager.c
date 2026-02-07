#include "config_manager.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#include "log.h"
#include "target.h"

static const char *TAG = "CONFIG";

app_config_t g_app_config;

static void set_default_config(void);
static void parse_line(char *line);

static bool str_to_bool(const char *str);
static void trim(char *str);


#include "nvs_flash.h"
#include "nvs.h"

// Public getters (declare in config_manager.h):
// bool config_manager_get_dac_cal_points(int channel /*0:DAC1, 1:DAC2*/,
//                                        uint16_t* v0_mv, uint16_t* vfs_mv);

bool config_manager_load(void) {
    // Initialize NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    set_default_config();

    LOG_I(TAG, "Loading configuration from %s", SD_MOUNT_PATH "/configuration.ini");
    FILE *f = fopen(SD_MOUNT_PATH "/configuration.ini", "r");
    if (!f) {
        LOG_W(TAG, "configuration.ini not found, using defaults");
        return false;
    }

    char line[128];
    while (fgets(line, sizeof(line), f)) {
        parse_line(line);
    }
    fclose(f);
    return true;
}

bool config_manager_load_stm_config(logger_config_t *cfg) {
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READONLY, &my_handle);
    if (err != ESP_OK) return false;

    size_t required_size = sizeof(logger_config_t);
    err = nvs_get_blob(my_handle, "logger_cfg", cfg, &required_size);
    nvs_close(my_handle);

    if (!(err == ESP_OK && required_size == sizeof(logger_config_t))) {
        return false;
    }

    // Basic sanity check: reject corrupted/legacy blobs that do not carry config magic.
    return (cfg->magic == LOGGER_CONFIG_MAGIC);
}

bool config_manager_save_stm_config(const logger_config_t *cfg) {
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return false;

    err = nvs_set_blob(my_handle, "logger_cfg", cfg, sizeof(logger_config_t));
    if (err == ESP_OK) {
        err = nvs_commit(my_handle);
    }
    nvs_close(my_handle);
    return (err == ESP_OK);
}

bool config_manager_clear_stm_config(void) {
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return false;

    err = nvs_erase_key(my_handle, "logger_cfg");
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        err = ESP_OK;
    }
    if (err == ESP_OK) {
        err = nvs_commit(my_handle);
    }
    nvs_close(my_handle);
    return (err == ESP_OK);
}


static void set_default_config(void) {

    g_app_config.add_drop_counters = true;  // Enabled by default
}

static void parse_line(char *line) {
    char *key = strtok(line, "=");
    char *value = strtok(NULL, "\r\n");
    if (!key || !value) return;

    trim(key);
    trim(value);

    if (key[0] == '#') return;

    if (strcmp(key, "ADD_DROP_COUNTERS") == 0) {
        g_app_config.add_drop_counters = str_to_bool(value);
    }
}



static bool str_to_bool(const char *str) {
    if (!str) return false;
    return strcasecmp(str, "true") == 0 || strcmp(str, "1") == 0;
}

static void trim(char *str) {
    if (!str) return;
    char *start = str;
    while (*start == ' ' || *start == '\t') start++;
    if (start != str) memmove(str, start, strlen(start) + 1);
    char *end = str + strlen(str) - 1;
    while (end >= str && (*end == ' ' || *end == '\t')) {
        *end = '\0';
        end--;
    }
}
