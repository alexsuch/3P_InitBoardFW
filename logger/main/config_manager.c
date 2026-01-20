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


// Public getters (declare in config_manager.h):
// bool config_manager_get_dac_cal_points(int channel /*0:DAC1, 1:DAC2*/,
//                                        uint16_t* v0_mv, uint16_t* vfs_mv);

bool config_manager_load(void) {
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
