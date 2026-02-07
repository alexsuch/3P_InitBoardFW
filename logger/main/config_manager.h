#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "target.h"






#include "modules/logger_frame_link.h"

// Main configuration structure
typedef struct {


    bool add_drop_counters;

} app_config_t;

extern app_config_t g_app_config;

bool config_manager_load(void);
bool config_manager_load_stm_config(logger_config_t *cfg);
bool config_manager_save_stm_config(const logger_config_t *cfg);
bool config_manager_clear_stm_config(void);
