#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "target.h"





// Main configuration structure
typedef struct {


    bool add_drop_counters;

} app_config_t;

extern app_config_t g_app_config;

bool config_manager_load(void);
