#pragma once

#include <hal/storage_base.h>
#include <nvs_flash.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct hal_storage_s {
    nvs_handle_t handle;
} hal_storage_t;

#ifdef __cplusplus
}
#endif