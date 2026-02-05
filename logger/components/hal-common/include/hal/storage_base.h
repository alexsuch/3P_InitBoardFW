#pragma once

#include <stdbool.h>
#include <stddef.h>

typedef int hal_err_t;

#define HAL_ERR_NONE 0
#define HAL_ERR_FAILED -1
#define HAL_ERR_INVALID_ARG -2

typedef struct hal_storage_s hal_storage_t;

hal_err_t hal_storage_init(hal_storage_t *s, const char *name);
void hal_storage_deinit(hal_storage_t *s);
bool hal_storage_get_blob(hal_storage_t *s, const char *key, void *buf, size_t *size);
hal_err_t hal_storage_get_blob2(hal_storage_t *s, const char *key, void *buf, size_t *size, bool *found);
hal_err_t hal_storage_set_blob(hal_storage_t *s, const char *key, const void *buf, size_t size);
hal_err_t hal_storage_commit(hal_storage_t *s);