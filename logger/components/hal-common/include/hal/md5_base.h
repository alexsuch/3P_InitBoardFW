#pragma once

#include <stddef.h>
#include <stdint.h>

#define HAL_MD5_OUTPUT_SIZE 16

typedef struct hal_md5_ctx_s hal_md5_ctx_t;

// Forward declaration - actual type defined in platform-specific headers
typedef int hal_err_t;

hal_err_t hal_md5_init(hal_md5_ctx_t *ctx);
hal_err_t hal_md5_update(hal_md5_ctx_t *ctx, const void *input, size_t size);
hal_err_t hal_md5_digest(hal_md5_ctx_t *ctx, uint8_t output[HAL_MD5_OUTPUT_SIZE]);
hal_err_t hal_md5_destroy(hal_md5_ctx_t *ctx);
