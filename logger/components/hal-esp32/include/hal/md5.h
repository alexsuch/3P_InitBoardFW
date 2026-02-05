#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <hal/err.h>
#include <hal/md5_base.h>
#include <mbedtls/md5.h>

typedef struct hal_md5_ctx_s {
    mbedtls_md5_context ctx;
} hal_md5_ctx_t;

#ifdef __cplusplus
}
#endif