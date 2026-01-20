#include <assert.h>
#include <hal/md5.h>

hal_err_t hal_md5_init(hal_md5_ctx_t *ctx) {
    if (ctx == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    mbedtls_md5_init(&ctx->ctx);
    int ret = mbedtls_md5_starts(&ctx->ctx);
    return (ret == 0) ? ESP_OK : ESP_FAIL;
}

hal_err_t hal_md5_update(hal_md5_ctx_t *ctx, const void *input, size_t size) {
    if (ctx == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    mbedtls_md5_update(&ctx->ctx, (unsigned char *)input, size);
    return ESP_OK;
}

hal_err_t hal_md5_digest(hal_md5_ctx_t *ctx, uint8_t output[HAL_MD5_OUTPUT_SIZE]) {
    if (ctx == NULL || output == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    mbedtls_md5_finish(&ctx->ctx, output);
    return ESP_OK;
}

hal_err_t hal_md5_destroy(hal_md5_ctx_t *ctx) {
    if (ctx == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    mbedtls_md5_free(&ctx->ctx);
    return ESP_OK;
}