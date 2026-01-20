#pragma once

#include <esp_err.h>

typedef esp_err_t hal_err_t;

#define HAL_ERR_NONE ESP_OK
#define HAL_ERR_FAILED ESP_FAIL
#define HAL_ERR_INVALID_ARG ESP_ERR_INVALID_ARG
#define HAL_ERR_ASSERT_OK(e) ESP_ERROR_CHECK(e)
#define HAL_ERR_MEMORY ESP_ERR_NO_MEM
#define HAL_ERR_INVALID_STATE ESP_ERR_INVALID_STATE

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif