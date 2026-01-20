#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <hal/mutex_base.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct mutex_s {
    SemaphoreHandle_t sema;
} mutex_t;

#ifdef __cplusplus
}
#endif
