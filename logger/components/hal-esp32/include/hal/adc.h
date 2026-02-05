#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_oneshot.h>

// ADC configuration struct for new API
typedef struct adc_config_s {
    adc_unit_t unit;
    adc_channel_t channel;
    adc_atten_t atten;
    adc_bitwidth_t bitwidth;
    adc_oneshot_unit_handle_t oneshot_handle;
    adc_cali_handle_t cali_handle;
} adc_config_t;

#include <hal/adc_base.h>

#ifdef __cplusplus
}
#endif
