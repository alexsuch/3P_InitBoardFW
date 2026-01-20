#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>
#include <esp_adc/adc_oneshot.h>

// Include curve fitting for ESP32-C3, ESP32-S3, etc.
#if defined(CONFIG_IDF_TARGET_ESP32C3)
// #include <esp_adc/adc_cali_curve_fitting.h>
#endif

#include <hal/adc.h>

#include "hal/gpio.h"
#include "hal/log.h"

#define NO_OF_SAMPLES 10

static const char *TAG = "ESP_ADC";

hal_err_t hal_adc_init(adc_config_t *config) {
    esp_err_t err;
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = config->unit,
    };
    err = adc_oneshot_new_unit(&unit_cfg, &config->oneshot_handle);
    if (err != ESP_OK) {
        LOG_E(TAG, "Failed to create ADC unit: %d", err);
        return err;
    }

    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = config->bitwidth,
        .atten = config->atten,
    };
    err = adc_oneshot_config_channel(config->oneshot_handle, config->channel, &chan_cfg);
    if (err != ESP_OK) {
        LOG_E(TAG, "Failed to config ADC channel: %d", err);
        return err;
    }

    // Calibration (optional, but recommended)
#if defined(CONFIG_IDF_TARGET_ESP32)
    adc_cali_line_fitting_config_t cali_cfg = {
        .unit_id = config->unit,
        .atten = config->atten,
        .bitwidth = config->bitwidth,
        .default_vref = 1100,  // Default reference voltage in mV
    };
    err = adc_cali_create_scheme_line_fitting(&cali_cfg, &config->cali_handle);
    if (err != ESP_OK) {
        LOG_W(TAG, "Line fitting calibration creation failed: %d", err);
        config->cali_handle = NULL;
    }
#elif defined(CONFIG_IDF_TARGET_ESP32C3)
    // Curve fitting calibration is not available in this SDK, skip calibration
    LOG_W(TAG, "ADC calibration not supported for ESP32-C3 in this SDK version, skipping calibration.");
    config->cali_handle = NULL;
#else
    // Other targets: skip calibration
    config->cali_handle = NULL;
#endif
    return ESP_OK;
}

uint32_t get_adc_voltage(adc_config_t *config) {
    int raw = 0;
    int voltage = 0;
    int sum = 0;
    for (int i = 0; i < NO_OF_SAMPLES; i++) {
        if (adc_oneshot_read(config->oneshot_handle, config->channel, &raw) == ESP_OK) {
            sum += raw;
        }
    }
    raw = sum / NO_OF_SAMPLES;
    if (config->cali_handle) {
        if (adc_cali_raw_to_voltage(config->cali_handle, raw, &voltage) == ESP_OK) {
            return voltage;
        }
    }
    // If calibration not available, return raw value
    return raw;
}