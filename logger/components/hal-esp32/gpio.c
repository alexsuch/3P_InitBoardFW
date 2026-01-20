#include <driver/gpio.h>
#include <driver/rtc_io.h>
#include <esp_err.h>
#include <hal/gpio.h>
#include <hal/mutex.h>
#include <stdio.h>

// DAC support is only available on ESP32 (not ESP32-C3)
#if defined(CONFIG_IDF_TARGET_ESP32)
#include <driver/dac_oneshot.h>

// Global DAC channel handles for GPIO25 and GPIO26 (ESP32 only)
static dac_oneshot_handle_t dac_channel_1_handle = NULL;
static dac_oneshot_handle_t dac_channel_2_handle = NULL;
// Mutex to protect concurrent DAC initialization
static mutex_t dac_init_mutex;
#endif

// Global array to store the last configured interrupt type for each GPIO
static gpio_int_type_t gpio_intr_types[GPIO_NUM_MAX] = {GPIO_INTR_DISABLE};
// Mutex to protect concurrent access to gpio_intr_types array
static mutex_t gpio_intr_types_mutex;

static hal_err_t hal_gpio_enable(hal_gpio_t gpio) {
    hal_err_t err;
#if defined(CONFIG_IDF_TARGET_ESP32)
    // Disable DAC output on GPIO25 and GPIO26. It's enabled by default and
    // can alter the output levels otherwise.
    // "DAC channel 1 is attached to GPIO25, DAC channel 2 is attached to GPIO26"
    if (gpio == 25 || gpio == 26) {
        // Protect DAC initialization from race conditions
        if (!dac_init_mutex.sema) mutex_open(&dac_init_mutex);
        mutex_lock(&dac_init_mutex);

        switch (gpio) {
            case 25:
                // For ESP32, GPIO25 is DAC channel 1
                if (dac_channel_1_handle == NULL) {
                    // Create DAC channel handle if not already created
                    dac_oneshot_config_t config = {
                        .chan_id = DAC_CHAN_0,  // DAC_CHAN_0 corresponds to GPIO25 on ESP32
                    };
                    esp_err_t ret = dac_oneshot_new_channel(&config, &dac_channel_1_handle);
                    if (ret != ESP_OK) {
                        // If DAC driver is not available, just continue without error
                        // This can happen if the DAC driver is not enabled in the build
                        mutex_unlock(&dac_init_mutex);
                        break;
                    }
                }
                // Set output to 0V to effectively disable DAC output
                if (dac_channel_1_handle != NULL) {
                    if (dac_oneshot_output_voltage(dac_channel_1_handle, 0) != ESP_OK) {
                        mutex_unlock(&dac_init_mutex);
                        return HAL_ERR_FAILED;
                    }
                }
                break;
            case 26:
                // For ESP32, GPIO26 is DAC channel 2
                if (dac_channel_2_handle == NULL) {
                    // Create DAC channel handle if not already created
                    dac_oneshot_config_t config = {
                        .chan_id = DAC_CHAN_1,  // DAC_CHAN_1 corresponds to GPIO26 on ESP32
                    };
                    esp_err_t ret = dac_oneshot_new_channel(&config, &dac_channel_2_handle);
                    if (ret != ESP_OK) {
                        // If DAC driver is not available, just continue without error
                        // This can happen if the DAC driver is not enabled in the build
                        mutex_unlock(&dac_init_mutex);
                        break;
                    }
                }
                // Set output to 0V to effectively disable DAC output
                if (dac_channel_2_handle != NULL) {
                    if (dac_oneshot_output_voltage(dac_channel_2_handle, 0) != ESP_OK) {
                        mutex_unlock(&dac_init_mutex);
                        return HAL_ERR_FAILED;
                    }
                }
                break;
        }

        mutex_unlock(&dac_init_mutex);
    }
#endif

    // See https://github.com/espressif/esp-idf/blob/master/docs/api-reference/system/sleep_modes.rst
    // Pins used for wakeup need to be manually unmapped from RTC
    if (rtc_gpio_is_valid_gpio(gpio)) {
#if defined(CONFIG_IDF_TARGET_ESP32)
        if ((err = rtc_gpio_deinit(gpio)) != HAL_ERR_NONE) {
            return err;
        }
#endif
    }
    // PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[gpio], PIN_FUNC_GPIO);
    return HAL_ERR_NONE;
}

static hal_err_t hal_gpio_set_dir(hal_gpio_t gpio, hal_gpio_dir_t dir) {
    gpio_mode_t mode = 0;
    switch (dir) {
        case HAL_GPIO_DIR_INPUT:
            mode = GPIO_MODE_INPUT;
            break;
        case HAL_GPIO_DIR_OUTPUT:
            mode = GPIO_MODE_OUTPUT;
            break;
        case HAL_GPIO_DIR_OUTPUT_OD:
            mode = GPIO_MODE_OUTPUT_OD;
            break;
        case HAL_GPIO_DIR_BIDIR:
            mode = GPIO_MODE_INPUT_OUTPUT;
            break;
    }
    return gpio_set_direction(gpio, mode);
}

static hal_err_t hal_gpio_set_pull(hal_gpio_t gpio, hal_gpio_pull_t pull) {
    gpio_pull_mode_t pull_mode = 0;
    switch (pull) {
        case HAL_GPIO_PULL_NONE:
            pull_mode = GPIO_FLOATING;
            break;
        case HAL_GPIO_PULL_UP:
            pull_mode = GPIO_PULLUP_ONLY;
            break;
        case HAL_GPIO_PULL_DOWN:
            pull_mode = GPIO_PULLDOWN_ONLY;
            break;
        case HAL_GPIO_PULL_BOTH:
            pull_mode = GPIO_PULLUP_PULLDOWN;
            break;
    }
    return gpio_set_pull_mode(gpio, pull_mode);
}

hal_err_t hal_gpio_setup(hal_gpio_t gpio, hal_gpio_dir_t dir, hal_gpio_pull_t pull) {
    hal_err_t err;

    if ((err = hal_gpio_enable(gpio)) != HAL_ERR_NONE) {
        return err;
    }

    if ((err = hal_gpio_set_dir(gpio, dir)) != HAL_ERR_NONE) {
        return err;
    }
    if ((err = hal_gpio_set_pull(gpio, pull)) != HAL_ERR_NONE) {
        return err;
    }
    return HAL_ERR_NONE;
}

hal_err_t hal_gpio_set_level(hal_gpio_t gpio, uint32_t level) { return gpio_set_level(gpio, level); }

int hal_gpio_get_level(hal_gpio_t gpio) { return gpio_get_level(gpio); }

hal_err_t hal_gpio_set_isr(hal_gpio_t gpio, hal_gpio_intr_t intr, hal_gpio_isr_t isr, const void *data) {
    hal_err_t err;

    // Validate GPIO number to prevent buffer overflow
    if (gpio >= GPIO_NUM_MAX) {
        return HAL_ERR_INVALID_ARG;
    }

    if (isr) {
        gpio_int_type_t intr_type = 0;
        switch (intr) {
            case HAL_GPIO_INTR_POSEDGE:
                intr_type = GPIO_INTR_POSEDGE;
                break;
            case HAL_GPIO_INTR_NEGEDGE:
                intr_type = GPIO_INTR_NEGEDGE;
                break;
            case HAL_GPIO_INTR_ANYEDGE:
                intr_type = GPIO_INTR_ANYEDGE;
                break;
            case HAL_GPIO_INTR_LOW_LEVEL:
                intr_type = GPIO_INTR_LOW_LEVEL;
                break;
            case HAL_GPIO_INTR_HIGH_LEVEL:
                intr_type = GPIO_INTR_HIGH_LEVEL;
                break;
        }
        // Store the interrupt type for later enable/disable operations
        if (!gpio_intr_types_mutex.sema) mutex_open(&gpio_intr_types_mutex);
        mutex_lock(&gpio_intr_types_mutex);
        gpio_intr_types[gpio] = intr_type;
        mutex_unlock(&gpio_intr_types_mutex);
        if ((err = gpio_set_intr_type(gpio, intr_type)) != HAL_ERR_NONE) {
            return err;
        }
        if ((err = gpio_isr_handler_add(gpio, (gpio_isr_t)isr, (void *)data)) != HAL_ERR_NONE) {
            return err;
        }
    } else {
        if (!gpio_intr_types_mutex.sema) mutex_open(&gpio_intr_types_mutex);
        mutex_lock(&gpio_intr_types_mutex);
        gpio_intr_types[gpio] = GPIO_INTR_DISABLE;
        mutex_unlock(&gpio_intr_types_mutex);
        if ((err = gpio_set_intr_type(gpio, GPIO_INTR_DISABLE)) != HAL_ERR_NONE) {
            return err;
        }
        if ((err = gpio_isr_handler_remove(gpio)) != HAL_ERR_NONE) {
            return err;
        }
    }

    return HAL_ERR_NONE;
}

hal_err_t hal_gpio_intr_enable(hal_gpio_t gpio) {
    // Validate GPIO number to prevent buffer overflow
    if (gpio >= GPIO_NUM_MAX) {
        return HAL_ERR_INVALID_ARG;
    }

    // Re-enable interrupts by setting the interrupt type back to the last configured type
    gpio_int_type_t intr_type;
    if (!gpio_intr_types_mutex.sema) mutex_open(&gpio_intr_types_mutex);
    mutex_lock(&gpio_intr_types_mutex);
    intr_type = gpio_intr_types[gpio];
    mutex_unlock(&gpio_intr_types_mutex);
    return gpio_set_intr_type(gpio, intr_type);
}

hal_err_t hal_gpio_intr_disable(hal_gpio_t gpio) {
    if (gpio >= GPIO_NUM_MAX) {
        return HAL_ERR_INVALID_ARG;
    }
    // Disable interrupts by setting the interrupt type to disable
    return gpio_set_intr_type(gpio, GPIO_INTR_DISABLE);
}

char *hal_gpio_toa(hal_gpio_t gpio, char *dst, size_t size) {
    snprintf(dst, size, "%02u", (unsigned)gpio);
    return dst;
}

// Initialize GPIO subsystem
void hal_gpio_init(void) {
    // Install ISR service once (no flags)
    gpio_install_isr_service(0);
    mutex_open(&gpio_intr_types_mutex);
#if defined(CONFIG_IDF_TARGET_ESP32)
    mutex_open(&dac_init_mutex);
#endif
}

// Cleanup function to delete DAC channel handles and mutex
void hal_gpio_cleanup(void) {
#if defined(CONFIG_IDF_TARGET_ESP32)
    // Clean up DAC handles for ESP32
    if (dac_channel_1_handle != NULL) {
        dac_oneshot_del_channel(dac_channel_1_handle);
        dac_channel_1_handle = NULL;
    }
    if (dac_channel_2_handle != NULL) {
        dac_oneshot_del_channel(dac_channel_2_handle);
        dac_channel_2_handle = NULL;
    }
    // Clean up DAC mutex
    mutex_close(&dac_init_mutex);
#endif
    // Clean up GPIO interrupt types mutex
    mutex_close(&gpio_intr_types_mutex);
    // Uninstall ISR service
    gpio_uninstall_isr_service();
}