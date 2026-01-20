#include "remote_accel_reader.h"

#include <driver/spi_master.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <stdbool.h>


#include "app_state.h"
#include "hal/gpio.h"
#include "hal/spi.h"
#include "log.h"
#include "logger_frame_link.h"  // Phase 5: New 640-byte LogFrame_t format
#include "logger_module.h"
#include "target.h"

static const char* TAG = "RACCEL";

#if defined(CONFIG_IDF_TARGET_ESP32S3)

static hal_spi_device_handle_t s_link_device;
static TaskHandle_t s_reader_task_handle = NULL;
static bool s_reader_ready = false;
static uint64_t s_last_frame_us = 0;
static uint64_t s_last_diag_us = 0;
static uint64_t s_last_stats_us = 0;
static uint32_t s_valid_frames = 0;
static uint32_t s_invalid_frames = 0;
static uint32_t s_drain_batches = 0;
static const uint64_t DIAG_IDLE_INTERVAL_US = 2000000ULL;  // 2 seconds

static void remote_accel_reader_cleanup_spi(void) {
    if (s_link_device.lock) {
        vSemaphoreDelete(s_link_device.lock);
        s_link_device.lock = NULL;
    }
    if (s_link_device.dev) {
        spi_bus_remove_device(s_link_device.dev);
        s_link_device.dev = NULL;
    }
}

static void IRAM_ATTR remote_link_isr(void* arg) {
    (void)arg;
    BaseType_t higher_priority_task_woken = pdFALSE;
    if (s_reader_task_handle) {
        vTaskNotifyGiveFromISR(s_reader_task_handle, &higher_priority_task_woken);
        if (higher_priority_task_woken) {
            portYIELD_FROM_ISR();
        }
    }
}

/**
 * @brief Receive and process LogFrame_t structures from STM32 SPI slave
 *
 * Phase 5: Dual-command SPI protocol
 *   - Command 42 (0x2A): Retrieve device configuration (logger_config_t, 64 bytes)
 *     Sent once at startup to get ADC/IMU device parameters
 *   - Command 45 (0x2D): Retrieve logging frames (LogFrame_t, 640 bytes)
 *     Sent repeatedly every 5ms to stream data
 *
 * Frame rate: ~390 Hz (one per 2.56 ms when available)
 * ADC data: 256 samples @ 10 kHz
 * IMU data: 0-10 samples @ 52 Hz (distributed across ADC window)
 *
 * Commands are sent regardless of GPIO state; frames are processed only when GPIO is HIGH.
 */
static void remote_accel_reader_drain(void) {
    if (!s_reader_ready) {
        return;
    }

    ++s_drain_batches;
    static bool s_configured = false;
    uint8_t tx_cmd[LOGGER_FRAME_SIZE_BYTES] = {0};
    logger_frame_t frame;

    // ===== SEND CONFIG COMMAND (ONCE AT STARTUP) =====
    // Repeatedly send config command until we successfully read the config
    while (!s_configured) {
        // Step 1: Send 5-byte config command (CMD 42)
        uint8_t config_tx[5] = {42, 0, 0, 0, 0};  // Command 42 + 4 padding bytes
        hal_err_t err = hal_spi_device_transmit(&s_link_device, 0, 0, config_tx, 5, NULL, 0);

        if (err != HAL_ERR_NONE) {
            LOG_E(TAG, "Config command send failed: %d", (int)err);
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        // Step 2: Check if GPIO went high (config ready)
        if (hal_gpio_get_level(LINK_INT_GPIO) == HAL_GPIO_HIGH) {
            // Step 3: Wait 5ms before reading config
            vTaskDelay(pdMS_TO_TICKS(10));

            // Step 4: Read config (sizeof(logger_config_t) = 64 bytes)
            uint8_t config_read_tx[sizeof(logger_config_t)] = {0};  // Dummy TX data for full duplex
            logger_config_t config_response = {0};
            hal_err_t read_err =
                hal_spi_device_transmit(&s_link_device, 0, 0, config_read_tx, sizeof(config_read_tx), (uint8_t*)&config_response, sizeof(logger_config_t));

            if (read_err == HAL_ERR_NONE) {
                // Step 5: Validate config magic (0xCAFE)
                if (config_response.magic == LOGGER_CONFIG_MAGIC) {
#if 1  // Disable UART output to prevent read data delay (approximately 55ms)
                    LOG_I(TAG, "Config received: magic=0x%04X version=%u.%u", config_response.magic, config_response.version_major, config_response.version_minor);

                    // Log ADC configuration
                    LOG_I(TAG, "=== ADC Configuration ===");
                    LOG_I(TAG, "  Sample Rate: %u kHz", config_response.adc_sample_rate_khz);
                    LOG_I(TAG, "  Block Size: %u samples", config_response.adc_block_size);

                    // Log IMU configuration
                    LOG_I(TAG, "=== IMU Configuration ===");
                    LOG_I(TAG, "  Accel Present: %u", config_response.imu_config.accel_present);
                    LOG_I(TAG, "  Gyro Present: %u", config_response.imu_config.gyro_present);
                    LOG_I(TAG, "  Chip ID: 0x%02X", config_response.imu_config.chip_id);
                    LOG_I(TAG, "  Accel ODR: %u Hz", config_response.imu_config.accel_odr_hz);
                    LOG_I(TAG, "  Gyro ODR: %u Hz", config_response.imu_config.gyro_odr_hz);
                    LOG_I(TAG, "  Accel Range: %u G", config_response.imu_config.accel_range_g);
                    LOG_I(TAG, "  Gyro Range: %u DPS", config_response.imu_config.gyro_range_dps);

                    // Write config header to log file
                    logger_module_set_stm_config(&g_logger_module, &config_response);
#endif
                    s_configured = true;
                    break;
                } else {
                    LOG_W(TAG, "Invalid config magic: 0x%04X (expected 0xCAFE)", config_response.magic);
                }
            } else {
                LOG_E(TAG, "Config read failed: %d", (int)read_err);
            }
        } else {
            LOG_D(TAG, "Config command sent but GPIO low, retrying...");
        }

        // Wait before retrying config command
        vTaskDelay(pdMS_TO_TICKS(6));
    }
#if 1
    // ===== REPEATEDLY SEND READ COMMANDS (EVERY 5MS) =====
    // Protocol:
    //   1. Send command 45 (read logging) - 5 bytes only
    //   2. Check GPIO to see if frame is ready
    //   3. If GPIO high: read 640-byte frame, process it
    //   4. If GPIO low: wait 5ms before next attempt
    while (s_reader_ready) {
        // Step 2: Check if GPIO is high (frame ready from STM32)
        if (hal_gpio_get_level(LINK_INT_GPIO) == HAL_GPIO_HIGH) {
            //  Step 3: Read 640-byte frame
            hal_gpio_set_level(13, !hal_gpio_get_level(13));
            logger_frame_t frame = {0};
            uint8_t read_tx[LOGGER_FRAME_SIZE_BYTES] = {0};  // Dummy TX data (SPI is full duplex, need to send something)
            hal_err_t read_err = hal_spi_device_transmit(&s_link_device, 0, 0, read_tx, sizeof(read_tx), (uint8_t*)&frame, sizeof(frame));

            if (read_err != HAL_ERR_NONE) {
                LOG_E(TAG, "SPI frame read failed: %d", (int)read_err);
                vTaskDelay(pdMS_TO_TICKS(5));
                continue;
            }

            // Validate frame magic (0x5A5A for LogFrame_t)
            if (frame.magic == LOGGER_FRAME_MAGIC) {
                s_last_frame_us = esp_timer_get_time();
                ++s_valid_frames;
                s_invalid_frames = 0;

                // Write the entire 852-byte frame to the log file
                logger_module_write_frame(&g_logger_module, &frame);

                // Invert GPIO 13 on valid frame
                hal_gpio_set_level(13, !hal_gpio_get_level(13));
            } else {
                // Toggle GPIO 13 LOW on invalid frame
                //hal_gpio_set_level(13, HAL_GPIO_LOW);
                LOG_D(TAG, "Invalid frame magic: 0x%04X (expected 0x5A5A)", (unsigned)frame.magic);
                ++s_invalid_frames;
            }

            // Frame received successfully, send next command immediately
            // delay required because TASK has highest priority  and without delay it occupies all CPU time
            vTaskDelay(pdMS_TO_TICKS(1));
        } else {
            // GPIO low: STM32 has no frame ready (queue empty)
            // Wait 5ms before next read command attempt
            // LOG_D(TAG, "GPIO low (no frame ready), waiting 5ms...");
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
#endif
}

// === OLD remote_accel_reader_drain() REMOVED ===
// Polling now happens continuously in remote_accel_reader_task()

static void remote_accel_reader_task(void* arg) {
    (void)arg;
    while (1) {
        uint32_t notified = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(500));
        if (notified) {
            remote_accel_reader_drain();
        }

        uint64_t now_us = esp_timer_get_time();
        if (s_valid_frames == 0 && now_us - s_last_diag_us >= DIAG_IDLE_INTERVAL_US) {
            int int_level = hal_gpio_get_level(LINK_INT_GPIO);
            LOG_W(TAG, "No accel link frames received yet (INT level=%d, host=%d, clk=%d miso=%d mosi=%d cs=%d)", int_level, (int)LINK_SPI_HOST,
                  (int)LINK_SPI_CLK_GPIO, (int)LINK_SPI_MISO_GPIO, (int)LINK_SPI_MOSI_GPIO, (int)LINK_SPI_CS_GPIO);
            s_last_diag_us = now_us;
        } else if (s_valid_frames > 0 && now_us - s_last_frame_us >= DIAG_IDLE_INTERVAL_US && now_us - s_last_diag_us >= DIAG_IDLE_INTERVAL_US) {
            int int_level = hal_gpio_get_level(LINK_INT_GPIO);
            LOG_W(TAG, "No new accel link frames for %llu ms (INT level=%d, total_valid=%u)", (unsigned long long)((now_us - s_last_frame_us) / 1000ULL),
                  int_level, (unsigned)s_valid_frames);
            s_last_diag_us = now_us;
        }

        if (now_us - s_last_stats_us >= DIAG_IDLE_INTERVAL_US) {
            LOG_W(TAG, "Accel link stats: valid=%u invalid=%u batches=%u INT=%d", (unsigned)s_valid_frames, (unsigned)s_invalid_frames,
                  (unsigned)s_drain_batches, hal_gpio_get_level(LINK_INT_GPIO));
            s_valid_frames = 0;
            s_invalid_frames = 0;
            s_drain_batches = 0;
            s_last_stats_us = now_us;
        }
    }
}

void remote_accel_reader_init(void) {
    hal_err_t err = hal_spi_bus_init(LINK_SPI_HOST, LINK_SPI_MISO_GPIO, LINK_SPI_MOSI_GPIO, LINK_SPI_CLK_GPIO);
    if (err != HAL_ERR_NONE) {
        LOG_E(TAG, "Failed to init link SPI bus (err=%d)", (int)err);
        return;
    }

    hal_spi_device_config_t devcfg = {
        .command_bits = 0,
        .address_bits = 0,
        .clock_speed_hz = ACCEL_SPI_FREQ_HZ,
        .spi_mode = 0,
        .cs = LINK_SPI_CS_GPIO,
        .flags = 0,
    };

    err = hal_spi_bus_add_device(LINK_SPI_HOST, &devcfg, &s_link_device);
    if (err != HAL_ERR_NONE) {
        LOG_E(TAG, "Failed to add SPI device for remote accel link (err=%d)", (int)err);
        return;
    }

    // Link INT is actively driven low by the accel reader when idle; use pull-down to avoid floating high when the wire is disconnected.
    hal_err_t gpio_err = hal_gpio_setup(LINK_INT_GPIO, HAL_GPIO_DIR_INPUT, HAL_GPIO_PULL_DOWN);
    if (gpio_err != HAL_ERR_NONE) {
        LOG_E(TAG, "Failed to configure LINK_INT_GPIO (err=%d)", (int)gpio_err);
        remote_accel_reader_cleanup_spi();
        return;
    }

    if (hal_gpio_set_isr(LINK_INT_GPIO, HAL_GPIO_INTR_POSEDGE, remote_link_isr, NULL) != 0) {
        LOG_E(TAG, "Failed to attach ISR for LINK_INT_GPIO");
        remote_accel_reader_cleanup_spi();
        return;
    }

    gpio_err = hal_gpio_intr_enable(LINK_INT_GPIO);
    if (gpio_err != HAL_ERR_NONE) {
        LOG_E(TAG, "Failed to enable interrupt on LINK_INT_GPIO (err=%d)", (int)gpio_err);
        remote_accel_reader_cleanup_spi();
        return;
    }

    BaseType_t ret = xTaskCreatePinnedToCore(remote_accel_reader_task, "REM_ACCEL", TASK_STACK_SIZE_ACCEL_TASK / sizeof(StackType_t), NULL,
                                             TASK_PRIORITY_ACCEL_TASK, &s_reader_task_handle, CORE_ACCEL_TASK);
    if (ret != pdPASS) {
        LOG_E(TAG, "Failed to create remote accel reader task");
        hal_gpio_intr_disable(LINK_INT_GPIO);
        remote_accel_reader_cleanup_spi();
        return;
    }

    s_reader_ready = true;
    LOG_I(TAG, "Remote accelerometer reader initialized (SPI master host=%d, IRQ GPIO=%d, clk=%d miso=%d mosi=%d cs=%d)", (int)LINK_SPI_HOST,
          (int)LINK_INT_GPIO, (int)LINK_SPI_CLK_GPIO, (int)LINK_SPI_MISO_GPIO, (int)LINK_SPI_MOSI_GPIO, (int)LINK_SPI_CS_GPIO);

    // Notify task to start draining immediately (don't wait for GPIO edge at startup)
    xTaskNotifyGive(s_reader_task_handle);
    s_last_diag_us = esp_timer_get_time();
}

#else  // not ESP32S3

void remote_accel_reader_init(void) {
    // No-op for unsupported targets.
}

#endif
