#include "remote_accel_reader.h"

#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_attr.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <stdbool.h>
#include <string.h>


#include "app_state.h"
#include "hal/gpio.h"
#include "hal/spi.h"
#include "log.h"
#include "logger_frame_link.h"  // SPI link frame format (logger_frame_t)
#include "logger_module.h"
#include "target.h"

static const char* TAG = "STMLINK";

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

// Keep the SPI reader task responsive even if STM32 is unplugged/not running.
    static const uint64_t CONFIG_INT_WAIT_TIMEOUT_US = 30000ULL;  // 30 ms for INT to assert after CMD 42
    static const uint32_t CONFIG_MAX_ATTEMPTS = 5;
    static const TickType_t CONFIG_RETRY_AFTER_RESET_DELAY = pdMS_TO_TICKS(100);
    static const TickType_t STM_RESET_LOW_TIME = pdMS_TO_TICKS(100);
    // Give STM32 time to boot after a reset before we start sending CMD 42.
    // If this value is too small, ESP can get stuck in a reset loop (STM never reaches Logger_Init()).
    static const uint64_t CONFIG_AFTER_RESET_GRACE_US = 500000ULL;  // 500 ms
    static const uint64_t CONFIG_RETRY_AFTER_INCOMPLETE_US = 150000ULL;
    static const uint64_t CONFIG_RETRY_NO_INT_BACKOFF_US = 200000ULL;
    static const uint32_t CONFIG_NO_INT_RETRY_WITHOUT_RESET = 2;

static bool s_configured = false;
static bool s_config_waiting_int = false;
    static uint32_t s_config_attempts = 0;
    static uint64_t s_config_next_attempt_us = 0;
    static uint64_t s_config_wait_deadline_us = 0;
    static uint32_t s_config_no_int_without_reset = 0;

static uint32_t s_last_adc_timestamp = 0;
static bool s_have_last_adc_timestamp = false;
static uint32_t s_expected_adc_delta_ticks = 0;
static uint32_t s_bad_adc_delta_count = 0;
static uint64_t s_last_bad_adc_delta_log_us = 0;
static const uint64_t BAD_ADC_DELTA_LOG_INTERVAL_US = 1000000ULL;  // 1 second

static uint64_t s_spi_frame_read_sum_us = 0;
static uint64_t s_spi_frame_read_max_us = 0;
static uint32_t s_spi_frame_read_count = 0;

// When false, the ISR will not wake the reader task on INT edges.
// This is used to ignore STM32 streaming interrupts while the logger is not in LOGGING mode.
static volatile bool s_wake_on_int = true;

static uint8_t s_config_cmd_tx[5] = {42, 0, 0, 0, 0};
static uint8_t s_config_dummy_tx[sizeof(logger_config_t)] = {0};
DMA_ATTR static uint8_t s_frame_dummy_tx[LOGGER_FRAME_SIZE_BYTES] = {0};
DMA_ATTR static logger_frame_t s_frame_rx __attribute__((aligned(4)));

static void remote_accel_reader_cleanup_spi(void);
static void remote_accel_reader_log_config(const logger_config_t* cfg);

    static void remote_accel_reader_reset_stm32(void) {
        LOG_W(TAG, "Resetting STM32 on GPIO %d", STM_RESET_GPIO);
        gpio_reset_pin(STM_RESET_GPIO);
        gpio_set_direction(STM_RESET_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(STM_RESET_GPIO, 0);
    vTaskDelay(STM_RESET_LOW_TIME);
    gpio_set_level(STM_RESET_GPIO, 1);
    vTaskDelay(CONFIG_RETRY_AFTER_RESET_DELAY);
        // Set back to Hi-Z (input) so ST-LINK can control STM32 reset for debugging.
        gpio_set_direction(STM_RESET_GPIO, GPIO_MODE_INPUT);
    }

static void remote_accel_reader_log_config(const logger_config_t* cfg) {
    if (!cfg) {
        return;
    }

    const uint8_t checksum_algo_id = cfg->reserved[0];
    const char* checksum_algo_name = "UNKNOWN";
    switch (checksum_algo_id) {
        case 1:
            checksum_algo_name = "CRC8";
            break;
        case 2:
            checksum_algo_name = "SUM8";
            break;
        case 3:
            checksum_algo_name = "CRC8_HW";
            break;
        default:
            break;
    }

    LOG_I(TAG, "STM32 config received:");
    LOG_I(TAG, "  magic=0x%04X version=%u.%u", (unsigned)cfg->magic, (unsigned)cfg->version_major, (unsigned)cfg->version_minor);
    LOG_I(TAG, "  ADC: sample_rate=%u kHz block_size=%u", (unsigned)cfg->adc_sample_rate_khz, (unsigned)cfg->adc_block_size);
    LOG_I(TAG, "  checksum: algo=%s (id=%u)", checksum_algo_name, (unsigned)checksum_algo_id);

    LOG_I(TAG, "  IMU: accel_present=%u gyro_present=%u chip_id=0x%02X", (unsigned)cfg->imu_config.accel_present,
          (unsigned)cfg->imu_config.gyro_present, (unsigned)cfg->imu_config.chip_id);
    LOG_I(TAG, "  IMU: accel_odr=%u Hz gyro_odr=%u Hz accel_range=%u g gyro_range=%u dps", (unsigned)cfg->imu_config.accel_odr_hz,
          (unsigned)cfg->imu_config.gyro_odr_hz, (unsigned)cfg->imu_config.accel_range_g, (unsigned)cfg->imu_config.gyro_range_dps);
    LOG_I(TAG, "  IMU regs: CTRL1_XL=0x%02X CTRL2_G=0x%02X CTRL3_C=0x%02X CTRL7_G=0x%02X CTRL4_C=0x%02X",
          (unsigned)cfg->imu_config.reserved0, (unsigned)cfg->imu_config.reserved1, (unsigned)cfg->imu_config.reserved2,
          (unsigned)cfg->imu_config.reserved3, (unsigned)cfg->imu_config.reserved4);

    LOG_I(TAG, "  MAVLink logging: %s", cfg->mavlink_logging_enabled ? "enabled" : "disabled");

    LOG_I(TAG,
          "  reserved[32]=%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X "
          "%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
          (unsigned)cfg->reserved[0], (unsigned)cfg->reserved[1], (unsigned)cfg->reserved[2], (unsigned)cfg->reserved[3],
          (unsigned)cfg->reserved[4], (unsigned)cfg->reserved[5], (unsigned)cfg->reserved[6], (unsigned)cfg->reserved[7],
          (unsigned)cfg->reserved[8], (unsigned)cfg->reserved[9], (unsigned)cfg->reserved[10], (unsigned)cfg->reserved[11],
          (unsigned)cfg->reserved[12], (unsigned)cfg->reserved[13], (unsigned)cfg->reserved[14], (unsigned)cfg->reserved[15],
          (unsigned)cfg->reserved[16], (unsigned)cfg->reserved[17], (unsigned)cfg->reserved[18], (unsigned)cfg->reserved[19],
          (unsigned)cfg->reserved[20], (unsigned)cfg->reserved[21], (unsigned)cfg->reserved[22], (unsigned)cfg->reserved[23],
          (unsigned)cfg->reserved[24], (unsigned)cfg->reserved[25], (unsigned)cfg->reserved[26], (unsigned)cfg->reserved[27],
          (unsigned)cfg->reserved[28], (unsigned)cfg->reserved[29], (unsigned)cfg->reserved[30], (unsigned)cfg->reserved[31]);
}

    void remote_accel_reader_restart_session(void) {
    const app_state_t *state = app_state_get_instance();
    if (state && state->system_error_code != APP_OK) {
        LOG_W(TAG, "STM32 link restart requested while app is in error state (err=%ld); ignoring", (long)state->system_error_code);
        return;
    }

    s_wake_on_int = true;

    // If the reader task has exited (e.g. due to previous fatal handshake failure), recreate it.
    if (!s_reader_ready || s_reader_task_handle == NULL) {
        remote_accel_reader_init();
        if (!s_reader_ready || s_reader_task_handle == NULL) {
            return;
        }
    }

    // Pause frame reception until we re-fetch the config.
    s_configured = false;
    s_config_waiting_int = false;
        s_config_attempts = 0;
        s_config_no_int_without_reset = 0;
    // Prevent the reader task from requesting config while the STM32 is being reset/booting.
    s_config_next_attempt_us = esp_timer_get_time() + CONFIG_AFTER_RESET_GRACE_US;
    s_config_wait_deadline_us = 0;

    // Reset stats/timestamps for cleaner diagnostics.
    s_valid_frames = 0;
    s_invalid_frames = 0;
    s_drain_batches = 0;
    s_last_frame_us = 0;
    s_last_diag_us = esp_timer_get_time();
    s_last_stats_us = s_last_diag_us;
    s_last_adc_timestamp = 0;
    s_have_last_adc_timestamp = false;
    s_expected_adc_delta_ticks = 0;
    s_bad_adc_delta_count = 0;
    s_last_bad_adc_delta_log_us = 0;
    s_spi_frame_read_sum_us = 0;
    s_spi_frame_read_max_us = 0;
    s_spi_frame_read_count = 0;

    // Ensure logger writes a fresh config header for the new session.
    g_logger_module.stm_config_received = false;

    // Restart STM32, wait for it to boot, then the reader task will re-run the config handshake.
        remote_accel_reader_reset_stm32();

    // Kick the task so it doesn't wait for a GPIO edge.
    xTaskNotifyGive(s_reader_task_handle);
    }

    static void remote_accel_reader_fail_config_and_retry(const char* reason) {
        ++s_config_attempts;

    if (s_config_attempts >= CONFIG_MAX_ATTEMPTS) {
        LOG_E(TAG, "STM32 config failed after %u attempts (%s). Entering ERROR state.", (unsigned)s_config_attempts, reason ? reason : "unknown");
        app_state_set_error(APP_ERR_STM32_CONFIG_TIMEOUT);
        s_reader_ready = false;
        hal_gpio_intr_disable(LINK_INT_GPIO);
        remote_accel_reader_cleanup_spi();
        LOG_E(TAG, "STM32 link task stopped due to config failure");
        s_reader_task_handle = NULL;
        vTaskDelete(NULL);
        return;
    }

        const bool no_int = (reason != NULL) && (strcmp(reason, "no config response (INT stayed low)") == 0);

        if (no_int && s_config_no_int_without_reset < CONFIG_NO_INT_RETRY_WITHOUT_RESET) {
            // Avoid reset-looping the STM32: it may still be booting / not yet arming SPI RX.
            // Back off and re-send CMD42 without resetting first.
            s_config_no_int_without_reset++;
            LOG_W(TAG,
                  "STM32 config failed (%s). Attempt %u/%u -> backing off %llu ms and retrying without reset (%u/%u)...",
                  reason, (unsigned)s_config_attempts, (unsigned)CONFIG_MAX_ATTEMPTS,
                  (unsigned long long)(CONFIG_RETRY_NO_INT_BACKOFF_US / 1000ULL),
                  (unsigned)s_config_no_int_without_reset, (unsigned)CONFIG_NO_INT_RETRY_WITHOUT_RESET);

            s_config_waiting_int = false;
            s_config_next_attempt_us = esp_timer_get_time() + CONFIG_RETRY_NO_INT_BACKOFF_US;
            s_config_wait_deadline_us = 0;
            return;
        }

        s_config_no_int_without_reset = 0;
        LOG_W(TAG, "STM32 config failed (%s). Attempt %u/%u -> resetting STM32 and retrying...",
              reason ? reason : "unknown", (unsigned)s_config_attempts, (unsigned)CONFIG_MAX_ATTEMPTS);

        s_config_waiting_int = false;
        s_config_next_attempt_us = esp_timer_get_time() + CONFIG_AFTER_RESET_GRACE_US;
        s_config_wait_deadline_us = 0;

        remote_accel_reader_reset_stm32();
    }

static void remote_accel_reader_cleanup_spi(void) {
    if (s_link_device.lock) {
        vSemaphoreDelete(s_link_device.lock);
        s_link_device.lock = NULL;
    }
    if (s_link_device.dev) {
        spi_bus_remove_device(s_link_device.dev);
        s_link_device.dev = NULL;
    }

    // Free the SPI bus so the reader can be re-created after a fatal failure.
    // (Without this, spi_bus_initialize() returns ESP_ERR_INVALID_STATE on re-init.)
    (void)spi_bus_free(LINK_SPI_HOST);
}

static void IRAM_ATTR remote_link_isr(void* arg) {
    (void)arg;
    if (!s_wake_on_int) {
        return;
    }
    BaseType_t higher_priority_task_woken = pdFALSE;
    if (s_reader_task_handle) {
        vTaskNotifyGiveFromISR(s_reader_task_handle, &higher_priority_task_woken);
        if (higher_priority_task_woken) {
            portYIELD_FROM_ISR();
        }
    }
}

    static void remote_accel_reader_step_config(uint64_t now_us) {
        if (!s_reader_ready || s_configured) {
            return;
        }

    const app_state_t *state = app_state_get_instance();
    if (state && state->system_error_code != APP_OK) {
        // Once the app is in an error state, do not keep resetting/handshaking with STM32 in the background.
        return;
    }

    if (s_config_next_attempt_us != 0 && now_us < s_config_next_attempt_us) {
        return;
    }

        if (s_config_waiting_int) {
            if (hal_gpio_get_level(LINK_INT_GPIO) == HAL_GPIO_HIGH) {
            // Give STM32 a short moment to prepare the config DMA buffer.
            vTaskDelay(pdMS_TO_TICKS(2));

            logger_config_t config_response = {0};
            hal_err_t read_err =
                hal_spi_device_transmit(&s_link_device, 0, 0, s_config_dummy_tx, sizeof(s_config_dummy_tx), (uint8_t*)&config_response, sizeof(logger_config_t));

            s_config_waiting_int = false;
            s_config_next_attempt_us = 0;

            if (read_err != HAL_ERR_NONE) {
                remote_accel_reader_fail_config_and_retry("spi config read failed");
                return;
            }

            if (config_response.magic != LOGGER_CONFIG_MAGIC) {
                remote_accel_reader_fail_config_and_retry("bad config magic");
                return;
            }

            const bool imu_cfg_missing =
                (config_response.imu_config.chip_id == 0) && (config_response.imu_config.accel_odr_hz == 0) && (config_response.imu_config.gyro_odr_hz == 0) &&
                (config_response.imu_config.reserved0 == 0) && (config_response.imu_config.reserved1 == 0);
                if (imu_cfg_missing) {
                // STM32 responded, but the IMU config fields are not populated yet (likely still initializing after a reset).
                // Wait a bit and re-request the config without resetting again.
                    LOG_W(TAG, "STM32 config received but IMU fields are not ready yet; retrying...");
                    s_config_next_attempt_us = now_us + CONFIG_RETRY_AFTER_INCOMPLETE_US;
                    return;
                }

                s_configured = true;
                s_config_attempts = 0;
                s_config_no_int_without_reset = 0;

            remote_accel_reader_log_config(&config_response);
            s_expected_adc_delta_ticks = (uint32_t)config_response.adc_block_size;
            s_have_last_adc_timestamp = false;
            s_bad_adc_delta_count = 0;
            s_last_bad_adc_delta_log_us = 0;
            s_spi_frame_read_sum_us = 0;
            s_spi_frame_read_max_us = 0;
            s_spi_frame_read_count = 0;

                logger_module_set_stm_config(&g_logger_module, &config_response);
                return;
            }

        if (now_us >= s_config_wait_deadline_us) {
            s_config_waiting_int = false;
            remote_accel_reader_fail_config_and_retry("no config response (INT stayed low)");
        }

        return;
        }

        LOG_I(TAG, "Requesting STM32 config (CMD 42), attempt %u/%u (INT=%d)",
              (unsigned)(s_config_attempts + 1u), (unsigned)CONFIG_MAX_ATTEMPTS, (int)hal_gpio_get_level(LINK_INT_GPIO));
        hal_err_t err = hal_spi_device_transmit(&s_link_device, 0, 0, s_config_cmd_tx, sizeof(s_config_cmd_tx), NULL, 0);
        if (err != HAL_ERR_NONE) {
            remote_accel_reader_fail_config_and_retry("spi config request send failed");
            return;
        }

    // Request sent: wait for INT to assert, but do not block the whole task.
    s_config_waiting_int = true;
    s_config_wait_deadline_us = now_us + CONFIG_INT_WAIT_TIMEOUT_US;
}

static void remote_accel_reader_drain_frames(uint64_t now_us) {
    if (!s_reader_ready || !s_configured) {
        return;
    }

    const app_state_t *state = app_state_get_instance();
    const bool should_write_frames = state && (state->current_mode == APP_MODE_LOGGING);
    if (!should_write_frames) {
        // In IDLE/other modes we only fetch the config and then ignore streaming data.
        // This prevents background SPI traffic and hides timestamp anomalies that should be debugged on the STM32 side.
        return;
    }

    ++s_drain_batches;
    logger_frame_t *frame = &s_frame_rx;

    // Drain as long as STM32 keeps INT asserted, but cap work per call to stay responsive.
    // If this slice is too small, the reader can fall behind and STM32 will start dropping frames.
    const uint64_t max_work_us = 8000ULL;  // 8 ms time slice
    const uint32_t max_frames_per_call = 16;
    uint32_t frames_read = 0;
    uint64_t batch_start_us = now_us;

    while (s_reader_ready && hal_gpio_get_level(LINK_INT_GPIO) == HAL_GPIO_HIGH) {
        if (frames_read >= max_frames_per_call || (esp_timer_get_time() - batch_start_us) >= max_work_us) {
            taskYIELD();
            break;
        }

        uint64_t frame_start_us = esp_timer_get_time();
        hal_err_t read_err =
            hal_spi_device_transmit(&s_link_device, 0, 0, s_frame_dummy_tx, LOGGER_FRAME_SIZE_BYTES, (uint8_t *)frame, sizeof(*frame));
        uint64_t dt_us = esp_timer_get_time() - frame_start_us;
        s_spi_frame_read_sum_us += dt_us;
        s_spi_frame_read_count++;
        if (dt_us > s_spi_frame_read_max_us) {
            s_spi_frame_read_max_us = dt_us;
        }

        if (read_err != HAL_ERR_NONE) {
            LOG_W(TAG, "Frame read failed: %d", (int)read_err);
            ++s_invalid_frames;
            break;
        }

        if (frame->magic == LOGGER_FRAME_MAGIC) {
            s_last_frame_us = esp_timer_get_time();
            ++s_valid_frames;
            s_invalid_frames = 0;

            if (s_expected_adc_delta_ticks != 0 && s_have_last_adc_timestamp) {
                uint32_t delta = (uint32_t)(frame->adc_timestamp - s_last_adc_timestamp);
                // Allow ±1 tick jitter. On STM32 the ADC timestamp is captured in/near DMA callbacks,
                // so depending on timing of the readout it can be off by 1 tick even when no samples
                // are dropped. Larger deltas still indicate drops/resets.
                const uint32_t expected = s_expected_adc_delta_ticks;
                const bool ok =
                    (delta == expected) ||
                    (delta == (expected + 1u)) ||
                    (expected > 0u && delta == (expected - 1u));

                if (!ok) {
                    ++s_bad_adc_delta_count;
                    if (now_us - s_last_bad_adc_delta_log_us >= BAD_ADC_DELTA_LOG_INTERVAL_US) {
                        LOG_W(TAG, "ADC delta mismatch: last=%u now=%u delta=%u expected=%u (bad=%u, mode=%u)",
                              (unsigned)s_last_adc_timestamp, (unsigned)frame->adc_timestamp, (unsigned)delta,
                              (unsigned)expected, (unsigned)s_bad_adc_delta_count,
                              (unsigned)(state ? state->current_mode : 0));
                        s_last_bad_adc_delta_log_us = now_us;
                    }
                }
            }
            s_last_adc_timestamp = frame->adc_timestamp;
            s_have_last_adc_timestamp = true;

            if (should_write_frames) {
                logger_module_write_frame(&g_logger_module, frame);
            }

            hal_gpio_set_level(13, !hal_gpio_get_level(13));
        } else {
            LOG_D(TAG, "Invalid frame magic: 0x%04X (expected 0x%04X)", (unsigned)frame->magic, (unsigned)LOGGER_FRAME_MAGIC);
            ++s_invalid_frames;
        }

        ++frames_read;
    }

    // If INT stays asserted, avoid monopolizing the core (APP_LOGIC/button runs at lower priority).
    // With CONFIG_FREERTOS_HZ=1000 this is a 1 ms delay; with lower tick rates we fall back to yield.
    if (hal_gpio_get_level(LINK_INT_GPIO) == HAL_GPIO_HIGH) {
#if CONFIG_FREERTOS_HZ >= 1000
        vTaskDelay(1);
#else
        taskYIELD();
#endif
    }
}

/**
 * @brief Receive and process logger frames from STM32 over the SPI link
 *
 * Protocol:
 *   - Startup: Send CMD 42 (0x2A) to request `logger_config_t` (64 bytes).
 *   - Streaming: After config is received, STM32 starts acquisition and asserts LINK_INT_GPIO when a frame is ready.
 *     When LINK_INT_GPIO is HIGH, read a full `logger_frame_t` (852 bytes).
 *
 * Notes:
 *   - This firmware does not perform any IMU/sensor operations; it only records frames produced by STM32.
 *   - Frame cadence is ~390 Hz (one frame per 2.56 ms at 100 kHz ADC / 256-sample blocks).
 */
static void remote_accel_reader_drain(void) {
    uint64_t now_us = esp_timer_get_time();
    remote_accel_reader_step_config(now_us);
    remote_accel_reader_drain_frames(now_us);
}

static void remote_accel_reader_task(void* arg) {
    (void)arg;
    while (1) {
        const app_state_t *state = app_state_get_instance();
        const bool should_stream = state && (state->current_mode == APP_MODE_LOGGING);
        // Wake on INT only when we are handshaking for config, or actively streaming frames in LOGGING mode.
        s_wake_on_int = (!s_configured) || should_stream;

        // Block most of the time (keeps button/app tasks responsive), but:
        // - wake on INT edges via task notifications
        // - avoid sleeping if INT is already asserted (backlog)
        TickType_t wait_ticks = pdMS_TO_TICKS(50);
        if (!s_configured && s_config_waiting_int) {
            wait_ticks = pdMS_TO_TICKS(5);
        }
        if (!s_configured && !s_config_waiting_int) {
            wait_ticks = pdMS_TO_TICKS(10);
        }
        if (should_stream && s_configured && hal_gpio_get_level(LINK_INT_GPIO) == HAL_GPIO_HIGH) {
            wait_ticks = 0;
        }
        ulTaskNotifyTake(pdTRUE, wait_ticks);
        remote_accel_reader_drain();

        uint64_t now_us = esp_timer_get_time();
        const bool diag_enabled = (!s_configured) || should_stream;
        if (diag_enabled) {
            if (s_valid_frames == 0 && now_us - s_last_diag_us >= DIAG_IDLE_INTERVAL_US) {
                int int_level = hal_gpio_get_level(LINK_INT_GPIO);
                LOG_W(TAG, "No STM32 link frames received yet (INT level=%d, host=%d, clk=%d miso=%d mosi=%d cs=%d)", int_level, (int)LINK_SPI_HOST,
                      (int)LINK_SPI_CLK_GPIO, (int)LINK_SPI_MISO_GPIO, (int)LINK_SPI_MOSI_GPIO, (int)LINK_SPI_CS_GPIO);
                s_last_diag_us = now_us;
            } else if (s_valid_frames > 0 && now_us - s_last_frame_us >= DIAG_IDLE_INTERVAL_US && now_us - s_last_diag_us >= DIAG_IDLE_INTERVAL_US) {
                int int_level = hal_gpio_get_level(LINK_INT_GPIO);
                LOG_W(TAG, "No new STM32 link frames for %llu ms (INT level=%d, total_valid=%u)",
                      (unsigned long long)((now_us - s_last_frame_us) / 1000ULL), int_level, (unsigned)s_valid_frames);
                s_last_diag_us = now_us;
            }

            if (now_us - s_last_stats_us >= DIAG_IDLE_INTERVAL_US) {
                LOG_W(TAG, "STM32 link stats: valid=%u invalid=%u batches=%u INT=%d", (unsigned)s_valid_frames, (unsigned)s_invalid_frames,
                      (unsigned)s_drain_batches, hal_gpio_get_level(LINK_INT_GPIO));
                if (s_spi_frame_read_count > 0 && s_spi_frame_read_sum_us > 0) {
                    const uint64_t bytes = (uint64_t)s_spi_frame_read_count * (uint64_t)LOGGER_FRAME_SIZE_BYTES;
                    const uint64_t bytes_per_sec = (bytes * 1000000ULL) / s_spi_frame_read_sum_us;
                    LOG_W(TAG, "STM32 link SPI read: frames=%u avg=%llu us max=%llu us throughput=%llu kB/s (clk=%u Hz)",
                          (unsigned)s_spi_frame_read_count,
                          (unsigned long long)(s_spi_frame_read_sum_us / (uint64_t)s_spi_frame_read_count),
                          (unsigned long long)s_spi_frame_read_max_us,
                          (unsigned long long)(bytes_per_sec / 1024ULL),
                          (unsigned)LINK_SPI_FREQ_HZ);
                }
                s_valid_frames = 0;
                s_invalid_frames = 0;
                s_drain_batches = 0;
                s_spi_frame_read_sum_us = 0;
                s_spi_frame_read_max_us = 0;
                s_spi_frame_read_count = 0;
                s_last_stats_us = now_us;
            }
        }
    }
}

void remote_accel_reader_init(void) {
    memset(&s_link_device, 0, sizeof(s_link_device));

    hal_err_t err = hal_spi_bus_init(LINK_SPI_HOST, LINK_SPI_MISO_GPIO, LINK_SPI_MOSI_GPIO, LINK_SPI_CLK_GPIO);
    if (err != HAL_ERR_NONE) {
        LOG_E(TAG, "Failed to init link SPI bus (err=%d)", (int)err);
        return;
    }

    hal_spi_device_config_t devcfg = {
        .command_bits = 0,
        .address_bits = 0,
        .clock_speed_hz = LINK_SPI_FREQ_HZ,
        .spi_mode = 0,
        .cs = LINK_SPI_CS_GPIO,
        .flags = 0,
    };

    err = hal_spi_bus_add_device(LINK_SPI_HOST, &devcfg, &s_link_device);
    if (err != HAL_ERR_NONE) {
        LOG_E(TAG, "Failed to add SPI device for STM32 link (err=%d)", (int)err);
        remote_accel_reader_cleanup_spi();
        return;
    }

    // Link INT is actively driven low by STM32 when idle; use pull-down to avoid floating high when the wire is disconnected.
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

    BaseType_t ret = xTaskCreatePinnedToCore(remote_accel_reader_task, "REM_STM", TASK_STACK_SIZE_STM_LINK / sizeof(StackType_t), NULL,
                                             TASK_PRIORITY_STM_LINK, &s_reader_task_handle, CORE_STM_LINK_TASK);
    if (ret != pdPASS) {
        LOG_E(TAG, "Failed to create STM32 link reader task");
        hal_gpio_intr_disable(LINK_INT_GPIO);
        remote_accel_reader_cleanup_spi();
        return;
    }

    s_reader_ready = true;
    LOG_I(TAG, "STM32 link reader initialized (SPI master host=%d, IRQ GPIO=%d, clk=%d miso=%d mosi=%d cs=%d)", (int)LINK_SPI_HOST,
          (int)LINK_INT_GPIO, (int)LINK_SPI_CLK_GPIO, (int)LINK_SPI_MISO_GPIO, (int)LINK_SPI_MOSI_GPIO, (int)LINK_SPI_CS_GPIO);
    LOG_I(TAG, "STM32 link SPI clock configured: %u Hz", (unsigned)LINK_SPI_FREQ_HZ);

    // Notify task to start draining immediately (don't wait for GPIO edge at startup)
    xTaskNotifyGive(s_reader_task_handle);
    s_last_diag_us = esp_timer_get_time();
}

#else  // not ESP32S3

void remote_accel_reader_init(void) {
    // No-op for unsupported targets.
}

void remote_accel_reader_restart_session(void) {
    // No-op for unsupported targets.
}

#endif
