#include "io_manager.h"

#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <errno.h>
#include <log.h>
#include <sdkconfig.h>
#include <stdlib.h>
#include <string.h>
#include <target.h>

#include "app_state.h"
#include "hal/sdcard.h"
#include "util/macros.h"

static const char *TAG = "IOM";

// Soft-reset helper for SD cards stuck after a bad boot.
// Implemented in sdcard.c (kept here as forward decl to avoid touching headers).
hal_err_t sdcard_spi_flush_clocks(hal_spi_bus_t bus, hal_gpio_t cs_pin, uint32_t clock_hz);

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

hal_err_t io_manager_init(io_manager_t *iom) {
    memset(iom, 0, sizeof(*iom));
    iom->sd_card_ok = false;

    LOG_I(TAG, "Initializing SPI buses...");

    vTaskDelay(pdMS_TO_TICKS(100));

    hal_err_t err;

    // Initialize SPI bus for SD Card
    err = hal_spi_bus_init(SDCARD_SPI_HOST, SD_SPI_MISO_GPIO, SD_SPI_MOSI_GPIO, SD_SPI_CLK_GPIO);
    if (err != HAL_ERR_NONE) {
        LOG_E(TAG, "Failed to initialize SD card SPI bus");
        return err;
    }
    iom->sdcard_spi_bus = SDCARD_SPI_HOST;

#ifdef CONFIG_IDF_TARGET_ESP32S3
    // For ESP32-S3, explicitly configure pull-ups for SD card SPI pins
    // This is critical for reliable SD card communication, especially MISO line
    // SD card SPI pins are defined in target.h
    gpio_set_pull_mode(SD_SPI_MISO_GPIO, GPIO_PULLUP_ONLY);  // MISO needs pull-up
    gpio_set_pull_mode(SD_SPI_MOSI_GPIO, GPIO_PULLUP_ONLY);  // MOSI pull-up for stability
    // CLK pull-down can break SD SPI entry / CRC on some cards.
    // Prefer pull-up (or floating if you have external pull-ups).
    gpio_set_pull_mode(SD_SPI_CLK_GPIO, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(SD_SPI_CS_GPIO, GPIO_PULLUP_ONLY);  // CS pull-up (active low)

    LOG_I(TAG, "Configured pull-ups for SD card SPI pins: MISO=%d, MOSI=%d, CLK=%d, CS=%d", SD_SPI_MISO_GPIO, SD_SPI_MOSI_GPIO, SD_SPI_CLK_GPIO,
          SD_SPI_CS_GPIO);
#endif

    uint32_t freq_khz = SDCARD_SPI_FREQ_KHZ;
    const uint32_t min_freq_khz = 10000U;
    const uint32_t step_khz = 1000U;
    err = HAL_ERR_FAILED;

    while (freq_khz >= min_freq_khz) {
        // Give card clean idle clocks with CS high before (re)trying init.
        // This helps if the card booted into a weird state and ignores CMD0.
        (void)sdcard_unmount();      // best-effort cleanup if previous attempt half-mounted
        (void)sdcard_unmount_raw();  // best-effort cleanup if raw was left around
        hal_err_t flush_err = sdcard_spi_flush_clocks(iom->sdcard_spi_bus, SD_SPI_CS_GPIO, 400000 /*400kHz*/);
        if (flush_err != HAL_ERR_NONE) {
            LOG_W(TAG, "SD flush clocks failed (err=%d), continuing to init anyway", flush_err);
        }
        err = sdcard_init(iom->sdcard_spi_bus, SD_SPI_CS_GPIO, SD_MOUNT_PATH, true, freq_khz, SDCARD_MAX_FILES, SDCARD_ALLOC_UNIT);
        if (err == HAL_ERR_NONE) {
            LOG_I(TAG, "SD card initialized successfully at %lu kHz", (unsigned long)freq_khz);
            break;
        }

        LOG_W(TAG, "SD card init failed at %lu kHz (err=%d). Retrying with lower frequency...", (unsigned long)freq_khz, err);
        if (freq_khz <= min_freq_khz) {
            break;
        }
        if (freq_khz > min_freq_khz + step_khz) {
            freq_khz -= step_khz;
        } else {
            freq_khz = min_freq_khz;
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }

    if (err != HAL_ERR_NONE) {
        LOG_E(TAG, "Failed to initialize SD card HAL after stepping down to %lu kHz. SD card functions will be disabled.", (unsigned long)freq_khz);
        iom->sd_card_ok = false;
        app_state_set_error(APP_ERR_IO_INIT_FAILED);
    } else {
        iom->sd_card_ok = true;
    }

    LOG_I(TAG, "IO Manager Initialized Successfully");
    iom->initialized = true;
    return HAL_ERR_NONE;
}


void io_manager_shutdown(io_manager_t *iom) {
    (void)iom;
    // No UART resources to clean up
}
