#include "hal/sdcard.h"

#include <driver/gpio.h>
#include <driver/sdspi_host.h>
#include <driver/spi_master.h>
#include <esp_vfs_fat.h>
#include <sdmmc_cmd.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

static sdmmc_card_t *g_card;
static bool g_mounted;
static char g_mount_path[32];
static hal_spi_bus_t g_bus;
static hal_gpio_t g_cs;
static bool g_format;
static uint32_t g_max_freq_khz;
static int g_max_files;
static size_t g_alloc_unit;

// ---------------------------------------------------------------------------
// Soft flush clocks for SD SPI (CS high + >=80 dummy clocks @ low freq).
// Helps recover cards that got stuck after a bad init without full power-cycle.
// ---------------------------------------------------------------------------
hal_err_t sdcard_spi_flush_clocks(hal_spi_bus_t bus, hal_gpio_t cs_pin, uint32_t clock_hz) {
    // Ensure CS is high (inactive)
    gpio_set_direction(cs_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(cs_pin, 1);

    spi_device_interface_config_t devcfg;
    memset(&devcfg, 0, sizeof(devcfg));
    devcfg.clock_speed_hz = (int)clock_hz;
    devcfg.mode = 0;
    devcfg.spics_io_num = -1;  // CS is controlled manually
    devcfg.queue_size = 1;
    devcfg.flags = SPI_DEVICE_HALFDUPLEX;

    spi_device_handle_t dev = NULL;
    esp_err_t err = spi_bus_add_device(bus, &devcfg, &dev);
    if (err != ESP_OK) {
        ESP_LOGW("SDCARD", "flush_clocks: spi_bus_add_device failed: %s", esp_err_to_name(err));
        return err;
    }

    // 20 bytes of 0xFF => 160 clocks (>> 74 required)
    uint8_t ff[20];
    memset(ff, 0xFF, sizeof(ff));
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = sizeof(ff) * 8;
    t.tx_buffer = ff;

    err = spi_device_transmit(dev, &t);
    spi_bus_remove_device(dev);

    if (err != ESP_OK) {
        ESP_LOGW("SDCARD", "flush_clocks: transmit failed: %s", esp_err_to_name(err));
    }
    return err;
}

hal_err_t sdcard_init(hal_spi_bus_t bus, hal_gpio_t cs_pin, const char *mount_path, bool format_if_mount_failed, uint32_t max_freq_khz, int max_files,
                      size_t alloc_unit) {
    // SPI bus is already initialized by the main application
    // No need to initialize it again here

    // Configure SD SPI device
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = cs_pin;
    slot_config.host_id = bus;

    // Configure mount options
    esp_vfs_fat_mount_config_t mount_config = {
        .format_if_mount_failed = format_if_mount_failed,
        .max_files = max_files,
        .allocation_unit_size = alloc_unit,
    };

    // Configure SDMMC host
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = bus;
    host.max_freq_khz = max_freq_khz;

    // Mount the SD card using the requested frequency
    esp_err_t ret = esp_vfs_fat_sdspi_mount(mount_path, &host, &slot_config, &mount_config, &g_card);
    if (ret != ESP_OK) {
        ESP_LOGE("SDCARD", "Failed to mount SD card: %s (0x%x)", esp_err_to_name(ret), ret);
        return ret;
    }

    ESP_LOGI("SDCARD", "SD card mounted at %lu kHz", (unsigned long)max_freq_khz);
    // remember params for later re-mount
    g_bus = bus;
    g_cs = cs_pin;
    g_format = format_if_mount_failed;
    g_max_freq_khz = max_freq_khz;
    g_max_files = max_files;
    g_alloc_unit = alloc_unit;
    strncpy(g_mount_path, mount_path, sizeof(g_mount_path) - 1);
    g_mount_path[sizeof(g_mount_path) - 1] = '\0';
    g_mounted = true;
    return HAL_ERR_NONE;
}

hal_err_t sdcard_open_file(const char *path, const char *mode, sdcard_file_handle_t *handle) {
    FILE *f = fopen(path, mode);
    if (!f) {
        return HAL_ERR_FAILED;
    }
    *handle = f;
    return HAL_ERR_NONE;
}

int sdcard_write(sdcard_file_handle_t handle, const void *data, size_t size) { return fwrite(data, 1, size, (FILE *)handle); }

hal_err_t sdcard_fsync(sdcard_file_handle_t handle) { return fsync(fileno((FILE *)handle)) == 0 ? HAL_ERR_NONE : HAL_ERR_FAILED; }

hal_err_t sdcard_close_file(sdcard_file_handle_t handle) { return fclose((FILE *)handle) == 0 ? HAL_ERR_NONE : HAL_ERR_FAILED; }

hal_err_t sdcard_deinit(const char *mount_path) {
    esp_vfs_fat_sdcard_unmount(mount_path, g_card);
    g_card = NULL;
    g_mounted = false;
    return HAL_ERR_NONE;
}

// -------- New helpers ----------
hal_err_t sdcard_unmount(void) {
    if (!g_mounted || g_card == NULL) return HAL_ERR_NONE;
    esp_vfs_fat_sdcard_unmount(g_mount_path, g_card);
    g_card = NULL;
    g_mounted = false;
    return HAL_ERR_NONE;
}

hal_err_t sdcard_mount(void) {
    if (g_mounted) return HAL_ERR_NONE;
    // Recreate configs (same as in sdcard_init)
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = g_cs;
    slot_config.host_id = g_bus;

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = g_bus;
    host.max_freq_khz = g_max_freq_khz;

    esp_vfs_fat_mount_config_t mount_config = {
        .format_if_mount_failed = g_format,
        .max_files = g_max_files,
        .allocation_unit_size = g_alloc_unit,
    };
    esp_err_t ret = esp_vfs_fat_sdspi_mount(g_mount_path, &host, &slot_config, &mount_config, &g_card);
    if (ret != ESP_OK) {
        ESP_LOGE("SDCARD", "Remount failed: %s (0x%x)", esp_err_to_name(ret), ret);
        return HAL_ERR_FAILED;
    }
    g_mounted = true;
    return HAL_ERR_NONE;
}

const char *sdcard_get_mount_point(void) { return g_mount_path; }
sdmmc_card_t *sdcard_get_card(void) { return g_card; }

// -------- Raw Access for MSC ----------
static sdspi_dev_handle_t g_spi_handle;

hal_err_t sdcard_mount_raw(void) {
    if (g_mounted) {
        sdcard_unmount();
    }
    if (g_card) {
        // Already raw mounted?
        return HAL_ERR_NONE;
    }

    // Add device to SPI bus
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = g_cs;
    slot_config.host_id = g_bus;

    esp_err_t ret = sdspi_host_init_device(&slot_config, &g_spi_handle);
    if (ret != ESP_OK) {
        ESP_LOGE("SDCARD", "Failed to init SPI device: %s", esp_err_to_name(ret));
        return HAL_ERR_FAILED;
    }

    // Initialize card
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = g_bus;
    host.max_freq_khz = g_max_freq_khz;

    g_card = (sdmmc_card_t *)malloc(sizeof(sdmmc_card_t));
    if (!g_card) {
        sdspi_host_remove_device(g_spi_handle);
        return HAL_ERR_MEMORY;
    }

    ret = sdmmc_card_init(&host, g_card);
    if (ret != ESP_OK) {
        ESP_LOGE("SDCARD", "Failed to init SD card: %s", esp_err_to_name(ret));
        free(g_card);
        g_card = NULL;
        sdspi_host_remove_device(g_spi_handle);
        return HAL_ERR_FAILED;
    }

    return HAL_ERR_NONE;
}

hal_err_t sdcard_unmount_raw(void) {
    if (g_mounted) {
        return HAL_ERR_INVALID_STATE;  // Should be unmounted first
    }
    if (!g_card) {
        return HAL_ERR_NONE;
    }

    // Deinit card (no specific function, just free memory)
    free(g_card);
    g_card = NULL;

    // Remove device
    sdspi_host_remove_device(g_spi_handle);
    return HAL_ERR_NONE;
}
