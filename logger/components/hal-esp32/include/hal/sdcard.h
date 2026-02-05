#pragma once

#include <sdmmc_cmd.h>  // sdmmc_card_t for MSC
#include <stddef.h>

#include "hal/err.h"
#include "hal/spi.h"

#ifdef __cplusplus
extern "C" {
#endif

// Opaque file handle
typedef void *sdcard_file_handle_t;

// Public API
// max_freq_khz: SPI clock for SDSPI host (e.g., 20000 for 20 MHz)
// max_files: maximum simultaneously open files in VFS FAT layer
// alloc_unit: FAT allocation unit size in bytes (e.g., 16*1024)
hal_err_t sdcard_init(hal_spi_bus_t bus,
                      hal_gpio_t cs_pin,
                      const char *mount_path,
                      bool format_if_mount_failed,
                      uint32_t max_freq_khz,
                      int max_files,
                      size_t alloc_unit);
hal_err_t sdcard_open_file(const char *path, const char *mode, sdcard_file_handle_t *handle);
int sdcard_write(sdcard_file_handle_t handle, const void *data, size_t size);
hal_err_t sdcard_fsync(sdcard_file_handle_t handle);
hal_err_t sdcard_close_file(sdcard_file_handle_t handle);
hal_err_t sdcard_deinit(const char *mount_path);

// New helpers for MSC flow
hal_err_t sdcard_unmount(void);            // unmount VFS/FAT (safe to call if not mounted)
hal_err_t sdcard_mount(void);              // re-mount using params from initial init()
const char *sdcard_get_mount_point(void);  // e.g. "/sdcard"
sdmmc_card_t *sdcard_get_card(void);       // raw card pointer for MSC backend
hal_err_t sdcard_mount_raw(void);          // init card for raw access (no VFS)
hal_err_t sdcard_unmount_raw(void);        // deinit raw card access


#ifdef __cplusplus
}
#endif
