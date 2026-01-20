#pragma once

#include <hal/spi.h>  // Use the canonical SPI type definitions from hal-common
// Removed local struct and typedef for hal_spi_device_config_s and hal_spi_device_config_t to avoid redefinition

// SPI HAL functions
hal_err_t hal_spi_bus_init(hal_spi_bus_t bus, hal_gpio_t miso, hal_gpio_t mosi, hal_gpio_t sck);
hal_err_t hal_spi_bus_add_device(hal_spi_bus_t bus, const hal_spi_device_config_t *cfg, hal_spi_device_handle_t *dev);
hal_err_t hal_spi_device_transmit(const hal_spi_device_handle_t *dev, uint16_t cmd, uint32_t addr, const void *tx, size_t tx_size, void *rx,
                                  size_t rx_size);
hal_err_t hal_spi_device_transmit_u8(const hal_spi_device_handle_t *dev, uint16_t cmd, uint32_t addr, uint8_t c, uint8_t *out);