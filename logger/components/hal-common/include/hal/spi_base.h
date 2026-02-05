#pragma once

#include <hal/err.h>
#include <hal/gpio.h>
#include <stdint.h>

typedef struct hal_spi_device_config_s {
    uint8_t command_bits;
    uint8_t address_bits;
    uint32_t clock_speed_hz;
    uint8_t spi_mode;
    hal_gpio_t cs;
    uint32_t flags;
} hal_spi_device_config_t;

hal_err_t hal_spi_bus_init(hal_spi_bus_t bus, hal_gpio_t miso, hal_gpio_t mosi, hal_gpio_t sck);
hal_err_t hal_spi_bus_add_device(hal_spi_bus_t bus, const hal_spi_device_config_t *cfg, hal_spi_device_handle_t *dev);
// tx_size and rx_size are expressed in bytes. rx_size must be <= tx_size.
// Passing rx_size == 0 means rx_size is the same as tx_size.
hal_err_t hal_spi_device_transmit(const hal_spi_device_handle_t *dev, uint16_t cmd, uint32_t addr, const void *tx, size_t tx_size, void *rx, size_t rx_size);
hal_err_t hal_spi_device_transmit_u8(const hal_spi_device_handle_t *dev, uint16_t cmd, uint32_t addr, uint8_t c, uint8_t *out);
hal_err_t hal_spi_device_transmit_bits(
    const hal_spi_device_handle_t *dev, uint16_t cmd, uint32_t addr, const void *tx, size_t tx_size, void *rx, size_t rx_size);

// «extended» RX-transaction: cmd/addr/dummy + RX (without TX buffer)
// - cmd_bits/addr_bits/dummy_bits set the length of the service phases in bits
// - cmd — command value (0..2^cmd_bits-1), addr — address value (0..2^addr_bits-1)
// - rx_size — number of bytes to read after dummy
// - trans_flags: bit 0 -> keep CS low after transaction (CS_KEEP_ACTIVE)
hal_err_t hal_spi_device_receive_ext(const hal_spi_device_handle_t *dev, uint8_t cmd_bits, uint16_t cmd, uint8_t addr_bits, uint32_t addr, uint8_t dummy_bits,
    void *rx, size_t rx_size, uint32_t trans_flags);