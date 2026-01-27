#include <driver/spi_master.h>
#include <hal/spi.h>
#include <string.h>

hal_err_t hal_spi_bus_init(hal_spi_bus_t bus, hal_gpio_t miso, hal_gpio_t mosi, hal_gpio_t sck) {
    spi_bus_config_t cfg = {
        .miso_io_num = miso, .mosi_io_num = mosi, .sclk_io_num = sck, .quadwp_io_num = -1, .quadhd_io_num = -1, .max_transfer_sz = 16 * 1024};
    esp_err_t err = spi_bus_initialize(bus, &cfg, SPI_DMA_CH_AUTO);
    if (err == ESP_ERR_INVALID_STATE) {
        // Treat already-initialized SPI bus as success to keep init idempotent.
        return ESP_OK;
    }
    return err;
}

hal_err_t hal_spi_bus_add_device(hal_spi_bus_t bus, const hal_spi_device_config_t *cfg, hal_spi_device_handle_t *dev) {
    spi_device_interface_config_t devcfg;
    memset(&devcfg, 0, sizeof(devcfg));
    devcfg.command_bits = cfg->command_bits;
    devcfg.address_bits = cfg->address_bits;
    devcfg.clock_speed_hz = cfg->clock_speed_hz;
    devcfg.mode = cfg->spi_mode;  // mode 0-3
    devcfg.spics_io_num = cfg->cs;
    devcfg.queue_size = 1;        // we are using sync access //devcfg.queue_size = 4;
    devcfg.flags = 0;             // CS active low (default behavior)
    devcfg.duty_cycle_pos = 128;  // 50% duty cycle
    devcfg.cs_ena_pretrans = 0;   // No CS setup time
    devcfg.cs_ena_posttrans = 0;  // No CS hold time
                                  // Attach the device

    esp_err_t err = spi_bus_add_device(bus, &devcfg, &dev->dev);
    if (err != ESP_OK) return err;
    dev->lock = xSemaphoreCreateMutex();
    if (dev->lock == NULL) {
        spi_bus_remove_device(dev->dev);
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

hal_err_t hal_spi_device_transmit(const hal_spi_device_handle_t *dev, uint16_t cmd, uint32_t addr, const void *tx, size_t tx_size, void *rx, size_t rx_size) {
    // NOTE: For large payloads, callers should prefer DMA-capable buffers (e.g. `DMA_ATTR` or `heap_caps_malloc(..., MALLOC_CAP_DMA)`),
    // otherwise ESP-IDF may need to bounce/copy buffers internally which adds overhead.
    spi_transaction_t t = {
        .cmd = cmd,
        .addr = addr,
        .length = tx_size * 8,
        .rxlength = rx_size * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
        .flags = 0,
    };
    xSemaphoreTake(dev->lock, portMAX_DELAY);
    esp_err_t r = spi_device_transmit(dev->dev, &t);
    xSemaphoreGive(dev->lock);
    return r;
}

hal_err_t hal_spi_device_transmit_u8(const hal_spi_device_handle_t *dev, uint16_t cmd, uint32_t addr, uint8_t c, uint8_t *out) {
    spi_transaction_t t = {
        .cmd = cmd,
        .addr = addr,
        .length = 8,
        .rxlength = 8,
        .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
    };
    t.tx_data[0] = c;

    xSemaphoreTake(dev->lock, portMAX_DELAY);
    esp_err_t err = spi_device_transmit(dev->dev, &t);
    xSemaphoreGive(dev->lock);

    if (out) {
        *out = t.rx_data[0];
    }
    return err;
}

hal_err_t hal_spi_device_transmit_bits(const hal_spi_device_handle_t *dev, uint16_t cmd, uint32_t addr, const void *tx, size_t tx_size, void *rx,
                                       size_t rx_size) {
    spi_transaction_t t = {
        .cmd = cmd,
        .addr = addr,
        .length = tx_size,
        .rxlength = rx_size,
        .tx_buffer = tx,
        .rx_buffer = rx,
        .flags = 0,
    };
    xSemaphoreTake(dev->lock, portMAX_DELAY);
    esp_err_t r = spi_device_transmit(dev->dev, &t);
    xSemaphoreGive(dev->lock);
    return r;
}

/**
 * @brief Receive over SPI using explicit command/address/dummy phases followed by RX data (no TX payload).
 *
 * This helper issues a single SPI transaction built on top of ESP-IDF's `spi_transaction_ext_t`
 * with VARIABLE command/address/dummy lengths. It is intended for slaves that require the master
 * to (1) send a command byte, optionally (2) send address bits, then (3) generate extra "dummy"
 * clock cycles before the slave starts driving MISO. Typical examples are sensors (e.g. ICM-20948),
 * NOR flashes, and some display controllers.
 *
 * Why use this instead of crafting a manual TX buffer or splitting into two transfers?
 *  - Works cleanly with half-duplex + DMA, because the transaction contains RX only (no simultaneous
 *    MOSI+MISO phases that ESP-IDF forbids in half-duplex with DMA).
 *  - Keeps CS asserted across all phases; you can also request CS to remain low after the transfer.
 *  - Avoids ad-hoc “dummy TX bytes” allocation and off-by-one mistakes (e.g., discarding the first
 *    garbage byte).
 *
 * @param dev         SPI device handle returned by hal_spi_bus_add_device().
 * @param cmd_bits    Number of command bits to send (0..16). Set to 0 to omit the command phase.
 * @param cmd         Command value to send (LSB-aligned within cmd_bits).
 * @param addr_bits   Number of address bits to send after the command (0..32). Set to 0 to omit.
 * @param addr        Address value to send (LSB-aligned within addr_bits).
 * @param dummy_bits  Number of dummy clock bits to generate after cmd/addr before RX (often a multiple of 8).
 * @param rx          Destination buffer for the received payload. May be NULL when rx_size == 0.
 * @param rx_size     Number of BYTES to receive after the dummy phase.
 * @param trans_flags Transaction flags; bit 0 = keep CS active after the transfer (equivalent to SPI_TRANS_CS_KEEP_ACTIVE).
 *
 * @return HAL_OK on success; ESP_ERR_INVALID_ARG on bad parameters; other esp_err_t codes propagated
 *         from spi_device_transmit() on driver errors.
 *
 * @note If the slave encodes the READ bit inside a “register command” byte (e.g., reg | 0x80),
 *       pass that value via @p cmd (or @p addr if the protocol puts R/W in the address field).
 * @note For devices that don’t need separate phases, prefer hal_spi_device_transmit().
 *
 * @example
 *   // Read WHO_AM_I (0x00) from ICM-20948: 8-bit command with READ bit, no address, 8 dummy bits, then 1 byte RX.
 *   uint8_t who = 0;
 *   hal_spi_device_receive_ext(&imu_dev,
 *                              /\*cmd_bits=*\/8, /\*cmd=*\/ (0x00 | 0x80),
 *                              /\*addr_bits=*\/0, /\*addr=*\/0,
 *                              /\*dummy_bits=*\/8,
 *                              /\*rx=*\/&who, /\*rx_size=*\/1,
 *                              /\*trans_flags=*\/0);
 */

hal_err_t hal_spi_device_receive_ext(const hal_spi_device_handle_t *dev, uint8_t cmd_bits, uint16_t cmd, uint8_t addr_bits, uint32_t addr, uint8_t dummy_bits,
                                     void *rx, size_t rx_size, uint32_t trans_flags) {
    if (!rx && rx_size) return ESP_ERR_INVALID_ARG;

    spi_transaction_ext_t t;
    memset(&t, 0, sizeof(t));

    // base part
    t.base.cmd = cmd;
    t.base.addr = addr;
    t.base.length = 0;              // NO TX data
    t.base.rxlength = rx_size * 8;  // how many bits to read after dummy
    t.base.tx_buffer = NULL;
    t.base.rx_buffer = rx;

    // service phases we manage at the level of TRANSACTION
    t.base.flags = SPI_TRANS_VARIABLE_CMD | SPI_TRANS_VARIABLE_ADDR | SPI_TRANS_VARIABLE_DUMMY;
    if (trans_flags & 0x1) {  // bit0 -> CS keep active
        t.base.flags |= SPI_TRANS_CS_KEEP_ACTIVE;
    }

    t.command_bits = cmd_bits;   // 0..16
    t.address_bits = addr_bits;  // 0..32
    t.dummy_bits = dummy_bits;   // 0..255 (usually multiple of 8)

    xSemaphoreTake(dev->lock, portMAX_DELAY);
    esp_err_t r = spi_device_transmit(dev->dev, (spi_transaction_t *)&t);
    xSemaphoreGive(dev->lock);
    return r;
}
