#include <hal/i2c.h>

hal_err_t hal_i2c_bus_init(hal_i2c_bus_t bus, hal_gpio_t sda, hal_gpio_t scl, uint32_t freq_hz) {
    esp_err_t err;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = sda;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = scl;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = freq_hz;
    conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;
    if ((err = i2c_param_config(bus, &conf)) != ESP_OK) {
        return err;
    }
    // Buffers are only used in slave mode
    return i2c_driver_install(bus, conf.mode, 0, 0, 0);
}

hal_err_t hal_i2c_bus_deinit(hal_i2c_bus_t bus) { return i2c_driver_delete(bus); }

hal_err_t hal_i2c_cmd_init(hal_i2c_cmd_t *cmd) {
    if (!cmd) return ESP_ERR_INVALID_ARG;
    cmd->handle = i2c_cmd_link_create();
    return cmd->handle ? ESP_OK : ESP_ERR_NO_MEM;
}

hal_err_t hal_i2c_cmd_destroy(hal_i2c_cmd_t *cmd) {
    if (!cmd || !cmd->handle) return ESP_ERR_INVALID_ARG;
    i2c_cmd_link_delete(cmd->handle);
    cmd->handle = NULL;
    return ESP_OK;
}

hal_err_t hal_i2c_cmd_master_start(hal_i2c_cmd_t *cmd) { return (cmd && cmd->handle) ? i2c_master_start(cmd->handle) : ESP_ERR_INVALID_ARG; }

hal_err_t hal_i2c_cmd_master_stop(hal_i2c_cmd_t *cmd) { return (cmd && cmd->handle) ? i2c_master_stop(cmd->handle) : ESP_ERR_INVALID_ARG; }

hal_err_t hal_i2c_cmd_master_write_byte(hal_i2c_cmd_t *cmd, uint8_t data, bool ack_en) {
    return (cmd && cmd->handle) ? i2c_master_write_byte(cmd->handle, data, ack_en) : ESP_ERR_INVALID_ARG;
}

hal_err_t hal_i2c_cmd_master_write(hal_i2c_cmd_t *cmd, uint8_t *data, size_t data_len, bool ack_en) {
    return (cmd && cmd->handle && data) ? i2c_master_write(cmd->handle, data, data_len, ack_en) : ESP_ERR_INVALID_ARG;
}

hal_err_t hal_i2c_cmd_master_read_byte(hal_i2c_cmd_t *cmd, uint8_t *data, uint8_t ack) {
    return (cmd && cmd->handle && data) ? i2c_master_read_byte(cmd->handle, data, ack) : ESP_ERR_INVALID_ARG;
}

hal_err_t hal_i2c_cmd_master_read(hal_i2c_cmd_t *cmd, uint8_t *data, size_t data_len, uint8_t ack) {
    return (cmd && cmd->handle && data) ? i2c_master_read(cmd->handle, data, data_len, ack) : ESP_ERR_INVALID_ARG;
}

hal_err_t hal_i2c_cmd_master_exec(hal_i2c_bus_t bus, hal_i2c_cmd_t *cmd) {
    // Increased timeout for better reliability with sensors
    return (cmd && cmd->handle) ? i2c_master_cmd_begin(bus, cmd->handle, pdMS_TO_TICKS(100)) : ESP_ERR_INVALID_ARG;
}