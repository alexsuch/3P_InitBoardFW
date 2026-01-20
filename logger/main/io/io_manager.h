#pragma once

#include <hal/gpio_base.h>

#include "hal_spi.h"

typedef struct io_manager_s {
    hal_spi_bus_t accel_spi_bus;
    hal_spi_bus_t sdcard_spi_bus;
    bool initialized;
    bool sd_card_ok;
} io_manager_t;

hal_err_t io_manager_init(io_manager_t *iom);

void io_manager_shutdown(io_manager_t *iom);
