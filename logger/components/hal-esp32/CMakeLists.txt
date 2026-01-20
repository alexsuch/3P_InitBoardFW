idf_component_register(
    SRCS "adc.c" "gpio.c" "i2c.c" "init.c" "md5.c" "mutex.c" "pwm.c" "rand.c" "sdcard.c" "spi.c" "storage.c" "ws2812.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "include"
    REQUIRES "hal-common" "driver" "esp_adc" "mbedtls" "nvs_flash" "fatfs" "console"
)
