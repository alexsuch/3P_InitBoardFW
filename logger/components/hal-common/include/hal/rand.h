#pragma once

#include <stdint.h>

/**
 * @brief Generate a 32-bit random value
 *
 * @warning Security caveat: The returned value may not be fully secure when
 * WiFi and Bluetooth are both disabled, as esp_random() can be weaker in that
 * state. Callers must not assume cryptographic-grade randomness in this scenario.
 *
 * @return uint32_t A 32-bit random value
 */
uint32_t hal_rand_u32(void);
