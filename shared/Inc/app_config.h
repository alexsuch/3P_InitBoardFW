
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __APP_CONFIG_H
#define __APP_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "init_brd.h"

const config_data_t* AppConfig_GetConfiguration(void);
bool AppConfig_SaveConfiguration(config_data_t* config);
bool AppConfig_ApplyDefaultConfig(void);
bool AppGonfig_IsAllElements(uint8_t* ptr, uint16_t len, uint8_t val);
#ifdef __cplusplus
}
#endif

#endif /* __APP_CONFIG_H */
