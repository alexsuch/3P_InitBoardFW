
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __APP_H
#define __APP_H
   
#ifdef __cplusplus
extern "C" {
#endif

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "init_brd.h"

#define SAFE_TIMEOUT_RAPID_INDICATION_START_SEC       (0u)
#define SAFE_TIMER_LAST_SEC_SOLID_SIGNAL_CNT          (1u)
#define FUSE_DETECT_RETRY                             (2u)
#define ADC_MEASURE_RETRY_CNT                         (5u)

void App_Task (void);
void App_InitRun (void);
void App_SetSafe (bool ind_enable);
uint8_t*  App_GetConfiguration(uint8_t* size);
bool App_SetConfiguration(uint8_t* data, uint8_t size);
uint8_t App_GetAppStatusConfiguration(void);
void App_RefreshConfig (void);

#ifdef __cplusplus
}
#endif

#endif /* __LIS2DH12_H */
