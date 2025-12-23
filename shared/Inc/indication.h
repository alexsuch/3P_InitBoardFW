
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __INDICATION_H
#define __INDICATION_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "init_brd.h"
#include "solution_wrapper.h"

#define IND_NO_ERROR_FOUND (0xFFu)
#define IND_RAPID_TICK_PER_SEC_NUBM (4u)
#define IND_RAPID_TICK_REPIOD (1000u / IND_RAPID_TICK_PER_SEC_NUBM)

#define STATUS_TYPE (0u)
#define ERROR_TYPE (1u)

typedef enum {
    IND_PATTERN_NONE,
    IND_PATTERN_SOLID,
    IND_PATTERN_SINGLE_SHORT,
    IND_PATTERN_SINGLE_LONG,
    IND_PATTERN_VERY_LONG,
    IND_PATTERN_SINGLE_5_SEC,
    IND_PATTERN_PERIODIC_50_50,
    IND_PATTERN_PERIODIC_1_SHORT_PLUS_PAUSE,
    IND_PATTERN_PERIODIC_2_SHORT_PLUS_PAUSE,
    IND_PATTERN_PERIODIC_2_SHORT_PLUS_LONG_PAUSE,
} ind_pattern_t;

typedef struct {
    uint32_t err_code_mask;
    uint32_t warn_code_mask;
    ind_status_t curr_status;
    volatile uint8_t stat_retry_cnt;
    volatile uint8_t err_retry_cnt;
    ind_pattern_t stat_pattern;
    ind_pattern_t err_pattern;
    volatile bool handle_err_tmr_cbk;
    volatile bool handle_stat_tmr_cbk;
    uint8_t* system_error_code_ptr;  // Pointer to system error code in app layer
} indication_status_t;

void Indication_Task(void);
void Indication_Reset(uint8_t* system_error_code_ptr);
void Indication_SetError(err_code_t err_code);
void Indication_SetStatus(ind_status_t status_code, uint32_t user_data);
void Indication_ClearError(err_code_t err_code);

#ifdef __cplusplus
}
#endif

#endif /* __INDICATION_H */
