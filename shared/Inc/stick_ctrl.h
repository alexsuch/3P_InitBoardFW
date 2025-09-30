
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STICK_CTRL_H
#define __STICK_CTRL_H
   
#ifdef __cplusplus
extern "C" {
#endif

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "prj_config.h"
#include "init_brd.h"


#define PWM_CH_1                        (0u)
#define PWM_CH_2                        (1u)

#define FC_PWM_DETECTION_NUM  (2u)
#define FC_LOST_DETECTION_NUM (8u)
// below value must be more than FC_PWM_DETECTION_NUM
#define FC_PWM_SIGNAL_INVALID_MAX_NUM (10u)
// below value must be more than FC_PWM_DETECTION_NUM
#define FC_PWM_PERIOD_INVALID_MAX_NUM (10u)
// all below values in 10us scale
#define FC_PWM_PERIOD_MIN (1800u)
#define FC_PWM_PERIOD_MAX (2200u)
#define FC_PWM_POS_1_MIN (90u)
#define FC_PWM_POS_1_MAX (110u)
#define FC_PWM_POS_2_MIN (140u)
#define FC_PWM_POS_2_MAX (160u)
#define FC_PWM_POS_3_MIN (190u)
#define FC_PWM_POS_3_MAX (220u)
#define FC_PWM_POS_LOST_MIN (115u)
#define FC_PWM_POS_LOST_MAX (135u)

typedef enum
{
	STICK_STATE_NONE,
	STICK_STATE_NO_STICK,
	STICK_STATE_LOST,
	STICK_STATE_ACTIVE_POS_1,
	STICK_STATE_ACTIVE_POS_2,
	STICK_STATE_ACTIVE_POS_3,
}stick_state_t;

typedef struct
{
	stick_state_t state;
	volatile uint16_t countVal;
	stick_state_t new_state;
	stick_state_t old_state;
	bool is_stick_scan_enabled;
	volatile uint16_t fallingIdx;
	uint16_t processedIdx;
	uint16_t signalInvalidCount;
	uint16_t periodInvalidCount;
	volatile uint16_t lastRisingVal;
	volatile uint16_t periodValue;
	volatile uint16_t signalValue;
	uint8_t pos1Counts;
	uint8_t pos2Counts;
	uint8_t pos3Counts;
	uint8_t lostCounts;
	bool process_flag;
}stickStatus_t;

void Stick_Task (void);
#if (CONTROL_MODE == PWM_CTRL_SUPP)
#if MINING_MODE_SUPP
void Stick_Reset (app_cbk_fn sys_cbk, bool mining_mode);
#else
void Stick_Reset (app_cbk_fn sys_cbk);
#endif
void Stick_Deinit (void);
void Stick_ProcessEdgeCbk(system_evt_t evt, uint32_t usr_data);
#endif /* PWM_CTRL_SUPP */

#ifdef __cplusplus
}
#endif

#endif /* __STICK_CTRL_H */
