
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LIS2DH12_H
#define __LIS2DH12_H
   
#ifdef __cplusplus
extern "C" {
#endif

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "init_brd.h"

#define LIS2DH12_MAX_BUF_SIZE   (7u)
#define LIS2DH12_MAX_RETRY_CNT  (10u)

#define LIS2DH12_MAX_TIMEOUT_CNT      (100u)

/*************************** REGISTER MAP ***************************/
#define LIS2DH12_WHO_AM_I              0x0FU
/** Device Identification (Who am I) **/
#define LIS2DH12_ID                    0x33U
#define LIS2DH12_CTRL_REG1             0x20U
#if ACC_12_BIT
#define LIS2DH12_CTRL_REG1_VAL         0x97U //Normal mode  + 1.344 kHz
#define LIS2DH12_CTRL_REG1_MOVE_VAL    0x27U //Normal mode  + 10Hz
#else
#define LIS2DH12_CTRL_REG1_VAL         0x8FU //Enable Low power + 1620Hz  //0x8FU
#endif
#define LIS2DH12_CTRL_REG4             0x23U
//#define LIS2DH12_CTRL_REG4_VAL         0x30U //16G 10bit
#define LIS2DH12_CTRL_REG4_VAL         0xB8U //16G 12bit
#define LIS2DH12_CTRL_REG4_MOVE_VAL    0xA8U //8G 12bit
#define LIS2DH12_CTRL_REG3             0x22U
#define LIS2DH12_CTRL_REG3_VAL         0x10U //Enable data available interrupt
#define LIS2DH12_CTRL_REG5             0x24U
#define LIS2DH12_CTRL_REG5_VAL         0x80U //Reboot memory content




typedef enum
{
	LIS2DH12_STATE_IDLE,
	LIS2DH12_STATE_CHECK_ID,
	LIS2DH12_STATE_SET_HIT_PARAMS,
	LIS2DH12_STATE_SET_MOVE_PARAMS,
	LIS2DH12_STATE_GET_DATA,
	LIS2DH12_STATE_ERROR

}lis2dh12_state_t;
    
typedef struct
{
	lis2dh12_state_t state;
	bool wait_flag;
	bool read_flag;
	uint8_t retry_cnt;
	uint8_t rd_buff[LIS2DH12_MAX_BUF_SIZE];
	/* Below values are in format -16g->0g->16g -> -128->0->127 */
	int16_t x_axis;
	int16_t y_axis;
	int16_t z_axis;
}lis2dh12_Status_t;

void Lis2dh12_Task (void);
void Lis2dh12_Reset (app_cbk_fn sys_cbk, int16_t** x_axis, int16_t** y_axis, int16_t** z_axis);
void Lis2dh12_ResetData(void);
void Lis2dh12_Deinit (void);
void Lis2dh12_GotoMoveMode (void);
void Lis2dh12_GotoHitMode (void);
#ifdef __cplusplus
}
#endif

#endif /* __LIS2DH12_H */
