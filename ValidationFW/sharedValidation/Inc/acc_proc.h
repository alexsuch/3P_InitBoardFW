/*
 * acc_proc.h
 *
 *  Created on: Jul 29, 2024
 *      Author: Volodymyr
 */

#ifndef INC_ACC_PROC_H_
#define INC_ACC_PROC_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "init_brd.h"
#include "prj_config.h"

#define MOVE_DETECT_INIT_DELAY_SKIP_CNT    (5u)

#if ACC_12_BIT
#define ACC_RES_MG_PER_BIT                 (12u)
#else
#define ACC_RES_MG_PER_BIT                 (192u)
#endif

#define ACC_TEST_LOW_THRESHOLD_MG              (700u)
#define ACC_TEST_HIGH_THRESHOLD_MG             (1300u)

#define ACC_SQRT_LOW_TRESHOLD              ((ACC_TEST_LOW_THRESHOLD_MG/ACC_RES_MG_PER_BIT) * (ACC_TEST_LOW_THRESHOLD_MG/ACC_RES_MG_PER_BIT))
#define ACC_SQRT_HIGH_TRESHOLD              ((ACC_TEST_HIGH_THRESHOLD_MG/ACC_RES_MG_PER_BIT) * (ACC_TEST_HIGH_THRESHOLD_MG/ACC_RES_MG_PER_BIT))

typedef struct
{
	// processing data
	uint8_t idx;
	int16_t x_buff[ACC_BUFF_SIZE + 1];
	int16_t y_buff[ACC_BUFF_SIZE + 1];
	int16_t z_buff[ACC_BUFF_SIZE + 1];
	int32_t x_sum;
	int32_t y_sum;
	int32_t z_sum;
	// handling data
	bool data_ready;
	uint8_t init_skip_counts;
	// hit detect status
	bool hitDetected;
	bool hit_detection_enabled;
	// move detect status
	bool move_detection_enabled;
	bool move_detected;
	uint8_t move_detection_retry_cnt;
	uint32_t move_threshold;

}accProcStatus_t;

void AccProc_Reset (app_cbk_fn sys_cbk);
void AccProc_Task (void);
void AccProc_HitDetectionStart (void);
void AccProc_Stop (void);
void AccProc_MoveDetectionStart (uint32_t move_threshold);

#ifdef __cplusplus
}
#endif

#endif /* INC_ACC_PROC_H_ */
