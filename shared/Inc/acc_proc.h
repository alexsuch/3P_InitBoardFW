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

/* Accelerometer resolution depends on sensor type and configuration */
#if (LIS2DH12_ACC_ENABLE == 1u)
	/* LIS2DH12: ±16g, 12-bit HR mode → 12 mg/LSB */
	#define ACC_RES_MG_PER_BIT                 (12u)
	#define ACC_SQRT_TRESHOLD                  ((ACC_HIT_THRESHOLD_MG/ACC_RES_MG_PER_BIT) * (ACC_HIT_THRESHOLD_MG/ACC_RES_MG_PER_BIT))
#elif (LSM6DS3_ACC_ENABLE == 1u)
	/* LSM6DS3: ±16g, 16-bit mode → 0.488 mg/LSB
	 * Use scaled integer math: multiply by 1000/488 ≈ 2.049
	 * Threshold_LSB = Threshold_mg × (1000/488)
	 * For 12000mg: 12000 × 1000 / 488 ≈ 24590 LSB */
	#define ACC_LSB_SCALE_NUM                  (1000u)
	#define ACC_LSB_SCALE_DEN                  (488u)
	#define ACC_THRESHOLD_LSB                  ((ACC_HIT_THRESHOLD_MG * ACC_LSB_SCALE_NUM) / ACC_LSB_SCALE_DEN)
	#define ACC_SQRT_TRESHOLD                  (ACC_THRESHOLD_LSB * ACC_THRESHOLD_LSB)
#else
	/* Default fallback */
	#define ACC_RES_MG_PER_BIT                 (12u)
	#define ACC_SQRT_TRESHOLD                  ((ACC_HIT_THRESHOLD_MG/ACC_RES_MG_PER_BIT) * (ACC_HIT_THRESHOLD_MG/ACC_RES_MG_PER_BIT))
#endif

#if ACC_SHAKE_DETECTION_ENABLE
#define ACC_SHAKE_SQRT_TRESHOLD            ((ACC_SHAKE_THRESHOLD_MG/ACC_RES_MG_PER_BIT) * (ACC_SHAKE_THRESHOLD_MG/ACC_RES_MG_PER_BIT))
#endif /* ACC_SHAKE_DETECTION_ENABLE */

#define APPROX_G_TO_RAW_FACTOR             0.9f      // компенсація похибки апроксимації
#define ACC_NET_TRESHOLD                   ((ACC_HIT_NET_DIFF_THRESHOLD_MG/ACC_RES_MG_PER_BIT) * ACC_NET_BUFF_SIZE * APPROX_G_TO_RAW_FACTOR)



typedef struct
{
	// processing data
	uint8_t idx;
	int16_t x_buff[ACC_BUFF_SIZE];
	int16_t y_buff[ACC_BUFF_SIZE];
	int16_t z_buff[ACC_BUFF_SIZE];
	int32_t x_sum;
	int32_t y_sum;
	int32_t z_sum;
	uint8_t validCount;

#if LSM6DS3_ACC_ENABLE
	// Gyroscope data processing (only for LSM6DS3)
	int16_t gyro_x_buff[ACC_BUFF_SIZE];
	int16_t gyro_y_buff[ACC_BUFF_SIZE];
	int16_t gyro_z_buff[ACC_BUFF_SIZE];
	int32_t gyro_x_sum;
	int32_t gyro_y_sum;
	int32_t gyro_z_sum;
	// Averaged gyroscope values (accessible from outside)
	int16_t gyro_x_avg;
	int16_t gyro_y_avg;
	int16_t gyro_z_avg;
#endif /* LSM6DS3_ACC_ENABLE */

#if NET_DETECTION_ENABLE
    int16_t net_x_raw_buff[ACC_NET_BUFF_SIZE];
    int16_t net_y_raw_buff[ACC_NET_BUFF_SIZE];
    int16_t net_z_raw_buff[ACC_NET_BUFF_SIZE];

    int32_t net_x_sum;
    int32_t net_y_sum;
    int32_t net_z_sum;

    int32_t net_x_sum_buff[ACC_NET_BUFF_SIZE];
    int32_t net_y_sum_buff[ACC_NET_BUFF_SIZE];
    int32_t net_z_sum_buff[ACC_NET_BUFF_SIZE];

    uint8_t net_validCount;
    uint8_t net_idx;
    uint8_t net_check_counter;
#endif
	// handling data
	volatile bool data_ready;
	uint8_t init_skip_counts;
	// hit detect status
	bool hitDetected;
	volatile bool hit_detection_enabled;
	// move detect status
	volatile bool move_detection_enabled;
	bool move_detected;
	uint8_t move_detection_retry_cnt;
	uint32_t move_threshold;
#if ACC_SHAKE_DETECTION_ENABLE
	// shake detect status
	volatile bool shake_detection_enabled;
#endif /* ACC_SHAKE_DETECTION_ENABLE */

}accProcStatus_t;

void AccProc_Reset (app_cbk_fn sys_cbk);
void AccProc_Task (void);
void AccProc_HitDetectionStart (void);
void AccProc_Stop (void);
void AccProc_MoveDetectionStart (uint32_t move_threshold);
#if ACC_SHAKE_DETECTION_ENABLE
void AccProc_ShakeDetectionStart (void);
#endif /* ACC_SHAKE_DETECTION_ENABLE */

#if LSM6DS3_ACC_ENABLE
/* Get averaged gyroscope data (only for LSM6DS3) */
void AccProc_GetGyroAverage(int16_t* gyro_x_avg, int16_t* gyro_y_avg, int16_t* gyro_z_avg);
#endif /* LSM6DS3_ACC_ENABLE */

#ifdef __cplusplus
}
#endif

#endif /* INC_ACC_PROC_H_ */
