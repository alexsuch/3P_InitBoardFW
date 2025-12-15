
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LSM6DS3_H
#define __LSM6DS3_H
   
#ifdef __cplusplus
extern "C" {
#endif

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "prj_config.h"

#if (LSM6DS3_ACC_ENABLE == 1u)

#include "init_brd.h"

#define LSM6DS3_MAX_BUF_SIZE   (13u) /* 1 dummy + 6 gyro + 6 accel */
#define LSM6DS3_MAX_RETRY_CNT  (10u)

#define LSM6DS3_MAX_TIMEOUT_CNT      (100u)

/*************************** REGISTER MAP ***************************/
#define LSM6DS3_WHO_AM_I              0x0FU
/** Device Identification (Who am I) **/
#define LSM6DS3_ID                    0x69U

/* Control registers */
#define LSM6DS3_CTRL1_XL              0x10U  /* Accelerometer control */
#define LSM6DS3_CTRL2_G               0x11U  /* Gyroscope control */
#define LSM6DS3_CTRL3_C               0x12U  /* Common control */
#define LSM6DS3_CTRL4_C               0x13U
#define LSM6DS3_CTRL5_C               0x14U
#define LSM6DS3_CTRL6_C               0x15U
#define LSM6DS3_CTRL7_G               0x16U
#define LSM6DS3_CTRL8_XL              0x17U
#define LSM6DS3_CTRL9_XL              0x18U
#define LSM6DS3_CTRL10_C              0x19U

/* Status register */
#define LSM6DS3_STATUS_REG            0x1EU

/* Accelerometer output registers */
#define LSM6DS3_OUTX_L_XL             0x28U
#define LSM6DS3_OUTX_H_XL             0x29U
#define LSM6DS3_OUTY_L_XL             0x2AU
#define LSM6DS3_OUTY_H_XL             0x2BU
#define LSM6DS3_OUTZ_L_XL             0x2CU
#define LSM6DS3_OUTZ_H_XL             0x2DU

/* Gyroscope output registers */
#define LSM6DS3_OUTX_L_G              0x22U
#define LSM6DS3_OUTX_H_G              0x23U
#define LSM6DS3_OUTY_L_G              0x24U
#define LSM6DS3_OUTY_H_G              0x25U
#define LSM6DS3_OUTZ_L_G              0x26U
#define LSM6DS3_OUTZ_H_G              0x27U

/* Temperature output registers */
#define LSM6DS3_OUT_TEMP_L            0x20U
#define LSM6DS3_OUT_TEMP_H            0x21U

/* Interrupt configuration */
#define LSM6DS3_INT1_CTRL             0x0DU
#define LSM6DS3_INT2_CTRL             0x0EU

/* FIFO registers */
#define LSM6DS3_FIFO_CTRL1            0x06U
#define LSM6DS3_FIFO_CTRL2            0x07U
#define LSM6DS3_FIFO_CTRL3            0x08U
#define LSM6DS3_FIFO_CTRL4            0x09U
#define LSM6DS3_FIFO_CTRL5            0x0AU

/*************************** Configuration Values ***************************/
/* CTRL1_XL: Accelerometer configurations */
/* Dynamic ODR based on project configuration */
#if defined(LSM6DS3_ODR_VALUE)
    #define LSM6DS3_CTRL1_XL_HIT_VAL      ((LSM6DS3_ODR_VALUE & 0xF0) | 0x04U)  /* Configured Hz, ±16g */
    #define LSM6DS3_CTRL1_XL_MOVE_VAL     ((LSM6DS3_ODR_VALUE & 0xF0) | 0x00U)  /* Configured Hz, ±2g */
#else
    #define LSM6DS3_CTRL1_XL_HIT_VAL      0xA4U  /* Default: 6.66 kHz, ±16g (ODR=1010, FS=01) */
    #define LSM6DS3_CTRL1_XL_MOVE_VAL     0x40U  /* Default: 104 Hz, ±2g */
#endif

/* CTRL2_G: Gyroscope configurations */
/* Dynamic ODR based on project configuration */
#if defined(LSM6DS3_ODR_VALUE)
    #define LSM6DS3_CTRL2_G_HIT_VAL       ((LSM6DS3_ODR_VALUE & 0xF0) | 0x0CU)  /* Configured Hz, 2000 dps */
    #define LSM6DS3_CTRL2_G_MOVE_VAL      ((LSM6DS3_ODR_VALUE & 0xF0) | 0x00U)  /* Configured Hz, 245 dps */
#else
    #define LSM6DS3_CTRL2_G_HIT_VAL       0xACU  /* Default: 6.66 kHz, 2000 dps (ODR=1010, FS=11) */
    #define LSM6DS3_CTRL2_G_MOVE_VAL      0x40U  /* Default: 104 Hz, 245 dps (ODR=0100, FS=00) */
#endif

/* CTRL3_C: Common configurations */
#define LSM6DS3_CTRL3_C_VAL           0x44U  /* BDU=1, IF_INC=1 - block data update */
#define LSM6DS3_CTRL3_C_RESET_VAL     0x01U  /* Software reset */

/* INT1_CTRL: Interrupt configurations */
#define LSM6DS3_INT1_CTRL_DRDY_VAL    0x01U  /* Enable INT1_DRDY_XL - accelerometer data ready on INT1 */

/* TAP_CFG: Interrupt/Latch configuration */
#define LSM6DS3_TAP_CFG               0x58U
#define LSM6DS3_TAP_CFG_VAL           0x00U  /* LIR=0: interrupt not latched */

/* Accelerometer range values */
#define LSM6DS3_ACCEL_RANGE_2G        0x00U
#define LSM6DS3_ACCEL_RANGE_4G        0x08U  /* FS_XL = 10b */
#define LSM6DS3_ACCEL_RANGE_8G        0x0CU  /* FS_XL = 11b */
#define LSM6DS3_ACCEL_RANGE_16G       0x04U  /* FS_XL = 01b */

/* Gyroscope range values */
#define LSM6DS3_GYRO_RANGE_245DPS     0x00U
#define LSM6DS3_GYRO_RANGE_500DPS     0x04U  /* FS_G = 01b */
#define LSM6DS3_GYRO_RANGE_1000DPS    0x08U  /* FS_G = 10b */
#define LSM6DS3_GYRO_RANGE_2000DPS    0x0CU  /* FS_G = 11b */

/* Output Data Rate (ODR) values */
#define LSM6DS3_ODR_POWER_DOWN        0x00U
#define LSM6DS3_ODR_13HZ              0x10U
#define LSM6DS3_ODR_26HZ              0x20U
#define LSM6DS3_ODR_52HZ              0x30U
#define LSM6DS3_ODR_104HZ             0x40U
#define LSM6DS3_ODR_208HZ             0x50U
#define LSM6DS3_ODR_416HZ             0x60U
#define LSM6DS3_ODR_833HZ             0x70U
#define LSM6DS3_ODR_1666HZ            0x80U

typedef enum
{
	LSM6DS3_STATE_IDLE,
	LSM6DS3_STATE_CHECK_ID,
	LSM6DS3_STATE_SET_HIT_PARAMS,
	LSM6DS3_STATE_SET_MOVE_PARAMS,
	LSM6DS3_STATE_SET_SHAKE_PARAMS,
	LSM6DS3_STATE_GET_DATA,
	LSM6DS3_STATE_ERROR

}lsm6ds3_state_t;
    
typedef struct
{
	lsm6ds3_state_t state;
	volatile bool wait_flag;
	bool read_flag;
	uint8_t retry_cnt;
	uint8_t rd_buff[LSM6DS3_MAX_BUF_SIZE];
	bool reset_done;  /* Track software reset completion */
	
	/* Accelerometer values in format -16g->0g->16g -> -32768->0->32767 */
	int16_t x_axis;
	int16_t y_axis;
	int16_t z_axis;
	
	/* Gyroscope values in format -2000dps->0->2000dps -> -32768->0->32767 */
	int16_t gyro_x;
	int16_t gyro_y;
	int16_t gyro_z;
	
	/* Temperature (optional) */
	int16_t temperature;
	
}lsm6ds3_Status_t;

void Lsm6ds3_Task (void);
void Lsm6ds3_Reset (app_cbk_fn sys_cbk, int16_t** x_axis, int16_t** y_axis, int16_t** z_axis);
void Lsm6ds3_ResetData(void);
void Lsm6ds3_Deinit (void);
void Lsm6ds3_GotoMoveMode (void);
void Lsm6ds3_GotoHitMode (void);
#if ACC_SHAKE_DETECTION_ENABLE
void Lsm6ds3_GotoShakeMode (void);
#endif /* ACC_SHAKE_DETECTION_ENABLE */

/* Additional functions for gyroscope access */
void Lsm6ds3_GetGyroData(int16_t** gyro_x, int16_t** gyro_y, int16_t** gyro_z);

/**
 * @brief Start reading accelerometer and gyroscope data
 *
 * Transitions the LSM6DS3 state machine from current state to LSM6DS3_STATE_GET_DATA,
 * which will begin polling for sensor data on next Lsm6ds3_Task() invocation.
 *
 * This is a convenience function that can be called to initiate continuous data
 * reading after the sensor has been initialized (after Lsm6ds3_GotoHitMode(),
 * Lsm6ds3_GotoMoveMode(), etc.).
 *
 * Returns: void
 *
 * Usage:
 *   Lsm6ds3_GotoHitMode();        // Initialize sensor
 *   // ... wait for ACC_EVT_HIT_INIT_OK callback ...
 *   Lsm6ds3_StartReadData();      // Begin continuous data reading
 */
void Lsm6ds3_StartReadData(void);

#endif /* LSM6DS3_ACC_ENABLE */

#ifdef __cplusplus
}
#endif

#endif /* __LSM6DS3_H */
