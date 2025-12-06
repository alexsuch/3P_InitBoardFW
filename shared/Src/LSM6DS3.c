
#include "prj_config.h"

#if (LSM6DS3_ACC_ENABLE == 1u)

#include <LSM6DS3.h>
#include <stdlib.h>
#include <stdio.h>
#include "init_brd.h"
#include "timer.h"
#include "solution_wrapper.h"
#include "acc_proc.h"

static lsm6ds3_Status_t lsm6ds3_Stat;
static app_cbk_fn  lsm6ds3_sys_cbk = NULL;
static volatile uint16_t timeoutCnt = 0u;

static void Lsm6ds3_ErrHandler (void);

static void Lsm6ds3_ResetTimeout (void)
{
	timeoutCnt = 0;
}

void Lsm6ds3_Reset (app_cbk_fn sys_cbk, int16_t** x_axis, int16_t** y_axis, int16_t** z_axis)
{
	memset(&lsm6ds3_Stat, 0u, sizeof(lsm6ds3_Stat));

	lsm6ds3_sys_cbk = sys_cbk;

	/* Set pointers to the x/y/z accelerometer values */
	*x_axis = &lsm6ds3_Stat.x_axis;
	*y_axis = &lsm6ds3_Stat.y_axis;
	*z_axis = &lsm6ds3_Stat.z_axis;

	/* Set initial state */
	lsm6ds3_Stat.state = LSM6DS3_STATE_CHECK_ID;
}

void Lsm6ds3_GetGyroData(int16_t** gyro_x, int16_t** gyro_y, int16_t** gyro_z)
{
	/* Set pointers to the gyroscope values */
	*gyro_x = &lsm6ds3_Stat.gyro_x;
	*gyro_y = &lsm6ds3_Stat.gyro_y;
	*gyro_z = &lsm6ds3_Stat.gyro_z;
}

void Lsm6ds3_GotoHitMode (void)
{
	/* Reset the flag for new initialization */
	lsm6ds3_Stat.reset_done = false;
	/* Set hit detection parameters */
	lsm6ds3_Stat.state = LSM6DS3_STATE_SET_HIT_PARAMS;
}

void Lsm6ds3_GotoMoveMode (void)
{
	/* Reset the flag for new initialization */
	lsm6ds3_Stat.reset_done = false;
	/* Set move detection parameters */
	lsm6ds3_Stat.state = LSM6DS3_STATE_SET_MOVE_PARAMS;
}

#if ACC_SHAKE_DETECTION_ENABLE
void Lsm6ds3_GotoShakeMode (void)
{
	/* Set shake detection parameters - same as hit detection */
	lsm6ds3_Stat.state = LSM6DS3_STATE_SET_SHAKE_PARAMS;
}
#endif /* ACC_SHAKE_DETECTION_ENABLE */

void Lsm6ds3_Deinit (void)
{
	memset(&lsm6ds3_Stat, 0u, sizeof(lsm6ds3_Status_t));
	Timer_Stop (ACC_TIMEOUT_TMR);
}

static void Lsm6ds3_PauseCbk (uint8_t timer_id)
{
    (void)timer_id;

    lsm6ds3_Stat.wait_flag = false;
}

static void Lsm6ds3_ErrHandler (void)
{
	/* Handle potential error */
	if (lsm6ds3_Stat.retry_cnt++ > LSM6DS3_MAX_RETRY_CNT)
	{
		/* Report Error */
		if (lsm6ds3_sys_cbk != NULL)
		{
			lsm6ds3_sys_cbk(SYSTEM_EVT_ERROR, 0u);
		}
		lsm6ds3_Stat.wait_flag = true;
		lsm6ds3_Stat.retry_cnt = 0u;
		lsm6ds3_Stat.state = LSM6DS3_STATE_ERROR;
	}
	else
	{
		lsm6ds3_Stat.wait_flag = true;
		/* Start retry timer */
		Timer_Start (ACC_TIMEOUT_TMR, ACC_TIMEOUT_TMR_RETRY_PERIOD_MS, Lsm6ds3_PauseCbk);
	}
}

static void Lsm6ds3_CheckID (void)
{
	uint8_t rdVal = 0xFF;  /* Initialize with invalid value for debugging */
	bool stat = false;
	volatile bool spi_result = false;  /* DEBUG: check SPI function result */

	/* Read device id */
	spi_result = SpiReadRegister(LSM6DS3_WHO_AM_I, &rdVal, 1);
	
	if (spi_result != false)
	{
		/* DEBUG: Set breakpoint here and check rdVal */
		if (rdVal == LSM6DS3_ID)
		{
			/* Init is done */
			if (lsm6ds3_sys_cbk != NULL)
			{
				/* move to idle if response is correct */
				lsm6ds3_Stat.state = LSM6DS3_STATE_IDLE;
				lsm6ds3_Stat.retry_cnt = 0u;
				lsm6ds3_sys_cbk(SYSTEM_EVT_READY, ACC_EVT_INIT_OK);
				stat = true;
			}
		}
		else
		{
			/* DEBUG: Wrong ID received - check rdVal value */
			Test2Toggle();  /* Toggle LED to indicate wrong ID */
		}
	}
	else
	{
		/* DEBUG: SPI read failed completely */
		LedErrorSet(true);  /* Turn on error LED */
	}

	if (stat == false)
	{
		/* handle the error */
		Lsm6ds3_ErrHandler();
	}
}

static void Lsm6ds3_SetHitParams (void)
{
	uint8_t rdVal = 0u;
	bool stat = false;
#if 1
	/* Software reset first (only once) */
	if (!lsm6ds3_Stat.reset_done)
	{
		if (SpiWriteSingleRegister(LSM6DS3_CTRL3_C, LSM6DS3_CTRL3_C_RESET_VAL) != false)
		{
			lsm6ds3_Stat.reset_done = true;
			/* Wait for reset to complete */
			Timer_Start(ACC_TIMEOUT_TMR, 60, Lsm6ds3_PauseCbk);
			lsm6ds3_Stat.wait_flag = true;
			return;
		}
		else
		{
			/* Reset failed */
			Lsm6ds3_ErrHandler();
			return;
		}
	}
#endif

	/* Set Gyroscope FIRST: 6.66 kHz, 2000 dps */
	if (SpiWriteSingleRegister(LSM6DS3_CTRL2_G, LSM6DS3_CTRL2_G_HIT_VAL) != false)
	{
		if (SpiReadRegister(LSM6DS3_CTRL2_G, &rdVal, 1) != false)
		{
			if (rdVal == LSM6DS3_CTRL2_G_HIT_VAL)
			{
			    stat = true;
			}
		}
	}
	if (stat == false)
	{
		Lsm6ds3_ErrHandler();
		return;
	}
	else
	{
		stat = false;
	}

	/* Set Accelerometer: 6.66 kHz, ±16g */
	if (SpiWriteSingleRegister(LSM6DS3_CTRL1_XL, LSM6DS3_CTRL1_XL_HIT_VAL) != false)
	{
		/* Check if value was written */
		if (SpiReadRegister(LSM6DS3_CTRL1_XL, &rdVal, 1) != false)
		{
			if (rdVal == LSM6DS3_CTRL1_XL_HIT_VAL)
			{
			    stat = true;
			}
		}
	}
	if (stat == false)
	{
		Lsm6ds3_ErrHandler();
		return;
	}
	else
	{
		stat = false;
	}

	/* Set common control register: BDU=1 (Block Data Update), IF_INC=1 (auto-increment) */
	if (SpiWriteSingleRegister(LSM6DS3_CTRL3_C, LSM6DS3_CTRL3_C_VAL) != false)
	{
		if (SpiReadRegister(LSM6DS3_CTRL2_G, &rdVal, 1) != false)
		{
			if (rdVal == LSM6DS3_CTRL2_G_HIT_VAL)
			{
			    stat = true;
			}
		}
	}
	if (stat == false)
	{
		Lsm6ds3_ErrHandler();
		return;
	}
	else
	{
		stat = false;
	}

	/* Enable accelerometer axes (CTRL9_XL = 0x18) */
	if (SpiWriteSingleRegister(0x18, 0x38) != false)
	{
		/* Check if value was written */
		if (SpiReadRegister(0x18, &rdVal, 1) != false)
		{
			if (rdVal == 0x38)
			{
			    stat = true;
			}
		}
	}
	if (stat == false)
	{
		Lsm6ds3_ErrHandler();
		return;
	}
	else
	{
		stat = false;
	}

	/* Set CTRL7_G for gyroscope high-performance mode (0x16 = 0x00) */
	if (SpiWriteSingleRegister(0x16, 0x00) != false)
	{
		if (SpiReadRegister(0x16, &rdVal, 1) != false)
		{
			if (rdVal == 0x00)
			{
			    stat = true;
			}
		}
	}
	if (stat == false)
	{
		Lsm6ds3_ErrHandler();
		return;
	}
	else
	{
		stat = false;
	}

	/* Ensure CTRL4_C DRDY_MASK=0 (0x13 = 0x00) */
	if (SpiWriteSingleRegister(0x13, 0x00) != false)
	{
		/* Check if value was written */
		if (SpiReadRegister(0x13, &rdVal, 1) != false)
		{
			if (rdVal == 0x00)
			{
			    stat = true;
			}
		}
	}
	if (stat == false)
	{
		Lsm6ds3_ErrHandler();
		return;
	}
	else
	{
		stat = false;
	}

	/* Disable interrupts on INT1 (0x0D = 0x00) */
	if (SpiWriteSingleRegister(LSM6DS3_INT1_CTRL, LSM6DS3_INT1_CTRL_DRDY_VAL) != false)
	{
		/* Check if value was written */
		if (SpiReadRegister(LSM6DS3_INT1_CTRL, &rdVal, 1) != false)
		{
			if (rdVal == LSM6DS3_INT1_CTRL_DRDY_VAL)
			{
			    stat = true;
			}
		}
	}
	if (stat == false)
	{
		Lsm6ds3_ErrHandler();
		return;
	}

	/* Wait 20ms for sensor to stabilize after configuration */
	lsm6ds3_Stat.wait_flag = true;
	Timer_Start(ACC_TIMEOUT_TMR, 20, Lsm6ds3_PauseCbk);
	
	/* DEBUG: Read back CTRL2_G to verify gyroscope is enabled */
	uint8_t ctrl2g_readback = 0;
	SpiReadRegister(LSM6DS3_CTRL2_G, &ctrl2g_readback, 1);
	
	/* Read STATUS_REG (0x1E) to check data ready flags */
	uint8_t statusReg = 0;
	SpiReadRegister(0x1E, &statusReg, 1);

	/* Init is done - send appropriate callback based on current state */
	if (lsm6ds3_sys_cbk != NULL)
	{
#if ACC_SHAKE_DETECTION_ENABLE
		if (lsm6ds3_Stat.state == LSM6DS3_STATE_SET_SHAKE_PARAMS)
		{
			lsm6ds3_sys_cbk(SYSTEM_EVT_READY, ACC_EVT_SHAKE_INIT_OK);
		}
		else
#endif /* ACC_SHAKE_DETECTION_ENABLE */
		{
			lsm6ds3_sys_cbk(SYSTEM_EVT_READY, ACC_EVT_HIT_INIT_OK);
		}
	}

	lsm6ds3_Stat.retry_cnt = 0u;
	lsm6ds3_Stat.state = LSM6DS3_STATE_GET_DATA;
}

static void Lsm6ds3_SetMoveParams (void)
{
	uint8_t rdVal = 0u;
	bool stat = false;
#if 1
	/* Software reset first (only once) */
	if (!lsm6ds3_Stat.reset_done)
	{
		if (SpiWriteSingleRegister(LSM6DS3_CTRL3_C, LSM6DS3_CTRL3_C_RESET_VAL) != false)
		{
			lsm6ds3_Stat.reset_done = true;
			/* Wait 50ms for reset to complete */
			Timer_Start(ACC_TIMEOUT_TMR, 50, Lsm6ds3_PauseCbk);
			lsm6ds3_Stat.wait_flag = true;
			return;
		}
		else
		{
			/* Reset failed */
			Lsm6ds3_ErrHandler();
			return;
		}
	}
#endif

	/* Set common control register: BDU=1, IF_INC=1 */
	if (SpiWriteSingleRegister(LSM6DS3_CTRL3_C, LSM6DS3_CTRL3_C_VAL) != false)
	{
		if (SpiReadRegister(LSM6DS3_CTRL3_C, &rdVal, 1) != false)
		{
			if (rdVal == LSM6DS3_CTRL3_C_VAL)
			{
			    stat = true;
			}
		}
	}
	if (stat == false)
	{
		Lsm6ds3_ErrHandler();
		return;
	}
	else
	{
		stat = false;
	}

	/* Set Accelerometer: 104 Hz, ±2g */
	if (SpiWriteSingleRegister(LSM6DS3_CTRL1_XL, LSM6DS3_CTRL1_XL_MOVE_VAL) != false)
	{
		if (SpiReadRegister(LSM6DS3_CTRL1_XL, &rdVal, 1) != false)
		{
			if (rdVal == LSM6DS3_CTRL1_XL_MOVE_VAL)
			{
			    stat = true;
			}
		}
	}
	if (stat == false)
	{
		Lsm6ds3_ErrHandler();
		return;
	}
	else
	{
		stat = false;
	}

	/* Set Gyroscope: 104 Hz, 245 dps */
	if (SpiWriteSingleRegister(LSM6DS3_CTRL2_G, LSM6DS3_CTRL2_G_MOVE_VAL) != false)
	{
		if (SpiReadRegister(LSM6DS3_CTRL2_G, &rdVal, 1) != false)
		{
			if (rdVal == LSM6DS3_CTRL2_G_MOVE_VAL)
			{
			    stat = true;
			}
		}
	}
	if (stat == false)
	{
		Lsm6ds3_ErrHandler();
		return;
	}
	else
	{
		stat = false;
	}

	/* Enable accelerometer axes (CTRL9_XL = 0x18) */
	if (SpiWriteSingleRegister(0x18, 0x38) != false)
	{
		/* Check if value was written */
		if (SpiReadRegister(0x18, &rdVal, 1) != false)
		{
			if (rdVal == 0x38)
			{
			    stat = true;
			}
		}
	}
	if (stat == false)
	{
		Lsm6ds3_ErrHandler();
		return;
	}
	else
	{
		stat = false;
	}

	/* Set CTRL7_G for gyroscope high-performance mode (0x16 = 0x00) */
	if (SpiWriteSingleRegister(0x16, 0x00) != false)
	{
		if (SpiReadRegister(0x16, &rdVal, 1) != false)
		{
			if (rdVal == 0x00)
			{
			    stat = true;
			}
		}
	}
	if (stat == false)
	{
		Lsm6ds3_ErrHandler();
		return;
	}
	else
	{
		stat = false;
	}

	/* Ensure CTRL4_C DRDY_MASK=0 (0x13 = 0x00) */
	if (SpiWriteSingleRegister(0x13, 0x00) != false)
	{
		if (SpiReadRegister(0x13, &rdVal, 1) != false)
		{
			if (rdVal == 0x00)
			{
			    stat = true;
			}
		}
	}
	if (stat == false)
	{
		Lsm6ds3_ErrHandler();
		return;
	}
	else
	{
		stat = false;
	}

	/* Disable interrupts on INT1 (0x0D = 0x00) */
	if (SpiWriteSingleRegister(LSM6DS3_INT1_CTRL, LSM6DS3_INT1_CTRL_DRDY_VAL) != false)
	{
		if (SpiReadRegister(LSM6DS3_INT1_CTRL, &rdVal, 1) != false)
		{
			if (rdVal == LSM6DS3_INT1_CTRL_DRDY_VAL)
			{
			    stat = true;
			}
		}
	}

	if (stat == false)
	{
		Lsm6ds3_ErrHandler();
		return;
	}

	/* Wait 20ms for sensor to stabilize after configuration */
	lsm6ds3_Stat.wait_flag = true;
	Timer_Start(ACC_TIMEOUT_TMR, 20, Lsm6ds3_PauseCbk);
	
	/* Read STATUS_REG (0x1E) to clear any pending interrupts */
	uint8_t statusReg = 0;
	SpiReadRegister(0x1E, &statusReg, 1);

	/* Init is done */
	lsm6ds3_Stat.retry_cnt = 0u;
	lsm6ds3_Stat.state = LSM6DS3_STATE_GET_DATA;
	if (lsm6ds3_sys_cbk != NULL)
	{
		lsm6ds3_sys_cbk(SYSTEM_EVT_READY, ACC_EVT_MOVE_INIT_OK);
	}
}

static void Lsm6ds3_GetDataCbk (system_evt_t evt, uint32_t usr_data)
{
	/* Stop timeout timer */
	//Timer_Stop (ACC_TIMEOUT_TMR);
	
	if (evt == SYSTEM_EVT_READY)
	{
		/* We read starting from 0x22 (OUTX_L_G) with auto-increment via IF_INC, 13 bytes total */
		/* Read address is 0xA2 = 0x22 | 0x80 (READ bit, auto-increment via CTRL3_C.IF_INC) */
		/* Data format: 1 dummy byte + 6 gyro bytes + 6 accel bytes */
		
		/* Convert gyroscope data: signed 16bit */
		/* Data format: OUTX_L_G, OUTX_H_G, OUTY_L_G, OUTY_H_G, OUTZ_L_G, OUTZ_H_G */
		/* Note: With HAL_SPI_TransmitReceive, first byte [0] is dummy, data starts at [1] */
		lsm6ds3_Stat.gyro_x = (int16_t)(lsm6ds3_Stat.rd_buff[1] | (lsm6ds3_Stat.rd_buff[2] << 8));
		lsm6ds3_Stat.gyro_y = (int16_t)(lsm6ds3_Stat.rd_buff[3] | (lsm6ds3_Stat.rd_buff[4] << 8));
		lsm6ds3_Stat.gyro_z = (int16_t)(lsm6ds3_Stat.rd_buff[5] | (lsm6ds3_Stat.rd_buff[6] << 8));
		
		/* Convert accelerometer data: signed 16bit */
		/* Data format: OUTX_L_XL, OUTX_H_XL, OUTY_L_XL, OUTY_H_XL, OUTZ_L_XL, OUTZ_H_XL */
		lsm6ds3_Stat.x_axis = (int16_t)(lsm6ds3_Stat.rd_buff[7] | (lsm6ds3_Stat.rd_buff[8] << 8));
		lsm6ds3_Stat.y_axis = (int16_t)(lsm6ds3_Stat.rd_buff[9] | (lsm6ds3_Stat.rd_buff[10] << 8));
		lsm6ds3_Stat.z_axis = (int16_t)(lsm6ds3_Stat.rd_buff[11] | (lsm6ds3_Stat.rd_buff[12] << 8));
		
		/* Report data is ready */
		if (lsm6ds3_sys_cbk != NULL)
		{
			lsm6ds3_sys_cbk(SYSTEM_EVT_READY, ACC_EVT_DATA_READY);
		}
	}
	else
	{
		/* handle the error */
		Lsm6ds3_ErrHandler();
	}
}

static void Lsm6ds3_GetData (void)
{
	/* Read accelerometer and gyroscope data in one burst read
	 * LSM6DS3 supports auto-increment, so we can read all data at once
	 * Starting from OUTX_L_XL (0x28) to OUTZ_H_G (0x27) 
	 * Total: 6 bytes accel + 6 bytes gyro = 12 bytes
	 */
	if (SpiGetAccData (lsm6ds3_Stat.rd_buff, Lsm6ds3_GetDataCbk) != false)
	{
		lsm6ds3_Stat.retry_cnt = 0u;
		/* Start timeout counter */
		Lsm6ds3_ResetTimeout();
	}
	else
	{
		/* handle the error */
		Lsm6ds3_ErrHandler();
	}
}

void Lsm6ds3_Task (void)
{
	if (lsm6ds3_Stat.wait_flag == false)
	{
#if 0
		if (timeoutCnt++ > LSM6DS3_MAX_TIMEOUT_CNT)
		{
			Lsm6ds3_ResetTimeout();
		}
#endif
		switch (lsm6ds3_Stat.state)
		{
		    case LSM6DS3_STATE_CHECK_ID:
		    	Lsm6ds3_CheckID();
		    	break;
#if ACC_SHAKE_DETECTION_ENABLE
		    case LSM6DS3_STATE_SET_SHAKE_PARAMS:
#endif /* ACC_SHAKE_DETECTION_ENABLE */
		    case LSM6DS3_STATE_SET_HIT_PARAMS:
		    	Lsm6ds3_SetHitParams();
		    	break;
		    case LSM6DS3_STATE_SET_MOVE_PARAMS:
		    	Lsm6ds3_SetMoveParams();
		    	break;
		    case LSM6DS3_STATE_GET_DATA:
		    	if (ReadAccIntGpio () != false)
		    	{
                    //Test1Toggle();
		    	    Lsm6ds3_GetData();
		    	}
		    	break;
		    default:
		    	break;
		}
	}
}

#endif /* LSM6DS3_ACC_ENABLE */
