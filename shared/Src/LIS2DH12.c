
#include <LIS2DH12.h>
#include <stdlib.h>
#include <stdio.h>
#include "timer.h"
#include "init_brd.h"
#include "solution_wrapper.h"
#include "acc_proc.h"

static lis2dh12_Status_t lis2dh12_Stat;
static app_cbk_fn  lis2dh12_sys_cbk = NULL;
static uint16_t timeoutCnt = 0u;

static void Lis2dh12_ErrHandler (void);

static void Lis2dh12_ResetTimeout (void)
{
	timeoutCnt = 0;
}

void Lis2dh12_Reset (app_cbk_fn sys_cbk, int16_t** x_axis, int16_t** y_axis, int16_t** z_axis)
{
	memset(&lis2dh12_Stat, 0u, sizeof(lis2dh12_Stat));

	lis2dh12_sys_cbk = sys_cbk;

	/* Set pointers to the x/y/z values */
	*x_axis = &lis2dh12_Stat.x_axis;
	*y_axis = &lis2dh12_Stat.y_axis;
	*z_axis = &lis2dh12_Stat.z_axis;

	/* Set initial state */
	lis2dh12_Stat.state = LIS2DH12_STATE_CHECK_ID;
}

void Lis2dh12_GotoHitMode (void)
{
	/* Set hit detection parameters */
	lis2dh12_Stat.state = LIS2DH12_STATE_SET_HIT_PARAMS;
}

void Lis2dh12_GotoMoveMode (void)
{
	/* Set move detection parameters */
	lis2dh12_Stat.state = LIS2DH12_STATE_SET_MOVE_PARAMS;
}

void Lis2dh12_Deinit (void)
{
	memset(&lis2dh12_Stat, 0u, sizeof(lis2dh12_Status_t));
	Timer_Stop (ACC_TIMEOUT_TMR);
}

static void Lis2dh12_PauseCbk (uint8_t timer_id)
{
    (void)timer_id;

    lis2dh12_Stat.wait_flag = false;
}

static void Lis2dh12_ErrHandler (void)
{
	/* Handle potential error */
	if (lis2dh12_Stat.retry_cnt++ > LIS2DH12_MAX_RETRY_CNT)
	{
		/* Report Error */
		if (lis2dh12_sys_cbk != NULL)
		{
			lis2dh12_sys_cbk(SYSTEM_EVT_ERROR, 0u);
		}
		lis2dh12_Stat.wait_flag = true;
		lis2dh12_Stat.retry_cnt = 0u;
		lis2dh12_Stat.state = LIS2DH12_STATE_ERROR;
	}
	else
	{
		lis2dh12_Stat.wait_flag = true;
		/* Start retry timer */
		Timer_Start (ACC_TIMEOUT_TMR, ACC_TIMEOUT_TMR_RETRY_PERIOD_MS, Lis2dh12_PauseCbk);
	}
}

static void Lis2dh12_CheckID (void)
{
	uint8_t rdVal = 0u;
	bool stat = false;

	/* Read device id */
	if (SpiReadRegister(LIS2DH12_WHO_AM_I, &rdVal, 1) != false)
	{
		if (rdVal == LIS2DH12_ID)
		{
			/* Init is done */
			if (lis2dh12_sys_cbk != NULL)
			{
				/* move to idle if response is correct */
				lis2dh12_Stat.state = LIS2DH12_STATE_IDLE;
				lis2dh12_Stat.retry_cnt = 0u;
				lis2dh12_sys_cbk(SYSTEM_EVT_READY, ACC_EVT_INIT_OK);
				stat = true;
			}
		}
	}

	if (stat == false)
	{
		/* handle the error */
		Lis2dh12_ErrHandler();
	}
}

static void Lis2dh12_SetHitParams (void)
{
	uint8_t rdVal = 0u;
	bool stat = false;

	/* Set 16G range */
	if (SpiWriteSingleRegister(LIS2DH12_CTRL_REG4, LIS2DH12_CTRL_REG4_VAL) != false)
	{
		/* Check if value was written */
		if (SpiReadRegister(LIS2DH12_CTRL_REG4, &rdVal, 1) != false)
		{
			/* Value was written successfully - go to the next step */
			if (rdVal == LIS2DH12_CTRL_REG4_VAL)
			{
			    stat = true;
			}
		}
	}
	if (stat == false)
	{
		/* handle the error */
		Lis2dh12_ErrHandler();
		return;
	}
	else
	{
		stat = false;
	}

	/* Set 1620Hz rate */
	if (SpiWriteSingleRegister(LIS2DH12_CTRL_REG1, LIS2DH12_CTRL_REG1_VAL) != false)
	{
		/* Check if value was written */
		if (SpiReadRegister(LIS2DH12_CTRL_REG1, &rdVal, 1) != false)
		{
			/* Value was written successfully - go to the next step */
			if (rdVal == LIS2DH12_CTRL_REG1_VAL)
			{
			    stat = true;
			}
		}
	}
	if (stat == false)
	{
		/* handle the error */
		Lis2dh12_ErrHandler();
		return;
	}
	else
	{
		stat = false;
	}

	/* Set Interrupt mask */
	if (SpiWriteSingleRegister(LIS2DH12_CTRL_REG3, LIS2DH12_CTRL_REG3_VAL) != false)
	{
		/* Check if value was written */
		if (SpiReadRegister(LIS2DH12_CTRL_REG3, &rdVal, 1) != false)
		{
			/* Value was written successfully - go to the next step */
			if (rdVal == LIS2DH12_CTRL_REG3_VAL)
			{
			    stat = true;
			}
		}
	}
	if (stat == false)
	{
		/* handle the error */
		Lis2dh12_ErrHandler();
		return;
	}
	else
	{
		stat = false;
	}

	/* Clear memory */
	if (SpiWriteSingleRegister(LIS2DH12_CTRL_REG5, LIS2DH12_CTRL_REG5_VAL) != false)
	{
		/* Check if value was written */
		if (SpiReadRegister(LIS2DH12_CTRL_REG5, &rdVal, 1) != false)
		{
			/* Value was written successfully - go to the next step */
			if (rdVal == LIS2DH12_CTRL_REG5_VAL)
			{
			    stat = true;
			}
		}
	}
	if (stat == false)
	{
		/* handle the error */
		Lis2dh12_ErrHandler();
		return;
	}
	else
	{
		//stat = false;
	}

#if 0
	/* Set Interrupt mask */
	if (SpiWriteSingleRegister(LIS2DH12_POWER_CTL, LIS2DH12_MEASURE_EN_SET) != false)
	{
		/* Check if value was written */
		if (SpiReadRegister(LIS2DH12_POWER_CTL, &rdVal, 1) != false)
		{
			/* Value was written successfully - go to the next step */
			if (rdVal == LIS2DH12_MEASURE_EN_SET)
			{
			    stat = true;
			}
		}
	}
#endif

	if (stat == false)
	{
		/* handle the error */
		Lis2dh12_ErrHandler();
		return;
	}
	else
	{
		lis2dh12_Stat.retry_cnt = 0u;
		lis2dh12_Stat.state = LIS2DH12_STATE_GET_DATA;
		/* Init is done */
		if (lis2dh12_sys_cbk != NULL)
		{
			lis2dh12_sys_cbk(SYSTEM_EVT_READY, ACC_EVT_HIT_INIT_OK);
		}
	}
}

static void Lis2dh12_SetMoveParams (void)
{
	uint8_t rdVal = 0u;
	bool stat = false;

	/* Set 8G range */
	if (SpiWriteSingleRegister(LIS2DH12_CTRL_REG4, LIS2DH12_CTRL_REG4_MOVE_VAL) != false)
	{
		/* Check if value was written */
		if (SpiReadRegister(LIS2DH12_CTRL_REG4, &rdVal, 1) != false)
		{
			/* Value was written successfully - go to the next step */
			if (rdVal == LIS2DH12_CTRL_REG4_MOVE_VAL)
			{
			    stat = true;
			}
		}
	}
	if (stat == false)
	{
		/* handle the error */
		Lis2dh12_ErrHandler();
		return;
	}
	else
	{
		stat = false;
	}

	/* Set 10Hz rate */
	if (SpiWriteSingleRegister(LIS2DH12_CTRL_REG1, LIS2DH12_CTRL_REG1_MOVE_VAL) != false)
	{
		/* Check if value was written */
		if (SpiReadRegister(LIS2DH12_CTRL_REG1, &rdVal, 1) != false)
		{
			/* Value was written successfully - go to the next step */
			if (rdVal == LIS2DH12_CTRL_REG1_MOVE_VAL)
			{
			    stat = true;
			}
		}
	}
	if (stat == false)
	{
		/* handle the error */
		Lis2dh12_ErrHandler();
		return;
	}
	else
	{
		stat = false;
	}

	/* Set Interrupt mask */
	if (SpiWriteSingleRegister(LIS2DH12_CTRL_REG3, LIS2DH12_CTRL_REG3_VAL) != false)
	{
		/* Check if value was written */
		if (SpiReadRegister(LIS2DH12_CTRL_REG3, &rdVal, 1) != false)
		{
			/* Value was written successfully - go to the next step */
			if (rdVal == LIS2DH12_CTRL_REG3_VAL)
			{
			    stat = true;
			}
		}
	}
	if (stat == false)
	{
		/* handle the error */
		Lis2dh12_ErrHandler();
		return;
	}
	else
	{
		stat = false;
	}

	/* Clear memory */
	if (SpiWriteSingleRegister(LIS2DH12_CTRL_REG5, LIS2DH12_CTRL_REG5_VAL) != false)
	{
		/* Check if value was written */
		if (SpiReadRegister(LIS2DH12_CTRL_REG5, &rdVal, 1) != false)
		{
			/* Value was written successfully - go to the next step */
			if (rdVal == LIS2DH12_CTRL_REG5_VAL)
			{
			    stat = true;
			}
		}
	}

	if (stat == false)
	{
		/* handle the error */
		Lis2dh12_ErrHandler();
		return;
	}
	else
	{
		lis2dh12_Stat.retry_cnt = 0u;
		lis2dh12_Stat.state = LIS2DH12_STATE_GET_DATA;
		/* Init is done */
		if (lis2dh12_sys_cbk != NULL)
		{
			lis2dh12_sys_cbk(SYSTEM_EVT_READY, ACC_EVT_MOVE_INIT_OK);
		}
	}
}

// https://community.st.com/t5/mems-sensors/lis2dh12-measure-conversion/td-p/420708
int16_t x,y,z;
static void Lis2dh12_GetDataCbk (system_evt_t evt, uint32_t usr_data)
{
	/* Stop timeout timer */
	//Timer_Stop (ACC_TIMEOUT_TMR);
	int16_t tmp;
	if (evt == SYSTEM_EVT_READY)
	{
		//TestLedToggle();
		/* Convert data: signed 12bit into signed 8bit */
#if 0
		lis2dh12_Stat.x_axis = lis2dh12_Stat.rd_buff[2];
		lis2dh12_Stat.y_axis = lis2dh12_Stat.rd_buff[4];
		lis2dh12_Stat.z_axis = lis2dh12_Stat.rd_buff[6];
#endif
		//adxl345Status.x_axis = adxl345Status.rd_buff[1] >> 2 | adxl345Status.rd_buff[2] << 6;
		//adxl345Status.y_axis = adxl345Status.rd_buff[3] >> 2 | adxl345Status.rd_buff[4] << 6;
		//adxl345Status.z_axis = adxl345Status.rd_buff[5] >> 2 | adxl345Status.rd_buff[6] << 6;

		//lis2dh12_Stat.x_axis = convert_lis2dh12_data(lis2dh12_Stat.rd_buff[1], lis2dh12_Stat.rd_buff[2]);
		//lis2dh12_Stat.y_axis = convert_lis2dh12_data(lis2dh12_Stat.rd_buff[3], lis2dh12_Stat.rd_buff[4]);;
		//lis2dh12_Stat.z_axis = convert_lis2dh12_data(lis2dh12_Stat.rd_buff[5], lis2dh12_Stat.rd_buff[6]);


		//lis2dh12_Stat.x_axis = (*(int16_t*) &lis2dh12_Stat.rd_buff[1]) >> 6;//lis2dh12_Stat.rd_buff[1] >> 2 | lis2dh12_Stat.rd_buff[2] << 6;
		//lis2dh12_Stat.y_axis = (*(int16_t*) &lis2dh12_Stat.rd_buff[3]) >> 6;//lis2dh12_Stat.rd_buff[3] >> 2 | lis2dh12_Stat.rd_buff[4] << 6;
		//lis2dh12_Stat.z_axis = (*(int16_t*) &lis2dh12_Stat.rd_buff[5]) >> 6;//lis2dh12_Stat.rd_buff[5] >> 2 | lis2dh12_Stat.rd_buff[6] << 6;

		//float x,y,z;
		//x = convert_lis2dh12_acceleration(lis2dh12_Stat.rd_buff[2], lis2dh12_Stat.rd_buff[1]);
		//y = convert_lis2dh12_acceleration(lis2dh12_Stat.rd_buff[4], lis2dh12_Stat.rd_buff[3]);
		//z = convert_lis2dh12_acceleration(lis2dh12_Stat.rd_buff[6], lis2dh12_Stat.rd_buff[5]);

		tmp = ((int16_t)lis2dh12_Stat.rd_buff[2] << 8) + lis2dh12_Stat.rd_buff[1];
		lis2dh12_Stat.x_axis = tmp >> 4;
		tmp = ((int16_t)lis2dh12_Stat.rd_buff[4] << 8) + lis2dh12_Stat.rd_buff[3];
		lis2dh12_Stat.y_axis = tmp >> 4;
		tmp = ((int16_t)lis2dh12_Stat.rd_buff[6] << 8) + lis2dh12_Stat.rd_buff[5];
		lis2dh12_Stat.z_axis = tmp >> 4;

#if 0
		x = lis2dh12_Stat.x_axis * 12;
		y = lis2dh12_Stat.y_axis * 12;
		z = lis2dh12_Stat.z_axis * 12;
#endif

		/* Report data is ready */
		if (lis2dh12_sys_cbk != NULL)
		{
			lis2dh12_sys_cbk(SYSTEM_EVT_READY, ACC_EVT_DATA_READY);
		}
	}
	else
	{
		/* handle the error */
		Lis2dh12_ErrHandler();
	}
}

static void Lis2dh12_GetData (void)
{
	/* Read data */
	if (SpiGetAccData (lis2dh12_Stat.rd_buff, Lis2dh12_GetDataCbk) != false)
	{
		lis2dh12_Stat.retry_cnt = 0u;
		/* Start timeout counter */
		Lis2dh12_ResetTimeout();
	}
	else
	{
		/* handle the error */
		Lis2dh12_ErrHandler();
	}
}

void Lis2dh12_Task (void)
{
	if (lis2dh12_Stat.wait_flag == false)
	{
#if 0
		if (timeoutCnt++ > LIS2DH12_MAX_TIMEOUT_CNT)
		{
			Lis2dh12_ResetTimeout();
		}
#endif
		switch (lis2dh12_Stat.state)
		{
		    case LIS2DH12_STATE_CHECK_ID:
		    	Lis2dh12_CheckID();
		    	break;
		    case LIS2DH12_STATE_SET_HIT_PARAMS:
		    	Lis2dh12_SetHitParams();
		    	break;
		    case LIS2DH12_STATE_SET_MOVE_PARAMS:
		    	Lis2dh12_SetMoveParams();
		    	break;
		    case LIS2DH12_STATE_GET_DATA:
		    	if (ReadAccIntGpio () != false)
		    	{
		    	    Lis2dh12_GetData();
		    	}
		    	break;
		    default:
		    	break;
		}
	}
}
