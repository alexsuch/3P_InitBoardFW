#if LIS2DH12_ACC_ENABLE
#include <LIS2DH12.h>
#endif
#include <string.h>
#include "init_brd.h"
#include "acc_proc.h"
#include "prj_config.h"
#include "solution_wrapper.h"

static accProcStatus_t accProcStatus;

static app_cbk_fn acc_sys_cbk = NULL;
static int16_t *x_axis;
static int16_t *y_axis;
static int16_t *z_axis;

const uint32_t accSqrtLowThreshold = ACC_SQRT_LOW_TRESHOLD;
const uint32_t accSqrtHighThreshold = ACC_SQRT_HIGH_TRESHOLD;

static uint8_t prev_idx;
static uint32_t tmp32;
static int16_t x_tmp;
static int16_t y_tmp;
static int16_t z_tmp;

static uint32_t measureCnt = 0;
static acc_status_t failflag = ACC_TEST_STATUS_VALID_DATA_OK;

/*--------------------------Hit detection block -------------------------------*/

static void App_AccCbk (system_evt_t evt, uint32_t usr_data)
{
	if (evt == SYSTEM_EVT_ERROR)
	{
		if (acc_sys_cbk != NULL)
		{
			/* Pass the error to the higher layer */
			acc_sys_cbk(SYSTEM_EVT_ERROR, ACC_TEST_STATUS_FAILED_COMMUNICATION);
		}
	}
	else if (evt == SYSTEM_EVT_READY)
	{
		if (usr_data == ACC_EVT_INIT_OK)
		{
			if (acc_sys_cbk != NULL)
			{
				/* Pass the init event to the higher layer */
				//acc_sys_cbk(SYSTEM_EVT_READY, usr_data);
			}
		}
		else if (usr_data == ACC_EVT_HIT_INIT_OK)
		{
			/* Start hit processing */
			accProcStatus.hit_detection_enabled  = true;
		}
		else if (usr_data == ACC_EVT_DATA_READY)
		{
			/* Data is ready for processing */
			if (accProcStatus.hit_detection_enabled != false)
			{
				/* Process the data */
				accProcStatus.data_ready = true;
			}
		}
		else
		{
			//Empty handler
		}
	}
}

static void AccProc_Processing(void)
 {
	if (accProcStatus.data_ready != false)
	{
		if (accProcStatus.init_skip_counts == 0)
		{
			if (accProcStatus.hit_detection_enabled != false)
			{
				/* save data */
				prev_idx = (accProcStatus.idx + 1) % (ACC_BUFF_SIZE + 1);
				accProcStatus.x_buff[accProcStatus.idx] = *x_axis;
				accProcStatus.y_buff[accProcStatus.idx] = *y_axis;
				accProcStatus.z_buff[accProcStatus.idx] = *z_axis;


				accProcStatus.x_sum += accProcStatus.x_buff[accProcStatus.idx];
				accProcStatus.x_sum -= accProcStatus.x_buff[prev_idx];
				accProcStatus.y_sum += accProcStatus.y_buff[accProcStatus.idx];
				accProcStatus.y_sum -= accProcStatus.y_buff[prev_idx];
				accProcStatus.z_sum += accProcStatus.z_buff[accProcStatus.idx];
				accProcStatus.z_sum -= accProcStatus.z_buff[prev_idx];

				accProcStatus.idx = prev_idx;

				// process data
				x_tmp = accProcStatus.x_sum / (int8_t)ACC_BUFF_SIZE;
				y_tmp = accProcStatus.y_sum / (int8_t)ACC_BUFF_SIZE;
				z_tmp = accProcStatus.z_sum / (int8_t)ACC_BUFF_SIZE;

				tmp32 = x_tmp * x_tmp + y_tmp * y_tmp + z_tmp * z_tmp;

				if (    (measureCnt > 10) &&
						((tmp32 < accSqrtLowThreshold) ||
						 (tmp32 > accSqrtHighThreshold))
					)
				{
					failflag = ACC_TEST_STATUS_VALID_DATA_FAILED;
				}

				if (
						(*x_axis > 30) ||
						(*x_axis < -30) ||
						(*y_axis > 30) ||
						(*y_axis < -30) ||
						(*z_axis > 300) ||
						(*z_axis < -300)
					)
				{
					failflag = ACC_TEST_STATUS_VALID_DATA_SPIKE;
				}


				if (measureCnt++ > 2000)
				{
					if (failflag == ACC_TEST_STATUS_VALID_DATA_OK)
					{
							// We report only transitions to ignition state
							if (acc_sys_cbk != NULL)
							{
								acc_sys_cbk(SYSTEM_EVT_READY, ACC_TEST_STATUS_VALID_DATA_OK);
							}
					}
					else
					{
						// We report only transitions to ignition state
						if (acc_sys_cbk != NULL)
						{
							acc_sys_cbk(SYSTEM_EVT_ERROR, failflag);
						}
					}

					AccProc_Stop();
				}

			}
			/* reset a flag */
			accProcStatus.data_ready = false;
		}
		else
		{
			/* Skip a few first measurements */
			accProcStatus.init_skip_counts--;
			accProcStatus.data_ready = false;
		}
	}
}

void AccProc_Stop (void)
{
	/* Reset all data and SM */
	memset(&accProcStatus, 0u, sizeof(accProcStatus_t));

	/* Reset local variables */
	measureCnt = 0;
	failflag = ACC_TEST_STATUS_VALID_DATA_OK;

	/* Skip first measurements results */
	accProcStatus.init_skip_counts = MOVE_DETECT_INIT_DELAY_SKIP_CNT;

#if	LIS2DH12_ACC_ENABLE
	/* Deinit Accelerometer functionality */
	Lis2dh12_Deinit();
#endif
}

void AccProc_Reset (app_cbk_fn sys_cbk)
{
	memset(&accProcStatus, 0u, sizeof(accProcStatus_t));
	acc_sys_cbk = sys_cbk;

#if	LIS2DH12_ACC_ENABLE
	/* Init Accelerometer functionality */
	Lis2dh12_Reset(App_AccCbk, &x_axis, &y_axis, &z_axis);
#endif
}

void AccProc_HitDetectionStart (void)
{
	if ((accProcStatus.hit_detection_enabled == false) && (acc_sys_cbk != NULL))
	{
		/* Reset all data and SM */
		AccProc_Stop();
#if	LIS2DH12_ACC_ENABLE
		/* Set hit mode parameters */
		Lis2dh12_GotoHitMode();
#endif
	}
}


void AccProc_Task ()
{
	AccProc_Processing();

#if LIS2DH12_ACC_ENABLE
	/* Accelerometer tasks*/
	Lis2dh12_Task();
#endif
}
