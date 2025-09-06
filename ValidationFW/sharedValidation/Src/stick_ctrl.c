
#include <stdlib.h>
#include <stdio.h>
#include "timer.h"
#include "init_brd.h"
#include "stick_ctrl.h"
#include "solution_wrapper.h"

static stickStatus_t stickStatus[2];
static bool periodsValid, signalsValid;
static app_cbk_fn sys_cbk = NULL;
static uint8_t pwm_ch;
static bool mining_mode_enabled = false;

// TODO LOW For some reason we read not expected period and signal value. Once at a time.
// This has to be investigated. Maybe GPIO interrupt handler got interrupted, or it happens in debug only
// Consider to show the issue somehow to a pin and observe regularity

static void Stick_Process(void)
{

}

static void Stick_HandleLostState (uint8_t pwm_ch)
{
	if (stickStatus[pwm_ch].old_state < STICK_STATE_ACTIVE_POS_1)
	{
		/* Stick wasn't detected during the start-up  */
		stickStatus[pwm_ch].new_state = STICK_STATE_NO_STICK;
	}
	else
	{
		/* Report lost stick if previously stick was detected */
		stickStatus[pwm_ch].new_state = STICK_STATE_LOST;
	}

	/* Run stick handling logic */
	Stick_Process();

	stickStatus[pwm_ch].old_state = stickStatus[pwm_ch].new_state;
}

static void Stick1_LostCbk (uint8_t timer_id)
{
	Stick_HandleLostState(PWM_CH_1);
}

static void Stick2_LostCbk (uint8_t timer_id)
{
	Stick_HandleLostState(PWM_CH_2);
}

void Stick_Reset (app_cbk_fn system_cbk, bool mining_mode)
{
	memset(&stickStatus, 0u, sizeof(stickStatus_t));
	sys_cbk = system_cbk;
	mining_mode_enabled = mining_mode;
	stickStatus[PWM_CH_1].is_stick_scan_enabled = true;
	//stickStatus[PWM_CH_2].is_stick_scan_enabled = true;

	//STICK_LOST_PERIOD_MS has to be less 500ms
	// Start stick lost timer
	Timer_Start (STICK1_LOST_TMR, STICK_LOST_PERIOD_MS, Stick1_LostCbk);
	//Timer_Start (STICK2_LOST_TMR, STICK_LOST_PERIOD_MS, Stick2_LostCbk);
}

void Stick_Deinit (void)
{
	memset(&stickStatus, 0u, sizeof(stickStatus));
}

void Stick_ProcessEdgeCbk (system_evt_t idx, uint32_t usr_data)
{
	/* Disable interrupts */
	uint32_t prim = __get_PRIMASK();
	__disable_irq();

	//Test2LedToggle();
	//TestLedToggle();

	if (idx <= PWM_CH_2)
	{
		stickStatus[idx].countVal = ReadStickCounter10Us();

		if (usr_data == true)
		{
			// rising edge
			if (stickStatus[idx].countVal > stickStatus[idx].lastRisingVal)
			{
				stickStatus[idx].periodValue = stickStatus[idx].countVal - stickStatus[idx].lastRisingVal;
			}
			else // counter overlap happen
			{
				stickStatus[idx].periodValue = 0xffff - (stickStatus[idx].lastRisingVal - stickStatus[idx].countVal);
			}
			stickStatus[idx].lastRisingVal = stickStatus[idx].countVal;
		}
		else
		{
			// falling edge
			if (stickStatus[idx].countVal > stickStatus[idx].lastRisingVal)
			{
				stickStatus[idx].signalValue = stickStatus[idx].countVal - stickStatus[idx].lastRisingVal;
			}
			else // counter overlap happen
			{
				stickStatus[idx].signalValue = 0xffff - (stickStatus[idx].lastRisingVal - stickStatus[idx].countVal);
			}
			stickStatus[idx].fallingIdx++;
		}
	}

	// enable interrupts
	if (!prim) {
		__enable_irq();
	}

}

void Stick_Task (void)
{
	for (pwm_ch = PWM_CH_1; pwm_ch < (PWM_CH_2 + 1); pwm_ch++)
	{
		if (stickStatus[pwm_ch].is_stick_scan_enabled == false)
			return;

		// check if we need to analyze data
		if (stickStatus[pwm_ch].fallingIdx != stickStatus[pwm_ch].processedIdx)
		{
			stickStatus[pwm_ch].processedIdx = stickStatus[pwm_ch].fallingIdx;

			if (pwm_ch == PWM_CH_1)
			{
				// Reset stick lost timer
				Timer_Start (STICK1_LOST_TMR, STICK_LOST_PERIOD_MS, Stick1_LostCbk);
			}
			if (pwm_ch == PWM_CH_2)
			{
				// Reset stick lost timer
				Timer_Start (STICK2_LOST_TMR, STICK_LOST_PERIOD_MS, Stick2_LostCbk);
			}

			// check periods
			periodsValid = true;
			periodsValid =  ((stickStatus[pwm_ch].periodValue > FC_PWM_PERIOD_MIN) && (stickStatus[pwm_ch].periodValue < FC_PWM_PERIOD_MAX));
			// It is kind of filter
			if (periodsValid == true)
			{
				stickStatus[pwm_ch].periodInvalidCount = 0;
			}
			else
			{
				// most likely transition period or some kind of dithering
				stickStatus[pwm_ch].periodInvalidCount++;
				if (stickStatus[pwm_ch].periodInvalidCount < FC_PWM_PERIOD_INVALID_MAX_NUM)
				{
					periodsValid = true;
				}
			}

			// check if signal values are valid
			signalsValid = true;
			if ((stickStatus[pwm_ch].signalValue > FC_PWM_POS_1_MIN) && (stickStatus[pwm_ch].signalValue < FC_PWM_POS_1_MAX))
			{
				stickStatus[pwm_ch].pos1Counts++;
				stickStatus[pwm_ch].pos2Counts = 0;
				stickStatus[pwm_ch].pos3Counts = 0;
				stickStatus[pwm_ch].lostCounts = 0;
			}
			else if ((stickStatus[pwm_ch].signalValue > FC_PWM_POS_2_MIN) && (stickStatus[pwm_ch].signalValue < FC_PWM_POS_2_MAX))
			{
				stickStatus[pwm_ch].pos1Counts = 0;
				stickStatus[pwm_ch].pos2Counts++;
				stickStatus[pwm_ch].pos3Counts = 0;
				stickStatus[pwm_ch].lostCounts = 0;
			}
			else if ((stickStatus[pwm_ch].signalValue > FC_PWM_POS_3_MIN) && (stickStatus[pwm_ch].signalValue < FC_PWM_POS_3_MAX))
			{
				stickStatus[pwm_ch].pos1Counts = 0;
				stickStatus[pwm_ch].pos2Counts = 0;
				stickStatus[pwm_ch].pos3Counts++;
				stickStatus[pwm_ch].lostCounts = 0;
			}
			else if ((stickStatus[pwm_ch].signalValue > FC_PWM_POS_LOST_MIN) && (stickStatus[pwm_ch].signalValue < FC_PWM_POS_LOST_MAX))
			{
				stickStatus[pwm_ch].pos1Counts = 0;
				stickStatus[pwm_ch].pos2Counts = 0;
				stickStatus[pwm_ch].pos3Counts = 0;
				stickStatus[pwm_ch].lostCounts++;
			}
			else
			{
				signalsValid = false;
				stickStatus[pwm_ch].pos1Counts = 0;
				stickStatus[pwm_ch].pos2Counts = 0;
				stickStatus[pwm_ch].pos3Counts = 0;
				stickStatus[pwm_ch].lostCounts = 0;
			}
			// It is kind of filter
			if (signalsValid == true)
			{
				stickStatus[pwm_ch].signalInvalidCount = 0;
			}
			else
			{
				// most likely transition period or some kind of dithering
				stickStatus[pwm_ch].signalInvalidCount++;
				if (stickStatus[pwm_ch].signalInvalidCount < FC_PWM_SIGNAL_INVALID_MAX_NUM)
				{
					signalsValid = true;
				}
			}

			// perform flag processing into status
			if ((periodsValid == false) || (signalsValid == false))
			{
				if (stickStatus[pwm_ch].old_state < STICK_STATE_LOST) //STICK_STATE_ACTIVE_POS_1
				{
					stickStatus[pwm_ch].new_state = STICK_STATE_NO_STICK;
				}
				else
				{
					/* Report lost stick if previously stick was detected */
					stickStatus[pwm_ch].new_state = STICK_STATE_LOST;
				}
			}
			if (stickStatus[pwm_ch].pos1Counts > FC_PWM_DETECTION_NUM)
			{
				stickStatus[pwm_ch].new_state = STICK_STATE_ACTIVE_POS_1;
			}
			if (stickStatus[pwm_ch].pos2Counts > FC_PWM_DETECTION_NUM)
			{
				stickStatus[pwm_ch].new_state = STICK_STATE_ACTIVE_POS_2;
			}
			if (stickStatus[pwm_ch].pos3Counts > FC_PWM_DETECTION_NUM)
			{
				stickStatus[pwm_ch].new_state = STICK_STATE_ACTIVE_POS_3;
			}
			if (stickStatus[pwm_ch].lostCounts > FC_PWM_DETECTION_NUM)
			{
				stickStatus[pwm_ch].new_state = STICK_STATE_LOST;
			}

			// report status if needed
			if (stickStatus[pwm_ch].new_state != stickStatus[pwm_ch].old_state)
			{
				/* Run stick handling logic */
				//Stick_Process();
				if (sys_cbk != NULL)
				{
					sys_cbk(SYSTEM_EVT_READY, stickStatus[pwm_ch].new_state);
				}

				stickStatus[pwm_ch].old_state = stickStatus[pwm_ch].new_state;
			}
		}
	}
}
