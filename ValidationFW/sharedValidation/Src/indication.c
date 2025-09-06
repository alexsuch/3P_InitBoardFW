
#include <stdlib.h>
#include <stdio.h>
#include "timer.h"
#include "init_brd.h"
#include "solution_wrapper.h"
#include "indication.h"

static indication_status_t indStatus;

static void Indication_PeriodicCbk (uint8_t timer_id);
static void Indication_UpdateErr (err_code_t err_code);
static uint8_t Indication_GetHighErrIdx (uint32_t err_code_mask);

static void Indication_Stop (uint8_t timer_id)
{
	(timer_id == INDICATION_STATUS_TMR) ? SET_STATUS_LED(false) : SET_ERROR_LED(false);
	BuzzerDisable();

	/* Run device active indication */
	if (indStatus.curr_status == IND_STATUS_FUSE_POPULATED)
	{
		Timer_Start (timer_id, ONE_SECOND_TICK_TMR_PERIOD_MS, Indication_PeriodicCbk);
		indStatus.stat_pattern = IND_PATTERN_PERIODIC_2_SHORT_PLUS_LONG_PAUSE;
	}
}

static void Indication_Start (uint8_t timer_id)
{
	(timer_id == INDICATION_STATUS_TMR) ? SET_STATUS_LED(true) : SET_ERROR_LED(true);
	BuzzerEnable();
}

static void Indication_ClearStatusIndication (void)
{
	Timer_Stop (INDICATION_STATUS_TMR);
	/* Reset indication related flags */
	indStatus.stat_retry_cnt = 0u;
	Indication_Stop(INDICATION_STATUS_TMR);
}

static void Indication_ClearErrorIndication (void)
{
	Timer_Stop (INDICATION_ERROR_TMR);
	/* Reset indication related flags */
	indStatus.err_retry_cnt = 0u;
	Indication_Stop(INDICATION_ERROR_TMR);
}

static void Indication_PeriodicCbk (uint8_t timer_id)
{
	if (timer_id == INDICATION_STATUS_TMR)
	{
		indStatus.handle_stat_tmr_cbk = true;
	}
	else
	{
		indStatus.handle_err_tmr_cbk = true;
	}
}

static void Indication_HandleTmrCbk (uint8_t timer_id)
{
	uint8_t * retry_cnt   = (timer_id == INDICATION_STATUS_TMR) ? (&indStatus.stat_retry_cnt) : (&indStatus.err_retry_cnt);
	ind_pattern_t pattern = (timer_id == INDICATION_STATUS_TMR) ? (indStatus.stat_pattern) : (indStatus.err_pattern);

	switch (pattern)
	{
	    case IND_PATTERN_PERIODIC_50_50:
	    	if (*retry_cnt == 0u)
	    	{
	    		if ((timer_id == INDICATION_STATUS_TMR) && (indStatus.curr_status == IND_STATUS_CHARGING_START))
	    		{
	    			/* Set only LED for charging indication */
	    			SET_STATUS_LED(true);
	    			Timer_Start (timer_id, INDICATION_TMR_SHORT_SOUND_PERIOD_MS, Indication_PeriodicCbk);
	    		}
	    		else
	    		{
	    			/* Start indication for 50% of duty time */
	    			Indication_Start(timer_id);
	    			Timer_Start (timer_id, INDICATION_TMR_SHORT_SOUND_PERIOD_MS, Indication_PeriodicCbk);
	    		}
	    		(*retry_cnt)++;
	    	}
	    	else
	    	{
	    		/* Stop indication for 50% of duty time */
	    		Indication_Stop(timer_id);
	    		if ((timer_id == INDICATION_STATUS_TMR) && (indStatus.curr_status == IND_STATUS_CHARGING_START))
	    		{
	    			Timer_Start (timer_id, INDICATION_TMR_SHORT_SOUND_PERIOD_MS, Indication_PeriodicCbk);
	    		}
	    		else
	    		{
	    			Timer_Start (timer_id, INDICATION_TMR_SHORT_SOUND_PERIOD_MS, Indication_PeriodicCbk);
	    		}
	    		*retry_cnt = 0;
	    	}
	    	break;

	    case IND_PATTERN_PERIODIC_1_SHORT_PLUS_PAUSE:
	    	if (*retry_cnt == 0u)
	    	{
	    		/* Start indication for sequence 1 short + long pause */
	    		Indication_Start(timer_id);
	    		Timer_Start (timer_id, INDICATION_TMR_SHORT_SOUND_PERIOD_MS, Indication_PeriodicCbk);
	    		(*retry_cnt)++;
	    	}
	    	else
	    	{
	    		/* Stop indication for long pause */
	    		Indication_Stop(timer_id);
	    		Timer_Start (timer_id, INDICATION_TMR_VERY_LONG_SOUND_PERIOD_MS, Indication_PeriodicCbk);
	    		*retry_cnt = 0;
	    	}
	    	break;

	    case IND_PATTERN_PERIODIC_2_SHORT_PLUS_PAUSE:
	    	if ((*retry_cnt == 0u) || (*retry_cnt == 2u))
	    	{
	    		/* Start indication for sequence 1 short + 1 short pause + 1 short + 1 short pause + long pause */
	    		Indication_Start(timer_id);
	    		Timer_Start (timer_id, INDICATION_TMR_SHORT_SOUND_PERIOD_MS, Indication_PeriodicCbk);
	    		(*retry_cnt)++;
	    	}
	    	else if (*retry_cnt == 1u)
	    	{
	    		/* Stop indication for short pause */
	    		Indication_Stop(timer_id);
	    		Timer_Start (timer_id, INDICATION_TMR_SHORT_SOUND_PERIOD_MS, Indication_PeriodicCbk);
	    		(*retry_cnt)++;
	    	}
	    	else
	    	{
	    		/* One long pause  */
	    		Indication_Stop(timer_id);
	    		Timer_Start (timer_id, (INDICATION_TMR_VERY_LONG_SOUND_PERIOD_MS * 2), Indication_PeriodicCbk);
	    		*retry_cnt = 0;
	    	}
	    	break;
	    case IND_PATTERN_PERIODIC_2_SHORT_PLUS_LONG_PAUSE:
	    	if ((*retry_cnt == 0u) || (*retry_cnt == 2u))
	    	{
	    		/* Start indication for sequence 1 short + 1 short pause + 1 short + 1 short pause + very long pause */
	    		SET_STATUS_LED(true);
	    		Timer_Start (timer_id, INDICATION_TMR_SHORT_SOUND_PERIOD_MS, Indication_PeriodicCbk);
	    		(*retry_cnt)++;
	    	}
	    	else if (*retry_cnt == 1u)
	    	{
	    		/* Stop indication for short pause */
	    		SET_STATUS_LED(false);
	    		Timer_Start (timer_id, INDICATION_TMR_SHORT_SOUND_PERIOD_MS, Indication_PeriodicCbk);
	    		(*retry_cnt)++;
	    	}
	    	else
	    	{
	    		/* One long pause  */
	    		SET_STATUS_LED(false);
	    		Timer_Start (timer_id, (ONE_SECOND_TICK_TMR_PERIOD_MS * 2), Indication_PeriodicCbk);
	    		*retry_cnt = 0;
	    	}
	    	break;
	    default:
	    	break;
	}
}

static void Indication_SetIndication (ind_pattern_t ind_pattern, uint8_t ind_type)
{
	uint8_t tmr_id = (ind_type == STATUS_TYPE) ? INDICATION_STATUS_TMR : INDICATION_ERROR_TMR;

	/* Set indication type */
	if (ind_type == STATUS_TYPE)
	{
		Indication_ClearStatusIndication();
		indStatus.stat_pattern = ind_pattern;
	}
	else
	{
		Indication_ClearErrorIndication();
		indStatus.err_pattern = ind_pattern;
	}

	switch (ind_pattern)
	{
		case IND_PATTERN_SOLID:
			Indication_Start(tmr_id);
			break;
		case IND_PATTERN_SINGLE_SHORT:
			Indication_Start(tmr_id);
			Timer_Start (tmr_id, INDICATION_TMR_SHORT_SOUND_PERIOD_MS, Indication_Stop);
			break;
		case IND_PATTERN_SINGLE_LONG:
			Indication_Start(tmr_id);
			Timer_Start (tmr_id, INDICATION_TMR_LONG_SOUND_PERIOD_MS, Indication_Stop);
			break;
		case IND_PATTERN_SINGLE_5_SEC:
			(tmr_id == INDICATION_STATUS_TMR) ? SET_STATUS_LED(true) : SET_ERROR_LED(true);
			Timer_Start (tmr_id, INDICATION_TMR_5SEC_SOUND_PERIOD_MS, Indication_Stop);
			break;
		case IND_PATTERN_VERY_LONG:
			Indication_Start(tmr_id);
			Timer_Start (tmr_id, INDICATION_TMR_1SEC_SOUND_PERIOD_MS, Indication_Stop);
			break;
		case IND_PATTERN_PERIODIC_50_50:
		case IND_PATTERN_PERIODIC_1_SHORT_PLUS_PAUSE:
		case IND_PATTERN_PERIODIC_2_SHORT_PLUS_PAUSE:
		case IND_PATTERN_PERIODIC_2_SHORT_PLUS_LONG_PAUSE:
			Indication_PeriodicCbk(tmr_id);
			break;
		default:
			break;
	}
}

void Indication_Reset (void)
{
	memset(&indStatus, 0u, sizeof(indication_status_t));
	Indication_ClearStatusIndication ();
	Indication_ClearErrorIndication ();
}

static uint8_t Indication_GetHighErrIdx (uint32_t err_code_mask)
{
	uint8_t idx, ret = IND_NO_ERROR_FOUND;

	/* Go through the error codes and find the most prioritized */
	if (err_code_mask != 0u)
	{
		for (idx = 0; idx < ERR_CODE_MAX_ERROR; idx++)
		{
			if (Util_IsFlagChecked((uint32_t*)&err_code_mask, idx))
			{
				return idx;
			}
		}
	}

	return ret;
}

static void Indication_UpdateErr (err_code_t err_code)
{
	/* Clear pending indication */
	Indication_ClearErrorIndication();

	switch (err_code)
	{
	    case ERR_CODE_FUSE_INCORRECT_STATE:
	    case ERR_CODE_UNEXPECTED_ARM:
	    case ERR_CODE_UNEXPECTED_IGNITION:
	    case ERR_CODE_UNEXPECTED_MINING:
	    case ERR_CODE_UNEXPECTED_VUSA_SHORTED:
	    	if (indStatus.curr_status != IND_STATUS_CONFIGURATION_MODE)
	    	{
		    	/* Clear status indication */
		    	indStatus.curr_status = IND_STATUS_NONE;
	    		/* Clear indication only if not configuration mode */
	    		Indication_ClearStatusIndication();
	    	}
	    	/* Set continuous error */
	    	Indication_SetIndication(IND_PATTERN_SOLID, ERROR_TYPE);
	    	break;
	    case ERR_CODE_STRONG_ALARM:
	    	/* Set periodic error */
	    	Indication_SetIndication(IND_PATTERN_PERIODIC_50_50, ERROR_TYPE);
	    	break;
	    case ERR_CODE_BATTERY_LOW:
	    	/* Set periodic short 2 pulse error */
	    	Indication_SetIndication(IND_PATTERN_PERIODIC_2_SHORT_PLUS_PAUSE, ERROR_TYPE);
	    	break;
	    default:
	    	break;
	}
}

void Indication_SetError (err_code_t err_code)
{
	/* Ignore error if it's already set */
	if (Util_IsFlagChecked((uint32_t*)&indStatus.err_code_mask, err_code) != false)
	{
		return;
	}

	if (
			(indStatus.err_code_mask == 0) ||
		   ((indStatus.err_code_mask != 0) &&
			(Indication_GetHighErrIdx(indStatus.err_code_mask) > err_code))
		)
	{
		/* Set error code mask */
		Util_SetFlag((uint32_t*)&indStatus.err_code_mask, err_code);

		/* Process error code if incoming error has higher priority then pending one */
		Indication_UpdateErr (err_code);
	}

	/* Set error code mask */
	Util_SetFlag((uint32_t*)&indStatus.err_code_mask, err_code);
}

void Indication_ClearError (err_code_t err_code)
{
	/* Ignore clearing error if it's already cleared */
	if (Util_IsFlagChecked((uint32_t*)&indStatus.err_code_mask, err_code) == false)
	{
		return;
	}

	if (
			(indStatus.err_code_mask != 0) &&
			(Indication_GetHighErrIdx(indStatus.err_code_mask) >= err_code)
		)
	{
		/* Clear error code mask */
		Util_RemoveFlag((uint32_t*)&indStatus.err_code_mask, err_code);
		/* Clear current indication */
		Indication_ClearErrorIndication();

		/* Check if any pending error are present as set it */
		if (indStatus.err_code_mask != 0)
		{
			Indication_UpdateErr (Indication_GetHighErrIdx(indStatus.err_code_mask));
		}
	}
	else
	{
	    /* Clear error code mask as higher priority error is present */
	    Util_RemoveFlag((uint32_t*)&indStatus.err_code_mask, err_code);
	}
}

void Indication_SetStatus (ind_status_t status_code, uint32_t user_data)
{
	/* Update only if there no pending errors */

	if ((indStatus.err_code_mask == 0) || (status_code == IND_STATUS_CONFIGURATION_MODE))
	{
		if (
			   ((indStatus.curr_status == status_code)        &&
				(status_code == IND_STATUS_FUSE_POPULATED)) ||
				((indStatus.curr_status == IND_STATUS_CONFIGURATION_MODE) &&
				 (status_code != IND_STATUS_CONFIGURATION_APPLIED))
			)
		{
			//Skip indication clear
			return;
		}
		else if (indStatus.err_code_mask == 0)
		{
			/* Clear pending indication */
			Indication_ClearStatusIndication();
		}

		switch(status_code)
		{
		    case IND_STATUS_WAIT_TMR_SHORT_TICK:
		    case IND_STATUS_SET_SAFE_IND:
		    case IND_MANUAL_MINING_ENABLED:
		    	Indication_SetIndication(IND_PATTERN_SINGLE_SHORT, STATUS_TYPE);
		    	break;
		    case IND_STATUS_WAIT_TMR_RAPID_TICK:
		    	Indication_SetIndication(IND_PATTERN_PERIODIC_50_50, STATUS_TYPE);
		    	break;
		    case IND_STATUS_SAFE_TMR_ELAPSED:
		    case IND_STATUS_CONFIGURATION_APPLIED:
		    	Indication_SetIndication(IND_PATTERN_VERY_LONG, STATUS_TYPE);
		    	break;
		    case IND_STATUS_INIT_SUCCESS:
		    case IND_STATUS_CHARGING_STOP:
		    case IND_STATUS_ARMED:
		    	Indication_SetIndication(IND_PATTERN_SINGLE_LONG, STATUS_TYPE);
		    	break;
		    case IND_STATUS_BOOM_START:
		    	BuzzerEnable();
		    	//Indication_SetIndication(IND_PATTERN_SOLID, STATUS_TYPE);
		    	//Indication_SetIndication(IND_PATTERN_SOLID, ERROR_TYPE);
		    	break;
		    case IND_STATUS_FUSE_POPULATED:
		    	if (indStatus.curr_status != status_code)
		    	{
		    		if (user_data == 0u)
		    		{
		    			Indication_SetIndication(IND_PATTERN_SINGLE_LONG, STATUS_TYPE);
		    		}
		    		else
		    		{
		    			Indication_SetIndication(IND_PATTERN_VERY_LONG, STATUS_TYPE);
		    		}
		    	}
		    	break;
		    case IND_STATUS_CHARGING_START:
		    	Indication_SetIndication(IND_PATTERN_PERIODIC_50_50, STATUS_TYPE);
		    	break;
		    case IND_STATUS_DESTRUCTION_START:
		    	Indication_SetIndication(IND_PATTERN_PERIODIC_50_50, ERROR_TYPE);
		    	Indication_SetIndication(IND_PATTERN_PERIODIC_50_50, STATUS_TYPE);
		    	break;
		    case IND_STATUS_DESTRUCTION_STOP:
		    case IND_STATUS_BOOM_STOP:
		    	Indication_ClearErrorIndication();
		    	Indication_ClearStatusIndication();
		    	break;
		    case IND_STATUS_CONFIGURATION_MODE:
		    	Indication_SetIndication(IND_PATTERN_SINGLE_5_SEC, STATUS_TYPE);
				/* Force buzzer in case of fuse error */
				if (Util_IsFlagChecked((uint32_t*)&indStatus.err_code_mask, ERR_CODE_FUSE_INCORRECT_STATE) != false)
				{
					Indication_Start(INDICATION_ERROR_TMR);
				}
		    	break;
		    default:
		    	break;
		}

		indStatus.curr_status = status_code;
	}
}

void Indication_Task (void)
{
	if (indStatus.handle_stat_tmr_cbk != false)
	{
		Indication_HandleTmrCbk(INDICATION_STATUS_TMR);
		indStatus.handle_stat_tmr_cbk = false;
	}
	else if (indStatus.handle_err_tmr_cbk != false)
	{
		Indication_HandleTmrCbk(INDICATION_ERROR_TMR);
		indStatus.handle_err_tmr_cbk = false;
	}
#if 0
	if (
			(indStatus.stat_pattern != IND_PATTERN_NONE) &&
			(indStatus.curr_status == 0u)
		)
		{
			indStatus.stat_pattern = IND_PATTERN_NONE;
			Indication_ClearStatusIndication();
		}

	if (
			(indStatus.err_pattern != IND_PATTERN_NONE) &&
			(indStatus.err_code_mask == 0u) &&
			(indStatus.warn_code_mask == 0u)
		)
		{
			indStatus.err_pattern = IND_PATTERN_NONE;
			Indication_ClearErrorIndication();
		}
#endif
}
