
#include <stdlib.h>
#include <stdio.h>
#include "timer.h"
#include "solution_wrapper.h"


static timers_t timers[MAX_TIMER_NUMB];
static volatile bool tmr_tick_ready = false;
static uint8_t gl_tmr_idx;
static timers_t* gl_curr_tmr = NULL;


void Timer_ResetAll()
{
    memset(timers, 0, sizeof(timers));
}

bool Timer_IsActive(uint8_t timer_id)
{
    uint8_t tmr_idx = 0;
    bool ret = false;

    /* Disable interrupts */
    uint32_t prim = __get_PRIMASK();
    __disable_irq();
    
    /* go through all timers */
    for (tmr_idx = 0; tmr_idx < MAX_TIMER_NUMB; tmr_idx++)
    {
        if ((timers[tmr_idx].timer_id == timer_id) && (timers[tmr_idx].is_active == true))
        {
            ret = true;
            break;
        }
    }
    
    /* Enable interrupts back */
    __set_PRIMASK(prim);

    return ret;
}

bool Timer_Start(uint8_t timer_id, uint32_t timer_period_ms, tmr_cbk cbk)
{
    uint8_t tmr_idx = 0;
    bool stat = false;
    
    /* Disable interrupts */
    uint32_t prim = __get_PRIMASK();
    __disable_irq();

    Timer_Stop(timer_id);
    
    /* go through all timers */
    for (tmr_idx = 0; tmr_idx < MAX_TIMER_NUMB; tmr_idx++)
    {
        if (timers[tmr_idx].is_active != true)
        {
            timers[tmr_idx].is_active = true;
            /* NOTE: FW uses 10ms tick  */
            timers[tmr_idx].curr_time = timer_period_ms / 10;
            timers[tmr_idx].timer_id = timer_id;
            if (cbk != NULL)
            {
                timers[tmr_idx].cbk = cbk;
            }
            stat = true;
            break;
        }
    }

    /* Enable interrupts back */
    __set_PRIMASK(prim);
        
    return stat;
}

void Timer_Stop(uint8_t timer_id)
{
    uint8_t tmr_idx = 0;
    
    /* Disable interrupts */
    uint32_t prim = __get_PRIMASK();
    __disable_irq();

    /* go through all timers */
    for (tmr_idx = 0; tmr_idx < MAX_TIMER_NUMB; tmr_idx++)
    {
        if (timers[tmr_idx].timer_id == timer_id)
        {
            memset(&(timers[tmr_idx]), 0, sizeof(timers_t));
        }
    }

    /* Enable interrupts back */
    __set_PRIMASK(prim);
}

uint32_t Timer_GetTime(uint8_t timer_id)
{
    uint8_t tmr_idx;
    uint32_t ret_time = 0u;
    
    /* go through all timers */
    for (tmr_idx = 0; tmr_idx < MAX_TIMER_NUMB; tmr_idx++)
    {
        if (timers[tmr_idx].timer_id == timer_id)
        {
            ret_time = timers[tmr_idx].curr_time * 10;
        }
    }
    
    return ret_time;
}

void Timer_Task (void)
{
	if (tmr_tick_ready != false)
	{
	    /* go through all timers */
	    for (gl_tmr_idx = 0u; gl_tmr_idx < MAX_TIMER_NUMB; gl_tmr_idx++)
	    {
	    	gl_curr_tmr = &(timers[gl_tmr_idx]);
	        if (gl_curr_tmr->is_active != false)
	        {
	        	gl_curr_tmr->curr_time--;
	            if (gl_curr_tmr->curr_time <= 0u)
	            {
	                if (gl_curr_tmr->cbk != NULL)
	                {
	                    /* Run callback function */
	                	gl_curr_tmr->cbk(timers[gl_tmr_idx].timer_id);
	                }
	                /* Check if timer was modified */
	                if (gl_curr_tmr->curr_time == 0u)
	                    memset(gl_curr_tmr, 0u, sizeof(timers_t));
	            }
	        }
	    }

	    tmr_tick_ready = false;
	}
}

void Timer_TickCbk (void)
{
	tmr_tick_ready = true;
}

void Util_SetFlag(uint32_t *event_map, uint8_t event_sel)
{
    /* Disable interrupts */
	uint32_t prim = __get_PRIMASK();
    __disable_irq();

    *event_map |= (uint32_t)(1UL << event_sel);

    __set_PRIMASK(prim);
}

bool Util_IsFlagChecked(uint32_t *event_map, uint8_t event_sel)
{
	/* Disable interrupts */
	uint32_t prim = __get_PRIMASK();
	__disable_irq();

	uint32_t evt_stat = *event_map;
	bool ret = false;


	if ((evt_stat & (uint32_t)(1UL << event_sel)) != 0U)
		 ret = true;

    __set_PRIMASK(prim);

    return ret;
}

void Util_RemoveFlag(uint32_t *event_map, uint8_t event_sel)
{
	/* Disable interrupts */
	uint32_t prim = __get_PRIMASK();
	__disable_irq();

	*event_map &= (uint32_t)(~(1UL << event_sel));

    __set_PRIMASK(prim);
}

/* [] END OF FILE */
