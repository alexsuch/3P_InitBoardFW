
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TIMER_H
#define __TIMER_H
   
#ifdef __cplusplus
extern "C" {
#endif

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define MAX_TIMER_NUMB                               (20u)

#define DUMMY_TMR                                    (0xFFu)

#define ACC_TIMEOUT_TMR                              (0u)
#define ACC_TIMEOUT_TMR_RETRY_PERIOD_MS              (10u)
#define ACC_TIMEOUT_TMR_READ_RETRY_PERIOD_MS         (10u)

#define SAFE_TIMEOUT_TMR                             (1u)
#define ONE_SECOND_TICK_TMR_PERIOD_MS                (1000u) //1 second

#define SYS_INIT_TMR                                 (2u)
#define SYS_INIT_TMR_PERIOD_MS                       (500u)

#define INDICATION_STATUS_TMR                    	 (3u)
#define INDICATION_ERROR_TMR                    	 (4u)
#define INDICATION_TMR_SHORT_SOUND_PERIOD_MS  	     (100u)
#define INDICATION_TMR_LONG_SOUND_PERIOD_MS   	     (250u)
#define INDICATION_TMR_VERY_LONG_SOUND_PERIOD_MS     (500u)
#define INDICATION_TMR_1SEC_SOUND_PERIOD_MS          (1000u)
#define INDICATION_TMR_5SEC_SOUND_PERIOD_MS          (5000u)
    
#define ADC_MEASURE_TMR                              (5u)
#define ADC_START_MEASURE_TMR_PERIOD_MS  		  	 (50u)
#define LOW_POWER_MONITOR_TMR_PERIOD_MS  		     (10000u) //10 sec

#define FUSE_CHECK_TMR                               (6u)
#define FUSE_CHECK_TMR_POLL_PERIOD_MS                (100u)
#define FUSE_CHECK_DELAY_PERIOD_MS                   (20u)

#define VUSA_CHECK_TMR                               (7u)
#define VUSA_POLL_TMR_DELAY_PERIOD_MS                (200u)

#define SELF_DESTRUCTION_TMR                         (8u)
#define SELF_DESTRUCTION_TMR_LOST_PERIOD_MINUTES     (180u)

#define MINING_ACTIVATE_TMR                          (9u)

#define IGNITION_TMR                                 (10u)
#define IGNITION_TMR_LOW_SIDE_OFF_PERIOD_MS          (50u)

#if (CONTROL_MODE == PWM_CTRL_SUPP)
#define STICK1_LOST_TMR                        		 (11u)
#define STICK2_LOST_TMR                        		 (12u)
#define STICK_LOST_PERIOD_MS  		     			 (500u)
#elif (CONTROL_MODE == MAVLINK_V2_CTRL_SUPP)
// Mavlink Timer IDs
#define MAVLINK_INITBOARD_HEARTBEAT_TMR              (11u)
#define MAVLINK_AUTOPILOT_CONNECTION_TIMEOUT_TMR     (12u)
#endif

#define CHARGING_TMR                        		 (13u)

#define BLOCK_IGNITION_TMR                           (14u)

#define LOGGER_SEND_TMR                              (18u)
#define LOGGER_SEND_TMR_PERIOD_MS                    (1000u)  /* Send Logger packet every 1 second */

#define SELF_DISTRUCTION_IND_TMR                     (15u)

#define UART_CONFIGURATOR_TMR                        (16u)
#define UART_CONFIGURATOR_INIT_TIMEOUT_MS            (300u)
#define UART_CONFIGURATOR_PING_TIMEOUT_MS            (2500u)

#define FLIGHT_PARAMS_UPDATE_TMR                     (17u)
#define FLIGHT_PARAMS_UPDATE_TMR_OSD_PERIOD_MS       (500u)

#define MILISECONDS_IN_SECOND                        (1000u)
#define MILISECONDS_IN_MINUTE                        (MILISECONDS_IN_SECOND * 60u)
#define MILISECONDS_IN_HOUR                          (MILISECONDS_IN_MINUTE * 60u)
#define MINUTES_IN_HOUR                              (60u)
#define SECONDS_IN_MINUTE                            (60u)

typedef void (*tmr_cbk)(uint8_t tmr_id);    
    
typedef struct
{
    uint8_t timer_id;
    bool is_active;
    uint32_t curr_time;
    tmr_cbk cbk;
    
} timers_t;
   
bool Timer_IsActive(uint8_t timer_id);
bool Timer_Start(uint8_t timer_id, uint32_t timer_period_ms, tmr_cbk cbk);
void Timer_ResetAll(void);
void init_global_timer(void);
void Timer_Stop(uint8_t timer_id);
void Timer_TickCbk (void);
void Timer_Task (void);
uint32_t Timer_GetTime(uint8_t timer_id);

void Util_SetFlag(uint32_t *event_map, uint8_t event_sel);
bool Util_IsFlagChecked(uint32_t *event_map, uint8_t event_sel);
void Util_RemoveFlag(uint32_t *event_map, uint8_t event_sel);

#ifdef __cplusplus
}
#endif

#endif /* __TIMER_H */
