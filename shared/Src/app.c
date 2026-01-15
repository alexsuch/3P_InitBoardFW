#include "app.h"

#include <stdio.h>
#include <stdlib.h>

#include "acc_proc.h"
#include "app_config.h"
#include "indication.h"
#include "init_brd.h"
#include "solution_wrapper.h"
#include "timer.h"
#include "uart_configurator.h"
#if SPI_LOGGER_ENABLE
#include "logger.h"
#endif /* SPI_LOGGER_ENABLE */
#if (CONTROL_MODE == MAVLINK_V2_CTRL_SUPP)
#include "mavlink_uart.h"
#endif /* (CONTROL_MODE == MAVLINK_V2_CTRL_SUPP) */
#if (CONTROL_MODE == PWM_CTRL_SUPP)
#include "stick_ctrl.h"
#endif /* (CONTROL_MODE == PWM_CTRL_SUPP) */
#if (OSD_ENABLE == 1u)
#include "msp_osd.h"
#endif /* (OSD_ENABLE == 1u) */

static system_status_t sysStatus;
static uint8_t idx = 0u;
static bool start_up;

static void App_SafeTmrStop(void);
static void App_ClearError(uint32_t err_code);
static void App_SetError(err_type_t err_type, uint32_t usr_data);
static void App_SafeTmrPause(void);
static void App_SafeTmrRelease(void);
static void App_ArmRun(void);
#if VBAT_MEASURE_FEATURE
static void App_AdcGetDataCbk(system_evt_t evt, uint32_t usr_data);
static void App_AdcStop(void);
#endif /* VBAT_MEASURE_FEATURE */
static void App_IgnitionRun(void);
static void App_DisarmRun(bool ind_enable);
static void App_FuseCheckRun(void);
static void App_SafeTmrTick(uint8_t timer_id);
static void App_ChargingRun(system_state_t charge_state, bool reset_var);
static void App_IgnitionSwitchOff(void);
static void App_IgnitionRun(void);
static void App_ArmConfigurationSet(void);
#if !SELF_DESTROY_DISABLE
static void App_SelfDestroyTmrTick(uint8_t timer_id);
static void App_SelfDestroyTmrTickCbk(void);
#endif /* !SELF_DESTROY_DISABLE */
#if MINING_MODE_SUPP
static void App_MiningModeEnableCbk(uint8_t timer_id);
static void App_MiningActivateTmrTick(uint8_t timer_id);
static void App_MiningActivateTmrTickCbk(void);
#endif /* MINING_MODE_SUPP */
#if UART_ENABLE
static bool App_UartConfigurationTask(void);
#endif /* UART_ENABLE */
#if (CONTROL_MODE == MAVLINK_V2_CTRL_SUPP)
static bool App_MavlinkIsAble2Arm(void);
static bool App_MavlinkIgnitionTryRun(void);
static bool App_MavlinkTry2Arm(void);
static void App_MavlinkFlightParamsTimerCbk(uint8_t timer_id);
static void App_MavlinkFlightParamsCheck(void);
#endif /* (CONTROL_MODE == MAVLINK_V2_CTRL_SUPP) */

/***************************************** SET/GET CONFIGURATION HANDLERS HANDLERS ******************************/
void App_RefreshConfig(void) { sysStatus.config = AppConfig_GetConfiguration(); }

uint8_t App_GetAppStatusConfiguration(void) {
    /* Disable interrupts */
    uint32_t prim = __get_PRIMASK();
    __disable_irq();

    uint8_t stat = 0u;

    /* Update Fuse status */
    if (sysStatus.is_fuse_removed == false) {
        stat |= IS_FUSE_PRESENT_STATUS_MASK;
    }

    /* Update low battery flag */
    if (sysStatus.is_battery_low != false) {
        stat |= IS_BATTERY_LOW_STATUS_MASK;
    }

    /* Check if application is in hard error */
    if (sysStatus.state == SYSTEM_STATE_ERROR) {
        stat |= IS_APP_FAULT_STATUS_MASK;
    }
    /* Check if application is ready */
    else if (sysStatus.state == SYSTEM_STATE_CONFIGURE) {
        stat |= IS_APP_READY_STATUS_MASK;
    }

    __set_PRIMASK(prim);

    return stat;
}

uint8_t* App_GetConfiguration(uint8_t* size) {
    uint8_t* ret = NULL;

    if (size == NULL) {
        return ret;
    }

    ret = (uint8_t*)sysStatus.config;
    *size = sizeof(config_data_t);

    return ret;
}

bool App_SetConfiguration(uint8_t* data, uint8_t size) {
    bool ret = false;

    if (data == NULL) {
        return ret;
    }

    /* Configuration can be done only when Fuse is populated/ app version match/ sizes are match */
    if ((sysStatus.is_fuse_removed == false)
        //&& (sizeof(config_data_t) == size)
    ) {
        if (AppGonfig_IsAllElements(data, size, 0x00)) {
            /* Apply default configuration if required (all elements are zeros) */
            ret = AppConfig_ApplyDefaultConfig();
        } else {
            /* Save received configuration */
            ret = AppConfig_SaveConfiguration((config_data_t*)data);
        }
    }

    return ret;
}

/***************************************** ERROR HANDLERS ********************************************************/

static void App_SetError(err_type_t err_type, uint32_t usr_data) {
    /* Fatal error - need to reset power */
    if (err_type == ERR_TYPE_HARD_RST) {
        /* set application to the safe state */
        App_SetSafe(false);
        /* Stop all timers */
        Timer_ResetAll();
        /* Deinit blocks */
#if (CONTROL_MODE == PWM_CTRL_SUPP)
        Stick_Deinit();
#endif
#if ACC_SUPPORTED_ENABLE
        AccProc_Stop();  // TODO OSAV add force condition
#endif

        sysStatus.state = SYSTEM_STATE_ERROR;

        if (usr_data == ERR_CODE_BATTERY_LOW) {
            Indication_SetError(ERR_CODE_BATTERY_LOW);
        } else {
            /* Report the accelerometer error */
            Indication_SetError(ERR_CODE_ACCELEROMETER_FAIL);
        }

        /* Stop the system in case of strong error - handle indication only */
        while (1) {
            /* Leave just timer and indication tasks */
            Indication_Task();
            Timer_Task();
#if (CONTROL_MODE == MAVLINK_V2_CTRL_SUPP)
            /* Mavlink Processing */
            Mavlink_Process();
#endif
#if (OSD_ENABLE == 1u)
            /* OSD processing task */
            OsdMsp_Task();
#endif /* (OSD_ENABLE == 1u) */
        }
    }
    /* Error is active until error source will be removed. In that case Reload app layer */
    else if (err_type == ERR_TYPE_RELOADABLE) {
        if ((usr_data == ERR_CODE_UNEXPECTED_ARM) || (usr_data == ERR_CODE_UNEXPECTED_IGNITION) ||
#if MINING_MODE_SUPP
            (usr_data == ERR_CODE_UNEXPECTED_MINING) ||
#endif /* MINING_MODE_SUPP */
            (usr_data == ERR_CODE_FUSE_INCORRECT_STATE)
#if VUSA_ENABLE
            || (usr_data == ERR_CODE_UNEXPECTED_VUSA_SHORTED)
#endif /* VUSA_ENABLE */
        ) {
            /* Set error flag for tracking */
            Util_SetFlag((uint32_t*)&sysStatus.app_err_stat, usr_data);
            /* Report the error */
            Indication_SetError(usr_data);
        }
    }
}

static void App_ClearError(uint32_t err_code) {
    /* Clear the error */
    Util_RemoveFlag((uint32_t*)&sysStatus.app_err_stat, err_code);

    /* Report to clear pending errors */
    Indication_ClearError(err_code);
}

#if !SELF_DESTROY_DISABLE
/***************************************** SELF DESTRUCTION TIMER ********************************************/
static void App_SelfDestructionCbk(uint8_t timer_id) {
    if (SELF_DESTROY_INDICATE_LAST_SECONDS != 0u) {
        /* Stop indication */
        Indication_SetStatus(IND_STATUS_DESTRUCTION_STOP, 0u);
    }

    /* Check if system is ARMed */
    if ((sysStatus.state == SYSTEM_STATE_CHARGING) || (sysStatus.state == SYSTEM_STATE_DISARM)) {
        /* Start capacitor charging when ARM command comes */
        App_ChargingRun(SYSTEM_STATE_CHARGING, true);
        sysStatus.self_destroy_mode = SELF_DESTROY_STATE_CHARGING;
    } else if ((sysStatus.state == SYSTEM_STATE_ARMED) || (sysStatus.state == SYSTEM_STATE_ARMED_PUMP)) {
        App_IgnitionRun();
    }
}

static void App_SelfDestroyRunCbk(uint8_t timer_id) {
    /* Run self destruction handler */
    App_SelfDestructionCbk(DUMMY_TMR);
}

static void App_SelfDestroyInit(void) {
#if !SELF_DESTROY_DISABLE
    sysStatus.self_destroy_mode = SELF_DESTROY_STATE_STARTED;
    /* Initiate ignition immediately after self destroy timer elapsed for mining mode */
    App_SelfDestroyRunCbk(DUMMY_TMR);
#endif /* !SELF_DESTROY_DISABLE */
}

static void App_SelfDestroyTmrTick(uint8_t timer_id) {
    (void)timer_id;

    Util_SetFlag((uint32_t*)&sysStatus.app_task_mask, APP_TASK_SELF_DESTROY_TMR_TICK_CBK);
}

static void App_SelfDestroyTmrTickCbk(void) {
    /* Decrement timer tick */
    sysStatus.self_destroy_tmr_tick--;

#if MINING_MODE_SUPP
    /* Only update sys_info if Mining or Safe timer are not active (Mining + Safe have higher priority) */
    if ((!Timer_IsActive(MINING_ACTIVATE_TMR)) && (sysStatus.state != SYSTEM_STATE_SAFE_TIMER))
#endif /* MINING_MODE_SUPP */
    {
        /* Update timer mode */
        sysStatus.sys_info.timer_mode = TIMER_MODE_SELF_DESTROY;

        /* Update timer info */
        sysStatus.sys_info.timer_seconds = sysStatus.self_destroy_tmr_tick;
    }

    if (sysStatus.self_destroy_tmr_tick == 0u) {
        /* Timer expired - execute self destruction */
        Util_SetFlag((uint32_t*)&sysStatus.app_task_mask, APP_TASK_SELF_DESTROY_INIT_CBK);
    } else {
        /* Enable indication in the last seconds before self destroy. Not applicable for mining case */
        if ((SELF_DESTROY_INDICATE_LAST_SECONDS != 0u) && (sysStatus.self_destroy_tmr_tick == SELF_DESTROY_INDICATE_LAST_SECONDS)
#if MINING_MODE_SUPP
            && (sysStatus.mining_state != MINING_STATE_IGNITION)
#endif /* MINING_MODE_SUPP */
        ) {
            if (sysStatus.vbat_voltage_mv < BATTERY_VOLTAGE_LOW_NO_SOUND_THRESHOLD_MILIVOLTS) {
                /* If battery almost discharged provide indication without sound */
                Indication_SetStatus(IND_STATUS_DESTRUCTION_START_NO_SOUND, 0u);
            } else if (sysStatus.is_battery_low == false) {
                /* Indicate self destruction with sound if battery is not low */
                Indication_SetStatus(IND_STATUS_DESTRUCTION_START, 0u);
            }
        }

        /* Continue countdown - restart timer for next second */
        Timer_Start(SELF_DESTRUCTION_TMR, ONE_SECOND_TICK_TMR_PERIOD_MS, App_SelfDestroyTmrTick);
    }
}

static void App_SelfDestroyTimerStart(self_destroy_mode_t mode) {
#if SELF_DESTROY_DISABLE
    return;
#endif /* SELF_DESTROY_DISABLE */

    if (mode == SELF_DESTROY_MODE_NORMAL) {
        if (sysStatus.config->selfDestroyTimeoutMin != 0u) {
            sysStatus.self_destroy_tmr_tick = sysStatus.config->selfDestroyTimeoutMin * SECONDS_IN_MINUTE;  // Convert minutes to seconds
            Timer_Start(SELF_DESTRUCTION_TMR, ONE_SECOND_TICK_TMR_PERIOD_MS, App_SelfDestroyTmrTick);
        }

#if MINING_MODE_SUPP
        /* Check if mining feature is enabled */
        if ((sysStatus.config->miningMode == MINING_MODE_AUTO) && ((sysStatus.config->selfDestroyTimeoutMin > sysStatus.config->miningAutoActivationMin) ||
                                                                      // Low battery self destruction activation
                                                                      (sysStatus.config->selfDestroyTimeoutMin == 0u))) {
            /* Start mining activate timer with second-by-second countdown */
            sysStatus.mining_activate_tmr_tick = sysStatus.config->miningAutoActivationMin * SECONDS_IN_MINUTE;  // Convert minutes to seconds
            Timer_Start(MINING_ACTIVATE_TMR, ONE_SECOND_TICK_TMR_PERIOD_MS, App_MiningActivateTmrTick);
        }
#endif /* MINING_MODE_SUPP */
    } else if (mode == SELF_DESTROY_MODE_CTRL_LOST) {
        // Set 3 hours by default if connection was lost
        sysStatus.self_destroy_tmr_tick = SELF_DESTRUCTION_TMR_LOST_PERIOD_MINUTES * SECONDS_IN_MINUTE;  // Convert minutes to seconds
        Timer_Start(SELF_DESTRUCTION_TMR, ONE_SECOND_TICK_TMR_PERIOD_MS, App_SelfDestroyTmrTick);
    } else {
        // empty handler
    }
}

static void App_SelfDestroyTimerStop(void) {
    Timer_Stop(SELF_DESTRUCTION_TMR);
#if MINING_MODE_SUPP
    Timer_Stop(MINING_ACTIVATE_TMR);
#endif /* MINING_MODE_SUPP */
    Timer_Stop(SELF_DISTRUCTION_IND_TMR);
    /* Clear related indication */
    Indication_SetStatus(IND_STATUS_DESTRUCTION_STOP, 0u);

    /* Update system info */
    sysStatus.sys_info.timer_mode = TIMER_MODE_NONE;
    sysStatus.sys_info.timer_seconds = 0u;

    /* Reset self destroy timer counter */
    sysStatus.self_destroy_tmr_tick = sysStatus.config->selfDestroyTimeoutMin * SECONDS_IN_MINUTE;

#if MINING_MODE_SUPP
    /* Reset mining activate timer counter */
    sysStatus.mining_activate_tmr_tick = sysStatus.config->miningAutoActivationMin * SECONDS_IN_MINUTE;
#endif /* MINING_MODE_SUPP */
}
#endif /* !SELF_DESTROY_DISABLE */

#if MINING_MODE_SUPP
static void App_MiningActivateTmrTick(uint8_t timer_id) {
    (void)timer_id;

    Util_SetFlag((uint32_t*)&sysStatus.app_task_mask, APP_TASK_MINING_ACTIVATE_TMR_TICK_CBK);
}

static void App_MiningActivateTmrTickCbk(void) {
    /* Decrement timer tick */
    sysStatus.mining_activate_tmr_tick--;

    /* Only update sys_info if Safe timer is not active (Safe has higher priority) */
    if (sysStatus.state != SYSTEM_STATE_SAFE_TIMER) {
        /* Update timer mode to MINING (Mining timer has higher priority than Self Destroy) */
        sysStatus.sys_info.timer_mode = TIMER_MODE_MINING;

        /* Update timer info */
        sysStatus.sys_info.timer_seconds = sysStatus.mining_activate_tmr_tick;
    }

    if (sysStatus.mining_activate_tmr_tick == 0u) {
        /* Timer expired - enable mining feature */
        sysStatus.mining_state = MINING_STATE_ENABLING;

        /* Clear indication */
        Indication_SetStatus(IND_STATUS_NONE, 0u);

        /* Mining timer finished - restore Self Destroy timer info if it's still active */
        if (Timer_IsActive(SELF_DESTRUCTION_TMR)) {
            sysStatus.sys_info.timer_mode = TIMER_MODE_SELF_DESTROY;
            sysStatus.sys_info.timer_seconds = sysStatus.self_destroy_tmr_tick;
        } else {
            sysStatus.sys_info.timer_mode = TIMER_MODE_NONE;
            sysStatus.sys_info.timer_seconds = 0u;
        }

        if (sysStatus.state == SYSTEM_STATE_ARMED) {
            /* System is already charged */
            App_ArmRun();
        } else {
            /* Goto ARM state */
            App_ChargingRun(SYSTEM_STATE_CHARGING, true);
            /* Enable mining feature because of variables reset as a part of App_ChargingRun function */
            sysStatus.mining_state = MINING_STATE_ENABLING;
        }
    } else {
        /* Continue countdown - restart timer for next second */
        Timer_Start(MINING_ACTIVATE_TMR, ONE_SECOND_TICK_TMR_PERIOD_MS, App_MiningActivateTmrTick);
    }
}

static void App_MiningModeEnableCbk(uint8_t timer_id) {
    /* This is the old callback that will be replaced by the timer tick system */
    /* Enable mining feature */
    sysStatus.mining_state = MINING_STATE_ENABLING;

    /* Clear indication */
    Indication_SetStatus(IND_STATUS_NONE, 0u);

    if (sysStatus.state == SYSTEM_STATE_ARMED) {
        /* System is already charged */
        App_ArmRun();
    } else {
        /* Goto ARM state */
        App_ChargingRun(SYSTEM_STATE_CHARGING, true);
        /* Enable mining feature because of variables reset as a part of App_ChargingRun function */
        sysStatus.mining_state = MINING_STATE_ENABLING;
    }
}

static void App_MiningModeDisable(void) {
    Timer_Stop(MINING_ACTIVATE_TMR);

    if (sysStatus.mining_state != MINING_STATE_NONE) {
        if (sysStatus.state > SYSTEM_STATE_SAFE_TIMER) {
            Timer_Stop(SAFE_TIMEOUT_TMR);
        }

        /* Disable mining feature */
        sysStatus.mining_state = MINING_STATE_NONE;

        /* Update board state based on current system state */
        if ((sysStatus.state == SYSTEM_STATE_ARMED) || (sysStatus.state == SYSTEM_STATE_ARMED_PUMP)) {
            sysStatus.sys_info.board_state = BOARD_STATE_CHARGED;
        } else if (sysStatus.state == SYSTEM_STATE_CHARGING) {
            sysStatus.sys_info.board_state = BOARD_STATE_CHARGING;
        } else {
            sysStatus.sys_info.board_state = BOARD_STATE_INIT;
        }

        /* Mining timer stopped - restore Self Destroy timer info if it's still active */
        if (Timer_IsActive(SELF_DESTRUCTION_TMR)) {
            sysStatus.sys_info.timer_mode = TIMER_MODE_SELF_DESTROY;
            sysStatus.sys_info.timer_seconds = sysStatus.self_destroy_tmr_tick;
        } else {
            sysStatus.sys_info.timer_mode = TIMER_MODE_NONE;
            sysStatus.sys_info.timer_seconds = 0u;
        }

        /* Disable low battery self destruction mechanism */
        sysStatus.sys_info.low_pwr_self_dest_allowed = false;

        /* Disable any detection */
#if ACC_SUPPORTED_ENABLE
        AccProc_Stop();
#endif
    }
}
#endif /* MINING_MODE_SUPP */

#if VUSA_ENABLE
/***************************************** VUSA HANDLERS ********************************************************/
static void App_VusaHandleCbk(bool vusaShorted);

static void App_VusaShortCheckTmrCbk(uint8_t timer_id) {
    /* If timer elapsed - VUSA is not detected */
    App_VusaHandleCbk(false);
}

static void App_VusaRunCheckTmrCbk(void) {
    /* Run timer to detect if Vusa disconnected*/
    Timer_Start(VUSA_CHECK_TMR, VUSA_POLL_TMR_DELAY_PERIOD_MS, App_VusaShortCheckTmrCbk);
}

static void App_VusaHandleCbk(bool vusaShorted) {
    /* Get Vusa state */
    vusa_state_t new_vusa_state = (vusaShorted) ? VUSA_STATE_SHORTED : VUSA_STATE_NOT_SHORTED;

    if (new_vusa_state == VUSA_STATE_SHORTED) {
        /* Run timer to detect if VUSA not shorted */
        Util_SetFlag((uint32_t*)&sysStatus.app_task_mask, APP_TASK_VUSA_RUN_TMR_CBK);
    }

    if (sysStatus.vusa_state != new_vusa_state) {
        /* Save current fuse state */
        sysStatus.vusa_state = new_vusa_state;

        if (((sysStatus.state == SYSTEM_STATE_ARMED) || (sysStatus.state == SYSTEM_STATE_ARMED_PUMP)) && (new_vusa_state == VUSA_STATE_SHORTED)) {
            /* Initiate ignition */
            App_IgnitionRun();
        } else {
            if (new_vusa_state == VUSA_STATE_SHORTED) {
                /* We reporting error that VUSA is shorted but don't stop the safe timer */
                App_SetError(ERR_TYPE_RELOADABLE, ERR_CODE_UNEXPECTED_VUSA_SHORTED);
            } else {
                /* Clear the error */
                App_ClearError(ERR_CODE_UNEXPECTED_VUSA_SHORTED);

                /* Restore charging indication is it was interrupted */
                if (sysStatus.state == SYSTEM_STATE_CHARGING) {
                    Indication_SetStatus(IND_STATUS_CHARGING_START, 0u);
                }
            }
        }
    }
}

static void App_VusaCbk(system_evt_t evt, uint32_t usr_data) {
    if (evt == SYSTEM_EVT_READY) {
        /* Set flag to run Vusa analysis */
        Util_SetFlag((uint32_t*)&sysStatus.app_task_mask, APP_TASK_VUSA_DETECTED_CBK);
    }
}
#endif /* VUSA_ENABLE */

#if (OSD_ENABLE == 1u)
/************************************ OSD HANDLERS *******************************************************/
static uint8_t App_OsdCbk(system_evt_t evt, uint32_t usr_data, void* usr_ptr) {
    uint8_t ret = 0;

    if (evt == SYSTEM_EVT_READY) {
        ret = 1;
    }

    return ret;
}

void App_OsdInit(void) {
    /* Set OSD enabled flag */
    sysStatus.osd_enabled = true;
    /* Initialize OSD module */
    OsdMsp_Init(App_OsdCbk, &sysStatus.sys_info);
}
#endif /* (OSD_ENABLE == 1u) */

/***************************************** STICK CONTROL ********************************************************/

static void App_DisarmHandler(bool is_stick) {
#if !SELF_DESTROY_DISABLE
    /* Stop self destruction timer in case of the stick control */
    App_SelfDestroyTimerStop();
#endif /* !SELF_DESTROY_DISABLE */

    /* Disable any detection */
#if ACC_SUPPORTED_ENABLE
    AccProc_Stop();
#endif
    /* Set indication for disarmed state */
    Indication_SetStatus(IND_STATUS_DISARMED, 0u);

#if (CONTROL_MODE == PWM_CTRL_SUPP)
    /* Reset block flag and stop the timer */
    sysStatus.is_ignition_blocked = false;
    Timer_Stop(BLOCK_IGNITION_TMR);
#endif

    /* Handle the case when Arm or Ignition signal came while Safe timer is pending and now switched back */
    if ((sysStatus.state == SYSTEM_STATE_SAFE_TIMER) && (sysStatus.safe_tmr_pause != false)) {
        /* Release current Safe timer */
        App_SafeTmrRelease();
    }

    /* System switched to the Safe state */
    if (((sysStatus.state >= SYSTEM_STATE_DISARM) && (sysStatus.state <= SYSTEM_STATE_IGNITION)) || (is_stick == false)) {
        App_DisarmRun(true);
    }

    /* Check if any pending error for stick */
    if (((Util_IsFlagChecked((uint32_t*)&sysStatus.app_err_stat, ERR_CODE_UNEXPECTED_IGNITION)) ||
            (Util_IsFlagChecked((uint32_t*)&sysStatus.app_err_stat, ERR_CODE_UNEXPECTED_ARM))) &&
        (is_stick != false)) {
        /* Clear errors */
        App_ClearError(ERR_CODE_UNEXPECTED_ARM);
        App_ClearError(ERR_CODE_UNEXPECTED_IGNITION);
    }
}

#if (CONTROL_MODE == PWM_CTRL_SUPP)

static void App_StickIgnitionUnblockCbk(uint8_t timer_id) {
    /* If timer elapsed - ignition is ready */
    sysStatus.is_ignition_blocked = false;
}

static void App_StickCbk(system_evt_t evt, uint32_t usr_data) {
#if 0
	Test2LedToggle();
#endif

#if MINING_MODE_SUPP
    if ((sysStatus.config->miningMode == MINING_MODE_MANUAL) && (usr_data >= CONTROL_EVT_MINING_DISABLE)) {
        if (sysStatus.mining_stick_ctrl_stat != usr_data) {
            /* Enable Mining only when disarmed */
            if (usr_data == CONTROL_EVT_MINING_ENABLE) {
                if (sysStatus.stick_ctrl_stat != CONTROL_EVT_DISARM) {
                    /* Set application error */
                    App_SetError(ERR_TYPE_RELOADABLE, ERR_CODE_UNEXPECTED_MINING);
                } else if (sysStatus.state == SYSTEM_STATE_DISARM) {
                    /* Mining mode enable */
                    sysStatus.mining_state = MINING_STATE_START_SAFE_TMR;
                    /* Enable self destroy timer */
                    App_SelfDestroyTimerStart(SELF_DESTROY_MODE_NORMAL);
                    /* Run safe timer for mining */
                    Timer_Start(SAFE_TIMEOUT_TMR, (sysStatus.config->miningEnableDelaySec * MILISECONDS_IN_SECOND), App_MiningModeEnableCbk);
                    /* Indicate mining enabling by stick */
                    Indication_SetStatus(IND_MANUAL_MINING_ENABLED, 0u);
                }
                /* Handle the case when mining signal came while Safe timer is pending */
                else if (sysStatus.state == SYSTEM_STATE_SAFE_TIMER) {
                    /* Pause current Safe timer */
                    App_SafeTmrPause();
                    /* Set application error */
                    App_SetError(ERR_TYPE_RELOADABLE, ERR_CODE_UNEXPECTED_MINING);
                } else {
                    // Empty handler
                }
            } else if (usr_data == CONTROL_EVT_MINING_DISABLE) {
                /* Handle the case when mining signal came while Safe timer is pending and now switched back */
                if ((sysStatus.state == SYSTEM_STATE_SAFE_TIMER) && (sysStatus.safe_tmr_pause != false)) {
                    /* Release current Safe timer */
                    App_SafeTmrRelease();
                } else if ((sysStatus.state > SYSTEM_STATE_SAFE_TIMER) && (sysStatus.mining_state != MINING_STATE_NONE)) {
                    /* Mining mode disable run only after safe timer  */
                    App_MiningModeDisable();
                    App_DisarmHandler(false);
                }

                /* Check if pending mining error for stick */
                if (Util_IsFlagChecked((uint32_t*)&sysStatus.app_err_stat, ERR_CODE_UNEXPECTED_MINING)) {
                    /* Clear error */
                    App_ClearError(ERR_CODE_UNEXPECTED_MINING);
                }

                if (sysStatus.state == SYSTEM_STATE_CHARGING) {
                    /* Restore charging indication is it was interrupted */
                    Indication_SetStatus(IND_STATUS_CHARGING_START, 0u);
                }
            }
        }

        /* Save mining stick status */
        sysStatus.mining_stick_ctrl_stat = usr_data;
        return;
    }
#endif /* MINING_MODE_SUPP */

    if (usr_data == CONTROL_EVT_NO_EVT) {
        // No data to process
        return;
    }

    if (usr_data == CONTROL_EVT_ARM_WO_ACC) {
        /* Disable accelerometer hit detection functionality */
        sysStatus.acc_hit_detection_enabled = false;
        usr_data = CONTROL_EVT_ARM_WITH_ACC;
    } else if ((sysStatus.config->accEnable != false)
#if MINING_MODE_SUPP
               && (sysStatus.mining_state == MINING_STATE_NONE)
#endif /* MINING_MODE_SUPP */
    ) {
        /* Enable accelerometer functionality */
        sysStatus.acc_hit_detection_enabled = true;

        /* Start hit accelerometer measurement if stick moved from ARM wo acc to ARM with Acc */
        if ((usr_data == CONTROL_EVT_ARM_WITH_ACC) && (sysStatus.stick_ctrl_stat == CONTROL_EVT_ARM_WITH_ACC)) {
#if ACC_SUPPORTED_ENABLE
            AccProc_HitDetectionStart();
#endif
        }
    }

    if (sysStatus.state == SYSTEM_STATE_INIT) {
        /* Remove error flag as we received response from stick layer during initialization */
        Util_RemoveFlag((uint32_t*)&sysStatus.err_stat, SYSTEM_BLOCK_CTRL);  // TODO OSAV add code to notify if 1 or 2 PWM channels
    }

    if (evt == SYSTEM_EVT_READY) {
        sysStatus.is_ctrl_lost = false;

        if (usr_data == CONTROL_EVT_NO_CTRL) {
            /* Signal was lost */
            if (sysStatus.stick_ctrl_mode == STICK_MODE_STICK_PWM) {
#if !SELF_DESTROY_DISABLE
                if ((sysStatus.state == SYSTEM_STATE_DISARM)
#if MINING_MODE_SUPP
                    && (sysStatus.mining_state == MINING_STATE_NONE)
#endif /* MINING_MODE_SUPP */
                ) {
                    /* Start self destroy timer if stick connection is lost when system was in disarm state */
                    App_SelfDestroyTimerStart(SELF_DESTROY_MODE_CTRL_LOST);
                }
#endif /* !SELF_DESTROY_DISABLE */
                /* Don't safe new state if control was lost */
                sysStatus.is_ctrl_lost = true;
                sysStatus.is_ignition_blocked = false;
            }

            if (sysStatus.is_ctrl_lost == false) {
                /* No control stick mode */
                sysStatus.stick_ctrl_mode = STICK_MODE_BYPASS;
                sysStatus.stick_ctrl_stat = CONTROL_EVT_NO_CTRL;
            }

            /* Check if any pending error for stick */
            if ((Util_IsFlagChecked((uint32_t*)&sysStatus.app_err_stat, ERR_CODE_UNEXPECTED_IGNITION)) ||
                (Util_IsFlagChecked((uint32_t*)&sysStatus.app_err_stat, ERR_CODE_UNEXPECTED_ARM)) ||
                (Util_IsFlagChecked((uint32_t*)&sysStatus.app_err_stat, ERR_CODE_UNEXPECTED_MINING))) {
                uint32_t tmp_err = sysStatus.app_err_stat;
                /* Clear errors */
                App_ClearError(ERR_CODE_UNEXPECTED_ARM);
                App_ClearError(ERR_CODE_UNEXPECTED_IGNITION);
                App_ClearError(ERR_CODE_UNEXPECTED_MINING);

                if (((sysStatus.state == SYSTEM_STATE_DISARM) || (sysStatus.state == SYSTEM_STATE_CHARGING)) &&
                    (Util_IsFlagChecked((uint32_t*)&tmp_err, ERR_CODE_UNEXPECTED_IGNITION))) {
                    /* Set error flag for the future tracking */
                    Util_SetFlag((uint32_t*)&sysStatus.app_err_stat, ERR_CODE_UNEXPECTED_IGNITION);
                }
            }
        } else {
            if ((sysStatus.stick_ctrl_mode == STICK_MODE_BYPASS) && (Util_IsFlagChecked((uint32_t*)&sysStatus.app_err_stat, ERR_CODE_UNEXPECTED_IGNITION))) {
                /* Signal appeared after was lost in wrong position goto disarm */
                App_DisarmHandler(true);
            }
            /* Otherwise stick mode */
            sysStatus.stick_ctrl_mode = STICK_MODE_STICK_PWM;
        }

        if (usr_data == CONTROL_EVT_DISARM) {
            /* Handle disarm stick */
            App_DisarmHandler(true);
        } else if ((usr_data == CONTROL_EVT_ARM_WITH_ACC) || (usr_data == CONTROL_EVT_IGNITION)) {
            /* Check if any pending error for stick to clear */
            if ((usr_data == CONTROL_EVT_ARM_WITH_ACC) && (Util_IsFlagChecked((uint32_t*)&sysStatus.app_err_stat, ERR_CODE_UNEXPECTED_IGNITION))) {
                App_ClearError(ERR_CODE_UNEXPECTED_IGNITION);
            } else if ((usr_data == CONTROL_EVT_IGNITION) && (Util_IsFlagChecked((uint32_t*)&sysStatus.app_err_stat, ERR_CODE_UNEXPECTED_ARM))) {
                App_ClearError(ERR_CODE_UNEXPECTED_ARM);
            }

            if ((sysStatus.state == SYSTEM_STATE_DISARM)
#if MINING_MODE_SUPP
                || ((sysStatus.config->miningMode == MINING_MODE_MANUAL) && (sysStatus.mining_state != MINING_STATE_NONE))
#endif /* MINING_MODE_SUPP */
            ) {
                if (usr_data == CONTROL_EVT_ARM_WITH_ACC) {
                    if (sysStatus.stick_ctrl_stat == CONTROL_EVT_DISARM) {
                        /* Start capacitor charging when ARM command comes */
                        App_ChargingRun(SYSTEM_STATE_CHARGING, true);
#if !SELF_DESTROY_DISABLE
                        if (Timer_IsActive(SELF_DESTRUCTION_TMR) == false) {
                            /* Start self destroy timer if ARMed */
                            App_SelfDestroyTimerStart(SELF_DESTROY_MODE_NORMAL);
                        }
#endif /* !SELF_DESTROY_DISABLE */
                    } else if ((sysStatus.stick_ctrl_stat == CONTROL_EVT_IGNITION) || (sysStatus.stick_ctrl_stat == CONTROL_EVT_ARM_WITH_ACC)) {
                        /* Indicate not valid ARM stick */
                        App_SetError(ERR_TYPE_RELOADABLE, ERR_CODE_UNEXPECTED_ARM);
                    }
                } else if (usr_data == CONTROL_EVT_IGNITION) {
                    /* Indicate not valid ignition stick */
                    App_SetError(ERR_TYPE_RELOADABLE, ERR_CODE_UNEXPECTED_IGNITION);
                }
            } else if ((sysStatus.state == SYSTEM_STATE_ARMED) || (sysStatus.state == SYSTEM_STATE_ARMED_PUMP)
#if !SELF_DESTROY_DISABLE
                       || (sysStatus.self_destroy_mode == SELF_DESTROY_STATE_STARTED)
#endif /* !SELF_DESTROY_DISABLE */
            ) {
                /* Process stick control when system is ready */
                if ((usr_data == CONTROL_EVT_IGNITION) && (sysStatus.stick_ctrl_stat == CONTROL_EVT_ARM_WITH_ACC) && (sysStatus.is_ignition_blocked == false)) {
                    /* Initiate ignition */
                    App_IgnitionRun();
                }
            }
            /* Handle the case when Arm or Ignition signal came while Safe timer is pending */
            else if (sysStatus.state == SYSTEM_STATE_SAFE_TIMER) {
                /* Pause current Safe timer */
                App_SafeTmrPause();
                /* Set application error */
                App_SetError(ERR_TYPE_RELOADABLE, (usr_data == CONTROL_EVT_ARM_WITH_ACC) ? ERR_CODE_UNEXPECTED_ARM : ERR_CODE_UNEXPECTED_IGNITION);
            }

            /* Block ignition if block timer not expired */
            if ((usr_data == CONTROL_EVT_IGNITION) && (sysStatus.is_ignition_blocked != false)) {
                /* Run timer to unblock ignition after timeout */
                Timer_Stop(BLOCK_IGNITION_TMR);
            } else if (usr_data == CONTROL_EVT_ARM_WITH_ACC) {
                /* Add condition to ignore Ignition for some time */
                sysStatus.is_ignition_blocked = true;

                /* Run timer to unblock ignition after timeout */
                Timer_Start(BLOCK_IGNITION_TMR, DISARM_IGNITION_BLOCK_TIMEOUT_MS, App_StickIgnitionUnblockCbk);
            }
        }

        if (sysStatus.is_ctrl_lost == false) {
            /* Save current stick state if required */
            sysStatus.stick_ctrl_stat = usr_data;
        }
    }
}
#endif /* (CONTROL_MODE == PWM_CTRL_SUPP) */

/***************************************** MAVLINK CONTROL ********************************************************/
#if (CONTROL_MODE == MAVLINK_V2_CTRL_SUPP)
static bool App_MavlinkIsAble2Arm(void) {
    return (sysStatus.sys_info.prearm_flag != 0) && (sysStatus.sys_info.is_charge_enabled != 0) &&
#if ACC_SHAKE_DETECTION_ENABLE
           (sysStatus.sys_info.shake_detected != 0) &&
#endif /* ACC_SHAKE_DETECTION_ENABLE */
#if FLIGHT_DETECTION_ENABLE
           (sysStatus.sys_info.is_flying != 0) &&
#endif /* FLIGHT_DETECTION_ENABLE */
#if FLIGHT_SPEED_ALTITUDE_DETECTION_ENABLE
           (sysStatus.sys_info.speed_altitude_flag != 0) &&
#endif /* FLIGHT_SPEED_ALTITUDE_DETECTION_ENABLE */
           (sysStatus.arm_enabled != false);
}

static void App_MavlinkFlightParamsTimerCbk(uint8_t timer_id) {
    (void)timer_id;
    sysStatus.sys_info.speed_altitude_flag = 1;

    // Set indication that stable flight parameters are OK
    Indication_SetStatus(IND_STATUS_SPEED_ALTITUDE_OK, 0u);

    /* Try to arm and start charging */
    App_MavlinkTry2Arm();
}

static void App_MavlinkFlightParamsCheck(void) {
    // Skip check if speed_altitude_flag is already set (stable flight detected)
    if (sysStatus.sys_info.speed_altitude_flag == 1) {
        return;
    }

    // Check if both speed and altitude are above minimum thresholds
    if ((sysStatus.current_speed_ms >= FLIGHT_SPEED_MINIMUM_THRESHOLD_M_S) && (sysStatus.current_altitude_m >= FLIGHT_ALTITUDE_MINIMUM_THRESHOLD_M)) {
        // Both parameters are stable - start stability timer if not already running
        if (!Timer_IsActive(FLIGHT_PARAMS_UPDATE_TMR)) {
            Timer_Start(FLIGHT_PARAMS_UPDATE_TMR, (FLIGHT_STABLE_PARAMETERS_TIMEOUT_SEC * ONE_SECOND_TICK_TMR_PERIOD_MS), App_MavlinkFlightParamsTimerCbk);
        }
    } else {
        // At least one parameter is below threshold - stop stability timer
        Timer_Stop(FLIGHT_PARAMS_UPDATE_TMR);
    }
}

static bool App_MavlinkIgnitionTryRun(void) {
    bool ret = false;

    /* If prearm and arm are enabled - run ignition */
    if (sysStatus.sys_info.prearm_flag && sysStatus.arm_enabled) {
        App_IgnitionRun();
        ret = true;
    }

    return ret;
}

static bool App_MavlinkTry2Arm(void) {
    bool ret = false;

    /* Check if able to arm and start charging */
    if (App_MavlinkIsAble2Arm()) {
        App_ChargingRun(SYSTEM_STATE_CHARGING, true);
        ret = true;
    } else {
        /* Set system to disarm state if system is not able to arm */
        App_DisarmHandler(false);
    }

    return ret;
}

static uint8_t App_MavlinkCbk(system_evt_t evt, uint32_t usr_data, void* usr_ptr) {
    uint8_t ret = TRUE;

    if (evt == SYSTEM_EVT_READY) {
        switch ((mavlink_event_t)usr_data) {
            case MAVLINK_EVT_COMMAND_IGNITION:
                /* Set false by default */
                ret = FALSE;

                /* Go through conditions if we able to run ignition */
                if ((sysStatus.state == SYSTEM_STATE_ARMED) || (sysStatus.state == SYSTEM_STATE_ARMED_PUMP)
#if !SELF_DESTROY_DISABLE
                    || (sysStatus.self_destroy_mode == SELF_DESTROY_STATE_STARTED)
#endif /* !SELF_DESTROY_DISABLE */
                ) {
                    /* Try to run ignition */
                    App_IgnitionRun();
                }

                break;

            case MAVLINK_EVT_AUTOPILOT_CONNECTED:
                // Autopilot connection established
                sysStatus.is_ctrl_lost = false;
                /* Update FC control presence */
                sysStatus.sys_info.fc_control_present = 1;
                break;

            case MAVLINK_EVT_AUTOPILOT_DISCONNECTED:
                // Autopilot connection lost
                sysStatus.is_ctrl_lost = true;
                /* Update FC control presence */
                sysStatus.sys_info.fc_control_present = 0;
                break;

            case MAVLINK_EVT_AUTOPILOT_HEARTBEAT:
                // Regular autopilot HEARTBEAT received
                sysStatus.is_ctrl_lost = false;
                /* Update FC control presence */
                sysStatus.sys_info.fc_control_present = 1;
                break;

            case MAVLINK_EVT_AUTOPILOT_ARMED:
                // Autopilot changed to ARMED state
                /* Update arm enabled state */
                sysStatus.arm_enabled = true;

                if (sysStatus.state == SYSTEM_STATE_DISARM) {
                    /* Try to arm and start charging */
                    App_MavlinkTry2Arm();

#if !SELF_DESTROY_DISABLE
                    if (Timer_IsActive(SELF_DESTRUCTION_TMR) == false) {
                        /* Start self destroy timer if ARMed */
                        App_SelfDestroyTimerStart(SELF_DESTROY_MODE_NORMAL);
                    }
#endif /* !SELF_DESTROY_DISABLE */
                }
                break;

            case MAVLINK_EVT_AUTOPILOT_DISARMED:
                // Autopilot changed to DISARMED state
                /* Update arm enabled state */
                sysStatus.arm_enabled = false;

                /* Set system to disarm state if system is not able to arm */
                if (sysStatus.state > SYSTEM_STATE_DISARM) {
                    App_DisarmHandler(false);
                }
                break;

            case MAVLINK_EVT_AUTOPILOT_PREARM_ENABLED:
                // Autopilot PREARM enabled
                sysStatus.sys_info.prearm_flag = 1;
                /* Try to arm and start charging */
                App_MavlinkTry2Arm();
                break;

            case MAVLINK_EVT_AUTOPILOT_PREARM_DISABLED:
                // Autopilot PREARM disabled
                sysStatus.sys_info.prearm_flag = 0;
                /* Set system to disarm state if system is not able to arm */
                if (sysStatus.state > SYSTEM_STATE_DISARM) {
                    App_DisarmHandler(false);
                }
                break;

            case MAVLINK_EVT_AUTOPILOT_CHARGE:
                // Autopilot charge enabled
                sysStatus.sys_info.is_charge_enabled = 1;
                /* Try to arm and start charging */
                App_MavlinkTry2Arm();
                break;

            case MAVLINK_EVT_AUTOPILOT_DISCHARGE:
                // Autopilot charge disabled
                sysStatus.sys_info.is_charge_enabled = 0;
                /* Set system to disarm state if system is not able to arm */
                if (sysStatus.state > SYSTEM_STATE_DISARM) {
                    App_DisarmHandler(false);
                }
                break;

#if FLIGHT_SPEED_ALTITUDE_DETECTION_ENABLE
            case MAVLINK_EVT_SPEED_RECEIVED:
                // Speed data received from autopilot
                if (usr_ptr != NULL) {
                    sysStatus.current_speed_ms = *(uint16_t*)usr_ptr;

                    // Check flight parameters stability
                    App_MavlinkFlightParamsCheck();
                }
                break;

            case MAVLINK_EVT_ALTITUDE_RECEIVED:
                // Altitude data received from autopilot
                if (usr_ptr != NULL) {
                    sysStatus.current_altitude_m = *(int32_t*)usr_ptr;

                    // Check flight parameters stability
                    App_MavlinkFlightParamsCheck();
                }
                break;
#endif /* FLIGHT_SPEED_ALTITUDE_DETECTION_ENABLE */

#if FLIGHT_DETECTION_ENABLE
            case MAVLINK_EVT_AUTOPILOT_FLYING:
                // Autopilot is in flight (IN_AIR, TAKEOFF, or LANDING state)
                sysStatus.sys_info.is_flying = 1;
                /* Try to arm and start charging */
                App_MavlinkTry2Arm();
                break;

            case MAVLINK_EVT_AUTOPILOT_LANDED:
                // Autopilot is on ground (ON_GROUND state)
                sysStatus.sys_info.is_flying = 0;
                /* Set system to disarm state if system is not able to arm */
                if (sysStatus.state > SYSTEM_STATE_DISARM) {
                    App_DisarmHandler(false);
                }
                break;
#endif /* FLIGHT_DETECTION_ENABLE */
            default:
                // Unknown Mavlink event
                break;
        }
    }

    return ret;
}
#endif

/***************************************** ACCELEROMETER PROCESSING  ********************************************************/

static void App_AccProcCbk(system_evt_t evt, uint32_t usr_data) {
    if ((evt == SYSTEM_EVT_READY) || ((usr_data == ACC_EVT_INIT_OK))) {
        /* Remove initial error flag */
        Util_RemoveFlag((uint32_t*)&sysStatus.err_stat, SYSTEM_BLOCK_ACC);

#if ACC_SHAKE_DETECTION_ENABLE
        /* Start shake detection immediately after accelerometer initialization */
        if (usr_data == ACC_EVT_INIT_OK) {
            AccProc_ShakeDetectionStart();
        }
#endif /* ACC_SHAKE_DETECTION_ENABLE */
    } else if ((evt == SYSTEM_EVT_ERROR) && (sysStatus.state == SYSTEM_STATE_INIT)) {
        /* Set error only during initialization, otherwise we ignore acc data */
        Util_SetFlag((uint32_t*)&sysStatus.err_stat, SYSTEM_BLOCK_ACC);
    }

    /* Ignore accelerometer if disabled except of mining mode */
    if ((((sysStatus.config->accEnable == false) || (sysStatus.acc_hit_detection_enabled == false)) && (usr_data != ACC_EVT_MOVE_DETECTED))) {
        return;
    }

    if (evt == SYSTEM_EVT_READY) {
        if ((usr_data == ACC_EVT_HIT_DETECTED) && (Util_IsFlagChecked((uint32_t*)&sysStatus.err_stat, SYSTEM_BLOCK_ACC) == false)) {
#if ACC_HIT_DETECTED_STICKY_LED_FEATURE
            /* Turn on test LED to indicate that accelerometer detected the hit */
            TestLedSet(true);
#endif
            /* Hit was detected - goto ignition */
            if ((sysStatus.state == SYSTEM_STATE_ARMED) || (sysStatus.state == SYSTEM_STATE_ARMED_PUMP)) {
                /* Initiate ignition */
                App_IgnitionRun();
#if ACC_HIT_DETECTED_STICKY_LED_FEATURE
                /* Turn on test LED to indicate that accelerometer detected the hit */
                TestLedSet(true);
#endif
            }
        }
#if MINING_MODE_SUPP
        else if (usr_data == ACC_EVT_MOVE_DETECTED) {
            /* Check if self destruction allowed */
            if ((sysStatus.mining_state == MINING_STATE_ACTIVE) && (sysStatus.self_destroy_mode == SELF_DESTROY_STATE_NONE)) {
                /* Set ignition root cause  */
                sysStatus.mining_state = MINING_STATE_IGNITION;
                /* Set flag to run self destroy handling */
                Util_SetFlag((uint32_t*)&sysStatus.app_task_mask, APP_TASK_SELF_DESTROY_INIT_CBK);
            }
        }
#endif /* MINING_MODE_SUPP */
#if ACC_SHAKE_DETECTION_ENABLE
        else if (usr_data == ACC_EVT_STARTUP_SHAKE_DETECTED) {
            /* Set shake detected flag in system info */
            sysStatus.sys_info.shake_detected = 1;
            /* Indicate shake detected */
            Indication_SetStatus(IND_STATUS_SHAKE_DETECTED, 0u);
        }
#endif /* ACC_SHAKE_DETECTION_ENABLE */
    }
}

/*********************************** SAFE STATE ****************************************************/

void App_SetSafe(bool ind_enable) {
    /* Disable interrupts */
    uint32_t prim = __get_PRIMASK();
    __disable_irq();

    /* Stop all the timers */
    Timer_Stop(SAFE_TIMEOUT_TMR);
    Timer_Stop(SYS_INIT_TMR);
    Timer_Stop(IGNITION_TMR);
    Timer_Stop(CHARGING_TMR);

    /* Reset all flags */
    sysStatus.safe_tmr_tick = sysStatus.config->safeTimeoutSec;
    sysStatus.safe_tmr_pause = false;
    sysStatus.sys_info.board_state = BOARD_STATE_DISCHARGED;
#if !SELF_DESTROY_DISABLE
    sysStatus.self_destroy_mode = SELF_DESTROY_STATE_NONE;
    sysStatus.sys_info.low_pwr_self_dest_allowed = false;
#endif /* !SELF_DESTROY_DISABLE */

#if MINING_MODE_SUPP
    sysStatus.mining_state = MINING_STATE_NONE;
#endif /* MINING_MODE_SUPP */

#if 0
	TestLedSet(false);
#endif

    /* Stop all blocks */
    App_SafeTmrStop();
    /* Disable capacitor charging MOSFET */
    ChargingDisable();
    /* Disable Pump MOSFET */
    PumpDisable();
    /* Disable all switches */
    App_IgnitionSwitchOff();

    if (ind_enable) {
        Indication_SetStatus(IND_STATUS_SET_SAFE_IND, 0u);
    }

    /* Enable interrupts */
    __set_PRIMASK(prim);
}

/***************************************** ADC Battery measurement *****************************************************/
#if VBAT_MEASURE_FEATURE
static void App_AdcStop(void) {
    Timer_Stop(ADC_MEASURE_TMR);
    sysStatus.vbat_voltage_mv = 0u;
    sysStatus.adc_measure_retry_cnt = 0u;
    sysStatus.adc_raw_data = 0u;
}

static void App_AdcTmrCbk(uint8_t timer_id) {
    /* Set flag to run ADC measurement */
    Util_SetFlag((uint32_t*)&sysStatus.app_task_mask, APP_TASK_ADC_MEASURE_CBK);
}

static void App_AdcGetDataCbk(system_evt_t evt, uint32_t usr_data) {
    if (evt == SYSTEM_EVT_READY) {
        if (sysStatus.self_voltage_mv == 0u) {
            /* Save current voltage */
            sysStatus.self_voltage_mv = (uint16_t)usr_data;
            Timer_Start(ADC_MEASURE_TMR, ADC_START_MEASURE_TMR_PERIOD_MS, App_AdcTmrCbk);
        } else {
            /* Save averaged voltage */
            sysStatus.self_voltage_mv += (uint16_t)usr_data;
            sysStatus.self_voltage_mv >>= 1u;
            Util_SetFlag((uint32_t*)&sysStatus.app_task_mask, APP_TASK_ADC_ANALYZE_CBK);
        }

        if (sysStatus.vbat_voltage_mv == 0u) {
            /* Save current voltage */
            sysStatus.vbat_voltage_mv = (uint16_t)(usr_data >> 16);
            Timer_Start(ADC_MEASURE_TMR, ADC_START_MEASURE_TMR_PERIOD_MS, App_AdcTmrCbk);
        } else {
            /* Save averaged voltage */
            sysStatus.vbat_voltage_mv += (uint16_t)(usr_data >> 16);
            sysStatus.vbat_voltage_mv >>= 1u;
            Util_SetFlag((uint32_t*)&sysStatus.app_task_mask, APP_TASK_ADC_ANALYZE_CBK);
        }
    } else {
        /* Run next measurement if fail */
        Timer_Start(ADC_MEASURE_TMR, ADC_START_MEASURE_TMR_PERIOD_MS, App_AdcTmrCbk);
    }
}

static void App_AdcCbk(void) {
    /* Run ADC measurement */
    if (AdcGetVoltage(App_AdcGetDataCbk) == false) {
        /* Re-run ADC task */
        Timer_Start(ADC_MEASURE_TMR, ADC_START_MEASURE_TMR_PERIOD_MS, App_AdcTmrCbk);
    }
}

static void App_AdcInit(void) {
    App_AdcStop();
    /* Set flag to run ADC measurement */
    Util_SetFlag((uint32_t*)&sysStatus.app_task_mask, APP_TASK_ADC_MEASURE_CBK);
}

/**
 * @brief Calculate battery level in 10% increments (0-10)
 * @param voltage_mv Battery voltage in millivolts
 * @return Battery level: 0 = 0%, 1 = 10%, 2 = 20%, ..., 10 = 100%
 */
static uint8_t App_AdcCalculateBatteryLevel(uint16_t voltage_mv) {
    uint8_t battery_level = 0;

    // Clamp voltage to valid range
    if (voltage_mv >= BATTERY_VOLTAGE_100_PERCENT_THRESHOLD_MILIVOLTS) {
        battery_level = 10;  // 100%
    } else if (voltage_mv <= BATTERY_VOLTAGE_0_PERCENT_THRESHOLD_MILIVOLTS) {
        battery_level = 0;  // 0%
    } else {
        // Calculate percentage: (current - min) / (max - min) * 100
        uint32_t voltage_range = BATTERY_VOLTAGE_100_PERCENT_THRESHOLD_MILIVOLTS - BATTERY_VOLTAGE_0_PERCENT_THRESHOLD_MILIVOLTS;
        uint32_t current_range = voltage_mv - BATTERY_VOLTAGE_0_PERCENT_THRESHOLD_MILIVOLTS;

        // Calculate percentage and round to nearest 10%
        uint32_t percentage = (current_range * 100) / voltage_range;
        battery_level = (uint8_t)((percentage + 5) / 10);  // +5 for rounding

        // Ensure we don't exceed limits
        if (battery_level > 10) {
            battery_level = 10;
        }
    }

    return battery_level;
}

static void App_AdcDataAnalyze(void) {
    /* Set default threshold */
    uint16_t low_threshold_mV = BATTERY_VOLTAGE_LOW_SD_LOW_THRESHOLD_MILIVOLTS;

    if ((sysStatus.state >= SYSTEM_STATE_CONFIGURE) && (sysStatus.state <= SYSTEM_STATE_INIT_FUSE_CHECK)) {
        /* Low down threshold to 100mV considering fuse not populated and LED + Buzzer consume additional current */
        if (Util_IsFlagChecked((uint32_t*)&sysStatus.app_err_stat, ERR_CODE_FUSE_INCORRECT_STATE)) {
            low_threshold_mV -= BATTERY_THRESHOLD_ADJUSTMENT_MV;
        }
    }
#if !SELF_DESTROY_DISABLE
    else if ((sysStatus.sys_info.low_pwr_self_dest_allowed != false) && (sysStatus.self_destroy_mode == SELF_DESTROY_STATE_NONE)) {
        /* If system is already charged threshold can be 100mV higher as we don't need to charge the capacitor */
        if ((sysStatus.state == SYSTEM_STATE_ARMED) || (sysStatus.state == SYSTEM_STATE_ARMED_PUMP)) {
            low_threshold_mV = BATTERY_VOLTAGE_VERY_LOW_SD_LOW_THRESHOLD_MILIVOLTS;
        }
    }
#endif /* !SELF_DESTROY_DISABLE */

    /* Check if battery is low charge */
    if (sysStatus.vbat_voltage_mv < low_threshold_mV) {
        /* Provide debounce */
        if (sysStatus.adc_measure_retry_cnt++ > ADC_MEASURE_RETRY_CNT) {
            /* Set error in configuration mode */
            if (sysStatus.state == SYSTEM_STATE_CONFIGURE) {
                Indication_SetError(ERR_CODE_BATTERY_LOW);
            }
            sysStatus.is_battery_low = true;
            sysStatus.adc_measure_retry_cnt = 0u;

#if !SELF_DESTROY_DISABLE
            /* If self destroy for low battery is allowed - run self destroy */
            if ((sysStatus.sys_info.low_pwr_self_dest_allowed != false) && (sysStatus.self_destroy_mode == SELF_DESTROY_STATE_NONE)) {
                /* Set flag to run self destroy handling */
                Util_SetFlag((uint32_t*)&sysStatus.app_task_mask, APP_TASK_SELF_DESTROY_INIT_CBK);
                /* Do not run Battery level measurement again */
                return;
            }
#endif /* !SELF_DESTROY_DISABLE */
        }
    } else {
        /* Clear the error in configuration mode */
        if (sysStatus.state == SYSTEM_STATE_CONFIGURE) {
            Indication_ClearError(ERR_CODE_BATTERY_LOW);
        }
        sysStatus.is_battery_low = false;
        sysStatus.adc_measure_retry_cnt = 0u;
    }

    /* Update battery level */
    sysStatus.sys_info.battery_level = App_AdcCalculateBatteryLevel(sysStatus.vbat_voltage_mv);

    /* Run next measurement */
    Timer_Start(ADC_MEASURE_TMR, ADC_START_MEASURE_TMR_PERIOD_MS, App_AdcTmrCbk);
}
#endif /* VBAT_MEASURE_FEATURE */

/***************************************** IGNITION STATE ********************************************************/
static void App_IgnitionSwitchOn(void) {
    /* Enable low side first */
    DetonLowSideSwitchSet(true);
    delay_us(20);  // 20us delay to avoid shoot through
    DetonHighSideSwithSet(true);
}

static void App_IgnitionDisableLowSideCbk(uint8_t timer_id) {
    /* Disable low side switch */
    DetonLowSideSwitchSet(false);
}

static void App_IgnitionSwitchOff(void) {
    /* Disable High and Low side switches */
    DetonHighSideSwithSet(false);
    /* Run timer to disable low side driver */
    Timer_Start(IGNITION_TMR, IGNITION_TMR_LOW_SIDE_OFF_PERIOD_MS, App_IgnitionDisableLowSideCbk);
}

static void App_IgnitionOffCbk(void) {
    /* Disable MOSFETS */
    App_IgnitionSwitchOff();

    /* Stop indication */
    Indication_SetStatus(IND_STATUS_BOOM_STOP, 0u);

    /* Set system to disarm state */
    App_DisarmRun(false);

#if !SELF_DESTROY_DISABLE
    /* Stop self destroy timer */
    App_SelfDestroyTimerStop();
#endif /* !SELF_DESTROY_DISABLE */
}

static void App_IgnitionExitTmrCbk(uint8_t timer_id) {
    /* Disable ignition */
    Util_SetFlag((uint32_t*)&sysStatus.app_task_mask, APP_TASK_IGNITION_OFF_CBK);
}

static void App_IgnitionOnCbk(void) {
#if !SELF_DESTROY_DISABLE
    /* Clear self destruction flag */
    sysStatus.self_destroy_mode = SELF_DESTROY_STATE_NONE;
#endif /* !SELF_DESTROY_DISABLE */

    /* Set ignition done flag */
    sysStatus.sys_info.is_ignition_done = 1;

    /* Set board state to ignition */
    sysStatus.sys_info.board_state = BOARD_STATE_BOOM;

    /* Provide indication */
    Indication_SetStatus(IND_STATUS_BOOM_START, 0u);

    /* Run timer to exit from ignition state */
    Timer_Start(IGNITION_TMR, IGNITION_OFF_TMR_PERIOD_MS, App_IgnitionExitTmrCbk);

    /* Enable power MOSFETS */
    App_IgnitionSwitchOn();
}

static void App_IgnitionDelayTmrCbk(uint8_t timer_id) {
    /* Run ignition */
    Util_SetFlag((uint32_t*)&sysStatus.app_task_mask, APP_TASK_IGNITION_ON_CBK);
}

static void App_IgnitionRun(void) {
#if !SELF_DESTROY_DISABLE
    App_SelfDestroyTimerStop();
#endif /* !SELF_DESTROY_DISABLE */
    /* Reset related times and variables */
    App_ArmConfigurationSet();

#if MINING_MODE_SUPP
    /* Disable mining before ignition */
    App_MiningModeDisable();
#endif /* MINING_MODE_SUPP */

    sysStatus.state = SYSTEM_STATE_IGNITION;

    if ((sysStatus.config->ignitionDelayMiliSec != 0u)
#if !SELF_DESTROY_DISABLE
        && (sysStatus.self_destroy_mode == SELF_DESTROY_STATE_NONE)
#endif /* !SELF_DESTROY_DISABLE */
    ) {
        /* Delay ignition if required. */
        Timer_Start(IGNITION_TMR, sysStatus.config->ignitionDelayMiliSec, App_IgnitionDelayTmrCbk);
    } else {
        /* Run ignition */
        App_IgnitionDelayTmrCbk(IGNITION_TMR);
    }
}

/***************************************** ARM STATE ********************************************************/
static void App_ArmConfigurationSet(void) {
    App_IgnitionSwitchOff();
#if 0
	Timer_Stop (VBAT_TEMP_MEASURE_TMR);
#endif
    Timer_Stop(IGNITION_TMR);
}

static void App_ArmRun(void) {
    bool ret = false;

    /* Reset related times and variables */
    App_ArmConfigurationSet();

    sysStatus.state = SYSTEM_STATE_ARMED;

    /* Set board state to CHARGED */
    sysStatus.sys_info.board_state = BOARD_STATE_CHARGED;

    /* Check if any ignition controls is already triggered */
    if ((sysStatus.vusa_state == VUSA_STATE_SHORTED)
#if !SELF_DESTROY_DISABLE
        || (sysStatus.self_destroy_mode == SELF_DESTROY_STATE_CHARGING)
#endif /* !SELF_DESTROY_DISABLE */
    ) {
        /* Vusa is shorted */
        App_IgnitionRun();
        return;
    }

#if MINING_MODE_SUPP
    /* Process the mining mode if enabled */
    if (sysStatus.mining_state >= MINING_STATE_ENABLING) {
        if (sysStatus.config->selfDestroyTimeoutMin == 0u) {
            /* Allow low battery self destruction mechanism */
            sysStatus.sys_info.low_pwr_self_dest_allowed = true;  // OSAV TODO: check if required in mining mode
        }

        /* Activate mining/trap mode  */
        sysStatus.mining_state = MINING_STATE_ACTIVE;

        /* Update board state in system info */
        sysStatus.sys_info.board_state = BOARD_STATE_MINING;

        /* Enable move detection */
#if ACC_SUPPORTED_ENABLE
        AccProc_MoveDetectionStart(sysStatus.config->dev_move_threshold);
#endif

#if MOVE_DETECTED_STICKY_LED_FEATURE
        /* Turn on test LED to indicate that accelerometer detected the hit */
        TestLedSet(true);
#endif
        ret = true;
    }
#endif /* MINING_MODE_SUPP */

    /* Check stick state */
    else if (sysStatus.stick_ctrl_mode == STICK_MODE_STICK_PWM) {
        if (sysStatus.stick_ctrl_stat == CONTROL_EVT_DISARM) {
            /* Set system to disarm state */
            App_DisarmHandler(false);
            ret = true;
        } else if ((sysStatus.stick_ctrl_stat == CONTROL_EVT_IGNITION) && (sysStatus.is_ignition_blocked == false)) {
            App_IgnitionRun();
            return;
        }
    }

#if ACC_SUPPORTED_ENABLE
    /* Enable Hit detection after armed */
    if ((sysStatus.acc_hit_detection_enabled != false) && (ret == false)) {
        /* Start hit accelerometer measurement */

        AccProc_HitDetectionStart();
    }
#endif
}

/***************************************** CHARGING STATE **************************************************/

static void App_ChargingStopCbk(void) {
    /* Notify that system is ready in case of first charing cycle */
    if ((sysStatus.state == SYSTEM_STATE_CHARGING)
#if MINING_MODE_SUPP
        && (sysStatus.mining_state != MINING_STATE_ENABLING)
#endif /* MINING_MODE_SUPP */
    ) {
        Indication_SetStatus(IND_STATUS_CHARGING_STOP, 0u);
    }

    /* Cap is charged - goto ARM state */
    App_ArmRun();
}

static void App_ChargingTmrCbk(uint8_t timer_id) {
    /* Set stop charge */
    Util_SetFlag((uint32_t*)&sysStatus.app_task_mask, APP_TASK_CHARGING_STOP_CBK);
}

static void App_ChargingRun(system_state_t charge_state, bool reset_var) {
    sysStatus.state = charge_state;

    if ((sysStatus.state == SYSTEM_STATE_CHARGING)
#if MINING_MODE_SUPP
        && (sysStatus.mining_state != MINING_STATE_ENABLING)
#endif /* MINING_MODE_SUPP */
    ) {
        Indication_SetStatus(IND_STATUS_CHARGING_START, 0u);
    }

    if (reset_var != false) {
        /* Reset all system blocks and variables for initial charging */
        App_SetSafe(false);

        /* Set board state to charging */
        sysStatus.sys_info.board_state = BOARD_STATE_CHARGING;
    }

    /* Enable capacitor charging MOSFET */
    ChargingEnable();

    /* Enable Pump MOSFET */
    PumpEnable();

    /* Enable charging timer. If Capacitor won't charge for that time - force charging disabling */
    Timer_Start(CHARGING_TMR, CHARGING_TMR_TIMEOUT_MS, App_ChargingTmrCbk);
}

/***************************************** DISARM STATE **************************************************/
static void App_DisarmRun(bool ind_enable) {
    /* Set board state to discharged */
    sysStatus.sys_info.board_state = BOARD_STATE_DISCHARGED;

    /* Reset all system blocks and variables */
    App_SetSafe(ind_enable);

    sysStatus.state = SYSTEM_STATE_DISARM;
}

/***************************************** SAFE TIMER STATE **************************************************/

static void App_SafeTmrTickCbk(void) {
    /* If timer pausing functionality is supported */
    if (sysStatus.safe_tmr_pause != false) {
        Timer_Start(SAFE_TIMEOUT_TMR, ONE_SECOND_TICK_TMR_PERIOD_MS, App_SafeTmrTick);
        return;
    }

    sysStatus.safe_tmr_tick--;

    /* Update timer info */
    sysStatus.sys_info.timer_seconds = sysStatus.safe_tmr_tick;
    sysStatus.sys_info.timer_mode = TIMER_MODE_SAFE;

    if (sysStatus.safe_tmr_tick == 0u) {
#if !SELF_DESTROY_DISABLE
        /* Set Self Destruction timer in sys info*/
        if (Timer_IsActive(SELF_DESTRUCTION_TMR)) {
            sysStatus.sys_info.timer_seconds = sysStatus.self_destroy_tmr_tick;
        } else {
            sysStatus.sys_info.timer_seconds = sysStatus.config->selfDestroyTimeoutMin * SECONDS_IN_MINUTE;
        }
#endif /* !SELF_DESTROY_DISABLE */

        /* Set safe timer timeout */
        sysStatus.safe_tmr_tick = sysStatus.config->safeTimeoutSec;

#if (CONTROL_MODE == PWM_CTRL_SUPP)
        if (sysStatus.stick_ctrl_mode != STICK_MODE_STICK_PWM) {
            /* Enable initial charging for non-stick control */
            App_ChargingRun(SYSTEM_STATE_CHARGING, true);
        } else {
            /* Check current stick position */
            if (sysStatus.stick_ctrl_stat == CONTROL_EVT_DISARM) {
                /* Set system to disarm state */
                App_DisarmHandler(false);
            } else {
                /* Reset all system blocks and variables for initial charging */
                App_ChargingRun(SYSTEM_STATE_CHARGING, true);
            }
        }
#elif (CONTROL_MODE == MAVLINK_V2_CTRL_SUPP)
        /* Try to arm the system */
        App_MavlinkTry2Arm();
#endif
    } else {
        Timer_Start(SAFE_TIMEOUT_TMR, ONE_SECOND_TICK_TMR_PERIOD_MS, App_SafeTmrTick);
        /* Notify user with rapid indication closer to the timeout */
        Indication_SetStatus(
            (sysStatus.safe_tmr_tick > SAFE_TIMEOUT_RAPID_INDICATION_START_SEC) ? IND_STATUS_WAIT_TMR_SHORT_TICK : IND_STATUS_WAIT_TMR_RAPID_TICK,
            sysStatus.safe_tmr_tick);

        if (sysStatus.safe_tmr_tick <= SAFE_TIMER_LAST_SEC_SOLID_SIGNAL_CNT) {
            Indication_SetStatus(IND_STATUS_SAFE_TMR_ELAPSED, 0u);
        }
    }
}

static void App_SafeTmrTick(uint8_t timer_id) {
    (void)timer_id;

    Util_SetFlag((uint32_t*)&sysStatus.app_task_mask, APP_TASK_SAFE_TMR_TICK_CBK);
}

static void App_SafeTmrStop(void) {
    sysStatus.safe_tmr_tick = sysStatus.config->safeTimeoutSec;
    Timer_Stop(SAFE_TIMEOUT_TMR);
    /* Reset variables */
    sysStatus.fuse_detect_retry = 0u;
    sysStatus.safe_tmr_pause = false;
    sysStatus.sys_info.timer_mode = TIMER_MODE_NONE;
}

static void App_SafeTmrPause(void) {
    /* Pause the safe timer */
    sysStatus.safe_tmr_pause = true;
    /* Clear Status */
    Indication_SetStatus(IND_STATUS_NONE, 0u);
}

static void App_SafeTmrRelease(void) {
    /* Release the safe timer */
    sysStatus.safe_tmr_pause = false;
}

static void App_SafeTmrRun(void) {
    /* Reset all system blocks and variables */
    App_SetSafe(false);

#if (CONTROL_MODE == PWM_CTRL_SUPP)
#if !SELF_DESTROY_DISABLE
    /* Start Self destroy timer for non stick control */
    if ((sysStatus.stick_ctrl_mode != STICK_MODE_STICK_PWM) || (sysStatus.is_ctrl_lost != false)) {
        App_SelfDestroyTimerStart(SELF_DESTROY_MODE_NORMAL);
    }
#endif /* !SELF_DESTROY_DISABLE */

    /* Check if stick is in disarm state, else pause safe timer. Note if signal lost - we continue with safe timer */
    if ((sysStatus.stick_ctrl_mode == STICK_MODE_STICK_PWM) && (sysStatus.stick_ctrl_stat > CONTROL_EVT_DISARM)) {
        App_SafeTmrPause();
    }
#endif /* PWM_CTRL_SUPP */

    sysStatus.state = SYSTEM_STATE_SAFE_TIMER;
    sysStatus.safe_tmr_tick = sysStatus.config->safeTimeoutSec;
    sysStatus.sys_info.timer_mode = TIMER_MODE_SAFE;
    Timer_Start(SAFE_TIMEOUT_TMR, ONE_SECOND_TICK_TMR_PERIOD_MS, App_SafeTmrTick);

    /* Notify user */
    Indication_SetStatus((sysStatus.safe_tmr_tick > SAFE_TIMEOUT_RAPID_INDICATION_START_SEC) ? IND_STATUS_WAIT_TMR_SHORT_TICK : IND_STATUS_WAIT_TMR_RAPID_TICK,
        sysStatus.safe_tmr_tick);
}

/***************************************** FUSE CHECK STATE **************************************************/

static void App_FusePollTmrCbk(uint8_t timer_id) { Util_SetFlag((uint32_t*)&sysStatus.app_task_mask, APP_TASK_FUSE_POLL_CBK); }

static void App_FusePollCbk(void) {
    /* Read fuse GPIO */
    bool new_fuse_state = false;
    uint16_t timer_val_ms = FUSE_CHECK_DELAY_PERIOD_MS;

    new_fuse_state = IsFuseRemoved();

    /* Process changed fuse state. NOTE: Ignore fuse changes during ignition because of schematic specifics */
    if ((sysStatus.is_fuse_removed != new_fuse_state) && (sysStatus.state != SYSTEM_STATE_IGNITION)) {
        if (sysStatus.fuse_detect_retry++ > FUSE_DETECT_RETRY) {
            /* Save current fuse state */
            sysStatus.is_fuse_removed = new_fuse_state;
            sysStatus.fuse_detect_retry = 0u;
            timer_val_ms = FUSE_CHECK_TMR_POLL_PERIOD_MS;

            /* Update fuse sys_info structure */
            sysStatus.sys_info.fuse_present = !new_fuse_state;

            if (sysStatus.configurator_active) {
                if (sysStatus.is_fuse_removed == false) {
                    /* Clear fuse error state just for a case */
                    App_ClearError(ERR_CODE_FUSE_INCORRECT_STATE);
                } else {
                    App_SetError(ERR_TYPE_RELOADABLE, ERR_CODE_FUSE_INCORRECT_STATE);
                }
            } else {
                /* Every time fuse is populated - go through all SM */
                if (sysStatus.is_fuse_removed == false) {
                    /* Move to the next start-up checks */
                    App_FuseCheckRun();
                }

                /* Clear fuse error state just for a case */
                App_ClearError(ERR_CODE_FUSE_INCORRECT_STATE);
            }
        }
    } else {
        sysStatus.fuse_detect_retry = 0u;
        timer_val_ms = FUSE_CHECK_TMR_POLL_PERIOD_MS;
    }

    /* Run fuse check timer */
    Timer_Start(FUSE_CHECK_TMR, timer_val_ms, App_FusePollTmrCbk);
}

static void App_FuseCheck(void) {
    /* Check if there's no control error */
    bool isCtrlOk = true;

#if (CONTROL_MODE == PWM_CTRL_SUPP)
    /* Check if stick position is safe */
    if (sysStatus.stick_ctrl_mode == STICK_MODE_STICK_PWM) {
        if ((sysStatus.stick_ctrl_stat > CONTROL_EVT_DISARM) && (sysStatus.stick_ctrl_stat < CONTROL_EVT_MINING_DISABLE) && (sysStatus.is_ctrl_lost == false)) {
            isCtrlOk = false;
            /* Set application error */
            App_SetError(ERR_TYPE_RELOADABLE, (sysStatus.stick_ctrl_stat == CONTROL_EVT_ARM_WITH_ACC) ? ERR_CODE_UNEXPECTED_ARM : ERR_CODE_UNEXPECTED_IGNITION);
        } else {
            App_ClearError(ERR_CODE_UNEXPECTED_ARM);
            App_ClearError(ERR_CODE_UNEXPECTED_IGNITION);
        }

        if (isCtrlOk != false) {
#if MINING_MODE_SUPP
            if (sysStatus.mining_stick_ctrl_stat == CONTROL_EVT_MINING_ENABLE) {
                isCtrlOk = false;
                /* Set application error for mining */
                App_SetError(ERR_TYPE_RELOADABLE, ERR_CODE_UNEXPECTED_MINING);
            } else if (sysStatus.mining_stick_ctrl_stat == CONTROL_EVT_MINING_DISABLE) {
                App_ClearError(ERR_CODE_UNEXPECTED_MINING);
            }
#endif /* MINING_MODE_SUPP */
        }
    }
#elif (CONTROL_MODE == MAVLINK_V2_CTRL_SUPP)
    if (sysStatus.sys_info.fc_control_present != 0) {
        if (sysStatus.arm_enabled) {
            isCtrlOk = false;
            /* Set application error */
            App_SetError(ERR_TYPE_RELOADABLE, ERR_CODE_UNEXPECTED_ARM);
        } else {
            App_ClearError(ERR_CODE_UNEXPECTED_ARM);
        }
    }

#endif
    /* Set validate state by default */
    Util_SetFlag((uint32_t*)&sysStatus.app_task_mask, APP_TASK_INIT_FUSE_CHECK_CBK);

    if (isCtrlOk != false) {
        /* If fuse is removed - can proceed */
        if (sysStatus.is_fuse_removed != false) {
            if (sysStatus.state >= SYSTEM_STATE_INIT_FUSE_CHECK) {
                /* Fuse is removed - goto safe timer */
                App_SafeTmrRun();
                Util_RemoveFlag((uint32_t*)&sysStatus.app_task_mask, APP_TASK_INIT_FUSE_CHECK_CBK);
            }
        } else {
            /* Indicate populated jumper */
            Indication_SetStatus(IND_STATUS_FUSE_POPULATED, start_up);
        }
    }

    start_up = false;
}

static void App_FuseCheckRun(void) {
    /* Set application to the Safe state */
    App_SetSafe(false);

#if !SELF_DESTROY_DISABLE
    /* Stop self destroy timer every time fuse is populated */
    App_SelfDestroyTimerStop();
#endif /* !SELF_DESTROY_DISABLE */

#if MINING_MODE_SUPP
    /* Disable mining */
    App_MiningModeDisable();
#endif /* MINING_MODE_SUPP */

    /* Update timer info */
    sysStatus.sys_info.timer_seconds = sysStatus.config->safeTimeoutSec;
    sysStatus.sys_info.fuse_present = !IsFuseRemoved();
    /* Clear ignition done flag */
    sysStatus.sys_info.is_ignition_done = 0;

#if (CONTROL_MODE == PWM_CTRL_SUPP)
    /* Reset Stick SM in case is connection was lost */
    if (sysStatus.is_ctrl_lost != false) {
        sysStatus.is_ctrl_lost = false;
        sysStatus.stick_ctrl_mode = STICK_MODE_NONE;
        sysStatus.stick_ctrl_stat = CONTROL_EVT_NO_CTRL;
#if MINING_MODE_SUPP
        sysStatus.mining_stick_ctrl_stat = CONTROL_EVT_NO_CTRL;
#endif /* MINING_MODE_SUPP */
    }
#elif (CONTROL_MODE == MAVLINK_V2_CTRL_SUPP)
    /* Reset flight parameters flag and stop related timer */
    sysStatus.sys_info.speed_altitude_flag = 0;
    Timer_Stop(FLIGHT_PARAMS_UPDATE_TMR);
#endif

    sysStatus.state = SYSTEM_STATE_INIT_FUSE_CHECK;

#if 1
    /* Check if fuse is present or removed  */
    if (sysStatus.is_fuse_removed != false) {
        start_up = false;
        /* Provide error */
        App_SetError(ERR_TYPE_RELOADABLE, ERR_CODE_FUSE_INCORRECT_STATE);
    } else {
        Util_SetFlag((uint32_t*)&sysStatus.app_task_mask, APP_TASK_INIT_FUSE_CHECK_CBK);
    }
#endif
}

/***************************************** INIT STATE **************************************************/

static void App_InitTmrCbk(uint8_t timer_id) {
    /* Set flag to run App_InitCbk */
    Util_SetFlag((uint32_t*)&sysStatus.app_task_mask, APP_TASK_INIT_CBK);
}

static void App_InitCbk(void) {
#if VBAT_MEASURE_FEATURE
    /* Check if Battery voltage is not lower than threshold */
    if (sysStatus.is_battery_low != false) {
        Util_SetFlag((uint32_t*)&sysStatus.err_stat, SYSTEM_BLOCK_BATTERY);
        /* Stop application and provide error */
        App_SetError(ERR_TYPE_HARD_RST, ERR_CODE_BATTERY_LOW);
        return;
    }
    /* If step-up converter failed then report */
    if (sysStatus.self_voltage_mv < SELF_VOLTAGE_LOW_SD_LOW_THRESHOLD_MILIVOLTS) {
        /* Set error only during initialization, otherwise we ignore acc data */
        Util_SetFlag((uint32_t*)&sysStatus.err_stat, SYSTEM_BLOCK_BATTERY);
    }
#endif

    /* Analyze errors */
    if (sysStatus.err_stat != 0) {
        /* Goto hard error state */
        App_SetError(ERR_TYPE_HARD_RST, 0);
    } else {
        /* Restart logic from the beginning */
        App_FuseCheckRun();
        /* Set board state to discharged */
        sysStatus.sys_info.board_state = BOARD_STATE_DISCHARGED;
    }
}

void App_InitFinish(void) {
    /* Set initial state */
    sysStatus.state = SYSTEM_STATE_INIT;

#if (CONTROL_MODE == PWM_CTRL_SUPP)
#if MINING_MODE_SUPP
    Stick_Reset(App_StickCbk, (sysStatus.config->miningMode == MINING_MODE_MANUAL));
#else
    Stick_Reset(App_StickCbk);
#endif /* MINING_MODE_SUPP */
    /* Set callback for FC PWM */
    FcPwmSetCbk(Stick_ProcessEdgeCbk);
#elif (CONTROL_MODE == MAVLINK_V2_CTRL_SUPP)
    /* Init Mavlink processing */
    Mavlink_Init(App_MavlinkCbk, &sysStatus.sys_info);
#endif /* CONTROL_MODE == MAVLINK_V2_CTRL_SUPP */

#if VUSA_ENABLE
    if (sysStatus.config->vusaEnable != false) {
        /* Start Vusa functionality */
        VusaStart(App_VusaCbk);
        /* Set flag to monitor Vusa */
        Util_SetFlag((uint32_t*)&sysStatus.app_task_mask, APP_TASK_VUSA_RUN_TMR_CBK);
    }
#endif /* VUSA_ENABLE */

    /* Run systems check timeout */
    Timer_Start(SYS_INIT_TMR, SYS_INIT_TMR_PERIOD_MS, App_InitTmrCbk);
}

void App_InitRun(void) {
    /* zero all variables */
    memset(&sysStatus, 0u, sizeof(system_status_t));

#if (SPI_LOGGER_ENABLE == 1u)
    Logger_Init();

    /* Reset all blocks */
    Timer_ResetAll();

    AccProc_Reset(App_AccProcCbk);

#if (CONTROL_MODE == MAVLINK_V2_CTRL_SUPP)
    /* Initialize flight parameters */
    sysStatus.current_speed_ms = 0;
    sysStatus.current_altitude_m = 0;
    sysStatus.sys_info.speed_altitude_flag = 0;

    /* Init Mavlink processing */
    Mavlink_Init(Logger_MavlinkCbk, &sysStatus.sys_info);
#endif /* (CONTROL_MODE == MAVLINK_V2_CTRL_SUPP) */
#else

    start_up = true;

    /* Load configuration */
    App_RefreshConfig();  // TODO OSAV checkconfig address

#if (CONTROL_MODE == MAVLINK_V2_CTRL_SUPP)
    /* Initialize flight parameters */
    sysStatus.current_speed_ms = 0;
    sysStatus.current_altitude_m = 0;
    sysStatus.sys_info.speed_altitude_flag = 0;
#endif /* (CONTROL_MODE == MAVLINK_V2_CTRL_SUPP) */

#if ACC_SHAKE_DETECTION_ENABLE
    /* Initialize shake detection flag */
    sysStatus.sys_info.shake_detected = 0;
#endif /* ACC_SHAKE_DETECTION_ENABLE */

    /* Set application to the Safe state */
    App_SetSafe(false);

    /* Reset all blocks */
    Timer_ResetAll();

#if VBAT_MEASURE_FEATURE
    /* Init ADC measurement */
    App_AdcInit();
#endif

    Indication_Reset(&sysStatus.sys_info.error_code);

    /* Set flag to monitor Fuse */
    Util_SetFlag((uint32_t*)&sysStatus.app_task_mask, APP_TASK_FUSE_POLL_CBK);

#if ACC_SUPPORTED_ENABLE
    /* Enable accelerometer if required */
    if ((sysStatus.config->accEnable != false)
#if MINING_MODE_SUPP
        || (sysStatus.config->miningMode != MINING_MODE_NONE)
#endif /* MINING_MODE_SUPP */
    ) {
        /* Set error by default - after successful init error will be erased */
        Util_SetFlag((uint32_t*)&sysStatus.err_stat, SYSTEM_BLOCK_ACC);
        /* Init Acc  handling */

        AccProc_Reset(App_AccProcCbk);

        if (sysStatus.config->accEnable != false) {
            sysStatus.acc_hit_detection_enabled = true;
        }
    }
#endif

#if UART_ENABLE
    /* Set UART flag to wait for UART connection */
    UartConfig_Init();

    sysStatus.state = SYSTEM_STATE_CONFIGURE;
    sysStatus.configurator_active = true;
#else
    App_InitFinish();
#endif
#endif /* SPI_LOGGER_ENABLE */
}

/***************************************** APP TASKS ********************************************************/

void App_Process() {
    /* No pending app tasks */
    if (sysStatus.app_task_mask == 0u) return;

    for (idx = 0; idx < APP_TASK_MAX_NUMB; idx++) {
        if (Util_IsFlagChecked((uint32_t*)&sysStatus.app_task_mask, idx)) {
            /* Remove flag */
            Util_RemoveFlag((uint32_t*)&sysStatus.app_task_mask, idx);

            switch ((app_task_t)idx) {
                case APP_TASK_INIT_CBK:
                    /* Init the system */
                    App_InitCbk();
                    break;
#if VBAT_MEASURE_FEATURE
                case APP_TASK_ADC_MEASURE_CBK:
                    /* Measure ADC voltage */
                    App_AdcCbk();
                    break;
                case APP_TASK_ADC_ANALYZE_CBK:
                    /* Analyze ADC data */
                    App_AdcDataAnalyze();
                    break;
#endif /* VBAT_MEASURE_FEATURE */
                case APP_TASK_FUSE_POLL_CBK:
                    /* Check fuse state */
                    App_FusePollCbk();
                    break;
#if VUSA_ENABLE
                case APP_TASK_VUSA_DETECTED_CBK:
                    /* Vusa detected */
                    App_VusaHandleCbk(true);
                    break;
                case APP_TASK_VUSA_RUN_TMR_CBK:
                    /* Run Vusa check timer */
                    App_VusaRunCheckTmrCbk();
                    break;
#endif /* VUSA_ENABLE */
                case APP_TASK_INIT_FUSE_CHECK_CBK:
                    /* Check fuse state */
                    App_FuseCheck();
                    break;
                case APP_TASK_IGNITION_ON_CBK:
                    /* Ignition on */
                    App_IgnitionOnCbk();
                    break;
                case APP_TASK_IGNITION_OFF_CBK:
                    /* Ignition off */
                    App_IgnitionOffCbk();
                    break;
                case APP_TASK_CHARGING_STOP_CBK:
                    /* Stop charging */
                    App_ChargingStopCbk();
                    break;
                case APP_TASK_SAFE_TMR_TICK_CBK:
                    /* Safe timer tick handling */
                    App_SafeTmrTickCbk();
                    break;
#if !SELF_DESTROY_DISABLE
                case APP_TASK_SELF_DESTROY_TMR_TICK_CBK:
                    /* Self destroy timer tick handling */
                    App_SelfDestroyTmrTickCbk();
                    break;
                case APP_TASK_SELF_DESTROY_INIT_CBK:
                    /* Initialize self destroy */
                    App_SelfDestroyInit();
                    break;
#endif /* !SELF_DESTROY_DISABLE */
#if MINING_MODE_SUPP
                case APP_TASK_MINING_ACTIVATE_TMR_TICK_CBK:
                    /* Mining activate timer tick handling */
                    App_MiningActivateTmrTickCbk();
                    break;
#endif /* MINING_MODE_SUPP */
                default:
                    break;
            }
        }
    }
}

#if UART_ENABLE
static bool App_UartConfigurationTask(void) {
    /* UART Configure task  */
    if (sysStatus.configurator_active) {
        sysStatus.configurator_active = UartConfig_Task();
        /* Exit from UART configuration - continue init board */
        if (sysStatus.configurator_active == false) {
#if (CONTROL_MODE == PWM_CTRL_SUPP)
            /* Check configuration of PWM2 channel */
            if (sysStatus.config->pwm2InputEnable == false) {
#if (OSD_ENABLE == 1u)
                /* Disable RX and keep TX enabled to transmit OSD data */
                UART_Configure(true, false);
                /* Initialize OSD */
                App_OsdInit();
#endif /* (OSD_ENABLE == 1u) */
                /* Configure only PWM1 channel */
                PWM_GPIO_Configure(true, false);
            } else {
                /* Configure both PWM channels */
                PWM_IN_GPIO_Init();
            }
#endif /* (CONTROL_MODE == PWM_CTRL_SUPP) */

            /* Finish application initialization */
            App_InitFinish();
        }
        return true;
    }
    return false;
}
#endif /* UART_ENABLE */

void App_Task(void) {
#if (SPI_LOGGER_ENABLE == 1u)

    Logger_Task();  // Phase 4: Frame builder - assembles LogFrame_t in unified queue

#if ACC_SUPPORTED_ENABLE
    AccProc_Task();
#endif

    /* Timer tick task */
    Timer_Task();

#if (CONTROL_MODE == MAVLINK_V2_CTRL_SUPP)
    /* Mavlink Processing */
    Mavlink_Process();
#endif
#else
    /* Timer tick task */
    Timer_Task();

    /* Pending Application tasks */
    App_Process();

    /* LED/Sound indication */
    Indication_Task();

    /* Acceleration processing task */
#if ACC_SUPPORTED_ENABLE
    AccProc_Task();
#endif

#if (OSD_ENABLE == 1u)
    /* OSD processing task */
    OsdMsp_Task();
#endif /* (OSD_ENABLE == 1u) */

#if UART_ENABLE
    /* UART Configuration Task */
    if (App_UartConfigurationTask()) {
        return;
    }
#endif /* UART_ENABLE */

#if (CONTROL_MODE == PWM_CTRL_SUPP)
    /* Stick monitoring */
    Stick_Task();
#elif (CONTROL_MODE == MAVLINK_V2_CTRL_SUPP)
    /* Mavlink Processing */
    Mavlink_Process();
#endif
#endif /* SPI_LOGGER_ENABLE */
}
