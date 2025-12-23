
#include "stick_ctrl.h"

#include <stdio.h>
#include <stdlib.h>

#include "init_brd.h"
#include "prj_config.h"
#include "solution_wrapper.h"
#include "timer.h"

#if (CONTROL_MODE == PWM_CTRL_SUPP)

static stickStatus_t stickStatus[2];
static bool periodsValid, signalsValid;
static control_evt_t old_ctrl_evt = CONTROL_EVT_NO_EVT;
#if MINING_MODE_SUPP
static control_evt_t old_mining_evt = CONTROL_EVT_NO_EVT;
#endif
static app_cbk_fn sys_cbk = NULL;
static uint8_t pwm_ch;
#if MINING_MODE_SUPP
static bool mining_mode_enabled = false;
#endif

// TODO LOW For some reason we read not expected period and signal value. Once at a time.
// This has to be investigated. Maybe GPIO interrupt handler got interrupted, or it happens in debug only
// Consider to show the issue somehow to a pin and observe regularity

static void Stick_Process(void) {
    control_evt_t new_ctrl_evt = old_ctrl_evt;
#if MINING_MODE_SUPP
    control_evt_t new_mining_evt = old_mining_evt;
#endif

    stick_state_t pwm_1_state = stickStatus[PWM_CH_1].new_state;
    stick_state_t pwm_2_state = stickStatus[PWM_CH_2].new_state;

    /* Next logic is following traceability matrix https://docs.google.com/spreadsheets/d/1osFT6v8AKOudRD-kBv0LlpidjTukqOLUr_APX6HuWQc/edit?usp=sharing */
    if ((pwm_1_state == STICK_STATE_NONE) || (pwm_2_state == STICK_STATE_NONE)) {
        return;
    }

    /* Use PWM1 channel only as control stick for mining mode */
#if MINING_MODE_SUPP
    if (mining_mode_enabled != false) {
        pwm_2_state = STICK_STATE_NO_STICK;
    }
#endif

    if ((pwm_1_state < STICK_STATE_ACTIVE_POS_1) || (pwm_2_state == STICK_STATE_LOST)) {
        new_ctrl_evt = CONTROL_EVT_NO_CTRL;
    } else if (pwm_2_state == STICK_STATE_ACTIVE_POS_3) {
        new_ctrl_evt = CONTROL_EVT_IGNITION;
    } else if (pwm_1_state == STICK_STATE_ACTIVE_POS_1) {
        new_ctrl_evt = CONTROL_EVT_DISARM;
    } else if ((pwm_1_state == STICK_STATE_ACTIVE_POS_2) && (pwm_2_state >= STICK_STATE_ACTIVE_POS_1)) {
        new_ctrl_evt = CONTROL_EVT_ARM_WO_ACC;
    } else if (((pwm_1_state == STICK_STATE_ACTIVE_POS_3) && (pwm_2_state >= STICK_STATE_ACTIVE_POS_1)) || (pwm_1_state == STICK_STATE_ACTIVE_POS_2)) {
        new_ctrl_evt = CONTROL_EVT_ARM_WITH_ACC;
    } else if (pwm_1_state == STICK_STATE_ACTIVE_POS_3) {
        new_ctrl_evt = CONTROL_EVT_IGNITION;
    } else {
#if MINING_MODE_SUPP
        if (mining_mode_enabled == false) {
            /* We should not hit this place */
            return;
        }
#else
        /* We should not hit this place */
        return;
#endif
    }

    if (sys_cbk != NULL) {
        /* Process control sticks */
        if (old_ctrl_evt != new_ctrl_evt) {
            sys_cbk(SYSTEM_EVT_READY, new_ctrl_evt);
            /* Save new event */
            old_ctrl_evt = new_ctrl_evt;
        }

        /* Process mining stick */
#if MINING_MODE_SUPP
        if (mining_mode_enabled != false) {
            /* Only stick 3 position can be interpreted as mining */
            new_mining_evt = (stickStatus[PWM_CH_2].new_state < STICK_STATE_ACTIVE_POS_3) ? CONTROL_EVT_MINING_DISABLE : CONTROL_EVT_MINING_ENABLE;

            if (old_mining_evt != new_mining_evt) {
                sys_cbk(SYSTEM_EVT_READY, new_mining_evt);
                /* Save new event */
                old_mining_evt = new_mining_evt;
            }
        }
#endif
    }
}

static void Stick_HandleLostState(uint8_t pwm_ch) {
    if (stickStatus[pwm_ch].old_state < STICK_STATE_ACTIVE_POS_1) {
        /* Stick wasn't detected during the start-up  */
        stickStatus[pwm_ch].new_state = STICK_STATE_NO_STICK;
    } else {
        /* Report lost stick if previously stick was detected */
        stickStatus[pwm_ch].new_state = STICK_STATE_LOST;
    }

    /* Run stick handling logic */
    Stick_Process();

    stickStatus[pwm_ch].old_state = stickStatus[pwm_ch].new_state;
}

static void Stick1_LostCbk(uint8_t timer_id) { Stick_HandleLostState(PWM_CH_1); }

static void Stick2_LostCbk(uint8_t timer_id) { Stick_HandleLostState(PWM_CH_2); }

#if MINING_MODE_SUPP
void Stick_Reset(app_cbk_fn system_cbk, bool mining_mode) {
    memset(&stickStatus, 0u, sizeof(stickStatus_t));
    sys_cbk = system_cbk;
    mining_mode_enabled = mining_mode;
    stickStatus[PWM_CH_1].is_stick_scan_enabled = true;
    stickStatus[PWM_CH_2].is_stick_scan_enabled = true;

    // STICK_LOST_PERIOD_MS has to be less 500ms
    //  Start stick lost timer
    Timer_Start(STICK1_LOST_TMR, STICK_LOST_PERIOD_MS, Stick1_LostCbk);
    Timer_Start(STICK2_LOST_TMR, STICK_LOST_PERIOD_MS, Stick2_LostCbk);
}
#else
void Stick_Reset(app_cbk_fn system_cbk) {
    memset(&stickStatus, 0u, sizeof(stickStatus_t));
    sys_cbk = system_cbk;
    stickStatus[PWM_CH_1].is_stick_scan_enabled = true;
    stickStatus[PWM_CH_2].is_stick_scan_enabled = true;

    // STICK_LOST_PERIOD_MS has to be less 500ms
    //  Start stick lost timer
    Timer_Start(STICK1_LOST_TMR, STICK_LOST_PERIOD_MS, Stick1_LostCbk);
    Timer_Start(STICK2_LOST_TMR, STICK_LOST_PERIOD_MS, Stick2_LostCbk);
}
#endif

void Stick_Deinit(void) { memset(&stickStatus, 0u, sizeof(stickStatus)); }

void Stick_ProcessEdgeCbk(system_evt_t idx, uint32_t usr_data) {
    if (idx <= PWM_CH_2) {
        stickStatus[idx].countVal = ReadStickCounter10Us();

        if (usr_data == true) {
            // rising edge
            if (stickStatus[idx].countVal > stickStatus[idx].lastRisingVal) {
                stickStatus[idx].periodValue = stickStatus[idx].countVal - stickStatus[idx].lastRisingVal;
            } else  // counter overlap happen
            {
                stickStatus[idx].periodValue = 0xffff - (stickStatus[idx].lastRisingVal - stickStatus[idx].countVal);
            }
            stickStatus[idx].lastRisingVal = stickStatus[idx].countVal;
        } else {
            // falling edge
            if (stickStatus[idx].countVal > stickStatus[idx].lastRisingVal) {
                stickStatus[idx].signalValue = stickStatus[idx].countVal - stickStatus[idx].lastRisingVal;
            } else  // counter overlap happen
            {
                stickStatus[idx].signalValue = 0xffff - (stickStatus[idx].lastRisingVal - stickStatus[idx].countVal);
            }
            stickStatus[idx].fallingIdx++;
        }
    }
}

void Stick_Task(void) {
    for (pwm_ch = PWM_CH_1; pwm_ch < (PWM_CH_2 + 1); pwm_ch++) {
        if (stickStatus[pwm_ch].is_stick_scan_enabled == false) return;

        // check if we need to analyze data
        if (stickStatus[pwm_ch].fallingIdx != stickStatus[pwm_ch].processedIdx) {
            stickStatus[pwm_ch].processedIdx = stickStatus[pwm_ch].fallingIdx;

            if (pwm_ch == PWM_CH_1) {
                // Reset stick lost timer
                Timer_Start(STICK1_LOST_TMR, STICK_LOST_PERIOD_MS, Stick1_LostCbk);
            }
            if (pwm_ch == PWM_CH_2) {
                // Reset stick lost timer
                Timer_Start(STICK2_LOST_TMR, STICK_LOST_PERIOD_MS, Stick2_LostCbk);
            }

            // check periods
            periodsValid = true;
            periodsValid = ((stickStatus[pwm_ch].periodValue > FC_PWM_PERIOD_MIN) && (stickStatus[pwm_ch].periodValue < FC_PWM_PERIOD_MAX));
            // It is kind of filter
            if (periodsValid == true) {
                stickStatus[pwm_ch].periodInvalidCount = 0;
            } else {
                // most likely transition period or some kind of dithering
                stickStatus[pwm_ch].periodInvalidCount++;
                if (stickStatus[pwm_ch].periodInvalidCount < FC_PWM_PERIOD_INVALID_MAX_NUM) {
                    periodsValid = true;
                }
            }

            // check if signal values are valid
            signalsValid = true;
            if ((stickStatus[pwm_ch].signalValue > FC_PWM_POS_1_MIN) && (stickStatus[pwm_ch].signalValue < FC_PWM_POS_1_MAX)) {
                stickStatus[pwm_ch].pos1Counts++;
                stickStatus[pwm_ch].pos2Counts = 0;
                stickStatus[pwm_ch].pos3Counts = 0;
                stickStatus[pwm_ch].lostCounts = 0;
            } else if ((stickStatus[pwm_ch].signalValue > FC_PWM_POS_2_MIN) && (stickStatus[pwm_ch].signalValue < FC_PWM_POS_2_MAX)) {
                stickStatus[pwm_ch].pos1Counts = 0;
                stickStatus[pwm_ch].pos2Counts++;
                stickStatus[pwm_ch].pos3Counts = 0;
                stickStatus[pwm_ch].lostCounts = 0;
            } else if ((stickStatus[pwm_ch].signalValue > FC_PWM_POS_3_MIN) && (stickStatus[pwm_ch].signalValue < FC_PWM_POS_3_MAX)) {
                stickStatus[pwm_ch].pos1Counts = 0;
                stickStatus[pwm_ch].pos2Counts = 0;
                stickStatus[pwm_ch].pos3Counts++;
                stickStatus[pwm_ch].lostCounts = 0;
            } else if ((stickStatus[pwm_ch].signalValue > FC_PWM_POS_LOST_MIN) && (stickStatus[pwm_ch].signalValue < FC_PWM_POS_LOST_MAX)) {
                stickStatus[pwm_ch].pos1Counts = 0;
                stickStatus[pwm_ch].pos2Counts = 0;
                stickStatus[pwm_ch].pos3Counts = 0;
                stickStatus[pwm_ch].lostCounts++;
            } else {
                signalsValid = false;
                stickStatus[pwm_ch].pos1Counts = 0;
                stickStatus[pwm_ch].pos2Counts = 0;
                stickStatus[pwm_ch].pos3Counts = 0;
                stickStatus[pwm_ch].lostCounts = 0;
            }
            // It is kind of filter
            if (signalsValid == true) {
                stickStatus[pwm_ch].signalInvalidCount = 0;
            } else {
                // most likely transition period or some kind of dithering
                stickStatus[pwm_ch].signalInvalidCount++;
                if (stickStatus[pwm_ch].signalInvalidCount < FC_PWM_SIGNAL_INVALID_MAX_NUM) {
                    signalsValid = true;
                }
            }

            // perform flag processing into status
            if ((periodsValid == false) || (signalsValid == false)) {
                if (stickStatus[pwm_ch].old_state < STICK_STATE_LOST)  // STICK_STATE_ACTIVE_POS_1
                {
                    stickStatus[pwm_ch].new_state = STICK_STATE_NO_STICK;
                } else {
                    /* Report lost stick if previously stick was detected */
                    stickStatus[pwm_ch].new_state = STICK_STATE_LOST;
                }
            }
            if (stickStatus[pwm_ch].pos1Counts > FC_PWM_DETECTION_NUM) {
                stickStatus[pwm_ch].new_state = STICK_STATE_ACTIVE_POS_1;
            }
            if (stickStatus[pwm_ch].pos2Counts > FC_PWM_DETECTION_NUM) {
                stickStatus[pwm_ch].new_state = STICK_STATE_ACTIVE_POS_2;
            }
            if (stickStatus[pwm_ch].pos3Counts > FC_PWM_DETECTION_NUM) {
                stickStatus[pwm_ch].new_state = STICK_STATE_ACTIVE_POS_3;
            }
            if (stickStatus[pwm_ch].lostCounts > FC_PWM_DETECTION_NUM) {
                stickStatus[pwm_ch].new_state = STICK_STATE_LOST;
            }

            // report status if needed
            if (stickStatus[pwm_ch].new_state != stickStatus[pwm_ch].old_state) {
                /* Run stick handling logic */
                Stick_Process();
                stickStatus[pwm_ch].old_state = stickStatus[pwm_ch].new_state;
            }
        }
    }
}

#endif /* PWM_CTRL_SUPP */
