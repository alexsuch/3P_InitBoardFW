
#include <stdlib.h>
#include <stdio.h>
#include "solution_wrapper.h"
#include "app.h"
#include "timer.h"
#include "init_brd.h"
#include "acc_proc.h"
#include "indication.h"
#include "stick_ctrl.h"
#include "app_config.h"
#include "uart_configurator.h"

static system_status_t sysStatus;
static uint8_t idx = 0u;

static void App_AdcGetDataCbk (system_evt_t evt, uint32_t usr_data);
static void App_AdcStop (void);
#if 0
static void App_PumpTmrCbk (uint8_t timer_id);
#endif

static void App_ChargingRun (system_state_t charge_state, bool reset_var);
static void App_IgnitionSwitchOff (void);


/***************************************** SET/GET CONFIGURATION HANDLERS HANDLERS ******************************/
void App_RefreshConfig (void)
{
	sysStatus.config = AppConfig_GetConfiguration();
}

uint8_t App_GetAppStatusConfiguration(void)
{
    /* Disable interrupts */
	uint32_t prim = __get_PRIMASK();
    __disable_irq();

	uint8_t stat = 0u;

	/* Update Fuse status */
	if (sysStatus.is_fuse_removed == false)
	{
		stat |= IS_FUSE_PRESENT_STATUS_MASK;
	}

	/* Update low battery flag */
	if (sysStatus.is_battery_low != false)
	{
		stat |= IS_BATTERY_LOW_STATUS_MASK;
	}

	/* Check if application is in hard error */
	if (sysStatus.state == SYSTEM_STATE_ERROR)
	{
		stat |= IS_APP_FAULT_STATUS_MASK;
	}
	/* Check if application is ready */
	else if (sysStatus.state == SYSTEM_STATE_CONFIGURE)
	{
		stat |= IS_APP_READY_STATUS_MASK;
	}

    if (!prim) {
        __enable_irq();
    }

    return stat;
}

uint8_t*  App_GetConfiguration(uint8_t* size)
{
	uint8_t* ret = NULL;

	ret = (uint8_t*)sysStatus.config;
	*size = sizeof(config_data_t);

	return ret;
}

bool App_SetConfiguration(uint8_t* data, uint8_t size)
{
	bool ret = false;

	/* Configuration can be done only when Fuse is populated/ app version match/ sizes are match */
	if (
			(sysStatus.is_fuse_removed == false)
			//&& (sizeof(config_data_t) == size)
		)
	{
		if (AppGonfig_IsAllElements(data, size, 0x00))
		{
			/* Apply default configuration if required (all elements are zeros) */
			ret = AppConfig_ApplyDefaultConfig();
		}
		else
		{
			/* Save received configuration */
			ret =  AppConfig_SaveConfiguration ((config_data_t*)data);
		}
	}

	return ret;
}

/***************************************** VUSA HANDLERS ********************************************************/

static void App_VusaHandleCbk (bool vusaShorted);

static void App_VusaShortCheckTmrCbk (uint8_t timer_id)
{
	/* If timer elapsed - VUSA is not detected */
	App_VusaHandleCbk (false);
}

static void App_VusaRunCheckTmrCbk (void)
{
	/* Run timer to detect if Vusa disconnected*/
	Timer_Start (VUSA_CHECK_TMR, VUSA_POLL_TMR_DELAY_PERIOD_MS, App_VusaShortCheckTmrCbk);
}

static void App_VusaHandleCbk (bool vusaShorted)
{
	/* Get Vusa state */
	vusa_state_t new_vusa_state = (vusaShorted) ? VUSA_STATE_SHORTED : VUSA_STATE_NOT_SHORTED;

	if (new_vusa_state == VUSA_STATE_SHORTED)
	{
		/* Run timer to detect if VUSA not shorted */
		Util_SetFlag((uint32_t*)&sysStatus.app_task_mask, APP_TASK_VUSA_RUN_TMR_CBK);
	}

	if (sysStatus.vusa_state != new_vusa_state)
	{
		/* Save current fuse state */
		sysStatus.vusa_state = new_vusa_state;
	}
}

static void App_VusaCbk (system_evt_t evt, uint32_t usr_data)
{
    if (evt == SYSTEM_EVT_READY)
	{
    	/* Set flag to run Vusa analysis */
    	Util_SetFlag((uint32_t*)&sysStatus.app_task_mask, APP_TASK_VUSA_DETECTED_CBK);
	}
}


/***************************************** STICK CONTROL ********************************************************/

static stick_state_t last_status = STICK_STATE_NONE;
static uint8_t stat_change_cnt = 0u;

static void App_StickVarClear (void)
{
	last_status = STICK_STATE_NONE;
	stat_change_cnt = 0u;
}


static void App_StickCbk (system_evt_t evt, uint32_t usr_data)
{
	if (sysStatus.pending_test_cmd == TEST_CMD_PWM_START)
	{
		if (last_status != usr_data)
		{
			stat_change_cnt++;
		}

		if (usr_data > STICK_STATE_NONE)
		{
			last_status = usr_data;
		}
	}
}

/***************************************** ACCELEROMETER PROCESSING  ********************************************************/

static void App_AccProcCbk (system_evt_t evt, uint32_t usr_data)
{

	if (sysStatus.pending_test_cmd == TEST_CMD_ACC_READY)
	{
		if (evt == SYSTEM_EVT_READY)
		{
			UartConfig_SendOneByteData (sysStatus.pending_test_cmd, TEST_CMD_STATUS_OK, 0);
		}
		else if (evt == SYSTEM_EVT_ERROR)
		{
			UartConfig_SendOneByteData (sysStatus.pending_test_cmd, TEST_CMD_STATUS_FAILED, usr_data);
		}
	}

	AccProc_Stop();
}

/*********************************** SAFE STATE ****************************************************/

void App_SetSafe (bool ind_enable)
{
    /* Disable interrupts */
	uint32_t prim = __get_PRIMASK();
    __disable_irq();

	/* Disable capacitor charging MOSFET */
	ChargingDisable();
	/* Disable Pump MOSFET */
	PumpDisable();
	/* Disable all switches */
	App_IgnitionSwitchOff();

	/* Enable interrupts */
    if (!prim) {
        __enable_irq();
    }
}

/***************************************** ADC Battery measurement *****************************************************/

static void App_AdcStop (void)
{
	Timer_Stop (ADC_MEASURE_TMR);
	sysStatus.vbat_voltage_mv       = 0u;
	sysStatus.adc_measure_retry_cnt = 0u;
	sysStatus.adc_raw_data          = 0u;
}

static void App_AdcTmrCbk (uint8_t timer_id)
{
	/* Set flag to run ADC measurement */
	Util_SetFlag((uint32_t*)&sysStatus.app_task_mask, APP_TASK_ADC_MEASURE_CBK);
}

static void App_AdcGetDataCbk (system_evt_t evt, uint32_t usr_data)
{
	if (evt == SYSTEM_EVT_READY)
	{
		if (sysStatus.self_voltage_mv == 0u)
		{
			/* Save current voltage */
			sysStatus.self_voltage_mv = (uint16_t)usr_data;
			Timer_Start (ADC_MEASURE_TMR, ADC_START_MEASURE_TMR_PERIOD_MS, App_AdcTmrCbk);
		}
		else
		{
			/* Save averaged voltage */
			sysStatus.self_voltage_mv += (uint16_t)usr_data;
			sysStatus.self_voltage_mv >>= 1u;
			Util_SetFlag((uint32_t*)&sysStatus.app_task_mask, APP_TASK_ADC_ANALYZE_CBK);
		}

		if (sysStatus.vbat_voltage_mv == 0u)
		{
			/* Save current voltage */
			sysStatus.vbat_voltage_mv = (uint16_t)(usr_data >> 16);
			Timer_Start (ADC_MEASURE_TMR, ADC_START_MEASURE_TMR_PERIOD_MS, App_AdcTmrCbk);
		}
		else
		{
			/* Save averaged voltage */
			sysStatus.vbat_voltage_mv += (uint16_t)(usr_data >> 16);
			sysStatus.vbat_voltage_mv >>= 1u;
			Util_SetFlag((uint32_t*)&sysStatus.app_task_mask, APP_TASK_ADC_ANALYZE_CBK);
		}
	}
	else
	{
		/* Run next measurement if fail */
		Timer_Start (ADC_MEASURE_TMR, ADC_START_MEASURE_TMR_PERIOD_MS, App_AdcTmrCbk);
	}
}

static void App_AdcCbk (void)
{
	/* Run ADC measurement */
	if (AdcGetVoltage (App_AdcGetDataCbk) == false)
	{
		/* Re-run ADC task */
		Timer_Start (ADC_MEASURE_TMR, ADC_START_MEASURE_TMR_PERIOD_MS, App_AdcTmrCbk);
	}
}

static void App_AdcInit (void)
{
	App_AdcStop();
	/* Set flag to run ADC measurement */
	Util_SetFlag((uint32_t*)&sysStatus.app_task_mask, APP_TASK_ADC_MEASURE_CBK);
}

static void App_AdcDataAnalyze(void)
{
	/* Run next measurement */
	Timer_Start (ADC_MEASURE_TMR, ADC_START_MEASURE_TMR_PERIOD_MS, App_AdcTmrCbk);
}


/***************************************** IGNITION STATE ********************************************************/
static void App_IgnitionSwitchOn (void)
{
	/* Enable low side first */
	DetonLowSideSwitchSet (true);
	delay_us(20);  //20us delay to avoid shoot through
	DetonHighSideSwithSet (true);
}

static void App_IgnitionDisableLowSideCbk (uint8_t timer_id)
{
	/* Disable low side switch */
	DetonLowSideSwitchSet (false);
}

static void App_IgnitionSwitchOff (void)
{
	/* Disable High and Low side switches */
	DetonHighSideSwithSet (false);
	/* Run timer to disable low side driver */
	Timer_Start (IGNITION_TMR, IGNITION_TMR_LOW_SIDE_OFF_PERIOD_MS, App_IgnitionDisableLowSideCbk);
}

/***************************************** CHARGING STATE **************************************************/

static void App_ChargingRun (system_state_t charge_state, bool reset_var)
{
	/* Enable capacitor charging MOSFET */
	ChargingEnable();

	/* Enable Pump MOSFET */
	PumpEnable();
}

/***************************************** FUSE CHECK STATE **************************************************/
static bool new_fuse_state = false;

static void App_TestFuseTmrCbk (uint8_t timer_id)
{
	UartConfig_SendOneByteData (sysStatus.pending_test_cmd, TEST_CMD_STATUS_RESULT, sysStatus.is_fuse_removed);
	sysStatus.pending_test_cmd = 0;
}

static void App_FusePoll (void)
{
	new_fuse_state = IsFuseRemoved();

	if (sysStatus.is_fuse_removed != new_fuse_state)
	{
		if (sysStatus.fuse_detect_retry++ > FUSE_DETECT_RETRY)
		{
			/* Save current fuse state */
			sysStatus.is_fuse_removed         = new_fuse_state;
			sysStatus.fuse_detect_retry       = 0u;
		}
	}
	else
	{
		sysStatus.fuse_detect_retry = 0u;
	}
}

/***************************************** INDICATION **************************************************/

static void App_IndicationTmrCbk (uint8_t timer_id)
{
	UartConfig_SendOneByteData (sysStatus.pending_test_cmd, TEST_CMD_STATUS_OK, 0);
	sysStatus.pending_test_cmd = 0;

	BuzzerDisable();
	SET_ERROR_LED(false);
	SET_STATUS_LED(false);
}

/***************************************** INIT STATE **************************************************/
static uint32_t uid = 0;

static uint32_t App_CreateUniqueId(uint32_t num1, uint32_t num2, uint32_t num3)
{
    // Define the FNV-1a hash constants
    const uint32_t fnv_prime = 0x811C9DC5;
    const uint32_t fnv_offset = 0x811C9DC5;

    // Initialize the hash value
    uint32_t id = fnv_offset;

    // Hash the three numbers together
    id = (id ^ num1) * fnv_prime;
    id = (id ^ num2) * fnv_prime;
    id = (id ^ num3) * fnv_prime;

    return id;
}

void App_InitRun(void)
{
	/* zero all variables */
	memset(&sysStatus, 0u, sizeof(system_status_t));

	/* Load configuration */
	//App_RefreshConfig();

	/* Set application to the Safe state */
	App_SetSafe(false);

	/* Init GPIOs for PWM */
	PWM_IN_GPIO_Init();

	/* Reset all blocks */
	Timer_ResetAll();

	/* Init ADC measurement */
	App_AdcInit();

	Indication_Reset();

	/* Init Acc  handling */
	AccProc_Reset(App_AccProcCbk);

	/* Set UART flag to wait for UART connection */
	UartConfig_Init();

	Stick_Reset(App_StickCbk, false);

	/* Set callback for FC PWM */
	FcPwmSetCbk(Stick_ProcessEdgeCbk);

	/* Start Vusa functionality */
	VusaStart(App_VusaCbk);
	/* Set flag to monitor Vusa */
	Util_SetFlag((uint32_t*)&sysStatus.app_task_mask, APP_TASK_VUSA_RUN_TMR_CBK);

	// Generate 32-bit unique ID
	uid = App_CreateUniqueId (HAL_GetUIDw0(), HAL_GetUIDw1(), HAL_GetUIDw2());
}

/**************************************** Config test cases************************************/

static uint8_t testCfgData[128];

static bool App_WriteCfg (uint8_t* data, uint8_t size)
{
	memset(&testCfgData, 0u, sizeof(testCfgData));

	memcpy(testCfgData, data, size);

	return AppConfig_SaveConfiguration ((config_data_t*)testCfgData);
}

static bool App_ReadCfg (void)
{
	memset(&testCfgData, 0u, sizeof(testCfgData));
	uint8_t* cpyPtr = (uint8_t*)AppConfig_GetConfiguration();

	if (cpyPtr == NULL)
	{
		return false;
	}

	memcpy(testCfgData, cpyPtr, sizeof(config_data_t));

	return true;
}

/***************************************** TEST APP TASKS ********************************************************/
uint8_t testData[32];

bool App_ParseTestCommand (uint8_t* cmd_ptr, uint8_t len)
{
	sysStatus.run_test_cmd = cmd_ptr[TEST_CMD_IDX];

	memcpy(testData, &cmd_ptr[TEST_DATA_START_IDX], (TEST_PKT_DATA_SIZE - TEST_DATA_START_IDX));

	/* Reset command if required */
	if (cmd_ptr[TEST_STATUS_IDX] == TEST_CMD_RESET_CURRENT_CMD)
	{
		sysStatus.run_test_cmd = 0;
		sysStatus.pending_test_cmd = 0;
	}

	return true;
}

void App_TestProcess (void)
{
	/* No pending app tasks */
	if (sysStatus.run_test_cmd == 0u)
		return;

    switch (sysStatus.run_test_cmd)
    {
    	case TEST_CMD_DISARM:
    		App_SetSafe(false);
    		UartConfig_SendOneByteData (sysStatus.run_test_cmd, TEST_CMD_STATUS_OK, 0);
    		break;
    	case TEST_CMD_FUSE_DFLT:
    	case TEST_CMD_FUSE_POPULATED:
    		Timer_Start (SEND_RESPONSE_TMR, SEND_RESPONSE_TMR_PERIOD_MS, App_TestFuseTmrCbk);
    		sysStatus.pending_test_cmd = sysStatus.run_test_cmd;
    		break;
    	case TEST_CMD_CAP_CHARGE:
    		App_ChargingRun (0, 0);
    		UartConfig_SendOneByteData (sysStatus.run_test_cmd, TEST_CMD_STATUS_OK, 0);
    		break;
    	case TEST_CMD_CAP_SWITCH:
    		ChargingEnable();
    		UartConfig_SendOneByteData (sysStatus.run_test_cmd, TEST_CMD_STATUS_OK, 0);
    		break;
    	case TEST_CMD_CAP_DISCHARGE:
    		ChargingDisable();
    		UartConfig_SendOneByteData (sysStatus.run_test_cmd, TEST_CMD_STATUS_OK, 0);
    		break;
    	case TEST_CMD_ACC_READY:
    		AccProc_HitDetectionStart();
    		sysStatus.pending_test_cmd = sysStatus.run_test_cmd;
    		break;
    	case TEST_CMD_VUSA_DFLT:
    	case TEST_CMD_VUSA_SHORTED:
    		UartConfig_SendOneByteData (sysStatus.run_test_cmd, TEST_CMD_STATUS_RESULT, sysStatus.vusa_state);
    		break;
    	case TEST_CMD_UART_COMM:
    		UartConfig_SendOneByteData (sysStatus.run_test_cmd, TEST_CMD_STATUS_OK, 0);
    		break;
    	case TEST_CMD_PWM_START:
    		UartConfig_SendOneByteData (sysStatus.run_test_cmd, TEST_CMD_STATUS_OK, 0);
    		sysStatus.pending_test_cmd = sysStatus.run_test_cmd;
    		App_StickVarClear();
    		break;
    	case TEST_CMD_PWM_READ:
    		UartConfig_SendTwoByteData (sysStatus.run_test_cmd, TEST_CMD_STATUS_RESULT, last_status, stat_change_cnt);
    		sysStatus.pending_test_cmd = sysStatus.run_test_cmd;
    		break;
    	case TEST_CMD_BAT_LVL:
    		UartConfig_SendTwoByteData (sysStatus.run_test_cmd, TEST_CMD_STATUS_RESULT, (sysStatus.self_voltage_mv >> 8), (sysStatus.self_voltage_mv & 0xFF));
    		break;
    	case TEST_CMD_DETONATOR_SWITCH:
    		App_IgnitionSwitchOn();
    		UartConfig_SendOneByteData (sysStatus.run_test_cmd, TEST_CMD_STATUS_OK, 0);
    		break;
    	case TEST_CMD_GET_SILICON_ID:
    		UartConfig_SendFourByteData (sysStatus.run_test_cmd, TEST_CMD_STATUS_RESULT, (uid >> 24) , (uid >> 16), (uid >> 8), (uid & 0xFF));
    		break;
    	case TEST_CMD_CONFIG_TABLE_WRITE:
    		if (App_WriteCfg (testData, (TEST_PKT_DATA_SIZE - TEST_DATA_START_IDX)) != false)
    		{
    			UartConfig_SendOneByteData (sysStatus.run_test_cmd, TEST_CMD_STATUS_OK, 0);
    		}
    		else
    		{
    			UartConfig_SendOneByteData (sysStatus.run_test_cmd, TEST_CMD_STATUS_FAILED, 0);
    		}
    		break;
    	case TEST_CMD_CONFIG_TABLE_READ:
    		if (App_ReadCfg () != false)
    		{
    			UartConfig_SendFourByteData (sysStatus.run_test_cmd, TEST_CMD_STATUS_RESULT, testCfgData[0], testCfgData[1], testCfgData[2], testCfgData[3]);
    		}
    		else
    		{
    			UartConfig_SendOneByteData (sysStatus.run_test_cmd, TEST_CMD_STATUS_FAILED, 0);
    		}

    		break;
    	case TEST_CMD_INDICATION:
    		BuzzerEnable();
    		SET_ERROR_LED(true);
    		SET_STATUS_LED(true);
    		Timer_Start (SEND_TEST_IND_TMR, SEND_TEST_IND_TMR_PERIOD_MS, App_IndicationTmrCbk);
    		sysStatus.pending_test_cmd = sysStatus.run_test_cmd;
    	default:
    		// Handle unknown command
			break;
    }

	sysStatus.run_test_cmd = 0;
}

void App_Process ()
{
	/* Poll fuse */
	App_FusePoll();

	/* No pending app tasks */
	if (sysStatus.app_task_mask == 0u)
		return;

	for (idx = 0; idx < APP_TASK_MAX_NUMB; idx++)
	{
		if (Util_IsFlagChecked ((uint32_t*)&sysStatus.app_task_mask, idx))
		{
			/* Remove flag */
			Util_RemoveFlag((uint32_t*)&sysStatus.app_task_mask, idx);

			switch ((app_task_t)idx)
			{
			    case APP_TASK_ADC_MEASURE_CBK:
			    	/* Measure ADC voltage */
			    	App_AdcCbk();
			    	break;
			    case APP_TASK_ADC_ANALYZE_CBK:
					/* Analyze ADC data */
					App_AdcDataAnalyze();
					break;
			    case APP_TASK_VUSA_DETECTED_CBK:
			    	/* Vusa detected */
			    	App_VusaHandleCbk(true);
			    	break;
			    case APP_TASK_VUSA_RUN_TMR_CBK:
			    	/* Run Vusa check timer */
			    	App_VusaRunCheckTmrCbk();
			    	break;
			    default:
			    	break;
			}
		}
	}
}


void App_Task (void)
{
	/* Timer tick task */
	Timer_Task();

	/* Pending Application tasks */
	App_Process();

	/* Pending Application tasks */
	App_TestProcess();

	/* LED/Sound indication */
	Indication_Task();

	/* Acceleration processing task */
	AccProc_Task();

	UartConfig_Task();

	/* Stick monitoring */
	Stick_Task();
}
