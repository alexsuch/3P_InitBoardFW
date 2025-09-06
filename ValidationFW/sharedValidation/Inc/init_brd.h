
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __INIT_BRD_H
#define __INIT_BRD_H
   
#ifdef __cplusplus
extern "C" {
#endif
   
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define ACC_12_BIT                                   (1u)

#define CONFIG_DATA_SIZE_BYTES                  (31u)

#define IS_FUSE_PRESENT_STATUS_MASK             (0x01u)
#define IS_BATTERY_LOW_STATUS_MASK              (0x02u)
#define IS_LOG_DATA_READY_STATUS_MASK           (0x04u)
#define IS_APP_READY_STATUS_MASK                (0x08u)
#define IS_APP_FAULT_STATUS_MASK                (0x10u)

#define ZERO                            (0u)
#define TRUE                            (1u)
#define FALSE                           (0u)
      
#define ON                              (1u)
#define OFF                             (0u)

typedef enum
{
    SYSTEM_BLOCK_ACC,
	SYSTEM_BLOCK_CTRL,
	SYSTEM_BLOCK_BATTERY,
	SYSTEM_BLOCK_DET,
	SYSTEM_BLOCK_MAX_BLOCK
}system_block_t;

typedef enum
{
    SYSTEM_STATE_IDLE,
	SYSTEM_STATE_ERROR,
	SYSTEM_STATE_CONFIGURE,
	SYSTEM_STATE_INIT,
	SYSTEM_STATE_INIT_FUSE_CHECK,
	SYSTEM_STATE_SAFE_TIMER,
	SYSTEM_STATE_DISARM,
	SYSTEM_STATE_CHARGING,
	SYSTEM_STATE_ARMED,
	SYSTEM_STATE_ARMED_PUMP,
	SYSTEM_STATE_IGNITION,
}system_state_t;

typedef enum
{
    VUSA_STATE_EMPTY,
	VUSA_STATE_NONE,
	VUSA_STATE_NOT_SHORTED,
	VUSA_STATE_SHORTED,
}vusa_state_t;

typedef enum
{
    STICK_MODE_NONE,
	STICK_MODE_BYPASS,
	STICK_MODE_STICK_PWM,
}stick_mode_t;

typedef enum
{
	SYSTEM_EVT_INIT_DONE,
	SYSTEM_EVT_READY,
	SYSTEM_EVT_ERROR,
}system_evt_t;

typedef enum
{
	CONTROL_EVT_NO_EVT,
	CONTROL_EVT_NO_CTRL,
	CONTROL_EVT_DISARM,
	CONTROL_EVT_ARM_WO_ACC,
	CONTROL_EVT_ARM_WITH_ACC,
	CONTROL_EVT_IGNITION,
	CONTROL_EVT_MINING_DISABLE,
	CONTROL_EVT_MINING_ENABLE,
}control_evt_t;

typedef enum
{
    ACC_EVT_IDLE,
	ACC_EVT_INIT_OK,
	ACC_EVT_HIT_INIT_OK,
	ACC_EVT_MOVE_INIT_OK,
	ACC_EVT_DATA_READY,
	ACC_EVT_HIT_DETECTED,
	ACC_EVT_MOVE_DETECTED,
}acc_evt_t;

typedef enum
{
	APP_TASK_FUSE_POLL_CBK,
	APP_TASK_INIT_FUSE_CHECK_CBK,
    APP_TASK_INIT_CBK,
	APP_TASK_SAFE_TMR_TICK_CBK,
	APP_TASK_PUMP_TMR_CBK,
	APP_TASK_IGNITION_ON_CBK,
	APP_TASK_IGNITION_OFF_CBK,
	APP_TASK_TEMP_MEASURE_CBK,
	APP_TASK_VUSA_RUN_TMR_CBK,
	APP_TASK_VUSA_DETECTED_CBK,
	APP_TASK_CHARGING_STOP_CBK,
	APP_TASK_ADC_MEASURE_CBK,
	APP_TASK_ADC_ANALYZE_CBK,
	APP_TASK_UART_CONFIGURATOR_CBK,
	APP_TASK_SELF_DESTROY_INIT_CBK,
	APP_TASK_HARD_ERROR,
	APP_TASK_MAX_NUMB,
}app_task_t;

typedef enum
{
    ERR_TYPE_RELOADABLE,
	ERR_TYPE_HARD_RST,
}err_type_t;

typedef enum
{
    ERR_CODE_NO_ERROR,
	ERR_CODE_STRONG_ALARM,
	ERR_CODE_BATTERY_LOW,
	ERR_CODE_FUSE_INCORRECT_STATE,
	ERR_CODE_UNEXPECTED_IGNITION,
	ERR_CODE_UNEXPECTED_ARM,
	ERR_CODE_UNEXPECTED_MINING,
	ERR_CODE_UNEXPECTED_VUSA_SHORTED,
	ERR_CODE_MAX_ERROR
}err_code_t;

typedef enum
{
    IND_STATUS_NONE,
	IND_STATUS_INIT_SUCCESS,
	IND_STATUS_WAIT_TMR_SHORT_TICK,
	IND_STATUS_WAIT_TMR_RAPID_TICK,
	IND_STATUS_SAFE_TMR_ELAPSED,
	IND_STATUS_CHARGING_START,
	IND_STATUS_CHARGING_STOP,
	IND_STATUS_SET_SAFE_IND,
	IND_STATUS_ARMED,
	IND_STATUS_BOOM_START,
	IND_STATUS_BOOM_STOP,
	IND_STATUS_FUSE_POPULATED,
	IND_STATUS_DESTRUCTION_START,
	IND_STATUS_DESTRUCTION_STOP,
	IND_MANUAL_MINING_ENABLED,
	IND_STATUS_CONFIGURATION_MODE,
	IND_STATUS_CONFIGURATION_APPLIED,
}ind_status_t;

typedef enum
{
    ADC_MEASURE_NONE,
	ADC_MEASURE_TEMP,
	ADC_MEASURE_VBAT,
	ADC_MEASURE_VCAP,
	ADC_MONITOR_VCAP,
}adc_measure_t;

typedef enum
{
    MINING_STATE_NONE,
	MINING_STATE_START_SAFE_TMR,
	MINING_STATE_ENABLING,
	MINING_STATE_ACTIVE,
	MINING_STATE_IGNITION,
}mining_state_t;

typedef enum
{
    MINING_MODE_NONE,
	MINING_MODE_AUTO,
	MINING_MODE_MANUAL,
}mining_mode_t;

typedef enum
{
    SELF_DESTROY_STATE_NONE,
	SELF_DESTROY_STATE_STARTED,
	SELF_DESTROY_STATE_CHARGING,
}self_destroy_state_t;

typedef enum
{
    SELF_DESTROY_MODE_NORMAL,
	SELF_DESTROY_MODE_CTRL_LOST,
}self_destroy_mode_t;




#define MAJOR_VER_SHIFT        (4u)
#define MINOR_VER_SHIFT        (0u)

#define TEST_CMD_IDX           (0u)
#define TEST_STATUS_IDX        (1u)
#define TEST_DATA_START_IDX    (2u)

#define TEST_PKT_DATA_SIZE     (6u)

typedef enum
{
    ACC_TEST_STATUS_FAILED_COMMUNICATION,
	ACC_TEST_STATUS_VALID_DATA_FAILED,
	ACC_TEST_STATUS_VALID_DATA_OK,
	ACC_TEST_STATUS_VALID_DATA_SPIKE,
}acc_status_t;


typedef enum
{
    TEST_CMD_STATUS_FAILED,
	TEST_CMD_STATUS_OK,
	TEST_CMD_STATUS_RESULT,
	TEST_CMD_RESET_CURRENT_CMD
}test_cmd_status_t;

typedef enum
{
	TEST_CMD_DISABLE_PWR                 = ((0u << MAJOR_VER_SHIFT) | (0u << MINOR_VER_SHIFT)),
	TEST_CMD_ENABLE_PWR                  = ((0u << MAJOR_VER_SHIFT) | (1u << MINOR_VER_SHIFT)),
    TEST_CMD_DISARM                      = ((0u << MAJOR_VER_SHIFT) | (2u << MINOR_VER_SHIFT)),
	TEST_CMD_FUSE_DFLT                   = ((1u << MAJOR_VER_SHIFT) | (0u << MINOR_VER_SHIFT)),
	TEST_CMD_FUSE_POPULATED              = ((1u << MAJOR_VER_SHIFT) | (1u << MINOR_VER_SHIFT)),
	TEST_CMD_CAP_CHARGE                  = ((2u << MAJOR_VER_SHIFT) | (0u << MINOR_VER_SHIFT)),
	TEST_CMD_CAP_SWITCH                  = ((2u << MAJOR_VER_SHIFT) | (1u << MINOR_VER_SHIFT)),
	TEST_CMD_CAP_DISCHARGE               = ((2u << MAJOR_VER_SHIFT) | (2u << MINOR_VER_SHIFT)),
	TEST_CMD_ACC_READY                   = ((3u << MAJOR_VER_SHIFT) | (0u << MINOR_VER_SHIFT)),
	TEST_CMD_VUSA_DFLT                   = ((4u << MAJOR_VER_SHIFT) | (0u << MINOR_VER_SHIFT)),
	TEST_CMD_VUSA_SHORTED                = ((4u << MAJOR_VER_SHIFT) | (1u << MINOR_VER_SHIFT)),
	TEST_CMD_UART_COMM                   = ((5u << MAJOR_VER_SHIFT) | (0u << MINOR_VER_SHIFT)),
	TEST_CMD_PWM_START                   = ((6u << MAJOR_VER_SHIFT) | (0u << MINOR_VER_SHIFT)),
	TEST_CMD_PWM_READ                    = ((6u << MAJOR_VER_SHIFT) | (1u << MINOR_VER_SHIFT)),
	TEST_CMD_BAT_LVL                     = ((7u << MAJOR_VER_SHIFT) | (0u << MINOR_VER_SHIFT)),
	TEST_CMD_DETONATOR_SWITCH            = ((8u << MAJOR_VER_SHIFT) | (0u << MINOR_VER_SHIFT)),
	TEST_CMD_GET_SILICON_ID              = ((9u << MAJOR_VER_SHIFT) | (0u << MINOR_VER_SHIFT)),
	TEST_CMD_CONFIG_TABLE_WRITE          = ((10u << MAJOR_VER_SHIFT) | (0u << MINOR_VER_SHIFT)),
	TEST_CMD_CONFIG_TABLE_READ           = ((10u << MAJOR_VER_SHIFT) | (1u << MINOR_VER_SHIFT)),
	TEST_CMD_INDICATION                  = ((11u << MAJOR_VER_SHIFT) | (0u << MINOR_VER_SHIFT)),
}test_cmd_t;


typedef struct __attribute__ ((__packed__))
{
	uint16_t safeTimeoutSec;   // 120sec default; Min - 60sec Max - 1000 sec
	uint16_t ignitionDelayMiliSec; // Default - 0; Max - 60 000	 MiliSec

	uint8_t  miningMode;  // Default - 0 - None; 1 - AUTO; 2 - MANUAL
	uint16_t miningAutoActivationMin; // Default - 30sec; Min - 10sec. Max - 10 000 sec.  After timeout can be triggered by movement or battery low state
	uint16_t miningEnableDelaySec; // Default  30 seconds; Min 10 sec; Max 1000 sec

	uint16_t selfDestroyTimeoutMin; // Default - 30 min; Max 255 min // Minimum 15 min

	uint8_t  accEnable;             // Default - Enable

	uint32_t dev_move_threshold; // MOVEMENT_THRESHOLD 72089LL // Граничне значення для детекції руху (наприклад 1.1 радіана.сек = FLOAT_TO_FIXED(1.1) = 72089

	uint8_t rsvd2[15]; //NOTE: Keep 31 byte data structure size
}config_data_t;

typedef struct
{
	config_data_t config_data;
	uint8_t crc;
}config_save_data_t;




typedef struct
{

	test_cmd_t run_test_cmd;
	test_cmd_t pending_test_cmd;

	// Application
	system_state_t state;
	uint32_t app_task_mask;
	// Errors
	uint32_t err_stat;
	uint32_t app_err_stat;
	//Safe timer
	uint16_t safe_tmr_tick;
	bool safe_tmr_pause;
	// Accelerometer
	bool acc_hit_detection_enabled;
	// Stick control
	stick_mode_t stick_ctrl_mode;
	control_evt_t stick_ctrl_stat;
	control_evt_t mining_stick_ctrl_stat;
	bool is_ctrl_lost;
	// Fuse
	bool is_fuse_removed;
	uint8_t fuse_detect_retry;
	// VUSA
	bool is_vusa_present;
	uint8_t vusa_detect_retry;
	vusa_state_t vusa_state;
	// ADC block
	uint16_t adc_raw_data;
	uint16_t adc_milivolts;
	uint8_t adc_measure_retry_cnt;
	uint16_t self_voltage_mv;
	uint16_t vbat_voltage_mv;
	uint16_t adc_temp;
	float temperature;
	bool is_battery_low;
	// Self destroy
	bool low_pwr_self_dest_allowed;
	self_destroy_state_t self_destroy_mode;
	mining_state_t mining_state;
	// Configuration
	const config_data_t* config;
}system_status_t;

typedef void (*app_cbk_fn)      (system_evt_t evt, uint32_t usr_data);

system_status_t* get_status(void);

#ifdef __cplusplus
}
#endif

#endif /* __INIT_BRD_H */
