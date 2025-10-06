
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
	ACC_EVT_SHAKE_INIT_OK,
	ACC_EVT_STARTUP_SHAKE_DETECTED,
}acc_evt_t;

typedef enum
{
	APP_TASK_FUSE_POLL_CBK,
	APP_TASK_INIT_FUSE_CHECK_CBK,
    APP_TASK_INIT_CBK,
	APP_TASK_SAFE_TMR_TICK_CBK,
	APP_TASK_SELF_DESTROY_TMR_TICK_CBK,
	APP_TASK_PUMP_TMR_CBK,
	APP_TASK_VUSA_DETECTED_CBK,
	APP_TASK_VUSA_RUN_TMR_CBK,
	APP_TASK_IGNITION_ON_CBK,
	APP_TASK_IGNITION_OFF_CBK,
	APP_TASK_ADC_MEASURE_CBK,
	APP_TASK_ADC_ANALYZE_CBK,
	APP_TASK_CHARGING_STOP_CBK,
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
	ERR_CODE_ACCELEROMETER_FAIL,
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
	IND_STATUS_DESTRUCTION_START_NO_SOUND,
	IND_STATUS_DESTRUCTION_STOP,
	IND_MANUAL_MINING_ENABLED,
	IND_STATUS_CONFIGURATION_MODE,
	IND_STATUS_CONFIGURATION_APPLIED,
	IND_STATUS_SHAKE_DETECTED,
	IND_STATUS_SPEED_ALTITUDE_OK,
}ind_status_t;

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

typedef enum
{
    DEVICE_TYPE_FOR_REGULAR_FPV = 0u,
    DEVICE_TYPE_FOR_REGULAR_WING = 1u,
    DEVICE_TYPE_FOR_CUSTOM_FPV = 2u,
    DEVICE_TYPE_FOR_CUSTOM_WING = 3u,
}device_type_t;

// Timer mode enumeration
typedef enum {
	TIMER_MODE_NONE = 0x00,         // No active timer
    TIMER_MODE_SAFE = 0x01,         // Safe timer active
    TIMER_MODE_SELF_DESTROY = 0x02  // Self destroy timer active
} timer_mode_t;

// Board state enumeration
typedef enum {
    BOARD_STATE_INIT = 0x00,        // No data available
    BOARD_STATE_DISARMED = 0x01,    // Disarmed
	BOARD_STATE_CHARGING = 0x02,    // Charging
    BOARD_STATE_ARMED = 0x03,       // Armed
    BOARD_STATE_BOOM = 0x04         // Boom
} board_state_t;

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

	uint8_t rsvd2[12]; // Reserved bytes 
	
	// Device identification
	uint8_t device_type;    // Device type identifier (see device_type_t enum)
	uint8_t customer_info;  // Customer info (reserved)
	uint8_t rsvd3;      // Reserved byte

	//NOTE: Keep 31 byte data structure size
}config_data_t;

typedef struct
{
	config_data_t config_data;
	uint8_t crc;
}config_save_data_t;

/**
 * @brief System State Data Structure
 *
 * This structure holds all system state values needed for InitBoard HEARTBEAT message.
 * It should be updated by the application when SYSTEM_EVT_INIT_DONE callback is called.
 */
typedef struct {
    uint16_t timer_seconds;      // Timer seconds (0-16383)
    uint8_t timer_mode;          // Timer mode (0-3)
    uint8_t fuse_present;        // Fuse present (0-1)
    uint8_t board_state;         // Board state (0-7)
    uint8_t battery_level;       // Battery level (0-10, where 10 = 100%)
    uint8_t error_code;          // Error code (0-15)
    uint8_t is_ignition_done;    // Ignition done flag (0-1)
    uint8_t fc_control_present;  // FC control connection status (0-1)
	uint8_t prearm_flag;         // Autopilot PREARM state (0-1)
	uint8_t speed_altitude_flag; // Autopilot speed/altitude control state (0-1)
	uint8_t shake_detected;      // Startup shake detection flag (0-1)
} init_board_system_info_t;

typedef struct
{
	// Application
	volatile system_state_t state;
	volatile uint32_t app_task_mask;
	// Errors
	uint32_t err_stat;
	uint32_t app_err_stat;
	//Safe timer
	uint16_t safe_tmr_tick;
	bool safe_tmr_pause;
	// Self destruction timer
	uint16_t self_destroy_tmr_tick;
	// Accelerometer
	bool acc_hit_detection_enabled;
	// Stick control
	stick_mode_t stick_ctrl_mode;
	control_evt_t stick_ctrl_stat;
	control_evt_t mining_stick_ctrl_stat;
	bool is_ctrl_lost;
	bool is_ignition_bloked;
	// Prearm
	bool arm_enabled;     // Mavlink autopilot ARM state
	// Fuse
	bool is_fuse_removed;
	uint8_t fuse_detect_retry;
	// VUSA
	bool is_vusa_present;
	volatile vusa_state_t vusa_state;
	// ADC block
	volatile uint16_t adc_raw_data;
	uint16_t adc_milivolts;
	volatile uint8_t adc_measure_retry_cnt;
	volatile uint16_t self_voltage_mv;
	volatile uint16_t vbat_voltage_mv;
	uint16_t adc_temp;
	float temperature;
	bool is_battery_low;
	// Self destroy
	bool low_pwr_self_dest_allowed;
	self_destroy_state_t self_destroy_mode;
	mining_state_t mining_state;
	// Mavlink data
	uint16_t current_speed_ms;      // Current speed in m/s (from VFR_HUD)
	int32_t current_altitude_m;     // Current altitude in meters (from VFR_HUD)
	// Board info
	init_board_system_info_t sys_info;
	// Configuration
	const config_data_t* config;
}system_status_t;

typedef void (*app_cbk_fn)      (system_evt_t evt, uint32_t usr_data);
typedef uint8_t (*app_ext_cbk_fn)      (system_evt_t evt, uint32_t usr_data, void* usr_ptr);

system_status_t* get_status(void);

#ifdef __cplusplus
}
#endif

#endif /* __INIT_BRD_H */
