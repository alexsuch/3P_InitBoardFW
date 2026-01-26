/*
 * mavlink_uart.h - STM32G0 Mavlink UART Communication Module Header
 *
 * This header defines the interface for Mavlink v2.0 communication via UART.
 * It includes all necessary constants, structures, and function prototypes.
 */

#ifndef MAVLINK_UART_H
#define MAVLINK_UART_H

#include <init_brd.h>
#include <stdint.h>

#include "solution_wrapper.h"

// ======================= MAVLINK CONSTANTS =======================
// Protocol Configuration
#define MAVLINK_V2_MAGIC 0xFD           // 253
#define MAVLINK_VERSION 0x03            // 3
#define MAVLINK_MAX_MESSAGE_SIZE 0x118  // 280
#define MAVLINK_MAX_PAYLOAD_SIZE 0xFF   // 255

// Mavlink v2.0 packet structure constants
#define MAVLINK_V2_HEADER_SIZE 10      // Header size in bytes
#define MAVLINK_V2_CRC_SIZE 2          // CRC size in bytes
#define MAVLINK_V2_MIN_PACKET_SIZE 12  // Header + CRC (minimum packet)

// Mavlink v2.0 header byte indices
#define MAVLINK_V2_MAGIC_INDEX 0           // Magic byte index
#define MAVLINK_V2_PAYLOAD_LEN_INDEX 1     // Payload length index
#define MAVLINK_V2_INCOMPAT_FLAGS_INDEX 2  // Incompatible flags index
#define MAVLINK_V2_COMPAT_FLAGS_INDEX 3    // Compatible flags index
#define MAVLINK_V2_SEQUENCE_INDEX 4        // Sequence number index
#define MAVLINK_V2_SYSTEM_ID_INDEX 5       // System ID index
#define MAVLINK_V2_COMPONENT_ID_INDEX 6    // Component ID index
#define MAVLINK_V2_MSG_ID_LOW_INDEX 7      // Message ID low byte index
#define MAVLINK_V2_MSG_ID_MID_INDEX 8      // Message ID mid byte index
#define MAVLINK_V2_MSG_ID_HIGH_INDEX 9     // Message ID high byte index
#define MAVLINK_V2_PAYLOAD_START_INDEX 10  // Payload start index

// HEARTBEAT message payload indices (relative to payload start)
#define HEARTBEAT_TYPE_INDEX 0             // MAV type
#define HEARTBEAT_AUTOPILOT_INDEX 1        // Autopilot type
#define HEARTBEAT_BASE_MODE_INDEX 2        // Base mode
#define HEARTBEAT_CUSTOM_MODE_INDEX 3      // Custom mode (4 bytes)
#define HEARTBEAT_SYSTEM_STATUS_INDEX 7    // System status
#define HEARTBEAT_MAVLINK_VERSION_INDEX 8  // Mavlink version
#define HEARTBEAT_PAYLOAD_SIZE 9           // HEARTBEAT payload size

// COMMAND_ACK message payload indices (relative to payload start)
#define COMMAND_ACK_COMMAND_INDEX 0  // Command ID (2 bytes)
#define COMMAND_ACK_RESULT_INDEX 2   // Result code
#define COMMAND_ACK_PAYLOAD_SIZE 3   // COMMAND_ACK payload size

// System and Component IDs
#define MAVLINK_SYSTEM_ID_INITBOARD 0x42  // 66 - Initiation Board
#define MAVLINK_COMP_ID_INITBOARD 0x42    // 66 - Initiation Board Component

// Message IDs
#define MAVLINK_MSG_ID_HEARTBEAT 0x00           // 0
#define MAVLINK_MSG_ID_VFR_HUD 0x4A             // 74
#define MAVLINK_MSG_ID_COMMAND_LONG 0x4C        // 76
#define MAVLINK_MSG_ID_COMMAND_ACK 0x4D         // 77
#define MAVLINK_MSG_ID_EXTENDED_SYS_STATE 0xF5  // 245

// EXTENDED_SYS_STATE message payload indices (relative to payload start)
#define EXTENDED_SYS_STATE_VTOL_STATE_INDEX 0    // VTOL state
#define EXTENDED_SYS_STATE_LANDED_STATE_INDEX 1  // Landed state
#define EXTENDED_SYS_STATE_PAYLOAD_SIZE 2        // EXTENDED_SYS_STATE payload size

// MAV_LANDED_STATE values
#define MAV_LANDED_STATE_UNDEFINED 0  // MAV landed state is unknown
#define MAV_LANDED_STATE_ON_GROUND 1  // MAV is landed (on ground)
#define MAV_LANDED_STATE_IN_AIR 2     // MAV is in air
#define MAV_LANDED_STATE_TAKEOFF 3    // MAV currently taking off
#define MAV_LANDED_STATE_LANDING 4    // MAV currently landing

// MAV_VTOL_STATE values
#define MAV_VTOL_STATE_UNDEFINED 0  // MAV is not configured as VTOL

// VFR_HUD message payload indices (relative to payload start)
#define VFR_HUD_AIRSPEED_INDEX 0     // Airspeed (4 bytes float)
#define VFR_HUD_GROUNDSPEED_INDEX 4  // Groundspeed (4 bytes float)
#define VFR_HUD_HEADING_INDEX 8      // Heading (2 bytes int16_t)
#define VFR_HUD_THROTTLE_INDEX 10    // Throttle (2 bytes uint16_t)
#define VFR_HUD_ALT_INDEX 12         // Altitude (4 bytes float)
#define VFR_HUD_CLIMB_INDEX 16       // Climb rate (4 bytes float)
#define VFR_HUD_PAYLOAD_SIZE 20      // VFR_HUD payload size

// MAV Types and States
#define MAV_TYPE_GENERIC 0x00       // 0
#define MAV_AUTOPILOT_INVALID 0x08  // 8
#define MAV_STATE_ACTIVE 0x04       // 4

// MAV base_mode flags
#define MAV_MODE_FLAG_SAFETY_ARMED (1 << 7)  // 0x80 - Vehicle armed (safety lock not armed)

// MAV Result codes
#define MAV_RESULT_ACCEPTED 0x00              // 0 - Command accepted
#define MAV_RESULT_TEMPORARILY_REJECTED 0x01  // 1 - Command temporarily rejected
#define MAV_RESULT_DENIED 0x02                // 2 - Command denied
#define MAV_RESULT_UNSUPPORTED 0x03           // 3 - Command not supported
#define MAV_RESULT_FAILED 0x04                // 4 - Command failed
#define MAV_RESULT_IN_PROGRESS 0x05           // 5 - Command in progress
#define MAV_RESULT_CANCELLED 0x06             // 6 - Command cancelled

// Mavlink flight parameters thresholds (use values from prj_config.h if defined)
#ifndef FLIGHT_SPEED_MINIMUM_THRESHOLD_M_S
#define FLIGHT_SPEED_MINIMUM_THRESHOLD_M_S (5u)  // Minimum flight speed 5 m/s
#endif

#ifndef FLIGHT_ALTITUDE_MINIMUM_THRESHOLD_M
#define FLIGHT_ALTITUDE_MINIMUM_THRESHOLD_M (50)  // Minimum flight altitude 50m
#endif

#ifndef FLIGHT_STABLE_PARAMETERS_TIMEOUT_SEC
#define FLIGHT_STABLE_PARAMETERS_TIMEOUT_SEC (3u)  // Stable parameters timeout 3 seconds
#endif

/**
 * @brief Custom Command IDs (used in param1[0] of MAV_CMD_USER_1)
 */
typedef enum {
    MAVLINK_CMD_IGNITION = 0x01,       // 1 - Custom ignition command
    MAVLINK_CMD_CHARGE = 0x02,         // 2 - Custom charge command
    MAVLINK_CMD_DISCHARGE = 0x03,      // 3 - Custom discharge command
    MAVLINK_CMD_PREARM_ENABLE = 0x04,  // 4 - Custom prearm enable command
    MAVLINK_CMD_PREARM_DISABLE = 0x05  // 5 - Custom prearm disable command
} mavlink_custom_command_t;

#define MAV_CMD_USER_1 31010  // MAVLink standard user command 1

/**
 * @brief Custom Command Structure via MAV_CMD_USER_1
 *
 * ESP32 sends custom commands using MAV_CMD_USER_1 (31010) with custom data in param1:
 * - command field = MAV_CMD_USER_1 (31010)
 * - param1[0] = custom_command_type (MAVLINK_CMD_IGNITION = 1, MAVLINK_CMD_CHARGE = 2,
 *                                    MAVLINK_CMD_DISCHARGE = 3, MAVLINK_CMD_PREARM_ENABLE = 4,
 *                                    MAVLINK_CMD_PREARM_DISABLE = 5)
 * - param1[1] = command_data (additional parameters specific to command type)
 * - param1[2] = reserved for future use
 * - param1[3] = reserved for future use
 *
 * STM32 processes by checking command_id == 31010, then switching on param1[0] for command type.
 *
 * Security: Commands are only processed if target_system == MAVLINK_SYSTEM_ID_INITBOARD (0x42)
 * AND target_component == MAVLINK_COMP_ID_INITBOARD (0x42). Broadcast commands are ignored.
 * Source filtering is not implemented.
 */

// ======================= TYPE DEFINITIONS =======================

// Autopilot ARM state enumeration
typedef enum {
    MAVLINK_AUTOPILOT_ARM_DISARMED = 0x00,  // 0 - Autopilot disarmed
    MAVLINK_AUTOPILOT_ARM_ARMED = 0x01      // 1 - Autopilot armed
} mavlink_autopilot_arm_state_t;

// Autopilot PREARM state enumeration
typedef enum {
    MAVLINK_AUTOPILOT_PREARM_DISABLED = 0x00,  // 0 - PREARM disabled
    MAVLINK_AUTOPILOT_PREARM_ENABLED = 0x01    // 1 - PREARM enabled
} mavlink_autopilot_prearm_state_t;

/**
 * @brief Mavlink Events for system callback
 */
typedef enum {
    MAVLINK_EVT_NO_EVT = 0x00,                     // 0 - No event
    MAVLINK_EVT_COMMAND_IGNITION = 0x01,           // 1 - Ignition command received
    MAVLINK_EVT_AUTOPILOT_CONNECTED = 0x02,        // 2 - Autopilot connection established
    MAVLINK_EVT_AUTOPILOT_DISCONNECTED = 0x03,     // 3 - Autopilot connection lost
    MAVLINK_EVT_AUTOPILOT_HEARTBEAT = 0x04,        // 4 - Autopilot HEARTBEAT received
    MAVLINK_EVT_AUTOPILOT_ARMED = 0x05,            // 5 - Autopilot changed to ARMED state
    MAVLINK_EVT_AUTOPILOT_DISARMED = 0x06,         // 6 - Autopilot changed to DISARMED state
    MAVLINK_EVT_AUTOPILOT_PREARM_ENABLED = 0x07,   // 7 - Autopilot PREARM enabled (command received)
    MAVLINK_EVT_AUTOPILOT_PREARM_DISABLED = 0x08,  // 8 - Autopilot PREARM disabled (command received)
    MAVLINK_EVT_VFR_HUD_RECEIVED = 0x09,           // 9 - VFR_HUD data received from autopilot
    MAVLINK_EVT_SPEED_RECEIVED = 0x0A,             // 10 - Speed data received from autopilot
    MAVLINK_EVT_ALTITUDE_RECEIVED = 0x0B,          // 11 - Altitude data received from autopilot
    MAVLINK_EVT_AUTOPILOT_CHARGE = 0x0C,           // 12 - Charge command received from autopilot
    MAVLINK_EVT_AUTOPILOT_DISCHARGE = 0x0D,        // 13 - Discharge command received from autopilot
    MAVLINK_EVT_AUTOPILOT_FLYING = 0x0E,           // 14 - Autopilot changed to FLYING state (landed_state=IN_AIR)
    MAVLINK_EVT_AUTOPILOT_LANDED = 0x0F            // 15 - Autopilot changed to LANDED state (landed_state=ON_GROUND)
} mavlink_event_t;

/**
 * @brief VFR_HUD Data Storage Structure (Integer-only for STM32G0)
 *
 * This structure stores VFR_HUD data using integer representations
 * to avoid floating-point operations on STM32G0 (no FPU).
 * All values are stored directly in meters/meters per second as integers.
 */
typedef struct {
    uint16_t airspeed_ms;       // Airspeed in m/s (converted from float)
    uint16_t groundspeed_ms;    // Groundspeed in m/s (converted from float)
    int16_t heading_deg;        // Heading in degrees (0-359, -1 for unknown)
    uint16_t throttle_percent;  // Throttle in percentage (0-100)
    int32_t altitude_m;         // Altitude in m (converted from float)
    int16_t climb_rate_ms;      // Climb rate in m/s (converted from float)
} mavlink_vfr_hud_data_t;

/**
 * @brief Custom Mode Bitfield Structure (32-bit total) for InitBoard HEARTBEAT
 *
 * This structure defines the custom mode bitfield that encodes system state
 * in the InitBoard HEARTBEAT message custom_mode field.
 */
typedef union {
    struct {
        uint32_t timer_sec : 12;           // Bits 0-11:  Timer seconds (0-4095)
        uint32_t timer_mode : 2;           // Bits 12-13: Timer mode (0-3)
        uint32_t fc_control_present : 1;   // Bit 14:   FC control present (0-1) - autopilot connection status
        uint32_t fuse_present : 1;         // Bit 15:     Fuse present (0-1)
        uint32_t board_state : 3;          // Bits 16-18: Board state (0-7)
        uint32_t battery_level : 4;        // Bits 19-22: Battery level 0-10 (encoded as 0-15)
        uint32_t error_code : 4;           // Bits 23-26: Error code (0-15)
        uint32_t is_ignition_done : 1;     // Bit 27:     Ignition done flag (0-1)
        uint32_t prearm_flag : 1;          // Bit 28:     Autopilot prearm flag
        uint32_t speed_altitude_flag : 1;  // Bit 29: Autopilot speed/altitude flag
        uint32_t is_flying : 1;            // Bit 30:     Flight state (0=LANDED, 1=FLYING)
        uint32_t reserved : 1;             // Bit 31:     Reserved for future use
    } bitfield;
    uint32_t raw;  // Raw 32-bit value
} mavlink_custom_mode_t;

/**
 * @brief Autopilot ARM State Structure (32-bit total) for Autopilot HEARTBEAT
 *
 * This structure defines the ARM/DISARM state that autopilot reads from ESP32 GPIO
 * and encodes in its HEARTBEAT message custom_mode field. 32-bit structure allows
 * for future expansion with additional states.
 */
typedef union {
    struct {
        uint32_t arm_state : 4;  // Bits 0-3:   ARM state (mavlink_autopilot_arm_state_t)
        uint32_t reserved1 : 4;  // Bits 4-7:   Reserved for future states
        uint32_t reserved2 : 8;  // Bits 8-15:  Reserved for future states
        uint32_t reserved3 : 8;  // Bits 16-23: Reserved for future use
        uint32_t reserved4 : 8;  // Bits 24-31: Reserved for future use
    } bitfield;
    uint32_t raw;  // Raw 32-bit value
} mavlink_autopilot_states_t;

/**
 * @brief RX State Machine States
 */
typedef enum {
    MAVLINK_RX_IDLE = 0x00,    // 0
    MAVLINK_RX_HEADER = 0x01,  // 1
    MAVLINK_RX_PAYLOAD = 0x02  // 2
} mavlink_rx_state_t;

/**
 * @brief Mavlink State Structure
 *
 * This structure maintains the complete state of the Mavlink communication system.
 */
typedef struct {
    // System configuration
    uint8_t system_id;
    uint8_t component_id;

    // Connection state
    uint8_t connected;
    volatile uint8_t initboard_heartbeat_send_flag;      // Flag to send InitBoard heartbeat
    volatile uint8_t autopilot_connection_timeout_flag;  // Flag for autopilot connection timeout

    // Autopilot ARM state (received from autopilot HEARTBEAT)
    uint8_t autopilot_arm_state;  // Last received ARM state from autopilot

    // VFR_HUD data from autopilot
    mavlink_vfr_hud_data_t vfr_hud_data;

    // RX state machine
    volatile mavlink_rx_state_t rx_state;
    volatile uint16_t rx_index;
    uint8_t payload_length;
    uint16_t expected_length;

    // Callback function and system state pointer
    app_ext_cbk_fn system_callback;
    init_board_system_info_t* system_info;  // Pointer to system state data
} mavlink_state_t;

// ======================= FUNCTION PROTOTYPES =======================
/**
 * @brief Initialize Mavlink UART communication
 * @param system_cbk Callback function for system events
 * @param system_state Pointer to system info structure (will be read when sending InitBoard HEARTBEAT)
 */
void Mavlink_Init(app_ext_cbk_fn system_cbk, init_board_system_info_t* system_info);

/**
 * @brief Process received UART byte (call from UART interrupt)
 * @param byte Received byte
 */
void Mavlink_UartRxByte(uint8_t byte);

/**
 * @brief Main periodic processing function (call from main loop)
 */
void Mavlink_Process(void);

/**
 * @brief Get autopilot ARM state (from last received HEARTBEAT)
 * @return mavlink_autopilot_arm_state_t ARM state (0xFF=unknown)
 */
uint8_t Mavlink_GetAutopilotArmState(void);

// Note: PREARM state is now handled via asynchronous commands (MAVLINK_CMD_PREARM_ENABLE/DISABLE)
// instead of being encoded in HEARTBEAT messages

/**
 * @brief Get last received VFR_HUD data
 * @return const mavlink_vfr_hud_data_t* - Pointer to VFR_HUD data structure
 */
const mavlink_vfr_hud_data_t* Mavlink_GetVfrHudData(void);

// ======================= APPLICATION INTERFACE =======================
/*
 * The application should update the mavlink_system_state_t structure
 * whenever system state changes, particularly when SYSTEM_EVT_INIT_DONE
 * callback is triggered.
 */

#endif  // MAVLINK_UART_H
