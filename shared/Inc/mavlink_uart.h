/*
 * mavlink_uart.h - STM32G0 Mavlink UART Communication Module Header
 * 
 * This header defines the interface for Mavlink v2.0 communication via UART.
 * It includes all necessary constants, structures, and function prototypes.
 */

#ifndef MAVLINK_UART_H
#define MAVLINK_UART_H

#include "stm32g0xx_hal.h"
#include <stdint.h>
#include <init_brd.h>

// ======================= MAVLINK CONSTANTS =======================
// Protocol Configuration
#define MAVLINK_V2_MAGIC                   0xFD    // 253
#define MAVLINK_VERSION                    0x03    // 3
#define MAVLINK_MAX_MESSAGE_SIZE           0x118   // 280
#define MAVLINK_MAX_PAYLOAD_SIZE           0xFF    // 255

// Mavlink v2.0 packet structure constants
#define MAVLINK_V2_HEADER_SIZE             10      // Header size in bytes
#define MAVLINK_V2_CRC_SIZE                2       // CRC size in bytes
#define MAVLINK_V2_MIN_PACKET_SIZE         12      // Header + CRC (minimum packet)

// Mavlink v2.0 header byte indices
#define MAVLINK_V2_MAGIC_INDEX             0       // Magic byte index
#define MAVLINK_V2_PAYLOAD_LEN_INDEX       1       // Payload length index
#define MAVLINK_V2_INCOMPAT_FLAGS_INDEX    2       // Incompatible flags index
#define MAVLINK_V2_COMPAT_FLAGS_INDEX      3       // Compatible flags index
#define MAVLINK_V2_SEQUENCE_INDEX          4       // Sequence number index
#define MAVLINK_V2_SYSTEM_ID_INDEX         5       // System ID index
#define MAVLINK_V2_COMPONENT_ID_INDEX      6       // Component ID index
#define MAVLINK_V2_MSG_ID_LOW_INDEX        7       // Message ID low byte index
#define MAVLINK_V2_MSG_ID_MID_INDEX        8       // Message ID mid byte index
#define MAVLINK_V2_MSG_ID_HIGH_INDEX       9       // Message ID high byte index
#define MAVLINK_V2_PAYLOAD_START_INDEX     10      // Payload start index

// HEARTBEAT message payload indices (relative to payload start)
#define HEARTBEAT_TYPE_INDEX               0       // MAV type
#define HEARTBEAT_AUTOPILOT_INDEX          1       // Autopilot type
#define HEARTBEAT_BASE_MODE_INDEX          2       // Base mode
#define HEARTBEAT_CUSTOM_MODE_INDEX        3       // Custom mode (4 bytes)
#define HEARTBEAT_SYSTEM_STATUS_INDEX      7       // System status
#define HEARTBEAT_MAVLINK_VERSION_INDEX    8       // Mavlink version
#define HEARTBEAT_PAYLOAD_SIZE             9       // HEARTBEAT payload size

// COMMAND_ACK message payload indices (relative to payload start)
#define COMMAND_ACK_COMMAND_INDEX          0       // Command ID (2 bytes)
#define COMMAND_ACK_RESULT_INDEX           2       // Result code
#define COMMAND_ACK_PAYLOAD_SIZE           3       // COMMAND_ACK payload size

// System and Component IDs
#define MAVLINK_SYSTEM_ID_INITBOARD        0x42    // 66 - Initiation Board
#define MAVLINK_COMP_ID_INITBOARD          0x42    // 66 - Initiation Board Component

// Message IDs
#define MAVLINK_MSG_ID_HEARTBEAT           0x00    // 0
#define MAVLINK_MSG_ID_VFR_HUD             0x4A    // 74
#define MAVLINK_MSG_ID_COMMAND_LONG        0x4C    // 76
#define MAVLINK_MSG_ID_COMMAND_ACK         0x4D    // 77

// VFR_HUD message payload indices (relative to payload start)
#define VFR_HUD_AIRSPEED_INDEX             0       // Airspeed (4 bytes float)
#define VFR_HUD_GROUNDSPEED_INDEX          4       // Groundspeed (4 bytes float)
#define VFR_HUD_HEADING_INDEX              8       // Heading (2 bytes int16_t)
#define VFR_HUD_THROTTLE_INDEX             10      // Throttle (2 bytes uint16_t)
#define VFR_HUD_ALT_INDEX                  12      // Altitude (4 bytes float)
#define VFR_HUD_CLIMB_INDEX                16      // Climb rate (4 bytes float)
#define VFR_HUD_PAYLOAD_SIZE               20      // VFR_HUD payload size

// MAV Types and States
#define MAV_TYPE_GENERIC                   0x00    // 0
#define MAV_AUTOPILOT_INVALID              0x08    // 8
#define MAV_STATE_ACTIVE                   0x04    // 4

// MAV Result codes
#define MAV_RESULT_ACCEPTED                0x00    // 0 - Command accepted
#define MAV_RESULT_TEMPORARILY_REJECTED    0x01    // 1 - Command temporarily rejected
#define MAV_RESULT_DENIED                  0x02    // 2 - Command denied
#define MAV_RESULT_UNSUPPORTED             0x03    // 3 - Command not supported
#define MAV_RESULT_FAILED                  0x04    // 4 - Command failed
#define MAV_RESULT_IN_PROGRESS             0x05    // 5 - Command in progress
#define MAV_RESULT_CANCELLED               0x06    // 6 - Command cancelled

// Command IDs  
#define MAVLINK_CMD_IGNITION               0x01    // 1 - Custom ignition command (used in param1[0])
#define MAV_CMD_USER_1                     31010   // MAVLink standard user command 1

/**
 * @brief Custom Command Structure via MAV_CMD_USER_1
 * 
 * ESP32 sends custom commands using MAV_CMD_USER_1 (31010) with custom data in param1:
 * - command field = MAV_CMD_USER_1 (31010)
 * - param1[0] = custom_command_type (MAVLINK_CMD_IGNITION = 1, future commands = 2,3,etc.)
 * - param1[1] = command_data (additional parameters specific to command type)
 * - param1[2] = reserved for future use
 * - param1[3] = reserved for future use
 * 
 * STM32 processes by checking command_id == 31010, then switching on param1[0] for command type.
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
    MAVLINK_EVT_AUTOPILOT_HEARTBEAT = 0x04,        // 4 - Autopilot HEARTBEAT with ARM/PREARM states received
    MAVLINK_EVT_AUTOPILOT_ARMED = 0x05,            // 5 - Autopilot changed to ARMED state
    MAVLINK_EVT_AUTOPILOT_DISARMED = 0x06,         // 6 - Autopilot changed to DISARMED state
    MAVLINK_EVT_AUTOPILOT_PREARM_ENABLED = 0x07,   // 7 - Autopilot PREARM enabled
    MAVLINK_EVT_AUTOPILOT_PREARM_DISABLED = 0x08,  // 8 - Autopilot PREARM disabled
    MAVLINK_EVT_VFR_HUD_RECEIVED = 0x09            // 9 - VFR_HUD data received from autopilot
} mavlink_event_t;

/**
 * @brief VFR_HUD Data Storage Structure (Integer-only for STM32G0)
 * 
 * This structure stores VFR_HUD data using integer representations
 * to avoid floating-point operations on STM32G0 (no FPU).
 */
typedef struct {
    uint16_t airspeed_cm_s;      // Airspeed in cm/s (converted from m/s float)
    uint16_t groundspeed_cm_s;   // Groundspeed in cm/s (converted from m/s float)
    int16_t heading_deg;         // Heading in degrees (0-359, -1 for unknown)
    uint16_t throttle_percent;   // Throttle in percentage (0-100)
    int32_t altitude_cm;         // Altitude in cm (converted from m float)
    int16_t climb_rate_cm_s;     // Climb rate in cm/s (converted from m/s float)
} mavlink_vfr_hud_data_t;

/**
 * @brief Custom Mode Bitfield Structure (32-bit total) for InitBoard HEARTBEAT
 * 
 * This structure defines the custom mode bitfield that encodes system state
 * in the InitBoard HEARTBEAT message custom_mode field.
 */
typedef union {
    struct {
        uint32_t timer_sec      : 14;  // Bits 0-13:  Timer seconds (0-16383)
        uint32_t timer_mode     : 2;   // Bits 14-15: Timer mode (0-3)
        uint32_t fc_control_present : 1; // Bit 16:   FC control present (0-1) - autopilot connection status
        uint32_t fuse_present   : 1;   // Bit 17:     Fuse present (0-1)
        uint32_t board_state    : 3;   // Bits 18-20: Board state (0-7)
        uint32_t battery_level  : 4;   // Bits 21-24: Battery level 0-10 (encoded as 0-15)
        uint32_t error_code     : 4;   // Bits 25-28: Error code (0-15)
        uint32_t is_ignition_done : 1; // Bit 29:     Ignition done flag (0-1)
        uint32_t reserved       : 2;   // Bits 30-31: Reserved for future use
    } bitfield;
    uint32_t raw;                      // Raw 32-bit value
} mavlink_custom_mode_t;

/**
 * @brief Autopilot ARM/PREARM States Structure (32-bit total) for Autopilot HEARTBEAT
 * 
 * This structure defines the ARM/PREARM states that autopilot reads from ESP32 GPIO
 * and encodes in its HEARTBEAT message custom_mode field. 32-bit structure allows
 * for future expansion with additional states.
 */
typedef union {
    struct {
        uint32_t arm_state      : 4;   // Bits 0-3:   ARM state (mavlink_autopilot_arm_state_t)
        uint32_t prearm_state   : 4;   // Bits 4-7:   PREARM state (mavlink_autopilot_prearm_state_t)
        uint32_t reserved1      : 8;   // Bits 8-15:  Reserved for future states
        uint32_t reserved2      : 8;   // Bits 16-23: Reserved for future use
        uint32_t reserved3      : 8;   // Bits 24-31: Reserved for future use
    } bitfield;
    uint32_t raw;                      // Raw 32-bit value
} mavlink_autopilot_states_t;

/**
 * @brief RX State Machine States
 */
typedef enum {
    MAVLINK_RX_IDLE = 0x00,     // 0
    MAVLINK_RX_HEADER = 0x01,   // 1
    MAVLINK_RX_PAYLOAD = 0x02   // 2
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
    uint8_t initboard_heartbeat_send_flag;     // Flag to send InitBoard heartbeat
    uint8_t autopilot_connection_timeout_flag; // Flag for autopilot connection timeout
    
    // Autopilot ARM/PREARM states (received from autopilot HEARTBEAT)
    uint8_t autopilot_arm_state;               // Last received ARM state from autopilot
    uint8_t autopilot_prearm_state;            // Last received PREARM state from autopilot
    
    // VFR_HUD data from autopilot
    mavlink_vfr_hud_data_t vfr_hud_data;
    
    // RX state machine
    mavlink_rx_state_t rx_state;
    uint16_t rx_index;
    uint8_t payload_length;
    uint16_t expected_length;
    
    // Callback function and system state pointer
    app_cbk_fn system_callback;
    init_board_system_info_t* system_info;  // Pointer to system state data
} mavlink_state_t;

// ======================= FUNCTION PROTOTYPES =======================
/**
 * @brief Initialize Mavlink UART communication
 * @param system_cbk Callback function for system events
 * @param system_state Pointer to system info structure (will be read when sending InitBoard HEARTBEAT)
 */
void Mavlink_Init(app_cbk_fn system_cbk, init_board_system_info_t* system_info);

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

/**
 * @brief Get autopilot PREARM state (from last received HEARTBEAT)
 * @return mavlink_autopilot_prearm_state_t PREARM state (0xFF=unknown)
 */
uint8_t Mavlink_GetAutopilotPrearmState(void);

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

#endif // MAVLINK_UART_H
