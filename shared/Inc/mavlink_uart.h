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

// System and Component IDs
#define MAVLINK_SYSTEM_ID_INITBOARD        0x42    // 66 - Initiation Board
#define MAVLINK_COMP_ID_USER1              0x42    // 66 - Initiation Board
#define MAVLINK_SYSTEM_ID_GCS              0xFF    // 255
#define MAVLINK_COMP_ID_GCS                0xBE    // 190

// Message IDs
#define MAVLINK_MSG_ID_HEARTBEAT           0x00    // 0
#define MAVLINK_MSG_ID_VFR_HUD             0x4A    // 74
#define MAVLINK_MSG_ID_COMMAND_LONG        0x4C    // 76
#define MAVLINK_MSG_ID_COMMAND_ACK         0x4D    // 77

// MAV Types and States
#define MAV_TYPE_GENERIC                   0x00    // 0
#define MAV_AUTOPILOT_INVALID              0x08    // 8
#define MAV_STATE_ACTIVE                   0x04    // 4

// Command IDs
#define MAV_CMD_USER_1                     0x7922  // 31010

// Result Codes
#define MAV_RESULT_ACCEPTED                0x00    // 0
#define MAV_RESULT_TEMPORARILY_REJECTED    0x01    // 1
#define MAV_RESULT_DENIED                  0x02    // 2
#define MAV_RESULT_UNSUPPORTED             0x03    // 3
#define MAV_RESULT_FAILED                  0x04    // 4

// Custom Command Types (matching ESP32 implementation)
#define MAVLINK_CMD_DISARM                 0x01    // 1
#define MAVLINK_CMD_ARM                    0x02    // 2
#define MAVLINK_CMD_IGNITION               0x03    // 3
#define MAVLINK_CMD_PREARM_ENABLED         0x04    // 4
#define MAVLINK_CMD_PREARM_DISABLED        0x05    // 5

// ======================= TYPE DEFINITIONS =======================

/**
 * @brief Mavlink Events for system callback
 */
typedef enum {
    MAVLINK_EVT_COMMAND_DISARM = 0x01,        // 1 - Disarm command received
    MAVLINK_EVT_COMMAND_ARM = 0x02,           // 2 - Arm command received
    MAVLINK_EVT_COMMAND_IGNITION = 0x03,      // 3 - Ignition command received
    MAVLINK_EVT_COMMAND_PREARM_ENABLED = 0x04, // 4 - PREARM enabled command received
    MAVLINK_EVT_COMMAND_PREARM_DISABLED = 0x05, // 5 - PREARM disabled command received
    MAVLINK_EVT_VFR_HUD_RECEIVED = 0x06,      // 6 - VFR_HUD data received
    MAVLINK_EVT_GCS_CONNECTED = 0x07,         // 7 - GCS connection established
    MAVLINK_EVT_GCS_DISCONNECTED = 0x08       // 8 - GCS connection lost
} mavlink_event_t;

/**
 * @brief Custom Mode Bitfield Structure (32-bit total)
 * 
 * This structure defines the custom mode bitfield that encodes system state
 * in the InitBoard HEARTBEAT message custom_mode field.
 */
typedef union {
    struct {
        uint32_t timer_sec      : 14;  // Bits 0-13:  Timer seconds (0-16383)
        uint32_t timer_mode     : 2;   // Bits 14-15: Timer mode (0-3)
        uint32_t fuse_present   : 1;   // Bit 16:     Fuse present (0-1)
        uint32_t board_state    : 3;   // Bits 17-19: Board state (0-7)
        uint32_t battery_level  : 4;   // Bits 20-23: Battery level 0-10 (encoded as 0-15)
        uint32_t error_code     : 4;   // Bits 24-27: Error code (0-15)
        uint32_t reserved       : 4;   // Bits 28-31: Reserved for future use
    } bitfield;
    uint32_t raw;                      // Raw 32-bit value
} mavlink_custom_mode_t;

/**
 * @brief VFR_HUD data structure (integer format)
 * 
 * This structure holds VFR_HUD message data converted to integer format
 * for use in integer-only STM32G0 operations.
 */
typedef struct {
    int32_t airspeed_cm_s;       // Airspeed in cm/s (original float * 100)
    int32_t groundspeed_cm_s;    // Groundspeed in cm/s (original float * 100)
    int32_t altitude_cm;         // Altitude in cm (original float * 100)
    int32_t climb_rate_cm_s;     // Climb rate in cm/s (original float * 100)
    int16_t heading_deg;         // Heading in degrees
    uint16_t throttle_percent;   // Throttle percentage (0-100)
} mavlink_vfr_hud_data_t;

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
    uint8_t gcs_connection_timeout_flag; // Flag for GCS connection timeout
    
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

// ======================= APPLICATION INTERFACE =======================
/*
 * The application should update the mavlink_system_state_t structure
 * whenever system state changes, particularly when SYSTEM_EVT_INIT_DONE
 * callback is triggered.
 */

#endif // MAVLINK_UART_H
