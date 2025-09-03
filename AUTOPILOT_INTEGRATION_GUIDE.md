# MAVLink Integration Guide for Autopilot Development

## Overview
This document provides complete integration specifications for MAVLink v2.0 communication with the InitBoard system. The InitBoard acts as a specialized payload device that requires bidirectional MAVLink communication for state monitoring and command execution.

## Communication Configuration

### UART Settings
```c
// UART Configuration
#define MAVLINK_UART_BAUD_RATE           9600      // Baud rate
#define MAVLINK_UART_DATA_BITS           8         // Data bits
#define MAVLINK_UART_STOP_BITS           1         // Stop bits  
#define MAVLINK_UART_PARITY              NONE      // No parity
#define MAVLINK_UART_FLOW_CONTROL        NONE      // No flow control
```

### System Identification
```c
// System and Component IDs
#define MAVLINK_SYSTEM_ID_INITBOARD      0x42      // 66 - InitBoard system ID
#define MAVLINK_COMP_ID_INITBOARD        0x42      // 66 - InitBoard component ID

// Message Protocol
#define MAVLINK_VERSION                  2         // MAVLink v2.0
#define MAVLINK_V2_MAGIC                 0xFD      // 253 - MAVLink v2.0 magic byte
```

### Timing Configuration
```c
// Message Frequencies
#define AUTOPILOT_HEARTBEAT_INTERVAL_MS  1000      // Send HEARTBEAT every 1000ms (1Hz)
#define INITBOARD_HEARTBEAT_INTERVAL_MS  750       // InitBoard sends HEARTBEAT every 750ms
#define CONNECTION_TIMEOUT_MS            3000      // Connection timeout after 3000ms of no messages
#define COMMAND_RESPONSE_TIMEOUT_MS      1000      // Wait for COMMAND_ACK up to 1000ms
```

## Message Types

### 1. HEARTBEAT Messages (Message ID: 0)

#### 1.1 Autopilot  InitBoard HEARTBEAT
**Purpose:** Send autopilot ARM/PREARM states to InitBoard
**Frequency:** 1Hz (every 1000ms)
**Direction:** Autopilot  InitBoard

```c
// Custom Mode Structure for Autopilot HEARTBEAT (32-bit)
typedef union {
    struct {
        uint32_t arm_state    : 4;   // Bits 0-3:   ARM state (0=DISARMED, 1=ARMED)
        uint32_t prearm_state : 4;   // Bits 4-7:   PREARM state (0=DISABLED, 1=ENABLED)
        uint32_t reserved1    : 8;   // Bits 8-15:  Reserved for future states
        uint32_t reserved2    : 8;   // Bits 16-23: Reserved for future use  
        uint32_t reserved3    : 8;   // Bits 24-31: Reserved for future use
    } bitfield;
    uint32_t raw;                    // Raw 32-bit value
} autopilot_custom_mode_t;

// ARM/PREARM State Values
#define AUTOPILOT_ARM_DISARMED           0    // Autopilot disarmed
#define AUTOPILOT_ARM_ARMED              1    // Autopilot armed
#define AUTOPILOT_PREARM_DISABLED        0    // PREARM disabled
#define AUTOPILOT_PREARM_ENABLED         1    // PREARM enabled
```

#### 1.2 InitBoard  Autopilot HEARTBEAT  
**Purpose:** Receive InitBoard system state information
**Frequency:** ~1.33Hz (every 750ms)
**Direction:** InitBoard  Autopilot

```c
// InitBoard Custom Mode Structure (32-bit)
typedef union {
    struct {
        uint32_t timer_sec          : 14; // Bits 0-13:  Timer seconds (0-16383)
        uint32_t timer_mode         : 2;  // Bits 14-15: Timer mode (see below)
        uint32_t fc_control_present : 1;  // Bit 16:     FC control present (0-1)
        uint32_t fuse_present       : 1;  // Bit 17:     Fuse present (0-1)
        uint32_t board_state        : 3;  // Bits 18-20: Board state (see below)
        uint32_t battery_level      : 4;  // Bits 21-24: Battery level 0-10 (encoded as 0-15)
        uint32_t error_code         : 4;  // Bits 25-28: Error code (see below)
        uint32_t is_ignition_done   : 1;  // Bit 29:     Ignition done flag (0-1)
        uint32_t reserved           : 2;  // Bits 30-31: Reserved for future use
    } bitfield;
    uint32_t raw;                         // Raw 32-bit value
} initboard_custom_mode_t;

// Timer Mode Values
#define TIMER_MODE_NONE              0    // No active timer
#define TIMER_MODE_SAFE              1    // Safe timer active  
#define TIMER_MODE_SELF_DESTROY      2    // Self destroy timer active

// Board State Values
#define BOARD_STATE_INIT             0    // Initialization
#define BOARD_STATE_DISARMED         1    // Disarmed
#define BOARD_STATE_CHARGING         2    // Charging
#define BOARD_STATE_ARMED            3    // Armed
#define BOARD_STATE_BOOM             4    // Boom (ignition completed)

// Error Code Values
#define ERR_CODE_NO_ERROR            0    // No error
#define ERR_CODE_ACCELEROMETER_FAIL  1    // Accelerometer failure
#define ERR_CODE_BATTERY_LOW         2    // Battery low
#define ERR_CODE_FUSE_INCORRECT      3    // Fuse incorrect state
#define ERR_CODE_UNEXPECTED_IGNITION 4    // Unexpected ignition
#define ERR_CODE_UNEXPECTED_ARM      5    // Unexpected arm
#define ERR_CODE_UNEXPECTED_MINING   6    // Unexpected mining
#define ERR_CODE_VUSA_SHORTED        7    // VUSA shorted
```

### 2. COMMAND_LONG Messages (Message ID: 76)

#### 2.1 Autopilot  InitBoard Commands
**Purpose:** Send commands to InitBoard (currently IGNITION only)
**Frequency:** On-demand (triggered by autopilot logic)
**Direction:** Autopilot  InitBoard

```c
// Custom Command Structure via MAV_CMD_USER_1
#define MAV_CMD_USER_1               31010   // MAVLink standard user command 1

// Custom Command Types (sent in param1[0])
#define MAVLINK_CMD_IGNITION         1       // Ignition command
// Future commands: 2, 3, 4, etc.

// Command Data Values (sent in param1[1]) 
#define IGNITION_DATA_NORMAL         0       // Normal ignition
#define IGNITION_DATA_DELAYED        1       // Delayed ignition
#define IGNITION_DATA_TEST           2       // Test mode ignition
// Future command data values as needed

// Custom Command Data Structure (packed into param1)
typedef union {
    struct {
        uint8_t command_type;   // Custom command type (MAVLINK_CMD_IGNITION = 1)
        uint8_t command_data;   // Command data (IGNITION_DATA_* values)
        uint8_t reserved1;      // Reserved (set to 0)
        uint8_t reserved2;      // Reserved (set to 0)
    } fields;
    float param1_float;         // Float representation for param1
} custom_command_data_t;
```

### 3. COMMAND_ACK Messages (Message ID: 77)

#### 3.1 InitBoard  Autopilot Command Acknowledgment
**Purpose:** Receive command execution results from InitBoard
**Frequency:** Response to each COMMAND_LONG
**Direction:** InitBoard  Autopilot

```c
// Command Result Codes
#define MAV_RESULT_ACCEPTED                0    // Command accepted and executed
#define MAV_RESULT_TEMPORARILY_REJECTED    1    // Command temporarily rejected
#define MAV_RESULT_DENIED                  2    // Command denied
#define MAV_RESULT_UNSUPPORTED             3    // Command not supported
#define MAV_RESULT_FAILED                  4    // Command failed
#define MAV_RESULT_IN_PROGRESS             5    // Command in progress
#define MAV_RESULT_CANCELLED               6    // Command cancelled
```
