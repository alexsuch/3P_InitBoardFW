# Mavlink InitBoard ↔ Autopilot Integration Guide - Final Version

## Overview
This document describes the integration between STM32G0 InitBoard and ESP32 Autopilot using Mavlink v2.0 protocol. The system has been refactored from GCS communication to Autopilot communication.

## Architecture Changes
- **Previous**: STM32G0 ↔ GCS communication
- **Current**: STM32G0 InitBoard ↔ ESP32 Autopilot communication
- **Communication**: Bidirectional UART at 9600 baud

## System Components

### 1. STM32G0 InitBoard
**Role**: Receives commands from autopilot, sends status updates
**Files**: `mavlink_uart.c`, `mavlink_uart.h`
**Key Functions**:
- Sends HEARTBEAT every 1Hz with system status
- Receives HEARTBEAT from autopilot with ARM/PREARM states
- Processes IGNITION commands from autopilot
- Monitors autopilot connection (5-second timeout)

### 2. ESP32 Autopilot Emulator
**Role**: Monitors GPIO states, sends commands to InitBoard
**Files**: `main.cpp`
**Key Functions**:
- Reads GPIO states (ARM/PREARM/IGNITION)
- Sends HEARTBEAT every 1Hz with ARM/PREARM states
- Sends IGNITION commands when GPIO0 goes LOW
- Provides debug output via Serial

## Message Flow

### 1. InitBoard → Autopilot: STATUS HEARTBEAT
```
Rate: 1Hz
Message: HEARTBEAT (0x00)
Purpose: System status notification
Custom Mode: 32-bit bitfield with timer, fuse, battery, error status
```

### 2. Autopilot → InitBoard: ARM/PREARM HEARTBEAT
```
Rate: 1Hz
Message: HEARTBEAT (0x00)
Purpose: ARM/PREARM state notification
Custom Mode: ARM state (bits 0-3), PREARM state (bits 4-7)
```

### 3. Autopilot → InitBoard: IGNITION COMMAND
```
Trigger: GPIO0 changes to LOW (IGNITION ON)
Message: COMMAND_LONG (0x4C)
Command: MAVLINK_CMD_IGNITION (0x01)
Response: COMMAND_ACK from InitBoard
```

## GPIO Configuration (ESP32)

| GPIO | Function | Direction | Pull-up | Logic |
|------|----------|-----------|---------|-------|
| 4    | ARM      | Input     | Yes     | LOW=DISARMED, HIGH=ARMED |
| 15   | PREARM   | Input     | Yes     | LOW=ENABLED, HIGH=DISABLED |
| 0    | IGNITION | Input     | Yes     | LOW=ON, HIGH=OFF |
| 16   | UART2 RX | Input     | No      | Receives from STM32G0 TX |
| 17   | UART2 TX | Output    | No      | Transmits to STM32G0 RX |

## Custom Mode Bitfield Specifications

### InitBoard HEARTBEAT Custom Mode (32-bit)
```c
typedef union {
    struct {
        uint32_t timer_sec      : 14;  // Timer seconds (0-16383)
        uint32_t timer_mode     : 2;   // Timer mode (NONE=0/SAFE=1/DESTROY=2)
        uint32_t fuse_present   : 1;   // Fuse status (0=No, 1=Yes)
        uint32_t board_state    : 3;   // Board state (NO_FC=0/DISARMED=1/ARMED=2/BOOM=3)
        uint32_t battery_level  : 4;   // Battery level (0-10)
        uint32_t error_code     : 4;   // Error code (0-15)
        uint32_t reserved       : 4;   // Reserved
    } bitfield;
    uint32_t raw;
} mavlink_custom_mode_t;
```

### Autopilot HEARTBEAT Custom Mode (32-bit)
```c
typedef union {
    struct {
        uint32_t arm_state      : 4;   // ARM state (DISARMED=0/ARMED=1)
        uint32_t prearm_state   : 4;   // PREARM state (DISABLED=0/ENABLED=1)
        uint32_t reserved1      : 8;   // Reserved for future states
        uint32_t reserved2      : 8;   // Reserved
        uint32_t reserved3      : 8;   // Reserved
    } bitfield;
    uint32_t raw;
} mavlink_autopilot_states_t;
```

## Key Enumerations

### Timer Mode
```c
typedef enum {
    MAVLINK_TIMER_MODE_NONE = 0x00,     // No active timer
    MAVLINK_TIMER_MODE_SAFE = 0x01,     // Safe timer active
    MAVLINK_TIMER_MODE_DESTROY = 0x02   // Self destroy timer active
} mavlink_timer_mode_t;
```

### Board State
```c
typedef enum {
    MAVLINK_BOARD_STATE_NO_FC = 0x00,       // No FC connected
    MAVLINK_BOARD_STATE_DISARMED = 0x01,    // Disarmed
    MAVLINK_BOARD_STATE_ARMED = 0x02,       // Armed
    MAVLINK_BOARD_STATE_BOOM = 0x03         // Boom
} mavlink_board_state_t;
```

### Autopilot States
```c
typedef enum {
    MAVLINK_AUTOPILOT_ARM_DISARMED = 0x00,  // Autopilot disarmed
    MAVLINK_AUTOPILOT_ARM_ARMED = 0x01      // Autopilot armed
} mavlink_autopilot_arm_state_t;

typedef enum {
    MAVLINK_AUTOPILOT_PREARM_DISABLED = 0x00,  // PREARM disabled
    MAVLINK_AUTOPILOT_PREARM_ENABLED = 0x01    // PREARM enabled
} mavlink_autopilot_prearm_state_t;
```

## Event System

### STM32G0 Mavlink Events
```c
typedef enum {
    MAVLINK_EVT_COMMAND_IGNITION = 0x01,           // Ignition command received
    MAVLINK_EVT_AUTOPILOT_CONNECTED = 0x02,        // Autopilot connected
    MAVLINK_EVT_AUTOPILOT_DISCONNECTED = 0x03,     // Autopilot disconnected
    MAVLINK_EVT_AUTOPILOT_HEARTBEAT = 0x04,        // Autopilot HEARTBEAT received
    MAVLINK_EVT_AUTOPILOT_ARMED = 0x05,            // Autopilot armed
    MAVLINK_EVT_AUTOPILOT_DISARMED = 0x06,         // Autopilot disarmed
    MAVLINK_EVT_AUTOPILOT_PREARM_ENABLED = 0x07,   // PREARM enabled
    MAVLINK_EVT_AUTOPILOT_PREARM_DISABLED = 0x08   // PREARM disabled
} mavlink_event_t;
```

## API Functions

### STM32G0 InitBoard
```c
// Initialization
void Mavlink_Init(app_cbk_fn system_cbk, init_board_system_info_t* system_info);

// Message processing
void Mavlink_UartRxByte(uint8_t byte);
void Mavlink_Process(void);

// State access
uint8_t Mavlink_GetAutopilotArmState(void);
uint8_t Mavlink_GetAutopilotPrearmState(void);
```

### ESP32 Autopilot
```c
// Main functions
void setup();
void loop();

// State reading
mavlink_arm_state_t read_arm_state();
mavlink_prearm_state_t read_prearm_state();
mavlink_ignition_state_t read_ignition_state();

// Message generation
void send_autopilot_heartbeat();
void send_ignition_command();
```

## Timing Configuration
- **HEARTBEAT Rate**: 1Hz (1000ms) - both directions
- **Autopilot Timeout**: 5 seconds
- **GPIO Debounce**: 300ms
- **UART Speed**: 9600 baud
- **Debug UART**: 115200 baud (ESP32 only)

## Connection Monitoring
1. **STM32G0**: Monitors autopilot connection via HEARTBEAT reception
2. **Timeout**: 5 seconds without HEARTBEAT triggers disconnection event
3. **Reconnection**: Automatic when HEARTBEAT received again
4. **Status**: Available via `mavlink_state.connected` flag

## Integration Checklist
- [ ] STM32G0 UART configured for 9600 baud
- [ ] ESP32 UART2 pins connected (GPIO 16/17)
- [ ] ESP32 GPIO inputs configured with pull-ups
- [ ] Timer IDs allocated for Mavlink functions
- [ ] System callback function implemented
- [ ] System info structure properly updated
- [ ] Debug output configured (ESP32 Serial)
- [ ] Connection timeout handling implemented

## Debug and Monitoring
- **ESP32 Debug**: Serial output at 115200 baud shows GPIO states and message traffic
- **STM32G0 Debug**: Can be added via UART or SWD
- **Message Validation**: CRC verification on all packets
- **State Monitoring**: Real-time ARM/PREARM state changes logged

This integration provides a robust bidirectional communication system between InitBoard and Autopilot with comprehensive state monitoring and command handling capabilities.
