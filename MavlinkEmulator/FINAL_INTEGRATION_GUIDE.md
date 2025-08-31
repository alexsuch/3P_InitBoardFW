# Mavlink Integration Guide - Final Implementation

## Overview
This document reflects the actual current implementation of the Mavlink communication system between STM32G0 InitBoard and ESP32 Autopilot Emulator. All information is based on the existing code and actual functionality.

## System Architecture

### Components
- **STM32G0 InitBoard:** Sends HEARTBEAT with custom mode bitfield, processes commands
- **ESP32 Autopilot Emulator:** Monitors ARM/PREARM GPIO states, decodes InitBoard status
- **Communication:** UART 9600 baud, Mavlink v2.0 protocol

### System IDs (Synchronized)
- **InitBoard System ID:** 0x42 (66) - MAVLINK_SYSTEM_ID_INITBOARD
- **InitBoard Component ID:** 0x42 (66) - MAVLINK_COMP_ID_USER1
- **Autopilot System ID:** 0x42 (66) - speaking TO InitBoard
- **Autopilot Component ID:** 0x42 (66) - MAVLINK_COMP_ID_INITBOARD

## STM32G0 Implementation

### File Structure
```
shared/Inc/mavlink_uart.h     - Header with definitions and structures
shared/Src/mavlink_uart.c     - Implementation with timer-based messaging
```

### Key Structures
```c
// Custom mode bitfield (32-bit)
typedef union {
    struct {
        uint32_t timer_sec      : 14;  // Bits 0-13:  Timer seconds (0-16383)
        uint32_t timer_mode     : 2;   // Bits 14-15: Timer mode (0-3)
        uint32_t fuse_present   : 1;   // Bit 16:     Fuse present (0-1)
        uint32_t board_state    : 3;   // Bits 17-19: Board state (0-7)
        uint32_t battery_level  : 4;   // Bits 20-23: Battery level 0-10
        uint32_t error_code     : 4;   // Bits 24-27: Error code (0-15)
        uint32_t reserved       : 4;   // Bits 28-31: Reserved
    } bitfield;
    uint32_t raw;
} mavlink_custom_mode_t;

// Mavlink state structure
typedef struct {
    uint8_t system_id;
    uint8_t component_id;
    uint8_t connected;
    uint8_t initboard_heartbeat_send_flag;
    uint8_t gcs_connection_timeout_flag;
    mavlink_rx_state_t rx_state;
    uint16_t rx_index;
    uint8_t payload_length;
    uint16_t expected_length;
    app_cbk_fn system_callback;
    init_board_system_info_t* system_info;  // Pointer to system state
} mavlink_state_t;
```

### Integration Steps

#### 1. Include Header
```c
#include "mavlink_uart.h"
```

#### 2. System Info Structure
Define the system info structure in your app:
```c
// In app.h or main.h
typedef struct {
    uint16_t timer_seconds;      // Current timer value (0-16383)
    uint8_t timer_mode;          // Timer mode (see mavlink_timer_mode_t)
    uint8_t fuse_present;        // Fuse detection status (0/1)
    uint8_t board_state;         // Board state (see mavlink_board_state_t)
    uint8_t battery_level;       // Battery level 0-10 (0%-100%)
    uint8_t error_code;          // Current error code (0-15)
} init_board_system_info_t;
```

#### 3. Callback Implementation
```c
// Mavlink event callback
static void App_MavlinkCbk(system_evt_t evt, uint32_t usr_data) {
    switch(evt) {
        case MAVLINK_EVT_COMMAND_DISARM:
            // Handle disarm command from autopilot
            break;
        case MAVLINK_EVT_COMMAND_ARM:
            // Handle arm command from autopilot
            break;
        case MAVLINK_EVT_COMMAND_IGNITION:
            // Handle ignition command from autopilot
            break;
        case MAVLINK_EVT_COMMAND_PREARM_ENABLED:
            // Handle prearm enabled command
            break;
        case MAVLINK_EVT_COMMAND_PREARM_DISABLED:
            // Handle prearm disabled command
            break;
        case MAVLINK_EVT_GCS_CONNECTED:
            // Handle autopilot connection established
            break;
        case MAVLINK_EVT_GCS_DISCONNECTED:
            // Handle autopilot connection lost
            break;
        default:
            break;
    }
}
```

#### 4. Initialization
```c
// Global system info structure
static init_board_system_info_t system_info = {0};

void App_Init(void) {
    // Initialize system info with default values
    system_info.timer_seconds = 0;
    system_info.timer_mode = MAVLINK_TIMER_MODE_NONE;
    system_info.fuse_present = 0;
    system_info.board_state = MAVLINK_BOARD_STATE_DISARMED;
    system_info.battery_level = 0;
    system_info.error_code = 0;
    
    // Initialize Mavlink with callback and system info pointer
    Mavlink_Init(App_MavlinkCbk, &system_info);
}
```

#### 5. Main Loop
```c
void main_loop(void) {
    while(1) {
        // Update system info when values change
        system_info.timer_seconds = get_current_timer_value();
        system_info.battery_level = calculate_battery_level();
        system_info.error_code = get_last_error_code();
        // ... update other fields as needed
        
        // Process Mavlink communication (handles timers and callbacks)
        Mavlink_Process();
        
        // Other application tasks
        // ...
    }
}
```

#### 6. UART Integration
```c
// UART RX callback - call from interrupt
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        static uint8_t rx_byte;
        
        // Process received byte
        Mavlink_UartRxByte(rx_byte);
        
        // Continue reception
        HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
    }
}

// Required UART send function (implement in your UART module)
bool UartSendData(uint8_t* wrBuff, uint8_t len) {
    HAL_StatusTypeDef status = HAL_UART_Transmit(&huart2, wrBuff, len, 1000);
    return (status == HAL_OK);
}
```

#### 7. Timer Configuration
Add these timer IDs to your timer configuration:
```c
// Timer IDs
MAVLINK_INITBOARD_HEARTBEAT_TMR,        // For 1Hz HEARTBEAT transmission
MAVLINK_GCS_CONNECTION_TIMEOUT_TMR,     // For connection timeout detection
```

## ESP32 Implementation

### Hardware Configuration
```cpp
#define MAVLINK_ESP32_UART2_RX_PIN 16       // RX2 pin
#define MAVLINK_ESP32_UART2_TX_PIN 17       // TX2 pin
#define MAVLINK_UART_BAUD_RATE 9600         // UART speed
#define MAVLINK_ESP32_ARM_PIN 4             // ARM state monitoring
#define MAVLINK_ESP32_PREARM_PIN 15         // PREARM state monitoring
```

### Key Functions
- `send_autopilot_heartbeat()` - Sends ARM/PREARM states to InitBoard
- `debug_mavlink_rx()` - Decodes InitBoard HEARTBEAT with custom mode
- `process_input_states()` - Monitors GPIO changes with debounce

### Debug Output Example
```
[MAVLINK_RX] Received packet: FD 09 00 00 71 42 42 00 00 00 ...
[MAVLINK_RX] Message type: HEARTBEAT
[DEBUG] Packet length: 21, System ID: 0x42 (expected: 0x42)
Timer: 1234 sec, Mode: 1, Fuse: 1, State: 2, Battery: 80%, Error: No Error
[TX] Autopilot HEARTBEAT: ARM=DISARMED, PREARM=DISABLED
```

## Message Flow

### 1. InitBoard → Autopilot (1Hz)
- **Message:** HEARTBEAT
- **Custom Mode:** 32-bit bitfield with system state
- **Content:** Timer, fuse status, battery level, error code

### 2. Autopilot → InitBoard (1Hz)  
- **Message:** HEARTBEAT
- **Custom Mode:** ARM/PREARM states from GPIO
- **Content:** Autopilot status for InitBoard

### 3. Command Exchange
- **Autopilot → InitBoard:** COMMAND_LONG (on GPIO state changes)
- **InitBoard → Autopilot:** COMMAND_ACK (response to commands)

## Custom Mode Bitfield Mapping

### InitBoard to Autopilot
```c
Bits 0-13:   timer_sec (0-16383)      - Current timer seconds
Bits 14-15:  timer_mode (0-3)         - Timer mode (NONE/SAFE/DESTROY)
Bit 16:      fuse_present (0-1)       - Fuse detection status
Bits 17-19:  board_state (0-7)        - Board state (NO_FC/DISARMED/ARMED/BOOM)
Bits 20-23:  battery_level (0-10)     - Battery percentage in 10% increments
Bits 24-27:  error_code (0-15)        - Current error code
Bits 28-31:  reserved (0)             - Future use
```

### Autopilot to InitBoard
```cpp
Bits 0-3:    arm_state (0-15)         - ARM state from GPIO4
Bits 4-7:    prearm_state (0-15)      - PREARM state from GPIO15
Bits 8-31:   reserved (0)             - Future use
```

## Current Implementation Status
- ✅ STM32G0 Mavlink module with custom mode bitfield encoding
- ✅ ESP32 Autopilot emulator with GPIO state monitoring
- ✅ Timer-based 1Hz HEARTBEAT transmission using Timer API
- ✅ System ID synchronization (0x42) between both systems
- ✅ Command processing (DISARM, ARM, IGNITION, PREARM)
- ✅ Connection timeout detection and recovery
- ✅ Comprehensive debug output and packet parsing
- ✅ Integer-only operations in STM32G0 (no floating point)
- ✅ Event-driven callback architecture for upper layer integration
