# Mavlink InitBoard Integration Guide - Updated Implementation

## Overview
This guide describes the current implementation of the Mavlink communication system between STM32G0 InitBoard and ESP32 Autopilot Emulator. The system uses custom mode bitfield encoding, timer-based messaging, and comprehensive state monitoring.

## System Architecture

### Hardware Configuration
- **STM32G0 InitBoard:** Main controller with Mavlink UART module
- **ESP32 Autopilot Emulator:** Monitors ARM/PREARM states, sends/receives Mavlink messages
- **Communication:** UART at 9600 baud (ESP32 GPIO16/17 ↔ STM32G0 UART2)
- **System ID:** 0x42 (66) synchronized across both systems
- **Component ID:** 0x42 (66) for InitBoard components

### Message Flow
1. **InitBoard → Autopilot:** HEARTBEAT with custom mode bitfield (1Hz)
2. **Autopilot → InitBoard:** HEARTBEAT with ARM/PREARM states (1Hz) 
3. **Command exchange:** COMMAND_LONG and COMMAND_ACK as needed

## STM32G0 Implementation

### 1. Files Structure
```
Core/Inc/mavlink_uart.h     - Mavlink module header
Core/Src/mavlink_uart.c     - Mavlink module implementation
```

### 2. Key Definitions (mavlink_uart.h)
```c
// System Configuration
#define MAVLINK_SYSTEM_ID_INITBOARD        0x42    // 66 - Initiation Board
#define MAVLINK_COMP_ID_USER1              0x42    // 66 - Initiation Board

// Timer Configuration
#define MAVLINK_INITBOARD_HEARTBEAT_INTERVAL_MS  1000  // 1Hz
#define MAVLINK_CONNECTION_TIMEOUT_MS            3000  // 3 seconds

// Custom Mode Bitfield Structure (32-bit)
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
```

### 3. Initialization
```c
// In your main.c or app.c
#include "mavlink_uart.h"

// System info structure
static init_board_system_info_t system_info = {0};

// Mavlink event callback
void App_MavlinkCbk(system_evt_t evt, uint32_t usr_data) {
    switch(evt) {
        case MAVLINK_EVT_COMMAND_DISARM:
            // Handle disarm command
            break;
        case MAVLINK_EVT_COMMAND_ARM:
            // Handle arm command
            break;
        case MAVLINK_EVT_COMMAND_IGNITION:
            // Handle ignition command
            break;
        // ... other events
    }
}

// In main() or app initialization
void App_Init(void) {
    // Initialize system info with default values
    system_info.timer_seconds = 0;
    system_info.timer_mode = MAVLINK_TIMER_MODE_NONE;
    system_info.fuse_present = 0;
    system_info.board_state = MAVLINK_BOARD_STATE_DISARMED;
    system_info.battery_level = 0;
    system_info.error_code = 0;
    
    // Initialize Mavlink
    Mavlink_Init(App_MavlinkCbk, &system_info);
}
```

### 4. Main Loop Integration
```c
void main_loop(void) {
    while(1) {
        // Update system info structure when values change
        system_info.timer_seconds = get_current_timer_value();
        system_info.battery_level = get_battery_level();
        // ... update other fields
        
        // Process Mavlink communication
        Mavlink_Process();
        
        // Other application tasks
        // ...
    }
}
```

### 5. UART Integration
```c
// UART RX Interrupt Handler
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        static uint8_t rx_byte;
        // Get received byte and process
        Mavlink_UartRxByte(rx_byte);
        
        // Restart reception
        HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
    }
}

// Required UART send function (implement in your UART module)
bool UartSendData(uint8_t* wrBuff, uint8_t len) {
    HAL_StatusTypeDef status = HAL_UART_Transmit(&huart2, wrBuff, len, 1000);
    return (status == HAL_OK);
}
```

### 6. Timer Integration
Add to your timer configuration:
```c
// Timer IDs (add to your timer enum)
MAVLINK_INITBOARD_HEARTBEAT_TMR,
MAVLINK_GCS_CONNECTION_TIMEOUT_TMR,
```

## ESP32 Implementation

### 1. Main Features
- **GPIO Monitoring:** GPIO4 (ARM state), GPIO15 (PREARM state)
- **UART Communication:** GPIO16/17 at 9600 baud
- **Debug Output:** Serial monitor at 115200 baud
- **Packet Parsing:** Decodes InitBoard custom mode bitfield

### 2. Key Functions
```cpp
// Send autopilot HEARTBEAT with ARM/PREARM states
void send_autopilot_heartbeat();

// Process received InitBoard HEARTBEAT
void debug_mavlink_rx(uint8_t* packet, uint8_t len);

// Monitor GPIO states with debounce
void process_input_states();
```

### 3. Hardware Connections
```
ESP32 GPIO16 (RX2) → STM32G0 UART2 TX
ESP32 GPIO17 (TX2) → STM32G0 UART2 RX
ESP32 GPIO4        → ARM state switch
ESP32 GPIO15       → PREARM state switch
Common GND
```

## Custom Mode Bitfield

### Encoding (STM32G0 → ESP32)
```c
// STM32G0 encodes system state
custom_mode.bitfield.timer_sec = current_timer_seconds;
custom_mode.bitfield.timer_mode = current_timer_mode;
custom_mode.bitfield.fuse_present = fuse_detected;
custom_mode.bitfield.board_state = current_board_state;
custom_mode.bitfield.battery_level = battery_level_0_to_10;
custom_mode.bitfield.error_code = current_error_code;
```

### Decoding (ESP32)
```cpp
// ESP32 decodes received bitfield
uint16_t timer_sec = custom_mode_raw & 0x3FFF;           // Bits 0-13
uint8_t timer_mode = (custom_mode_raw >> 14) & 0x03;     // Bits 14-15
uint8_t fuse_present = (custom_mode_raw >> 16) & 0x01;   // Bit 16
uint8_t board_state = (custom_mode_raw >> 17) & 0x07;    // Bits 17-19
uint8_t battery_level = (custom_mode_raw >> 20) & 0x0F;  // Bits 20-23
uint8_t error_code = (custom_mode_raw >> 24) & 0x0F;     // Bits 24-27
```

## Debugging

### ESP32 Debug Output
Monitor Serial (115200 baud) for:
```
[MAVLINK_RX] Received packet: FD 09 00 00 71 42 42 00 00 00 ...
[MAVLINK_RX] Message type: HEARTBEAT
[DEBUG] Packet length: 21, System ID: 0x42 (expected: 0x42)
Timer: 1234 sec, Mode: 1, Fuse: 1, State: 2, Battery: 80%, Error: No Error
```

### STM32G0 Debug
- Use SWD debugger with breakpoints in Mavlink callbacks
- Monitor system_info structure updates
- Verify timer callbacks execution

## Current Implementation Status
- ✅ STM32G0 Mavlink UART module with custom mode bitfield
- ✅ ESP32 Autopilot emulator with GPIO monitoring  
- ✅ Timer-based 1Hz HEARTBEAT transmission
- ✅ System ID synchronization (0x42)
- ✅ Custom mode bitfield encoding/decoding
- ✅ Command processing and acknowledgment
- ✅ Debug output and packet parsing
- ✅ Integer-only operations
