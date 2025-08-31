# Mavlink InitBoard Integration Guide - Updated

## Overview
This guide describes how to integrate the implemented Mavlink communication system between STM32G0 InitBoard and ESP32 Autopilot Emulator. The system features custom mode bitfield transmission, timer-based periodic messaging, and comprehensive system state monitoring.

## Generated Files

### ESP32 Files
- `main.cpp` - Complete ESP32 Autopilot Emulator with ARM/PREARM state monitoring

### STM32G0 Files
- `mavlink_uart.c` - Mavlink UART communication module with custom mode bitfield
- `mavlink_uart.h` - Header file with all definitions and data structures

## Key Features
- **Custom Mode Bitfield:** 32-bit system state encoding in HEARTBEAT messages
- **Timer-Based Messaging:** 1Hz periodic HEARTBEAT transmission using Timer API
- **ARM/PREARM Monitoring:** ESP32 GPIO state monitoring and transmission
- **System ID Synchronization:** Both systems use 0x42 (66) as system ID
- **Debug Output:** Comprehensive packet parsing and status display
- **Integer Operations:** No floating point operations in STM32G0 code

## STM32G0 Integration Steps

### 1. Add Files to Project
Copy the generated files to your STM32 project:
```
Core/Src/mavlink_uart.c
Core/Inc/mavlink_uart.h
```

### 2. Update CMakeLists.txt
Add the new source file to your CMakeLists.txt:
```cmake
# Add mavlink_uart.c to sources
set(SOURCES
    Core/Src/main.c
    Core/Src/mavlink_uart.c  # Add this line
    # ... other source files
)
```

### 3. Include Header in main.c
Add the include at the top of your main.c:
```c
#include "mavlink_uart.h"
```

### 4. Initialize in main() function
Add initialization call in your main() function with callback:
```c
// Callback function for Mavlink events
void mavlink_system_callback(system_evt_t evt, uint32_t usr_data) {
    if (evt == SYSTEM_EVT_READY) {
        mavlink_event_t mavlink_event = (mavlink_event_t)usr_data;
        
        switch (mavlink_event) {
            case MAVLINK_EVT_COMMAND_DISARM:
                // Handle disarm command
                app_process_disarm();
                break;
                
            case MAVLINK_EVT_COMMAND_ARM:
                // Handle arm command
                app_process_arm();
                break;
                
            case MAVLINK_EVT_COMMAND_IGNITION:
                // Handle ignition command
                app_process_ignition();
                break;
                
            case MAVLINK_EVT_COMMAND_PREARM_ENABLED:
                // Handle PREARM enabled
                app_process_prearm_enabled();
                break;
                
            case MAVLINK_EVT_COMMAND_PREARM_DISABLED:
                // Handle PREARM disabled
                app_process_prearm_disabled();
                break;
                
            case MAVLINK_EVT_VFR_HUD_RECEIVED:
                // VFR_HUD data received (optional processing)
                break;
                
            case MAVLINK_EVT_GCS_CONNECTED:
                // GCS connection established
                app_notify_gcs_connected();
                break;
                
            case MAVLINK_EVT_GCS_DISCONNECTED:
                // GCS connection lost
                app_notify_gcs_disconnected();
                break;
        }
    }
}

int main(void) {
    // ... existing HAL initialization code
    
    // Initialize Mavlink UART with callback
    Mavlink_Init(mavlink_system_callback);
    
    // ... rest of your code
}
```

### 5. Add to Main Loop
Add the periodic processing call in your main loop:
```c
while (1) {
    // Process Mavlink communication
    Mavlink_Process();
    
    // ... rest of your main loop
}
```

### 6. Add UART Callbacks
Add these callbacks to your main.c or stm32g0xx_it.c:

```c
// UART RX callback - call this from UART interrupt or HAL callback
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        static uint8_t rx_byte;
        Mavlink_UartRxByte(rx_byte);
        
        // Restart reception for next byte
        HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
    }
}

// UART TX callback - call this when UartSendData transmission is complete
void UartTxCompleteCallback(void) {
    Mavlink_UartTxComplete();
}
```

### 7. Required UART Function
The Mavlink module uses the public UART function `UartSendData()` for transmission.
Make sure this function is available in your project:

```c
// This function should be implemented in your UART module
bool UartSendData(uint8_t* wrBuff, uint8_t len);
```

The function should:
- Return `true` if transmission started successfully
- Return `false` if transmission could not be started
- Call `Mavlink_UartTxComplete()` when transmission is complete

### 8. Implement Application Interface Functions
Add these functions to your app.c file to provide system state data:

```c
// Get current timer seconds value (0-16383)
uint16_t app_get_timer_seconds(void) {
    // Return your timer seconds value
    return timer_current_seconds;
}

// Get current timer mode (0-3)
uint8_t app_get_timer_mode(void) {
    // Return your timer mode
    return current_timer_mode;
}

// Get fuse presence status (0 or 1)
uint8_t app_get_fuse_status(void) {
    // Return fuse status
    return fuse_detected ? 1 : 0;
}

// Get current board state (0-7)
uint8_t app_get_board_state(void) {
    // Return current board state
    return current_board_state;
}

// Get battery level (0-10, where 10 = 100%)
uint8_t app_get_battery_level(void) {
    // Return calculated battery level from ADC processing
    // This should return the result from App_CalculateBatteryLevel()
    return current_battery_level;
}

// Battery level calculation function (add to app.c)
uint8_t App_CalculateBatteryLevel(uint16_t adc_voltage_mv) {
    // AAA battery voltage range: 900mV (0%) to 1500mV (100%)
    if (adc_voltage_mv <= BATTERY_VOLTAGE_0_PERCENT_THRESHOLD_MILIVOLTS) {
        return 0;  // 0%
    }
    if (adc_voltage_mv >= BATTERY_VOLTAGE_100_PERCENT_THRESHOLD_MILIVOLTS) {
        return 10; // 100%
    }
    
    // Calculate level in 10% increments (0-10 scale)
    uint8_t level = (adc_voltage_mv - BATTERY_VOLTAGE_0_PERCENT_THRESHOLD_MILIVOLTS) * 10 / 
                    (BATTERY_VOLTAGE_100_PERCENT_THRESHOLD_MILIVOLTS - BATTERY_VOLTAGE_0_PERCENT_THRESHOLD_MILIVOLTS);
    
    return level;
}

// Get current error code (0-15)
uint8_t app_get_error_code(void) {
    // Return current error code
    return current_error_code;
}

// Process commands received via callback
void app_process_disarm(void) {
    // Implement disarm logic
}

void app_process_arm(void) {
    // Implement arm logic
}

void app_process_ignition(void) {
    // Implement ignition logic
}

void app_process_prearm_enabled(void) {
    // Handle PREARM enabled
}

void app_process_prearm_disabled(void) {
    // Handle PREARM disabled
}

void app_notify_gcs_connected(void) {
    // Handle GCS connection
}

void app_notify_gcs_disconnected(void) {
    // Handle GCS disconnection
}
```

## ESP32 Integration Steps

### 1. Replace main.cpp
Replace your existing ESP32 main.cpp with the generated implementation.

### 2. Hardware Connections
Connect the ESP32 to STM32G0:
- ESP32 GPIO16 (RX2) → STM32G0 UART2 TX
- ESP32 GPIO17 (TX2) → STM32G0 UART2 RX
- Common GND connection

### 3. Control Inputs
Connect control inputs to ESP32:
- 3-position switch A → GPIO4
- 3-position switch B → GPIO2  
- PREARM button → GPIO15
- All inputs use internal pull-ups

### 4. Upload and Test
Upload the code to ESP32 and verify:
- Serial monitor shows initialization messages
- Switch changes send commands to STM32
- HEARTBEAT and VFR_HUD messages are sent periodically

## System Operation

### Message Flow
1. ESP32 sends HEARTBEAT and VFR_HUD messages every 1000ms (with 500ms offset)
2. ESP32 monitors inputs and sends COMMAND_LONG messages on changes
3. STM32G0 receives commands and sends COMMAND_ACK responses
4. STM32G0 sends HEARTBEAT with custom mode bitfield containing system state

### Custom Mode Bitfield
The STM32G0 HEARTBEAT message includes a 32-bit custom mode field with:
- Bits 0-7: Switch 1 ADC value (0-255) 
- Bits 8-15: Switch 2 ADC value (0-255)
- Bits 16-19: Switch 3 ADC value (0-15)
- Bits 20-23: Battery level (0-10, representing 0%-100% in 10% increments)
- Bits 24-31: Reserved for future use

### Battery Level Encoding
- **Range:** 0-10 (4 bits)
- **Mapping:** 0 = 0%, 1 = 10%, 2 = 20%, ..., 10 = 100%
- **Voltage Range:** 900mV (0%) to 1500mV (100%) for AAA battery
- **Update:** Calculated in ADC processing and transmitted in HEARTBEAT custom mode

### Integration Requirements
Add to prj_config.h:
```c
#define BATTERY_VOLTAGE_100_PERCENT_THRESHOLD_MILIVOLTS  1500
#define BATTERY_VOLTAGE_0_PERCENT_THRESHOLD_MILIVOLTS    900
```

Add to app.c the App_CalculateBatteryLevel() function and integrate battery_level updates in ADC processing.

### Command Types
- DISARM (1): 3-position switch in disarm position
- ARM (2): 3-position switch in neutral position
- IGNITION (3): 3-position switch in ignition position
- PREARM_ENABLED (4): PREARM button pressed
- PREARM_DISABLED (5): PREARM button released

## Debugging

### ESP32 Debug Output
Monitor ESP32 Serial (115200 baud) for:
- Initialization messages
- Input state changes
- Transmitted command details
- Received message parsing

### STM32G0 Debug
Use your preferred STM32 debugging method:
- SWD debugger with breakpoints
- UART debug output (if available)
- LED indicators for status

## Testing Checklist

1. ✅ ESP32 initializes and shows all subsystems ready
2. ✅ UART2 communication established between ESP32 and STM32G0
3. ✅ 3-position switch changes trigger command transmission
4. ✅ PREARM button changes trigger command transmission
5. ✅ STM32G0 receives and acknowledges commands
6. ✅ STM32G0 sends periodic HEARTBEAT with valid custom mode
7. ✅ ESP32 receives and decodes STM32G0 HEARTBEAT messages
8. ✅ VFR_HUD simulation data changes dynamically
9. ✅ Connection timeout detection works correctly
10. ✅ All message timing prevents UART overlap

## Troubleshooting

### No Communication
- Check UART wiring (TX ↔ RX crossed)
- Verify baud rate (9600) on both sides
- Check GND connection
- Ensure UART2 is properly initialized on STM32G0

### Commands Not Working
- Verify input wiring and pull-up configuration
- Check debounce timing (300ms)
- Monitor ESP32 serial output for command transmission
- Verify STM32G0 command processing implementation

### Timing Issues
- Ensure main loop calls mavlink_uart_update() regularly
- Check HEARTBEAT interval timing
- Verify VFR_HUD offset prevents message overlap
- Monitor for UART transmission busy states

This integration guide should help you successfully implement the Mavlink communication system in your project!
