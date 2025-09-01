# Mavlink Integration Guide - Current Implementation Status

## Overview
This document reflects the actual implementation of the Mavlink communication system for InitBoard ↔ Autopilot communication. The system uses bidirectional HEARTBEAT messages and COMMAND_LONG for ignition control between STM32G0 InitBoard and ESP32 Autopilot Emulator.

## Key Implementation Features
- **Bidirectional Communication:** InitBoard sends status, Autopilot sends GPIO states
- **Timer-Based Messaging:** Uses Timer API for 1Hz periodic HEARTBEAT transmission
- **System ID Synchronization:** Both systems use 0x42 (66) as system ID
- **Event-Driven Architecture:** Callback-based event handling for autopilot events
- **Integer-Only Operations:** No floating point math in STM32G0 code
- **GPIO Monitoring:** Real-time autopilot ARM/PREARM/IGNITION state tracking

## Current Implementation Architecture

### STM32G0 InitBoard
- **File:** `mavlink_uart.c/.h` in `shared/Src/` and `shared/Inc/`
- **System ID:** 0x42 (66)
- **Component ID:** 1
- **Function:** Sends status HEARTBEAT, receives autopilot HEARTBEAT and IGNITION commands

### ESP32 Autopilot Emulator  
- **File:** `main.cpp` in `MavlinkEmulator/src/`
- **System ID:** 0x42 (66)
- **Component ID:** 2
- **Function:** Monitors GPIO 0/1/2 states, sends autopilot HEARTBEAT, sends IGNITION commands

### Communication Protocol
- **UART:** 9600 baud between ESP32 GPIO16/17 and STM32G0 UART2
- **Messages:** Mavlink v2.0 HEARTBEAT (1Hz), COMMAND_LONG (IGNITION), COMMAND_ACK
- **Timing:** Timer-based periodic transmission, GPIO event-triggered commands

### 4. Implement Mavlink Event Handler
Add to app.c:
```c
static void App_MavlinkEventHandler(mavlink_evt_t event_type, uint8_t event_data) {
    switch (event_type) {
        case MAVLINK_EVT_AUTOPILOT_CONNECTED:
            // Autopilot connection established
            break;
            
        case MAVLINK_EVT_AUTOPILOT_DISCONNECTED:
            // Autopilot connection lost - handle timeout
            break;
            
        case MAVLINK_EVT_COMMAND_IGNITION:
            // Handle IGNITION command from autopilot
            break;
            
        case MAVLINK_EVT_AUTOPILOT_HEARTBEAT:
            // Autopilot HEARTBEAT received - update connection status
            break;
            
        case MAVLINK_EVT_AUTOPILOT_ARMED:
            // Autopilot armed state detected
            break;
            
        case MAVLINK_EVT_AUTOPILOT_DISARMED:
            // Autopilot disarmed state detected
            break;
            
        case MAVLINK_EVT_AUTOPILOT_PREARM_ENABLED:
            // Autopilot PREARM enabled
            break;
            
        case MAVLINK_EVT_AUTOPILOT_PREARM_DISABLED:
            // Autopilot PREARM disabled
            break;
            
        default:
            // Unknown event
            break;
    }
}

### 5. Initialize Mavlink Module
Add to your initialization code:
```c
void App_Init(void) {
    // ... other initialization
    
    // Initialize Mavlink with event handler
    Mavlink_Init(App_MavlinkEventHandler);
    
    // ... continue initialization
}
```

### 6. Integrate Battery Level Updates
Add to ADC processing in app.c:
```c
void App_AdcDataAnalyze(void) {
    // ... existing ADC processing
    
    // Calculate and update battery level
    uint16_t battery_voltage_mv = /* get battery voltage from ADC */;
    battery_level = App_CalculateBatteryLevel(battery_voltage_mv);
    
    // ... continue ADC processing
}
```

### 7. Custom Mode Bitfield Structure
The Mavlink module automatically packs the following data into the custom_mode field:
- **Bits 0-7:** Switch 1 ADC value (0-255)
- **Bits 8-15:** Switch 2 ADC value (0-255)  
- **Bits 16-19:** Switch 3 ADC value (0-15)
- **Bits 20-23:** Battery level (0-10, representing 0%-100% in 10% increments)
- **Bits 24-31:** Reserved for future use

Data is automatically retrieved via getter functions and transmitted in HEARTBEAT messages at 1Hz.
            // Handle DISARM command
            handle_disarm_command();
            break;
            
        case MAVLINK_EVT_COMMAND_IGNITION:
            // Handle IGNITION command
            handle_ignition_command();
            break;
            
        case MAVLINK_EVT_COMMAND_PREARM_ENABLED:
            // Handle PREARM ENABLED command
            handle_prearm_enabled();
            break;
            
        case MAVLINK_EVT_COMMAND_PREARM_DISABLED:
            // Handle PREARM DISABLED command
            handle_prearm_disabled();
            break;
            
        case MAVLINK_EVT_VFR_HUD_RECEIVED:
            // Handle VFR_HUD data received
            handle_vfr_hud_data();
            break;
            
        case MAVLINK_EVT_GCS_CONNECTED:
            // Handle GCS connection
            on_gcs_connected();
            break;
            
        case MAVLINK_EVT_GCS_DISCONNECTED:
            // Handle GCS disconnection
            on_gcs_disconnected();
            break;
    }
}
```

### 4. Initialize Mavlink
```c
void app_init(void) {
    // Initialize system state with default values
    system_state.timer_seconds = 0;
    system_state.timer_mode = 0;
    system_state.fuse_present = 0;
    system_state.board_state = 0;
    system_state.battery_level = 0;
    system_state.error_code = 0;
    
    // Initialize Mavlink with callback and system state pointer
    Mavlink_Init(system_event_callback, &system_state);
}
```

### 5. Update System State Regularly
```c
void update_system_state(void) {
    // Update system state structure whenever values change
    system_state.timer_seconds = get_current_timer_seconds();
    system_state.timer_mode = get_current_timer_mode();
    system_state.fuse_present = is_fuse_present();
    system_state.board_state = get_board_state();
    system_state.battery_level = get_battery_level();
    system_state.error_code = get_error_code();
}

// Call this function:
// - When system state changes
// - Periodically in main loop
// - When SYSTEM_EVT_INIT_DONE callback is triggered
```

### 6. Main Loop Integration
```c
int main(void) {
    // System initialization
    HAL_Init();
    SystemClock_Config();
    MX_UART2_Init();
    
    // Application initialization
    app_init();
    
    // Main loop
    while (1) {
        // Process Mavlink messages
        Mavlink_Process();
        
        // Update system state periodically
        update_system_state();
        
        // Other application tasks
        app_process();
        
        // Small delay
        HAL_Delay(10);
    }
}
```

### 7. UART Integration
```c
// UART RX Interrupt Handler
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        // Process received byte
        Mavlink_UartRxByte(uart_rx_byte);
        
        // Restart UART reception
        HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1);
    }
}

// UART TX Complete Callback
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        // Notify Mavlink that transmission is complete
        Mavlink_UartTxComplete();
    }
}
```

## Benefits of New Approach

### Performance
- **Faster**: No function calls during HEARTBEAT generation
- **Efficient**: Single structure read vs 6 function calls
- **Predictable**: No function call overhead

### Code Quality
- **Cleaner**: Single structure vs multiple functions
- **Maintainable**: Easy to add new state fields
- **Flexible**: Application controls when to update state

### Memory Usage
- **Compact**: Structure uses only needed bytes
- **Cache-friendly**: All data in single memory location
- **No overhead**: Direct memory access

## System State Structure
```c
typedef struct {
    uint16_t timer_seconds;      // Timer seconds (0-16383)
    uint8_t timer_mode;          // Timer mode (0-3)
    uint8_t fuse_present;        // Fuse present (0-1)
    uint8_t board_state;         // Board state (0-7)
    uint8_t battery_level;       // Battery level (0-10)
    uint8_t error_code;          // Error code (0-15)
} mavlink_system_state_t;
```

## Example Implementation
```c
// Example of how to implement the system state functions
static mavlink_system_state_t system_state = {0};

uint16_t get_current_timer_seconds(void) {
    // Return current timer value from your timer implementation
    return timer_get_seconds();
}

uint8_t get_current_timer_mode(void) {
    // Return current timer mode
    return app_get_current_mode();
}

uint8_t is_fuse_present(void) {
    // Check fuse presence
    return gpio_read_fuse_pin();
}

uint8_t get_board_state(void) {
    // Return current board state
    return app_get_state();
}

uint8_t get_battery_level(void) {
    // Return battery level (0-10)
    return adc_get_battery_level();
}

uint8_t get_error_code(void) {
    // Return current error code
    return app_get_last_error();
}
```

This updated approach provides better performance, cleaner code, and more flexibility while maintaining all the functionality of the original design.
