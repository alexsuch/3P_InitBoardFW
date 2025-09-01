# STM32G0 InitBoard ↔ ESP32 Autopilot - Complete Documentation

## 📋 PROJECT OVERVIEW

**Objective:** Mavlink-based communication system between STM32G0 InitBoard and ESP32 Autopilot Emulator  
**Protocol:** MAVLink v2.0 with custom mode bitfields for comprehensive system state transmission  
**Architecture:** InitBoard ↔ Autopilot bidirectional communication (replacing previous GCS communication)

## 🏗️ SYSTEM ARCHITECTURE

### Components
1. **STM32G0 InitBoard** - Primary device that sends system status and receives commands
2. **ESP32 Autopilot Emulator** - Monitors GPIO states and sends commands to InitBoard

### Communication Flow
```
STM32G0 InitBoard ←→ ESP32 Autopilot Emulator
    (System Status)     (ARM/PREARM/IGNITION States)
    Custom Mode         GPIO Monitoring
    HEARTBEAT 750ms     COMMAND_LONG
```

## ⚡ HARDWARE CONFIGURATION

### STM32G0 InitBoard
- **System ID:** 0x42 (66)
- **Component ID:** 0x42 (66)
- **UART:** 9600 baud, 8N1
- **Function:** Timer management, ignition control, battery monitoring, fuse detection

### ESP32 Autopilot Emulator
- **System ID:** 0x42 (66)
- **Component ID:** 0x01 (1)
- **UART2:** GPIO 16 (RX), GPIO 17 (TX) - 9600 baud
- **GPIO Inputs (with pull-ups):**
  - **GPIO 4:** ARM state (LOW=ARMED, HIGH=DISARMED)
  - **GPIO 15:** PREARM state (LOW=ENABLED, HIGH=DISABLED)
  - **GPIO 0:** IGNITION trigger (LOW=IGNITION, HIGH=OFF)

### Physical Connections
```
STM32G0 UART ↔ ESP32 UART2 (GPIO 16/17)
STM32G0 GND  ↔ ESP32 GND
```

## 📡 MAVLINK PROTOCOL SPECIFICATION

### Message Types
1. **HEARTBEAT** - System status exchange (750ms interval)
2. **COMMAND_LONG** - Commands from autopilot to InitBoard

### Custom Mode Bitfields

#### InitBoard HEARTBEAT (32-bit custom_mode)
```
Bit Field Layout:
Bits 0-13:  timer_seconds (0-16383) - Current timer value
Bits 14-15: timer_mode (0=None, 1=Safe, 2=SelfDestroy) - Timer state
Bit 16:     fc_control_present (0=disconnected, 1=connected) - Autopilot connection
Bit 17:     fuse_present (0=removed, 1=present) - Fuse detection
Bits 18-20: board_state (0-7) - System state
  - 0: BOARD_STATE_INIT (No data)
  - 1: BOARD_STATE_DISARMED 
  - 2: BOARD_STATE_CHARGING
  - 3: BOARD_STATE_ARMED
  - 4: BOARD_STATE_BOOM
Bits 21-24: battery_level (0-10) - Battery percentage (0-100% in 10% increments)
Bits 25-28: error_code (0-15) - Error status
Bit 29:     is_ignition_done (0=not done, 1=completed) - Ignition completion flag
Bits 30-31: reserved - Future expansion
```

#### Autopilot HEARTBEAT (32-bit custom_mode)
```
Bit Field Layout:
Bits 0-3:   arm_state (0=DISARMED, 1=ARMED) - ARM state from GPIO4
Bits 4-7:   prearm_state (0=DISABLED, 1=ENABLED) - PREARM state from GPIO15
Bits 8-31:  reserved - Future expansion
```

### Command Protocol
**IGNITION Command (COMMAND_LONG):**
- **Command ID:** 1 (MAVLINK_CMD_IGNITION)
- **Trigger:** When ESP32 GPIO0 changes from HIGH to LOW
- **Response:** STM32G0 sends COMMAND_ACK

## 💻 IMPLEMENTATION DETAILS

### STM32G0 Files
- **`mavlink_uart.h`** - Header with protocol definitions, structures, constants
- **`mavlink_uart.c`** - Implementation with UART communication, message processing
- **Key Functions:**
  - `Mavlink_Init()` - Initialize communication
  - `Mavlink_SendInitBoardHeartbeat()` - Send system status
  - `Mavlink_ProcessCommandLong()` - Handle incoming commands
  - `Mavlink_EncodeCustomMode()` - Encode system state bitfield

### ESP32 Files  
- **`main.cpp`** - Complete autopilot emulator with GPIO monitoring
- **Key Functions:**
  - `send_autopilot_heartbeat()` - Send ARM/PREARM states
  - `send_ignition_command()` - Send IGNITION command
  - `process_input_states()` - Monitor GPIO changes
  - `debug_mavlink_rx()` - Parse and display received messages

### Configuration
- **HEARTBEAT Interval:** 750ms (configurable in prj_config.h)
- **Connection Timeout:** 3000ms
- **Debug Output:** Minimal (raw packet dumps commented out)

## 🔧 COMPILATION & SETUP

### STM32G0 Build
```bash
cd 3P_FPV_2_0
cmake -G Ninja -DCMAKE_BUILD_TYPE=Debug -S . -B build/Debug
cmake --build build/Debug
# Output: build/Debug/3P_FPV_2_0.hex
```

### ESP32 Build
```bash
cd MavlinkEmulator
pio run -t upload -t monitor
```

### Configuration Files
- **STM32G0:** `prj_config.h` - MAVLINK_INITBOARD_HEARTBEAT_INTERVAL_MS = 750u
- **ESP32:** `main.cpp` - GPIO pin definitions, system IDs

## 🧪 TESTING PROTOCOL

### 1. Basic Communication Test
1. Power both devices
2. Check serial output - should see HEARTBEAT exchange
3. Verify System ID match (0x42)

### 2. GPIO State Testing
```bash
# ARM Test
Connect GPIO4 to GND → ARM=ARMED in autopilot HEARTBEAT

# PREARM Test  
Connect GPIO15 to GND → PREARM=ENABLED in autopilot HEARTBEAT

# IGNITION Test
Connect GPIO0 to GND → sends IGNITION COMMAND_LONG to InitBoard
```

### 3. System State Monitoring
Monitor InitBoard HEARTBEAT custom_mode for:
- Timer countdown
- Fuse presence
- Battery level
- Error conditions
- Ignition completion

## 🐛 TROUBLESHOOTING

### No HEARTBEAT Exchange
- Check UART connections (TX ↔ RX, GND ↔ GND)
- Verify baud rate (9600)
- Confirm System ID match (0x42)

### GPIO Not Responding
- Ensure pull-up resistors enabled
- Check GPIO pin definitions
- Test with multimeter (should see HIGH→LOW transition)

### Compilation Errors
- Verify all includes present
- Check timer.h definitions
- Ensure solution_wrapper.h available

### Connection Issues
- Monitor fc_control_present bit in InitBoard HEARTBEAT
- Check connection timeout settings
- Verify MAVLink packet format

## 📊 SYSTEM STATE EXAMPLES

### Normal Operation
```
InitBoard HEARTBEAT:
- timer_seconds: 120 (safe timer countdown)
- timer_mode: 1 (Safe timer active)
- fc_control_present: 1 (Autopilot connected)
- fuse_present: 1 (Fuse detected)
- board_state: 1 (DISARMED)
- battery_level: 8 (80% battery)
- error_code: 0 (No errors)
- is_ignition_done: 0 (Not triggered)

Autopilot HEARTBEAT:
- arm_state: 0 (DISARMED - GPIO4 HIGH)
- prearm_state: 0 (DISABLED - GPIO15 HIGH)
```

### Armed State
```
InitBoard HEARTBEAT:
- timer_seconds: 30 (self-destroy countdown)  
- timer_mode: 2 (Self destroy timer)
- board_state: 3 (ARMED)
- fc_control_present: 1 (Connected)

Autopilot HEARTBEAT:
- arm_state: 1 (ARMED - GPIO4 LOW)
- prearm_state: 1 (ENABLED - GPIO15 LOW)
```

## 🔄 VERSION HISTORY

**Current Implementation:**
- ✅ Custom mode bitfields implemented
- ✅ Timer integration completed  
- ✅ GPIO monitoring functional
- ✅ IGNITION command processing
- ✅ Connection monitoring
- ✅ Debug output optimized
- ✅ 750ms HEARTBEAT interval
- ✅ fc_control_present field added
- ✅ is_ignition_done field added

**Future Enhancements:**
- Additional command types
- Extended error reporting
- Configuration persistence
- Wireless communication options

---

*This document consolidates all requirements, implementation details, and testing procedures for the STM32G0 InitBoard ↔ ESP32 Autopilot Mavlink communication system.*
