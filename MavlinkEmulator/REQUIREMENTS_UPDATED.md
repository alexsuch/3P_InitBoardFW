# Software Requirements Specification (SRS) - Updated
## Mavlink InitBoard System

### 1. SYSTEM OVERVIEW
**Objective:** Create a Mavlink communication system between STM32G0 InitBoard and ESP32 Autopilot Emulator
**Components:** 
- STM32G0 InitBoard (main controller with custom mode bitfield transmission)
- ESP32 DevKit V1 (Autopilot emulator with ARM/PREARM state monitoring)

### 2. HARDWARE ARCHITECTURE
- **STM32G0 ↔ ESP32:** UART connection (9600 baud)
- **ESP32 UART2:** GPIO 16 (RX2), GPIO 17 (TX2) - communication with STM32G0
- **ESP32 Autopilot Emulator:** UART-only communication (NO WiFi)
- **ESP32 State Monitoring:**
  - **GPIO 4:** ARM state input (pull-up enabled) - reads ARM/DISARM status
  - **GPIO 15:** PREARM state input (pull-up enabled) - reads PREARM status
- **ESP32 Debug Output:** UART0 (default Serial) for terminal debugging
- **Power:** 5V external supply

### 2.1 CONFIGURATION MANAGEMENT
- **System IDs:** 0x42 (66) for InitBoard, synchronized between STM32G0 and ESP32
- **Component IDs:** 0x42 (66) for InitBoard components
- **Message IDs:** Standard Mavlink v2.0 (HEARTBEAT: 0x00, COMMAND_LONG: 0x4C, etc.)

### 3. FUNCTIONAL REQUIREMENTS

#### 3.1 InitBoard (STM32G0) SHALL:
- **FR-01:** Send HEARTBEAT message every 1Hz via Timer-based callback
  - System ID: 0x42 (66) - MAVLINK_SYSTEM_ID_INITBOARD
  - Component ID: 0x42 (66) - MAVLINK_COMP_ID_USER1
  - Custom mode: 32-bit bitfield for system status
- **FR-02:** Receive and process incoming Mavlink messages via Mavlink_UartRxByte()
- **FR-03:** Process COMMAND_LONG (MAV_CMD_USER_1) commands with custom command types
- **FR-04:** Send COMMAND_ACK responses for received commands
- **FR-05:** Encode system state in custom mode bitfield (timer, fuse, battery, error status)
- **FR-06:** Support integer-only operations (no floating point)
- **FR-07:** Implement GCS connection timeout detection (3 seconds)
- **FR-08:** Provide callback interface for upper layer event notification

#### 3.1.1 STM32G0 Mavlink UART Module (mavlink_uart.c/.h) SHALL:
- **FR-01a:** Implement Mavlink_UartRxByte(uint8_t byte) for byte-by-byte reception
- **FR-01b:** Implement Mavlink_Init(app_cbk_fn system_cbk, init_board_system_info_t* system_info)
- **FR-01c:** Support callback function typedef: app_cbk_fn for system events
- **FR-01d:** Parse incoming Mavlink v2.0 packets with proper header validation
- **FR-01e:** Generate HEARTBEAT packets with custom mode bitfield encoding
- **FR-01f:** Implement timer-based periodic HEARTBEAT transmission (1Hz)
- **FR-01g:** Parse COMMAND_LONG messages and extract custom command types
- **FR-01h:** Generate COMMAND_ACK responses with proper result codes
- **FR-01i:** Support VFR_HUD message processing with float-to-integer conversion
- **FR-01j:** Validate message CRC and provide synchronization
- **FR-01k:** Use UartSendData() for packet transmission

#### 3.2 ESP32 Autopilot Emulator SHALL:
- **FR-06:** Implement configurable System ID (0x42) and Component ID (0x42)
- **FR-07:** Send periodic HEARTBEAT messages (1Hz) as autopilot to InitBoard
- **FR-08:** Monitor ARM state via GPIO4 and encode in autopilot HEARTBEAT custom mode
- **FR-09:** Monitor PREARM state via GPIO15 and encode in autopilot HEARTBEAT custom mode
- **FR-10:** Receive and decode InitBoard HEARTBEAT messages with custom mode bitfield
- **FR-11:** Parse and display InitBoard status (timer, fuse, battery, error) in human-readable format
- **FR-12:** Use UART2 (GPIO 16/17) at 9600 baud for STM32G0 communication
- **FR-13:** Implement 300ms debounce for GPIO state changes
- **FR-14:** Output debug messages via UART0 (Serial) for all Mavlink events
- **FR-15:** Decode received packets and show detailed parsing information
- **FR-16:** Implement proper Mavlink v2.0 packet generation and parsing
- **FR-17:** Provide error descriptions for InitBoard error codes

### 4. TECHNICAL PARAMETERS
- **Mavlink version:** v2.0
- **System ID InitBoard:** 0x42 (66) - MAVLINK_SYSTEM_ID_INITBOARD
- **Component ID InitBoard:** 0x42 (66) - MAVLINK_COMP_ID_INITBOARD
- **System ID Autopilot:** 0x42 (66) - MAVLINK_SYSTEM_ID_INITBOARD (autopilot speaking TO InitBoard)
- **Component ID Autopilot:** 0x42 (66) - MAVLINK_COMP_ID_INITBOARD
- **STM32 UART Speed:** 9600 baud
- **ESP32 UART2:** GPIO 16 (RX2), GPIO 17 (TX2) at 9600 baud
- **ESP32 Debug UART:** UART0 (Serial) for terminal output at 115200 baud
- **ESP32 Input Debounce:** 300ms for GPIO state stability
- **Message Rates:** 1Hz for HEARTBEAT messages
- **GCS Timeout Period:** 3000ms (3 seconds) - MAVLINK_CONNECTION_TIMEOUT_MS
- **InitBoard Heartbeat Interval:** 1000ms (1Hz) - MAVLINK_INITBOARD_HEARTBEAT_INTERVAL_MS

#### 4.1 Timer Configuration
```c
// Mavlink timer IDs
#define MAVLINK_INITBOARD_HEARTBEAT_TMR        // Timer for 1Hz InitBoard heartbeat
#define MAVLINK_GCS_CONNECTION_TIMEOUT_TMR     // Timer for GCS connection timeout

// Mavlink timing constants
#define MAVLINK_INITBOARD_HEARTBEAT_INTERVAL_MS  1000  // 1Hz InitBoard heartbeat
#define MAVLINK_CONNECTION_TIMEOUT_MS            3000  // 3 second GCS timeout
```

### 5. PROTOCOL DETAILS

#### 5.1 InitBoard HEARTBEAT Message
- **Message:** HEARTBEAT (ID: 0x00)
- **Rate:** 1Hz (every 1000ms)
- **Fields:**
  - `type`: MAV_TYPE_GENERIC (0)
  - `autopilot`: MAV_AUTOPILOT_INVALID (8)
  - `base_mode`: 0
  - `custom_mode`: 32-bit bitfield (see below)
  - `system_status`: MAV_STATE_ACTIVE (4)
  - `mavlink_version`: 3

#### 5.2 Custom Mode Bitfield Structure (32-bit)
```
Bit Position | Field Name     | Bits | Range/Values          | Description
-------------|----------------|------|----------------------|---------------------------
0-13         | timer_sec      | 14   | 0-16383              | Timer seconds value
14-15        | timer_mode     | 2    | 0-3                  | Timer mode (NONE/SAFE/DESTROY)
16           | fuse_present   | 1    | 0-1                  | Fuse present status (0=No, 1=Yes)
17-19        | board_state    | 3    | 0-7                  | Board state (NO_FC/DISARMED/ARMED/BOOM)
20-23        | battery_level  | 4    | 0-10                 | Battery level (0-10, representing 0%-100%)
24-27        | error_code     | 4    | 0-15                 | Error code (0-7 defined)
28-31        | reserved       | 4    | 0                    | Reserved for future use
```

#### 5.3 ESP32 Autopilot HEARTBEAT Message
- **Message:** HEARTBEAT (ID: 0x00)
- **Rate:** 1Hz (every 1000ms)
- **Fields:**
  - `type`: MAV_TYPE_QUADROTOR (2)
  - `autopilot`: MAV_AUTOPILOT_GENERIC (0)
  - `base_mode`: MAV_MODE_FLAG_CUSTOM_MODE_ENABLED (1)
  - `custom_mode`: ARM/PREARM states encoded
  - `system_status`: MAV_STATE_ACTIVE (4)
  - `mavlink_version`: 3

#### 5.4 Command Types (Custom Commands)
- **DISARM (1):** Disarm system
- **ARM (2):** Arm system  
- **IGNITION (3):** Trigger ignition
- **PREARM_ENABLED (4):** Enable prearm
- **PREARM_DISABLED (5):** Disable prearm

#### 5.5 Error Code Descriptions
```c
0: "No Error"
1: "Strong Alarm"
2: "Battery Low"
3: "Fuse Incorrect"
4: "Unexpected Ignition"
5: "Unexpected Arm"
6: "Unexpected Mining"
7: "VUSA Shorted"
8+: "Unknown Error"
```

### 6. MESSAGE FLOW
1. **InitBoard → Autopilot:** HEARTBEAT with custom mode bitfield (1Hz)
2. **Autopilot → InitBoard:** HEARTBEAT with ARM/PREARM states (1Hz)
3. **Autopilot → InitBoard:** COMMAND_LONG (on GPIO state changes)
4. **InitBoard → Autopilot:** COMMAND_ACK (response to commands)

### 7. IMPLEMENTATION STATUS
- ✅ STM32G0 Mavlink UART module with custom mode bitfield
- ✅ ESP32 Autopilot emulator with GPIO monitoring
- ✅ Timer-based periodic message transmission
- ✅ Command processing and acknowledgment
- ✅ System ID synchronization (0x42)
- ✅ Custom mode bitfield encoding/decoding
- ✅ Debug output and packet parsing
- ✅ Integer-only operations (no floating point)
