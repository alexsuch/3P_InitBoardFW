# Software Requirements Specification (SRS) - Final Update
## Mavlink InitBoard ↔ Autopilot Communication System

### 1. SYSTEM OVERVIEW
**Objective:** Create a Mavlink communication system between STM32G0 InitBoard and Autopilot (ESP32 emulator)
**Role Change:** STM32G0 InitBoard now communicates with Autopilot instead of GCS
**Components:** 
- STM32G0 InitBoard (receives commands from autopilot, sends status via custom mode bitfield)
- ESP32 DevKit V1 (Autopilot emulator with GPIO state monitoring for ARM/PREARM/IGNITION)

### 2. HARDWARE ARCHITECTURE
- **STM32G0 ↔ ESP32:** UART connection (9600 baud, Mavlink v2.0)
- **ESP32 UART2:** GPIO 16 (RX2), GPIO 17 (TX2) - communication with STM32G0
- **ESP32 Autopilot Role:** Reads physical GPIO states and transmits them via Mavlink
- **ESP32 State Monitoring:**
  - **GPIO 4:** ARM state input (pull-up enabled) - Physical ARM/DISARM switch
  - **GPIO 15:** PREARM state input (pull-up enabled) - Physical PREARM switch  
  - **GPIO 0:** IGNITION state input (pull-up enabled) - Physical IGNITION switch
- **ESP32 Debug Output:** UART0 (default Serial) for terminal debugging
- **Power:** 5V external supply

### 2.1 CONFIGURATION MANAGEMENT
- **System IDs:** 0x42 (66) for InitBoard, synchronized between STM32G0 and ESP32
- **Component IDs:** 0x42 (66) for InitBoard components
- **Message IDs:** Standard Mavlink v2.0 (HEARTBEAT: 0x00, COMMAND_LONG: 0x4C, etc.)
- **Communication Direction:** Bidirectional InitBoard ↔ Autopilot

### 3. FUNCTIONAL REQUIREMENTS

#### 3.1 InitBoard (STM32G0) SHALL:
- **FR-01:** Send HEARTBEAT message every 1Hz via Timer-based callback
  - System ID: 0x42 (66) - MAVLINK_SYSTEM_ID_INITBOARD
  - Component ID: 0x42 (66) - MAVLINK_COMP_ID_INITBOARD
  - Custom mode: 32-bit bitfield for system status
- **FR-02:** Receive and process HEARTBEAT messages from Autopilot with ARM/PREARM states
- **FR-03:** Process COMMAND_LONG (IGNITION commands) from Autopilot
- **FR-04:** Send COMMAND_ACK responses for received commands
- **FR-05:** Encode system state in custom mode bitfield (timer, fuse, battery, error status)
- **FR-06:** Support integer-only operations (no floating point)
- **FR-07:** Implement Autopilot connection timeout detection (5 seconds)
- **FR-08:** Provide ARM/PREARM state access functions for application layer
- **FR-09:** Monitor Autopilot connection status and notify application

#### 3.1.1 STM32G0 Mavlink UART Module (mavlink_uart.c/.h) SHALL:
- **FR-01a:** Implement Mavlink_UartRxByte(uint8_t byte) for byte-by-byte reception
- **FR-01b:** Implement Mavlink_Init(app_cbk_fn system_cbk, init_board_system_info_t* system_info)
- **FR-01c:** Support callback function typedef: app_cbk_fn for autopilot events
- **FR-01d:** Parse incoming Mavlink v2.0 packets with proper header validation
- **FR-01e:** Generate InitBoard HEARTBEAT packets with custom mode bitfield encoding
- **FR-01f:** Implement timer-based periodic HEARTBEAT transmission (1Hz)
- **FR-01g:** Parse Autopilot HEARTBEAT messages and extract ARM/PREARM states
- **FR-01h:** Parse COMMAND_LONG messages and extract IGNITION commands
- **FR-01i:** Generate COMMAND_ACK responses with proper result codes
- **FR-01j:** Provide Mavlink_GetAutopilotArmState() and Mavlink_GetAutopilotPrearmState()
- **FR-01k:** Support autopilot connection timeout with callback notification
- **FR-01l:** Use UartSendData() for packet transmission

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
#### 3.2 Autopilot (ESP32) SHALL:
- **FR-10:** Send HEARTBEAT message every 1Hz with ARM/PREARM states in custom mode
  - System ID: 0x42 (66) - same as InitBoard for communication
  - Component ID: 0x42 (66) - same as InitBoard for communication
  - Custom mode: ARM/PREARM states from GPIO readings
- **FR-11:** Read GPIO states with debouncing (300ms)
  - GPIO 4: ARM state (0=DISARMED, 1=ARMED)
  - GPIO 15: PREARM state (0=ENABLED, 1=DISABLED with pull-up)
  - GPIO 0: IGNITION state (0=ON, 1=OFF with pull-up)
- **FR-12:** Send COMMAND_LONG (IGNITION) when IGNITION GPIO state changes to ON
- **FR-13:** Process incoming InitBoard HEARTBEAT messages for monitoring
- **FR-14:** Provide debug output via Serial (115200 baud)
- **FR-15:** Implement connection timeout detection
- **FR-16:** Support custom mode encoding for ARM/PREARM states
- **FR-17:** Provide error descriptions for InitBoard error codes

### 4. TECHNICAL PARAMETERS
- **Mavlink version:** v2.0
- **System ID InitBoard:** 0x42 (66) - MAVLINK_SYSTEM_ID_INITBOARD
- **Component ID InitBoard:** 0x42 (66) - MAVLINK_COMP_ID_INITBOARD
- **System ID Autopilot:** 0x42 (66) - MAVLINK_SYSTEM_ID_INITBOARD (autopilot communicates with InitBoard)
- **Component ID Autopilot:** 0x42 (66) - MAVLINK_COMP_ID_INITBOARD
- **STM32 UART Speed:** 9600 baud
- **ESP32 UART2:** GPIO 16 (RX2), GPIO 17 (TX2) at 9600 baud
- **ESP32 Debug UART:** UART0 (Serial) for terminal output at 115200 baud
- **ESP32 Input Debounce:** 300ms for GPIO state stability
- **Message Rates:** 1Hz for HEARTBEAT messages (both directions)
- **Autopilot Timeout Period:** 5000ms (5 seconds) - MAVLINK_CONNECTION_TIMEOUT_MS
- **InitBoard Heartbeat Interval:** 1000ms (1Hz) - MAVLINK_INITBOARD_HEARTBEAT_INTERVAL_MS

#### 4.1 Timer Configuration
```c
// Mavlink timer IDs
#define MAVLINK_INITBOARD_HEARTBEAT_TMR        // Timer for 1Hz InitBoard heartbeat
#define MAVLINK_AUTOPILOT_CONNECTION_TIMEOUT_TMR // Timer for autopilot connection timeout

// Mavlink timing constants
#define MAVLINK_INITBOARD_HEARTBEAT_INTERVAL_MS  1000  // 1Hz InitBoard heartbeat
#define MAVLINK_CONNECTION_TIMEOUT_MS            5000  // 5 second autopilot timeout
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

#### 5.2 Custom Mode Bitfield Structure (32-bit) - InitBoard Status
```
Bit Position | Field Name     | Bits | Range/Values          | Description
-------------|----------------|------|----------------------|---------------------------
0-13         | timer_sec      | 14   | 0-16383              | Timer seconds value
14-15        | timer_mode     | 2    | 0-2                  | Timer mode (NONE=0/SAFE=1/DESTROY=2)
16           | fuse_present   | 1    | 0-1                  | Fuse present status (0=No, 1=Yes)
17-19        | board_state    | 3    | 0-3                  | Board state (NO_FC=0/DISARMED=1/ARMED=2/BOOM=3)
20-23        | battery_level  | 4    | 0-10                 | Battery level (0-10, representing 0%-100%)
24-27        | error_code     | 4    | 0-15                 | Error code (0-7 defined)
28-31        | reserved       | 4    | 0                    | Reserved for future use
```

#### 5.3 Autopilot Custom Mode Structure (32-bit) - ARM/PREARM States
```
Bit Position | Field Name     | Bits | Range/Values          | Description
-------------|----------------|------|----------------------|---------------------------
0-3          | arm_state      | 4    | 0-1                  | ARM state (DISARMED=0/ARMED=1)
4-7          | prearm_state   | 4    | 0-1                  | PREARM state (DISABLED=0/ENABLED=1)
8-15         | reserved1      | 8    | 0                    | Reserved for future states
16-23        | reserved2      | 8    | 0                    | Reserved for future use
24-31        | reserved3      | 8    | 0                    | Reserved for future use
```

#### 5.4 ESP32 Autopilot HEARTBEAT Message
- **Message:** HEARTBEAT (ID: 0x00)
- **Rate:** 1Hz (every 1000ms)
- **Purpose:** Send ARM/PREARM states from GPIO to InitBoard
- **Fields:**
  - `type`: MAV_TYPE_QUADROTOR (2)
  - `autopilot`: MAV_AUTOPILOT_GENERIC (0)
  - `base_mode`: MAV_MODE_FLAG_CUSTOM_MODE_ENABLED (1)
  - `custom_mode`: ARM/PREARM states encoded
  - `system_status`: MAV_STATE_ACTIVE (4)
  - `mavlink_version`: 3

#### 5.5 Command Types
- **IGNITION (1):** Only supported command - trigger ignition when GPIO0 goes LOW

#### 5.6 Autopilot Events (STM32G0)
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

#### 5.7 Error Code Descriptions
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

#### 6.1 InitBoard → Autopilot (Heartbeat)
```
Message: HEARTBEAT (ID: 0)
Frequency: 1 Hz
Source: InitBoard (sys_id: 0x42, comp_id: 1)
Target: Autopilot (sys_id: 0x42, comp_id: 2)

Custom Mode Bitfield:
├── [0] IGNITION_PIN_STATE (0 = HIGH, 1 = LOW)
├── [1] MINING_STATUS_BIT  (0 = LOW, 1 = HIGH) 
├── [2] ARM_STATUS_BIT     (0 = LOW, 1 = HIGH)
├── [3] IS_ARMED_BIT       (0 = DISARMED, 1 = ARMED)
├── [4] NEED_IGNITION_BIT  (0 = NO, 1 = YES)
├── [5-7] ERROR_CODE_LOW   (Error code bits 0-2)
└── [8-10] ERROR_CODE_HIGH (Error code bits 3-5)
```

#### 6.2 Autopilot → InitBoard (Heartbeat)
```
Message: HEARTBEAT (ID: 0)
Frequency: 1 Hz
Source: Autopilot (sys_id: 0x42, comp_id: 2)
Target: InitBoard (sys_id: 0x42, comp_id: 1)

Custom Mode Bitfield:
├── [0] ARM_GPIO_STATE     (GPIO1: 0 = HIGH, 1 = LOW)
├── [1] PREARM_GPIO_STATE  (GPIO2: 0 = HIGH, 1 = LOW)
├── [2] IGNITION_GPIO_STATE (GPIO0: 0 = HIGH, 1 = LOW)
└── [3-31] Reserved
```

#### 6.3 Autopilot → InitBoard (Commands)
```
Message: COMMAND_LONG (ID: 76)
Source: Autopilot (sys_id: 0x42, comp_id: 2)
Target: InitBoard (sys_id: 0x42, comp_id: 1)

Commands:
- IGNITION (command = 1): Trigger ignition when GPIO0 = LOW
```

#### 6.4 Message Flow Sequence
1. **InitBoard → Autopilot:** HEARTBEAT with status bitfield (1Hz)
2. **Autopilot → InitBoard:** HEARTBEAT with GPIO states (1Hz)
3. **Autopilot → InitBoard:** COMMAND_LONG (on GPIO0 LOW detection)
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
