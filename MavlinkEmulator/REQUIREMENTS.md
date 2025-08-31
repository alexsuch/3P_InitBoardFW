# Software Requirements Specification (SRS)
## Mavlink InitBoard System

### 1. SYSTEM OVERVIEW
**Objective:** Create a Mavlink node that can integrate into UAV communication network
**Components:** 
- STM32G0 InitBoard (main controller with custom mode bitfield transmission)
- ESP32 DevKit V1 (Autopilot emulator with ARM/PREARM state monitoring)

### 2. HARDWARE ARCHITECTURE
- **STM32G0 ↔ ESP32:** UART connection (9600 baud)
- **ESP32 UART2:** GPIO 16 (RX2), GPIO 17 (TX2) - communication with STM32G0
- **ESP32 Ground Station Emulator:** UART-only communication (NO WiFi)
- **ESP32 Control Inputs:**
  - **GPIO 4:** 3-position switch input (pull-up enabled)
  - **GPIO 2:** 3-position switch input (pull-up enabled)
  - **GPIO 15:** PREARM button input (pull-up enabled)
- **ESP32 Debug Output:** UART0 (default Serial) for terminal debugging
- **Power:** 5V external supply

### 2.1 CONFIGURATION MANAGEMENT
- **System IDs:** Configurable via defines in config file
- **Ground Station ID:** Configurable define
- **InitBoard ID:** Configurable define

### 3. FUNCTIONAL REQUIREMENTS

#### 3.1 InitBoard (STM32G0) SHALL:
- **FR-01:** Send HEARTBEAT message every 1Hz using Mavlink_SendData()
  - System ID: Configurable define
  - Component ID: MAVLINK_COMP_ID_USER1 (25)
  - Custom mode: 4 bytes for internal status
- **FR-02:** Receive VFR_HUD messages from ground station (1Hz) via Mavlink_ByteReceived()
- **FR-03:** Receive HEARTBEAT from ground station (1Hz) via Mavlink_ByteReceived()
- **FR-04:** Process COMMAND_LONG (MAVLINK_CMD_USER_1) commands via Mavlink_ByteReceived()
- **FR-05:** Send COMMAND_ACK responses using Mavlink_SendData()
- **FR-06:** Process float parameters in COMMAND_LONG using union-based conversion (no floating point operations)
- **FR-07:** Extract 2-byte custom data from COMMAND_LONG param1 field using bit manipulation
- **FR-08:** Parse custom command types from lower byte of 2-byte custom data (DISARM, ARM, TARGET_APPROACH, IGNITION)

#### 3.1.1 STM32G0 Mavlink UART Module (mavlink_uart.c/.h) SHALL:
- **FR-01a:** Implement Mavlink_ByteReceived(uint8_t byte) for byte-by-byte reception
- **FR-01b:** Implement Mavlink_SendData(uint8_t* wrBuff, uint8_t len) for packet transmission
- **FR-01c:** Implement Mavlink_Reset(mavlink_app_cbk_fn system_cbk) for initialization and callback setup
- **FR-01d:** Support callback function typedef: void (*mavlink_app_cbk_fn)(mavlink_system_evt_t evt, uint32_t usr_data)
- **FR-01e:** Parse incoming Mavlink packets and notify upper layers via callback
- **FR-01f:** Generate HEARTBEAT packets with configurable system ID
- **FR-01g:** Implement GCS connection timeout detection using Timer API
- **FR-01h:** Restart timeout timer on each valid GCS HEARTBEAT reception
- **FR-01i:** Notify upper layers when GCS connection is lost (timeout expired)
- **FR-01j:** Parse COMMAND_LONG messages and extract custom data using float-to-uint32 conversion
- **FR-01k:** Generate COMMAND_ACK responses with proper result codes
- **FR-01l:** Parse VFR_HUD messages and store data in integer-only format (no floating point operations)
- **FR-01m:** Validate VFR_HUD data freshness and provide access functions for upper layers

#### 3.2 ESP32 Ground Station Emulator SHALL:
- **FR-06:** Implement configurable System ID and Component ID via defines
- **FR-07:** Send VFR_HUD messages (1Hz) with realistic flight data via UART2 (GPIO 16/17) at 9600 baud
- **FR-08:** Send HEARTBEAT GCS messages (1Hz) via UART2 (GPIO 16/17) at 9600 baud with proper GCS identification
- **FR-09:** Send test COMMAND_LONG commands via UART2 (GPIO 16/17) at 9600 baud based on input states
- **FR-10:** Receive and display InitBoard HEARTBEAT and responses via UART2 (GPIO 16/17) at 9600 baud
- **FR-11:** Use configuration file with defines for system parameters
- **FR-12:** Implement 1-second timer for periodic HEARTBEAT transmission
- **FR-13:** Generate proper Mavlink v2.0 HEARTBEAT packet with GCS-specific fields
- **FR-14:** Use only UART2 communication - NO WiFi functionality
- **FR-15:** Monitor 3-position switch on GPIO 4/2 with pull-up configuration
- **FR-16:** Monitor PREARM button on GPIO 15 with pull-up configuration
- **FR-17:** Implement 300ms debounce for all input switches
- **FR-18:** Send commands only on state changes (not continuously)
- **FR-19:** Output debug messages via UART0 (Serial) for all Mavlink events
- **FR-20:** Decode and display human-readable command descriptions in debug output
- **FR-21:** Generate VFR_HUD messages (1Hz) with simulated flight data and dynamic parameter changes
- **FR-22:** Implement periodic message scheduling to prevent UART transmission overlap
- **FR-23:** Offset VFR_HUD transmission by 500ms from HEARTBEAT to distribute UART load

### 4. TECHNICAL PARAMETERS
- **Mavlink version:** v2.0
- **System ID InitBoard:** Configurable define (MAVLINK_SYSTEM_ID_INITBOARD)
- **System ID GCS Emulator:** Configurable define (MAVLINK_SYSTEM_ID_GCS)
- **Component ID InitBoard:** 25 (MAVLINK_COMP_ID_USER1)
- **Component ID GCS:** 190 (MAVLINK_COMP_ID_MISSIONPLANNER)
- **STM32 UART Speed:** 9600 baud
- **ESP32 UART2:** GPIO 16 (RX2), GPIO 17 (TX2) at 9600 baud
- **ESP32 Debug UART:** UART0 (Serial) for terminal output
- **ESP32 Input Debounce:** 300ms for switch stability
- **Message Rates:** 1Hz for all periodic messages
- **GCS Timeout Period:** 3000ms (3 seconds) - configurable in header file
- **InitBoard Heartbeat Interval:** 1000ms (1Hz)
- **Timer API:** Timer_Start(uint8_t timer_id, uint32_t timer_period_ms, tmr_cbk cbk)
- **Timer Callback:** typedef void (*tmr_cbk)(uint8_t tmr_id)
- **Battery Monitoring:** AAA battery voltage range 900mV-1500mV
- **Battery Level Resolution:** 10% increments (0-10 scale)
- **ADC Update Rate:** Configurable via ADC_START_MEASURE_TMR_PERIOD_MS

#### 4.1 Timer Configuration
```c
// Mavlink timer IDs (add to your timer configuration)
#define MAVLINK_INITBOARD_HEARTBEAT_TMR        // Timer for 1Hz InitBoard heartbeat
#define MAVLINK_GCS_CONNECTION_TIMEOUT_TMR     // Timer for GCS connection timeout

// Mavlink timing constants
#define MAVLINK_INITBOARD_HEARTBEAT_INTERVAL_MS  1000  // 1Hz InitBoard heartbeat
#define MAVLINK_CONNECTION_TIMEOUT_MS            3000  // 3 second GCS timeout
```
### 5. PROTOCOL DETAILS

#### 5.1 InitBoard HEARTBEAT Message
- **Message:** HEARTBEAT
- **Rate:** 1Hz (every 1000ms)
- **Fields:**
  - `type`: MAV_TYPE_QUADROTOR (2)
  - `autopilot`: MAV_AUTOPILOT_GENERIC (0) 
  - `base_mode`: MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
  - `custom_mode`: Bitfield containing switch and battery states
  - `system_status`: MAV_STATE_ACTIVE (4)
  - `mavlink_version`: 3

#### 5.2 Custom Mode Bitfield Structure
```
Bit Position | Field Name    | Bits | Range/Values          | Description
-------------|---------------|------|----------------------|---------------------------
0-7          | switch1       | 8    | 0-255                | Switch 1 ADC value/state
8-15         | switch2       | 8    | 0-255                | Switch 2 ADC value/state  
16-19        | switch3       | 4    | 0-15                 | Switch 3 ADC value/state
20-23        | battery_level | 4    | 0-10                 | Battery level (0%-100% in 10% increments)
24-31        | reserved      | 8    | 0                    | Reserved for future use
```

#### 5.3 Battery Level Calculation
- **Input Range:** 900mV - 1500mV (AAA battery voltage)
- **Output Scale:** 0-10 (representing 0%-100% in 10% increments)
- **Calculation Formula:**
  ```c
  battery_level = (adc_voltage_mv - 900) * 10 / (1500 - 900)
  battery_level = min(max(battery_level, 0), 10)
  ```
- **Integration:** Updated in ADC processing and transmitted via custom_mode field
### 6. GCS COMMAND PROCESSING

#### 6.1 Supported Commands from GCS
1. **HEARTBEAT** - GCS presence detection and connection establishment
2. **REQUEST_DATA_STREAM** - Request for periodic message transmission
3. **COMMAND_LONG** - General command execution interface
4. **MISSION_ITEM** - Mission waypoint/command handling
5. **SET_MODE** - Flight mode change requests

#### 6.2 GCS Connection Management
- **Connection Detection:** GCS considered connected when HEARTBEAT received
- **Timeout Handling:** 3-second timeout triggers connection lost state
- **Status Indication:** Connection status reflected in debug output
- **Automatic Recovery:** System automatically reconnects when GCS HEARTBEAT resumes

### 7. IMPLEMENTATION ARCHITECTURE

#### 7.1 STM32G0 InitBoard Components
- **Mavlink UART Module:** Message parsing and transmission
- **ADC Processing:** Battery and switch monitoring with configurable update rates
- **Event System:** Mavlink event handler with comprehensive switch-case processing
- **Timer Integration:** 1Hz heartbeat and connection timeout management
- **Battery Calculation:** AAA battery level mapping to 10% increments

#### 7.2 ESP32 GCS Emulator Components
- **Dual UART System:** UART2 for Mavlink, UART0 for debug output
- **Switch Input Processing:** GPIO debouncing with 300ms stability window
- **Command Generation:** Automated periodic message transmission
- **Debug Interface:** Human-readable message logging and status display
} mavlink_custom_mode_t;

// Timer mode enumeration
typedef enum {
    MAVLINK_TIMER_MODE_NONE = 0x00,         // No active timer
    MAVLINK_TIMER_MODE_SAFE = 0x01,         // Safe timer active
    MAVLINK_TIMER_MODE_DESTROY = 0x02       // Self destroy timer active
} mavlink_timer_mode_t;

// Board state enumeration
typedef enum {
    MAVLINK_BOARD_STATE_NO_FC = 0x00,       // No FC connected
    MAVLINK_BOARD_STATE_DISARMED = 0x01,    // Disarmed
    MAVLINK_BOARD_STATE_ARMED = 0x02,       // Armed
    MAVLINK_BOARD_STATE_BOOM = 0x03         // Boom
} mavlink_board_state_t;

// Error code enumeration
typedef enum {
    MAVLINK_ERR_CODE_NO_ERROR = 0x00,                   // No error
    MAVLINK_ERR_CODE_STRONG_ALARM = 0x01,               // Strong alarm
    MAVLINK_ERR_CODE_BATTERY_LOW = 0x02,                // Battery low
    MAVLINK_ERR_CODE_FUSE_INCORRECT_STATE = 0x03,       // Fuse incorrect state
    MAVLINK_ERR_CODE_UNEXPECTED_IGNITION = 0x04,        // Unexpected ignition
    MAVLINK_ERR_CODE_UNEXPECTED_ARM = 0x05,             // Unexpected arm
    MAVLINK_ERR_CODE_UNEXPECTED_MINING = 0x06,          // Unexpected mining
    MAVLINK_ERR_CODE_UNEXPECTED_VUSA_SHORTED = 0x07,    // Unexpected VUSA shorted
} mavlink_error_code_t;

// Helper macros for battery level conversion
#define MAVLINK_BATTERY_LEVEL_TO_PERCENT(level) ((level) * 10)  // Convert bitfield to percentage
#define MAVLINK_BATTERY_PERCENT_TO_LEVEL(percent) (((percent) + 5) / 10)  // Convert percentage to bitfield (with rounding)

// Battery level calculation thresholds for AAA battery
#define BATTERY_VOLTAGE_100_PERCENT_THRESHOLD_MILIVOLTS      (1500u) // 100% - 1.5V AAA battery
#define BATTERY_VOLTAGE_0_PERCENT_THRESHOLD_MILIVOLTS        (900u)  // 0% - 0.9V AAA battery

// Usage examples:
// mavlink_custom_mode_t custom_mode = {0};
// custom_mode.bitfield.timer_sec = 120;                       // 2 minutes timer
// custom_mode.bitfield.timer_mode = MAVLINK_TIMER_MODE_SAFE;
// custom_mode.bitfield.fuse_present = 1;                      // Fuse is present
// custom_mode.bitfield.board_state = MAVLINK_BOARD_STATE_ARMED;
// custom_mode.bitfield.battery_level = 0x08;                  // 80% battery (8 * 10%)
// custom_mode.bitfield.error_code = MAVLINK_ERR_CODE_NO_ERROR;  // No errors
// uint32_t custom_mode_value = custom_mode.raw;               // Use in HEARTBEAT message
```

#### 5.1 Battery Level Implementation
The STM32G0 automatically calculates battery level from AAA battery voltage:
```c
/**
 * @brief Calculate battery level in 10% increments (0-10)
 * Battery level is automatically updated in App_AdcDataAnalyze() function
 * and transmitted in HEARTBEAT custom mode bitfield
 */
static uint8_t App_CalculateBatteryLevel(uint16_t voltage_mv) {
    // AAA battery voltage range: 900mV (0%) to 1500mV (100%)
    // Returns: 0 = 0%, 1 = 10%, 2 = 20%, ..., 10 = 100%
}
```

#### 5.2 System State Structure
```c
/**
 * @brief System State Data Structure
 * This structure is automatically updated by the application and 
 * transmitted via Mavlink HEARTBEAT custom mode bitfield
 */
typedef struct {
    uint16_t timer_seconds;      // Timer seconds (0-16383)
    uint8_t timer_mode;          // Timer mode (0-3)
    uint8_t fuse_present;        // Fuse present (0-1)
    uint8_t board_state;         // Board state (0-7)
    uint8_t battery_level;       // Battery level (0-10, where 10 = 100%)
    uint8_t error_code;          // Error code (0-15)
} init_board_system_info_t;
```
```c
// Custom mode bitfield structure (32 bits total)
typedef union {
    uint32_t raw;
    struct {
        uint32_t safe_destroy_timer_sec : 14;  // Bits 0-13: Safe/destroy timer in seconds (0-16383)
        uint32_t timer_mode : 2;               // Bits 14-15: Timer mode (see enum below)
        uint32_t is_fuse_present : 1;          // Bit 16: Fuse presence indicator
        uint32_t board_state : 3;              // Bits 17-19: Board state (see enum below)
        uint32_t battery_level : 4;            // Bits 20-23: Battery level in 10% increments (0x0=0%, 0xA=100%)
        uint32_t error_code : 4;               // Bits 24-27: Error code (see enum below)
        uint32_t reserved : 4;                 // Bits 28-31: Reserved for future use
    } fields;
} mavlink_custom_mode_t;

// Timer mode enumeration
typedef enum {
    MAVLINK_TIMER_MODE_NONE = 0x00,         // No active timer
    MAVLINK_TIMER_MODE_SAFE = 0x01,         // Safe timer active
    MAVLINK_TIMER_MODE_DESTROY = 0x02       // Self destroy timer active
} mavlink_timer_mode_t;

// Board state enumeration
typedef enum {
    MAVLINK_BOARD_STATE_NO_FC = 0x00,       // No FC connected
    MAVLINK_BOARD_STATE_DISARMED = 0x01,    // Disarmed
    MAVLINK_BOARD_STATE_ARMED = 0x02,       // Armed
    MAVLINK_BOARD_STATE_BOOM = 0x03         // Boom
} mavlink_board_state_t;

// Error code enumeration
typedef enum {
    MAVLINK_ERR_CODE_NO_ERROR = 0x00,               // No error
    MAVLINK_ERR_CODE_ACCELEROMETER_FAILURE = 0x01,  // Accelerometer error
    MAVLINK_ERR_CODE_BATTERY_LOW = 0x02,            // Battery low warning
    MAVLINK_ERR_CODE_FUSE_INCORRECT_STATE = 0x03,   // Fuse in incorrect state
    MAVLINK_ERR_CODE_UNEXPECTED_IGNITION = 0x04,    // Unexpected ignition detected
    MAVLINK_ERR_CODE_UNEXPECTED_ARM = 0x05,         // Unexpected arm condition
    MAVLINK_ERR_CODE_UNEXPECTED_VUSA_SHORTED = 0x07, // Unexpected VUSA shorted
} mavlink_error_code_t;

// Helper macros for battery level conversion
#define MAVLINK_BATTERY_LEVEL_TO_PERCENT(level) ((level) * 10)  // Convert bitfield to percentage
#define MAVLINK_BATTERY_PERCENT_TO_LEVEL(percent) (((percent) + 5) / 10)  // Convert percentage to bitfield (with rounding)

// Usage examples:
// mavlink_custom_mode_t custom_mode = {0};
// custom_mode.fields.safe_destroy_timer_sec = 120;        // 2 minutes timer
// custom_mode.fields.timer_mode = MAVLINK_TIMER_MODE_SAFE;
// custom_mode.fields.is_fuse_present = 1;                 // Fuse is present
// custom_mode.fields.board_state = MAVLINK_BOARD_STATE_ARMED;
// custom_mode.fields.battery_level = 0x08;                // 80% battery (8 * 10%)
// custom_mode.fields.error_code = MAVLINK_ERR_CODE_NO_ERROR;  // No errors
// uint32_t custom_mode_value = custom_mode.raw;           // Use in HEARTBEAT message
```

#### 5.1 Custom Mode Helper Functions
```c
// Initialize custom mode structure
void Mavlink_InitCustomMode(mavlink_custom_mode_t* custom_mode) {
    custom_mode->raw = 0;  // Clear all fields
    custom_mode->fields.timer_mode = MAVLINK_TIMER_MODE_NONE;
    custom_mode->fields.board_state = MAVLINK_BOARD_STATE_NO_FC;
    custom_mode->fields.battery_level = 0x0A;  // Default to 100%
    custom_mode->fields.error_code = MAVLINK_ERR_CODE_NO_ERROR;
}

// Update timer information
void Mavlink_SetTimerInfo(mavlink_custom_mode_t* custom_mode, 
                         mavlink_timer_mode_t mode, 
                         uint16_t timer_sec) {
    custom_mode->fields.timer_mode = mode;
    custom_mode->fields.safe_destroy_timer_sec = (timer_sec > 16383) ? 16383 : timer_sec;
}

// Update board state
void Mavlink_SetBoardState(mavlink_custom_mode_t* custom_mode, 
                          mavlink_board_state_t state) {
    custom_mode->fields.board_state = state;
}

// Update battery level from percentage
void Mavlink_SetBatteryPercent(mavlink_custom_mode_t* custom_mode, 
                              uint8_t battery_percent) {
    if (battery_percent > 100) battery_percent = 100;
    custom_mode->fields.battery_level = MAVLINK_BATTERY_PERCENT_TO_LEVEL(battery_percent);
}

// Get battery percentage
uint8_t Mavlink_GetBatteryPercent(const mavlink_custom_mode_t* custom_mode) {
    return MAVLINK_BATTERY_LEVEL_TO_PERCENT(custom_mode->fields.battery_level);
}

// Set fuse presence
void Mavlink_SetFusePresent(mavlink_custom_mode_t* custom_mode, bool is_present) {
    custom_mode->fields.is_fuse_present = is_present ? 1 : 0;
}

// Set error code
void Mavlink_SetErrorCode(mavlink_custom_mode_t* custom_mode, mavlink_error_code_t error_code) {
    if (error_code < MAVLINK_ERR_CODE_MAX_ERROR) {
        custom_mode->fields.error_code = error_code;
    }
}

// Get error code
mavlink_error_code_t Mavlink_GetErrorCode(const mavlink_custom_mode_t* custom_mode) {
    return (mavlink_error_code_t)custom_mode->fields.error_code;
}

// Get error description string
const char* Mavlink_GetErrorDescription(mavlink_error_code_t error_code) {
    static const char* error_descriptions[] = {
        "No Error",
        "Strong Alarm",
        "Battery Low",
        "Fuse Incorrect State",
        "Unexpected Ignition",
        "Unexpected Arm",
        "Unexpected Mining",
        "Unexpected VUSA Shorted",
        "Unknown Error"
    };
    
    if (error_code < MAVLINK_ERR_CODE_MAX_ERROR) {
        return error_descriptions[error_code];
    }
    return error_descriptions[MAVLINK_ERR_CODE_MAX_ERROR];
}
```

### 6. MAVLINK MESSAGE SPECIFICATIONS

#### 6.1 HEARTBEAT Message (ID: 0)
**Ground Station → InitBoard:**
```c
typedef struct {
    uint8_t type;           // MAVLINK_TYPE_GCS = 6
    uint8_t autopilot;      // MAVLINK_AUTOPILOT_INVALID = 8 (for GCS)
    uint8_t base_mode;      // System mode bitmap (0 for GCS)
    uint32_t custom_mode;   // Custom mode (0 for GCS)
    uint8_t system_status;  // MAVLINK_STATE_ACTIVE = 4
    uint8_t mavlink_version; // Mavlink version (3 for v2.0)
} mavlink_heartbeat_t;
```

**InitBoard → Ground Station:**
```c
typedef struct {
    uint8_t type;           // MAVLINK_TYPE_GENERIC = 0
    uint8_t autopilot;      // MAVLINK_AUTOPILOT_GENERIC = 0
    uint8_t base_mode;      // System mode bitmap
    uint32_t custom_mode;   // 4 bytes custom status (see section 5)
    uint8_t system_status;  // MAVLINK_STATE_STANDBY/ACTIVE/etc
    uint8_t mavlink_version; // 3 for v2.0
} mavlink_heartbeat_t;
```

**Key Fields to Monitor:**
- **system_status:** Indicates if GCS is active/standby
- **custom_mode:** InitBoard's internal status (4 bytes bitfield - see section 5)
  - **safe_destroy_timer_sec:** Current timer value (0-16383 seconds)
  - **timer_mode:** Active timer type (none/safe/destroy)
  - **is_fuse_present:** Fuse detection status
  - **board_state:** Current board operational state
  - **battery_level:** Battery charge level in 10% increments
  - **error_code:** Current error state (see error enumeration)
- **base_mode:** Armed/disarmed state, safety switch, etc.

#### 6.2 ESP32 GCS HEARTBEAT Generation
**Packet Structure for ESP32 Emulator:**
```
Byte:   0    1    2    3    4    5    6    7    8    9   10  11  12  13  14  15  16  17
Value: [FD] [09] [00] [00] [XX] [FF] [BE] [00] [00] [00] [06] [08] [00] [00] [00] [00] [04] [03]
Field: |STX||LEN||INC||CMP||SEQ||SID||CID||MSG_ID ||  PAYLOAD (9 bytes)                 ||CRC|
```

**ESP32 Implementation Requirements:**
- **Byte 0:** 0xFD (Mavlink v2.0)
- **Byte 1:** 0x09 (9 bytes payload)
- **Byte 4:** Sequence counter (0-255, increment each packet)
- **Byte 5:** MAVLINK_SYSTEM_ID_GCS (configurable, default 255)
- **Byte 6:** MAVLINK_COMP_ID_GCS (configurable, default 190)
- **Byte 10:** 0x06 (MAVLINK_TYPE_GCS)
- **Byte 11:** 0x08 (MAVLINK_AUTOPILOT_INVALID)
- **Byte 16:** 0x04 (MAVLINK_STATE_ACTIVE)
- **Byte 17:** 0x03 (Mavlink version)
- **Last 2 bytes:** CRC16 checksum

**ESP32 UART2 Implementation:**
```cpp
// ESP32 UART2 setup - GPIO 16 (RX2), GPIO 17 (TX2)
void setup_uart2() {
    Serial2.begin(9600, SERIAL_8N1, 16, 17);  // RX=GPIO16, TX=GPIO17
}

// ESP32 GPIO setup for control inputs
void setup_control_inputs() {
    pinMode(4, INPUT_PULLUP);   // 3-position switch input A
    pinMode(2, INPUT_PULLUP);   // 3-position switch input B
    pinMode(15, INPUT_PULLUP);  // PREARM button input
}

// ESP32 timer callback - called every 1000ms
void heartbeat_timer_callback() {
    send_gcs_heartbeat_uart2();
}

void setup() {
    Serial.begin(115200);  // Debug output via UART0
    setup_uart2();
    setup_control_inputs();
    // Initialize 1-second timer for HEARTBEAT
    timer = timerBegin(0, 80, true);  // 1MHz timer
    timerAttachInterrupt(timer, &heartbeat_timer_callback, true);
    timerAlarmWrite(timer, 1000000, true);  // 1 second
    timerAlarmEnable(timer);
}
```

#### 6.3 VFR_HUD Message (ID: 74)
**Ground Station → InitBoard:**
```c
typedef struct {
    float airspeed;         // Current airspeed in m/s
    float groundspeed;      // Current ground speed in m/s  
    float alt;              // Current altitude (MSL) in meters
    float climb;            // Current climb rate in m/s
    int16_t heading;        // Current heading in degrees (0-360)
    uint16_t throttle;      // Current throttle setting (0-100%)
} mavlink_vfr_hud_t;
```

**STM32G0 VFR_HUD Storage Structure (No FPU):**
```c
// Integer-only storage for VFR_HUD data (STM32G0 - no floating point)
typedef struct {
    int32_t airspeed_cm_s;      // Airspeed in cm/s (converted from float m/s * 100)
    int32_t groundspeed_cm_s;   // Ground speed in cm/s (converted from float m/s * 100)
    int32_t altitude_cm;        // Altitude in cm (converted from float m * 100)
    int32_t climb_rate_cm_s;    // Climb rate in cm/s (converted from float m/s * 100)
    int16_t heading_deg;        // Heading in degrees (0-360)
    uint16_t throttle_percent;  // Throttle percentage (0-100)
    uint32_t last_update_ms;    // Timestamp of last update
    bool is_valid;              // Data validity flag
} mavlink_vfr_hud_data_t;
```

**STM32G0 VFR_HUD Processing Functions:**
```c
// Convert float to integer (cm/s from m/s)
int32_t Mavlink_FloatToIntCmS(float value_ms) {
    mavlink_float_uint32_converter_t converter;
    converter.f = value_ms;
    // Extract mantissa and exponent for conversion without FPU
    // Simplified: multiply by 100 using bit manipulation
    return (int32_t)(converter.u >> 8) & 0x00FFFFFF; // Simplified conversion
}

// Convert float to integer (cm from m)
int32_t Mavlink_FloatToIntCm(float value_m) {
    mavlink_float_uint32_converter_t converter;
    converter.f = value_m;
    // Simplified conversion for altitude
    return (int32_t)(converter.u >> 8) & 0x00FFFFFF; // Simplified conversion
}

// Process received VFR_HUD message
void Mavlink_ProcessVfrHud(const mavlink_vfr_hud_t* vfr_hud, mavlink_vfr_hud_data_t* storage) {
    storage->airspeed_cm_s = Mavlink_FloatToIntCmS(vfr_hud->airspeed);
    storage->groundspeed_cm_s = Mavlink_FloatToIntCmS(vfr_hud->groundspeed);
    storage->altitude_cm = Mavlink_FloatToIntCm(vfr_hud->alt);
    storage->climb_rate_cm_s = Mavlink_FloatToIntCmS(vfr_hud->climb);
    storage->heading_deg = vfr_hud->heading;
    storage->throttle_percent = vfr_hud->throttle;
    storage->last_update_ms = HAL_GetTick(); // STM32 HAL timestamp
    storage->is_valid = true;
}

// Check if VFR_HUD data is fresh (within last 3 seconds)
bool Mavlink_IsVfrHudDataFresh(const mavlink_vfr_hud_data_t* storage) {
    return storage->is_valid && 
           ((HAL_GetTick() - storage->last_update_ms) < 3000);
}
```

#### 6.4 Message Headers
```c
typedef struct {
    uint8_t magic;          // 0xFD for Mavlink v2.0
    uint8_t len;            // Payload length
    uint8_t incompat_flags; // Incompatible flags
    uint8_t compat_flags;   // Compatible flags
    uint8_t seq;            // Packet sequence
    uint8_t sysid;          // System ID
    uint8_t compid;         // Component ID
    uint24_t msgid;         // Message ID (3 bytes for v2.0)
} mavlink_header_t;
```

### 7. GCS CONNECTION MONITORING
**Timer API Integration:**
- **Timer Function:** Timer_Start(uint8_t timer_id, uint32_t timer_period_ms, tmr_cbk cbk)
- **Timer Callback:** typedef void (*tmr_cbk)(uint8_t tmr_id)
- **Timer ID:** #define MAVLINK_GCS_TIMEOUT_TIMER_ID 1
- **Timeout Period:** #define MAVLINK_GCS_TIMEOUT_PERIOD_MS 3000 (3 seconds)

**Connection Logic:**
```c
// Restart timer on each valid GCS HEARTBEAT
void Mavlink_ProcessGcsHeartbeat(uint8_t* packet) {
    if (Mavlink_ValidateGcsHeartbeat(packet)) {
        // Restart timeout timer - connection is alive
        Timer_Start(MAVLINK_GCS_TIMEOUT_TIMER_ID, MAVLINK_GCS_TIMEOUT_PERIOD_MS, Mavlink_GcsTimeoutCallback);
        Mavlink_NotifyGcsConnected();
    }
}

// Called when no HEARTBEAT received for 3 seconds
void Mavlink_GcsTimeoutCallback(uint8_t tmr_id) {
    Mavlink_NotifyGcsConnectionLost();
}
```

**Connection States:**
- **MAVLINK_GCS_CONNECTED:** GCS HEARTBEAT received within 3 seconds
- **MAVLINK_GCS_LOST:** No GCS HEARTBEAT for > 3 seconds  
- **MAVLINK_GCS_RECOVERING:** First HEARTBEAT after connection loss

### 8. CUSTOM COMMAND IMPLEMENTATION

#### 8.1 Command Enumeration

The system uses MAVLINK_CMD_USER_1 to transmit custom commands via float parameter encoding:

```c
// Custom command types encoded in param1 as float-to-uint32
typedef enum {
    MAVLINK_CMD_DISARM = 1,           // 0x0001 (low byte) - Switch: GPIO4=0, GPIO2=1
    MAVLINK_CMD_ARM = 2,              // 0x0002 (low byte) - Switch: GPIO4=1, GPIO2=1 (neutral)
    MAVLINK_CMD_IGNITION = 3,         // 0x0003 (low byte) - Switch: GPIO4=1, GPIO2=0
    MAVLINK_CMD_PREARM_ENABLED = 4,   // 0x0004 (low byte) - Button: GPIO15=0
    MAVLINK_CMD_PREARM_DISABLED = 5   // 0x0005 (low byte) - Button: GPIO15=1
} mavlink_custom_command_t;
```

#### 8.2 Command Encoding Protocol

Commands are transmitted using COMMAND_LONG messages with the following encoding:

```c
mavlink_command_long_t cmd;
cmd.command = MAVLINK_CMD_USER_1;
cmd.param1 = Mavlink_Uint32ToFloat(command_id);  // Use union conversion
cmd.param2 = 0.0f;  // Reserved for future use
cmd.param3 = 0.0f;  // Reserved for future use
cmd.param4 = 0.0f;  // Reserved for future use
cmd.param5 = 0.0f;  // Reserved for future use
cmd.param6 = 0.0f;  // Reserved for future use
cmd.param7 = 0.0f;  // Reserved for future use
```

#### 8.3 Command Processing

The InitBoard must:
1. Extract command_id from param1 using Mavlink_FloatToUint32() conversion
2. Check if (command_id & 0xFF) matches valid command enum
3. Execute appropriate action based on command type
4. Send COMMAND_ACK with MAVLINK_RESULT_ACCEPTED or MAVLINK_RESULT_FAILED

#### 8.4 Float-to-Integer Conversion (STM32G0 - No FPU)

```c
// Union for type conversion without floating point operations
typedef union {
    float f;
    uint32_t u;
} mavlink_float_uint32_converter_t;

// Convert uint32 to float representation (for transmission)
float Mavlink_Uint32ToFloat(uint32_t value) {
    mavlink_float_uint32_converter_t converter;
    converter.u = value;
    return converter.f;
}

// Convert float to uint32 (for command extraction)
uint32_t Mavlink_FloatToUint32(float value) {
    mavlink_float_uint32_converter_t converter;
    converter.f = value;
    return converter.u;
}

// Extract command from COMMAND_LONG param1
void Mavlink_ProcessCommandLong(mavlink_command_long_t* cmd) {
    if (cmd->command == MAVLINK_CMD_USER_1) {
        uint32_t param1_data = Mavlink_FloatToUint32(cmd->param1);
        uint8_t command_type = (uint8_t)(param1_data & 0xFF);
        
        switch (command_type) {
            case MAVLINK_CMD_DISARM:
                Mavlink_ExecuteDisarmCommand();
                break;
            case MAVLINK_CMD_ARM:
                Mavlink_ExecuteArmCommand();
                break;
            case MAVLINK_CMD_IGNITION:
                Mavlink_ExecuteIgnitionCommand();
                break;
            case MAVLINK_CMD_PREARM_ENABLED:
                Mavlink_ExecutePrearmEnabledCommand();
                break;
            case MAVLINK_CMD_PREARM_DISABLED:
                Mavlink_ExecutePrearmDisabledCommand();
                break;
            default:
                Mavlink_SendCommandAck(MAVLINK_RESULT_UNSUPPORTED);
                return;
        }
        Mavlink_SendCommandAck(MAVLINK_RESULT_ACCEPTED);
    }
}
```

### 9. ESP32 INPUT CONTROL SYSTEM

#### 9.1 Hardware Input Configuration
```cpp
// GPIO pin definitions
#define MAVLINK_ESP32_SWITCH_PIN_A 4    // 3-position switch input A
#define MAVLINK_ESP32_SWITCH_PIN_B 2    // 3-position switch input B  
#define MAVLINK_ESP32_PREARM_PIN 15     // PREARM button input
#define MAVLINK_ESP32_DEBOUNCE_MS 300   // Debounce time in milliseconds

// Input states enumeration
typedef enum {
    MAVLINK_SWITCH_STATE_DISARM = 0,    // GPIO4=0, GPIO2=1
    MAVLINK_SWITCH_STATE_ARM = 1,       // GPIO4=1, GPIO2=1 (neutral)
    MAVLINK_SWITCH_STATE_IGNITION = 2   // GPIO4=1, GPIO2=0
} mavlink_switch_state_t;

typedef enum {
    MAVLINK_PREARM_DISABLED = 0,        // GPIO15=1 (pull-up active)
    MAVLINK_PREARM_ENABLED = 1          // GPIO15=0 (button pressed)
} mavlink_prearm_state_t;
```

#### 9.2 Input Processing Logic
```cpp
// Global state variables
mavlink_switch_state_t current_switch_state = MAVLINK_SWITCH_STATE_ARM;
mavlink_prearm_state_t current_prearm_state = MAVLINK_PREARM_DISABLED;
unsigned long last_switch_change = 0;
unsigned long last_prearm_change = 0;

// Read 3-position switch state
mavlink_switch_state_t read_switch_state() {
    bool pin4 = digitalRead(MAVLINK_ESP32_SWITCH_PIN_A);
    bool pin2 = digitalRead(MAVLINK_ESP32_SWITCH_PIN_B);
    
    if (!pin4 && pin2) {
        return MAVLINK_SWITCH_STATE_DISARM;    // GPIO4=0, GPIO2=1
    } else if (pin4 && !pin2) {
        return MAVLINK_SWITCH_STATE_IGNITION;  // GPIO4=1, GPIO2=0
    } else {
        return MAVLINK_SWITCH_STATE_ARM;       // GPIO4=1, GPIO2=1 (neutral/default)
    }
}

// Read PREARM button state
mavlink_prearm_state_t read_prearm_state() {
    bool pin15 = digitalRead(MAVLINK_ESP32_PREARM_PIN);
    return pin15 ? MAVLINK_PREARM_DISABLED : MAVLINK_PREARM_ENABLED;
}

// Process input changes with debounce
void process_input_states() {
    unsigned long current_time = millis();
    
    // Process 3-position switch
    mavlink_switch_state_t new_switch_state = read_switch_state();
    if (new_switch_state != current_switch_state) {
        if (current_time - last_switch_change >= MAVLINK_ESP32_DEBOUNCE_MS) {
            current_switch_state = new_switch_state;
            last_switch_change = current_time;
            send_switch_command(current_switch_state);
        }
    }
    
    // Process PREARM button
    mavlink_prearm_state_t new_prearm_state = read_prearm_state();
    if (new_prearm_state != current_prearm_state) {
        if (current_time - last_prearm_change >= MAVLINK_ESP32_DEBOUNCE_MS) {
            current_prearm_state = new_prearm_state;
            last_prearm_change = current_time;
            send_prearm_command(current_prearm_state);
        }
    }
}
```

#### 9.3 Command Transmission
```cpp
// Send switch-based commands
void send_switch_command(mavlink_switch_state_t state) {
    mavlink_custom_command_t cmd;
    const char* state_name;
    
    switch (state) {
        case MAVLINK_SWITCH_STATE_DISARM:
            cmd = MAVLINK_CMD_DISARM;
            state_name = "DISARM";
            break;
        case MAVLINK_SWITCH_STATE_ARM:
            cmd = MAVLINK_CMD_ARM;
            state_name = "ARM";
            break;
        case MAVLINK_SWITCH_STATE_IGNITION:
            cmd = MAVLINK_CMD_IGNITION;
            state_name = "IGNITION";
            break;
    }
    
    Serial.printf("[DEBUG] Switch changed to: %s (GPIO4=%d, GPIO2=%d)\n", 
                  state_name, 
                  digitalRead(MAVLINK_ESP32_SWITCH_PIN_A),
                  digitalRead(MAVLINK_ESP32_SWITCH_PIN_B));
    
    send_mavlink_command(cmd);
}

// Send PREARM commands
void send_prearm_command(mavlink_prearm_state_t state) {
    mavlink_custom_command_t cmd;
    const char* state_name;
    
    if (state == MAVLINK_PREARM_ENABLED) {
        cmd = MAVLINK_CMD_PREARM_ENABLED;
        state_name = "PREARM_ENABLED";
    } else {
        cmd = MAVLINK_CMD_PREARM_DISABLED;
        state_name = "PREARM_DISABLED";
    }
    
    Serial.printf("[DEBUG] PREARM changed to: %s (GPIO15=%d)\n", 
                  state_name, digitalRead(MAVLINK_ESP32_PREARM_PIN));
    
    send_mavlink_command(cmd);
}
```

#### 9.4 Debug Output System
```cpp
// Debug message output via UART0
void debug_mavlink_tx(mavlink_custom_command_t cmd) {
    const char* cmd_names[] = {
        "UNKNOWN", "DISARM", "ARM", "IGNITION", "PREARM_ENABLED", "PREARM_DISABLED"
    };
    
    if (cmd >= 1 && cmd <= 5) {
        Serial.printf("[MAVLINK_TX] Sending command: %s (ID=%d)\n", cmd_names[cmd], cmd);
    } else {
        Serial.printf("[MAVLINK_TX] Sending unknown command: %d\n", cmd);
    }
}

void debug_mavlink_rx(uint8_t* packet, uint8_t len) {
    Serial.printf("[MAVLINK_RX] Received packet: ");
    for (int i = 0; i < len; i++) {
        Serial.printf("%02X ", packet[i]);
    }
    Serial.println();
    
    // Decode message type if it's a known message
    if (len >= 8) {
        uint32_t msg_id = (packet[7] << 16) | (packet[8] << 8) | packet[9];
        switch (msg_id) {
            case 0:
                Serial.println("[MAVLINK_RX] Message type: HEARTBEAT");
                // Decode custom mode if HEARTBEAT from InitBoard
                if (len >= 18) {
                    uint32_t custom_mode_raw = (packet[13] << 24) | (packet[14] << 16) | 
                                              (packet[15] << 8) | packet[16];
                    mavlink_custom_mode_t custom_mode;
                    custom_mode.raw = custom_mode_raw;
                    
                    Serial.printf("  Timer: %d sec, Mode: %d, Fuse: %d, State: %d, Battery: %d%%, Error: %s\n",
                                 custom_mode.fields.safe_destroy_timer_sec,
                                 custom_mode.fields.timer_mode,
                                 custom_mode.fields.is_fuse_present,
                                 custom_mode.fields.board_state,
                                 MAVLINK_BATTERY_LEVEL_TO_PERCENT(custom_mode.fields.battery_level),
                                 get_error_description(custom_mode.fields.error_code));
                }
                break;
            case 77:
                Serial.println("[MAVLINK_RX] Message type: COMMAND_ACK");
                break;
            default:
                Serial.printf("[MAVLINK_RX] Message type: Unknown (ID=%d)\n", msg_id);
                break;
        }
    }
}

// ESP32 helper function for error description
const char* get_error_description(uint8_t error_code) {
    static const char* error_descriptions[] = {
        "No Error", "Strong Alarm", "Battery Low", "Fuse Incorrect", 
        "Unexpected Ignition", "Unexpected Arm", "Unexpected Mining", 
        "VUSA Shorted", "Unknown Error"
    };
    
    return (error_code < 8) ? error_descriptions[error_code] : error_descriptions[8];
}
```

### 10. ESP32 PERIODIC MESSAGING SYSTEM

#### 10.1 Message Scheduling
```cpp
// Message timing configuration
#define MAVLINK_HEARTBEAT_INTERVAL_MS 1000    // 1Hz HEARTBEAT
#define MAVLINK_VFR_HUD_INTERVAL_MS 1000      // 1Hz VFR_HUD
#define MAVLINK_MESSAGE_OFFSET_MS 500         // 500ms offset between messages

// Message timing state
typedef struct {
    unsigned long heartbeat_last_sent;
    unsigned long vfr_hud_last_sent;
    unsigned long base_time;
} mavlink_timing_t;

mavlink_timing_t msg_timing = {0};

// Initialize message timing
void init_message_timing() {
    msg_timing.base_time = millis();
    msg_timing.heartbeat_last_sent = 0;
    msg_timing.vfr_hud_last_sent = MAVLINK_MESSAGE_OFFSET_MS; // Offset VFR_HUD
}

// Process periodic messages without overlap
void process_periodic_messages() {
    unsigned long current_time = millis();
    
    // Send HEARTBEAT every 1000ms at base time
    if (current_time - msg_timing.heartbeat_last_sent >= MAVLINK_HEARTBEAT_INTERVAL_MS) {
        send_gcs_heartbeat_uart2();
        msg_timing.heartbeat_last_sent = current_time;
        Serial.println("[PERIODIC] HEARTBEAT sent");
    }
    
    // Send VFR_HUD every 1000ms at base time + 500ms offset
    if (current_time - msg_timing.vfr_hud_last_sent >= MAVLINK_VFR_HUD_INTERVAL_MS) {
        send_vfr_hud_uart2();
        msg_timing.vfr_hud_last_sent = current_time;
        Serial.println("[PERIODIC] VFR_HUD sent");
    }
}
```

#### 10.2 VFR_HUD Message Generation
```cpp
// VFR_HUD simulation data
typedef struct {
    float airspeed;         // Simulated airspeed (m/s)
    float groundspeed;      // Simulated groundspeed (m/s)
    float altitude;         // Simulated altitude (m)
    float climb_rate;       // Simulated climb rate (m/s)
    int16_t heading;        // Simulated heading (degrees)
    uint16_t throttle;      // Simulated throttle (%)
} vfr_hud_sim_data_t;

vfr_hud_sim_data_t sim_data = {0};

// Initialize simulation data
void init_vfr_hud_simulation() {
    sim_data.airspeed = 25.5;      // 25.5 m/s
    sim_data.groundspeed = 24.8;   // 24.8 m/s
    sim_data.altitude = 120.0;     // 120 meters
    sim_data.climb_rate = 2.1;     // 2.1 m/s climb
    sim_data.heading = 45;         // 45 degrees heading
    sim_data.throttle = 75;        // 75% throttle
}

// Update simulation data (make it dynamic)
void update_vfr_hud_simulation() {
    static unsigned long last_update = 0;
    unsigned long current_time = millis();
    
    if (current_time - last_update >= 100) {  // Update every 100ms
        // Simulate dynamic changes
        sim_data.airspeed += (random(-10, 11) / 100.0);       // ±0.1 m/s variation
        sim_data.groundspeed += (random(-10, 11) / 100.0);    // ±0.1 m/s variation
        sim_data.altitude += (sim_data.climb_rate / 10.0);    // Altitude based on climb rate
        sim_data.climb_rate += (random(-5, 6) / 100.0);       // ±0.05 m/s variation
        sim_data.heading = (sim_data.heading + random(-1, 2)) % 360;  // ±1 degree variation
        sim_data.throttle = constrain(sim_data.throttle + random(-2, 3), 0, 100);
        
        last_update = current_time;
    }
}

// Send VFR_HUD message via UART2
void send_vfr_hud_uart2() {
    update_vfr_hud_simulation();
    
    // Create VFR_HUD packet (simplified - actual implementation needs proper Mavlink encoding)
    uint8_t vfr_hud_packet[28] = {0}; // Mavlink v2.0 header + 20 bytes payload + CRC
    
    // Header (8 bytes)
    vfr_hud_packet[0] = 0xFD;  // Mavlink v2.0 magic
    vfr_hud_packet[1] = 20;    // Payload length
    vfr_hud_packet[2] = 0;     // Incompat flags
    vfr_hud_packet[3] = 0;     // Compat flags
    vfr_hud_packet[4] = get_sequence_number();
    vfr_hud_packet[5] = MAVLINK_SYSTEM_ID_GCS;
    vfr_hud_packet[6] = MAVLINK_COMP_ID_GCS;
    vfr_hud_packet[7] = 74;    // VFR_HUD message ID (low byte)
    vfr_hud_packet[8] = 0;     // VFR_HUD message ID (mid byte)
    vfr_hud_packet[9] = 0;     // VFR_HUD message ID (high byte)
    
    // Payload (20 bytes) - Float values in IEEE 754 format
    memcpy(&vfr_hud_packet[10], &sim_data.airspeed, 4);     // bytes 10-13
    memcpy(&vfr_hud_packet[14], &sim_data.groundspeed, 4);  // bytes 14-17
    memcpy(&vfr_hud_packet[18], &sim_data.altitude, 4);     // bytes 18-21
    memcpy(&vfr_hud_packet[22], &sim_data.climb_rate, 4);   // bytes 22-25
    memcpy(&vfr_hud_packet[26], &sim_data.heading, 2);      // bytes 26-27
    memcpy(&vfr_hud_packet[28], &sim_data.throttle, 2);     // bytes 28-29
    
    // Calculate and add CRC16 (bytes 30-31)
    uint16_t crc = calculate_mavlink_crc(vfr_hud_packet, 30);
    vfr_hud_packet[30] = crc & 0xFF;
    vfr_hud_packet[31] = (crc >> 8) & 0xFF;
    
    // Send via UART2
    Serial2.write(vfr_hud_packet, 32);
    
    // Debug output
    Serial.printf("[VFR_HUD] A:%.1f G:%.1f Alt:%.1f C:%.1f H:%d T:%d%%\n",
                  sim_data.airspeed, sim_data.groundspeed, sim_data.altitude,
                  sim_data.climb_rate, sim_data.heading, sim_data.throttle);
}

// Get incrementing sequence number
uint8_t get_sequence_number() {
    static uint8_t seq = 0;
    return seq++;
}

// Simplified CRC calculation (actual implementation needed)
uint16_t calculate_mavlink_crc(const uint8_t* data, uint8_t len) {
    // Simplified CRC - actual Mavlink CRC-16-CCITT implementation needed
    uint16_t crc = 0xFFFF;
    for (int i = 0; i < len; i++) {
        crc ^= data[i] << 8;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
            else crc <<= 1;
        }
    }
    return crc;
}
```

#### 10.3 Updated Main Loop
```cpp
void loop() {
    // Process periodic messages (HEARTBEAT + VFR_HUD with timing offset)
    process_periodic_messages();
    
    // Process input states (switches and buttons)
    process_input_states();
    
    // Process incoming UART2 data
    process_uart2_incoming();
    
    // Small delay to prevent overwhelming the system
    delay(10);
}
```

### 11. DEVELOPMENT PHASES
1. **Phase 1:** Create STM32 mavlink_uart.c/.h module with basic packet parsing and VFR_HUD storage
2. **Phase 2:** Implement ESP32 ground station emulator with configurable IDs and periodic message scheduling
3. **Phase 3:** Basic Mavlink communication (HEARTBEAT + VFR_HUD) at 9600 baud with offset timing
4. **Phase 4:** Add ESP32 input control system with 3-position switch and PREARM button
5. **Phase 5:** Add COMMAND_LONG support with debug output and VFR_HUD simulation
6. **Phase 6:** Integration testing between STM32 and ESP32 with full command set and HUD data

### 12. FILE STRUCTURE
#### STM32G0 Files:
- **mavlink_uart.h** - Header with function prototypes, callback definitions, and timer constants
  - #define MAVLINK_GCS_TIMEOUT_TIMER_ID 1
  - #define MAVLINK_GCS_TIMEOUT_PERIOD_MS 3000
  - #define MAVLINK_SYSTEM_ID_INITBOARD (configurable)
  - #define MAVLINK_COMP_ID_USER1 25
  - mavlink_vfr_hud_data_t structure definition
- **mavlink_uart.c** - Implementation of UART Mavlink protocol handling with GCS timeout monitoring and VFR_HUD processing
- **config.h** - Configuration defines for system IDs

#### ESP32 Files:
- **config.h** - Configuration defines for ground station emulator
  - #define MAVLINK_SYSTEM_ID_GCS 255
  - #define MAVLINK_COMP_ID_GCS 190
  - #define MAVLINK_HEARTBEAT_INTERVAL_MS 1000
  - #define MAVLINK_ESP32_UART2_RX_PIN 16
  - #define MAVLINK_ESP32_UART2_TX_PIN 17
  - #define MAVLINK_ESP32_SWITCH_PIN_A 4
  - #define MAVLINK_ESP32_SWITCH_PIN_B 2
  - #define MAVLINK_ESP32_PREARM_PIN 15
  - #define MAVLINK_ESP32_DEBOUNCE_MS 300
  - #define MAVLINK_HEARTBEAT_INTERVAL_MS 1000
  - #define MAVLINK_VFR_HUD_INTERVAL_MS 1000
  - #define MAVLINK_MESSAGE_OFFSET_MS 500
- **main.cpp** - Ground station emulator implementation with UART2-based communication, input control system, and periodic VFR_HUD transmission (NO WiFi)

### 13. QUESTIONS TO CLARIFY
- [ ] ~~Preferred WiFi mode (AP with fixed SSID/password or Client)?~~ REMOVED - UART only
- [ ] ~~Should ESP32 bridge store/forward messages if connection lost?~~ REMOVED - UART only  
- [ ] What specific VFR_HUD parameters are most important?
- [ ] Any specific error handling requirements?
- [ ] ~~Integration with existing ground station software?~~ REMOVED - UART only

---
*Last updated: August 27, 2025*
