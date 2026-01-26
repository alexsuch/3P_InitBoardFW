#include <Arduino.h>
#include <HardwareSerial.h>

#include "mavlink_crc_helper.h"  // ??? ??????????????? CRC-?????? (v2: header ??? magic + payload + crc_extra)
// ======================= CONFIG SECTION =======================
// System Configuration - Autopilot reads switch states from ESP32 GPIO
#define MAVLINK_SYSTEM_ID_INITBOARD 0x42  // 66 - Initiation Board
#define MAVLINK_COMP_ID_INITBOARD 0x42    // 66 - Initiation Board

// Communication Configuration
#define MAVLINK_ESP32_UART2_RX_PIN 16
#define MAVLINK_ESP32_UART2_TX_PIN 17
#define MAVLINK_UART_BAUD_RATE 9600
#define MAVLINK_DEBUG_BAUD_RATE 115200

// Input Configuration - Autopilot reads these GPIO states
#define MAVLINK_ESP32_ARM_PIN 4            // ARM state pin (0=DISARMED, 1=ARMED)
#define MAVLINK_ESP32_PREARM_PIN 15        // PREARM state pin (0=PREARM_DISABLED, 1=PREARM_ENABLED)
#define MAVLINK_ESP32_IGNITION_PIN 0       // IGNITION state pin (0=OFF, 1=IGNITION)
#define MAVLINK_ESP32_VFR_BUTTON_PIN 25    // VFR control button pin (0=NOT_PRESSED, 1=PRESSED)
#define MAVLINK_ESP32_ARMING_PIN 32        // ARMING state pin (0=ARMED, 1=DISARMED) - encoded in base_mode
#define MAVLINK_ESP32_FLIGHT_STATE_PIN 27  // FLIGHT state pin (0=FLYING, 1=LANDED/default) - for landed_state info
#define MAVLINK_ESP32_DEBOUNCE_MS 300

// Message Timing Configuration
#define MAVLINK_HEARTBEAT_INTERVAL_MS 1000
#define MAVLINK_VFR_HUD_INTERVAL_MS 1000
#define MAVLINK_VFR_HUD_OFFSET_MS 500  // Offset VFR_HUD by 500ms from HEARTBEAT

// Command Configuration
#define MAVLINK_CMD_IGNITION 1      // Custom ignition command type (sent in param1[0])
#define MAVLINK_CMD_FLIGHT_STATE 6  // Custom flight state command (6 = IN_AIR/LANDED info)
#define MAV_CMD_USER_1 31010        // MAVLink standard user command 1

// MAVLink Message IDs
#define MAVLINK_MSG_ID_EXTENDED_SYS_STATE 245  // EXTENDED_SYS_STATE message ID
#define MAVLINK_MSG_ID_HEARTBEAT 0             // HEARTBEAT message ID
#define MAVLINK_MSG_ID_VFR_HUD 74              // VFR_HUD message ID
#define MAVLINK_MSG_ID_COMMAND_LONG 76         // COMMAND_LONG message ID

// MAVLink EXTENDED_SYS_STATE landed_state values (MAV_LANDED_STATE)
#define MAV_LANDED_STATE_UNDEFINED 0  // MAV landed state is unknown
#define MAV_LANDED_STATE_ON_GROUND 1  // MAV is landed (on ground)
#define MAV_LANDED_STATE_IN_AIR 2     // MAV is in air
#define MAV_LANDED_STATE_TAKEOFF 3    // MAV currently taking off
#define MAV_LANDED_STATE_LANDING 4    // MAV currently landing

// MAVLink EXTENDED_SYS_STATE vtol_state values (MAV_VTOL_STATE)
#define MAV_VTOL_STATE_UNDEFINED 0  // MAV is not configured as VTOL

// MAVLink HEARTBEAT base_mode flags
#define MAV_MODE_FLAG_SAFETY_ARMED (1 << 7)  // 0x80 - Vehicle armed (safety lock not armed)

// ======================= INITBOARD HEARTBEAT CUSTOM MODE STRUCTURE =======================
// InitBoard HEARTBEAT custom_mode bitfield structure (32-bit):
// Bits 0-13:   timer_sec (14 bits) - Timer seconds (0-16383)
// Bits 14-15:  timer_mode (2 bits) - Timer mode (0-3)
// Bit 16:      fc_control_present (1 bit) - FC control present (0-1) - autopilot connection status
// Bit 17:      fuse_present (1 bit) - Fuse present (0-1)
// Bits 18-20:  board_state (3 bits) - Board state (0-7)
// Bits 21-24:  battery_level (4 bits) - Battery level 0-10 (encoded as 0-15)
// Bits 25-28:  error_code (4 bits) - Error code (0-15)
// Bit 29:      is_ignition_done (1 bit) - Ignition done flag (0-1)
// Bits 30-31:  reserved (2 bits) - Reserved for future use

// ======================= TYPE DEFINITIONS =======================
// Arming states (read from GPIO 23) - encoded in HEARTBEAT base_mode field
typedef enum {
    MAVLINK_ARMING_STATE_DISARMED = 0,  // GPIO23=0 (DISARMED)
    MAVLINK_ARMING_STATE_ARMED = 1      // GPIO23=1 (ARMED)
} mavlink_arming_state_t;

// ARM and PREARM states (read from GPIO pins) - matching STM32G0 enum definitions
typedef enum {
    MAVLINK_ARM_STATE_DISARMED = 0,  // GPIO4=0 (MAVLINK_AUTOPILOT_ARM_DISARMED)
    MAVLINK_ARM_STATE_ARMED = 1      // GPIO4=1 (MAVLINK_AUTOPILOT_ARM_ARMED)
} mavlink_arm_state_t;

typedef enum {
    MAVLINK_PREARM_STATE_DISABLED = 0,  // GPIO15=0 (MAVLINK_AUTOPILOT_PREARM_DISABLED)
    MAVLINK_PREARM_STATE_ENABLED = 1    // GPIO15=1 (MAVLINK_AUTOPILOT_PREARM_ENABLED)
} mavlink_prearm_state_t;

typedef enum {
    MAVLINK_IGNITION_STATE_OFF = 0,  // GPIO2=0 (IGNITION_OFF)
    MAVLINK_IGNITION_STATE_ON = 1    // GPIO2=1 (IGNITION_ON)
} mavlink_ignition_state_t;

typedef enum {
    MAVLINK_VFR_BUTTON_NOT_PRESSED = 0,  // GPIO25=0 (Button not pressed)
    MAVLINK_VFR_BUTTON_PRESSED = 1       // GPIO25=1 (Button pressed)
} mavlink_vfr_button_state_t;

// Flight state (In air / Landed) - MAV_LANDED_STATE encoding
typedef enum {
    MAVLINK_FLIGHT_STATE_FLYING = 0,  // GPIO27=0 (FLYING/IN_AIR/TAKEOFF/LANDING)
    MAVLINK_FLIGHT_STATE_LANDED = 1   // GPIO27=1 (LANDED/default)
} mavlink_flight_state_t;

// Message timing state
typedef struct {
    unsigned long heartbeat_last_sent;
    unsigned long vfr_hud_last_sent;
    unsigned long base_time;
} mavlink_timing_t;

// VFR_HUD simulation data
typedef struct {
    float airspeed;     // m/s
    float groundspeed;  // m/s
    int16_t heading;    // degrees (0-359, -1=unknown)
    uint16_t throttle;  // percentage (0-100)
    float altitude;     // meters
    float climb_rate;   // m/s
} vfr_hud_sim_data_t;

// ======================= GLOBAL VARIABLES =======================
// State variables
mavlink_arming_state_t current_arming_state = MAVLINK_ARMING_STATE_DISARMED;
mavlink_arm_state_t current_arm_state = MAVLINK_ARM_STATE_DISARMED;
mavlink_prearm_state_t current_prearm_state = MAVLINK_PREARM_STATE_DISABLED;
mavlink_ignition_state_t current_ignition_state = MAVLINK_IGNITION_STATE_OFF;
mavlink_vfr_button_state_t current_vfr_button_state = MAVLINK_VFR_BUTTON_NOT_PRESSED;
mavlink_flight_state_t current_flight_state = MAVLINK_FLIGHT_STATE_LANDED;
unsigned long last_arming_change = 0;
unsigned long last_arm_change = 0;
unsigned long last_prearm_change = 0;
unsigned long last_ignition_change = 0;
unsigned long last_vfr_button_change = 0;
unsigned long last_flight_state_change = 0;

// Timing
mavlink_timing_t msg_timing = {0};
uint8_t sequence_number = 0;

// VFR_HUD simulation data
vfr_hud_sim_data_t vfr_hud_data = {
    .airspeed = 25.5f,     // 25.5 m/s
    .groundspeed = 24.8f,  // 24.8 m/s
    .heading = 180,        // 180 degrees (South)
    .throttle = 75,        // 75%
    .altitude = 150.0f,    // 150 meters
    .climb_rate = 2.1f     // 2.1 m/s climb
};

// ======================= FUNCTION DECLARATIONS =======================
void debug_mavlink_rx(uint8_t* packet, uint8_t len);
void debug_arm_state_change(mavlink_arm_state_t new_state);
void debug_prearm_state_change(mavlink_prearm_state_t new_state);
void debug_ignition_state_change(mavlink_ignition_state_t new_state);
void debug_vfr_button_state_change(mavlink_vfr_button_state_t new_state);
void send_ignition_command(void);
void send_custom_command(uint8_t command_type, uint8_t command_data);
void send_extended_sys_state(uint8_t landed_state);
void send_vfr_hud(void);
void update_vfr_hud_simulation(void);
const char* get_timer_mode_description(uint8_t timer_mode);
const char* get_board_state_description(uint8_t board_state);

// ======================= HELPER FUNCTIONS =======================
// ESP32 helper function for error description
const char* get_error_description(uint8_t error_code) {
    static const char* error_descriptions[] = {"No Error",
        "Strong Alarm",
        "Battery Low",
        "Fuse Incorrect",
        "Unexpected Ignition",
        "Unexpected Arm",
        "Unexpected Mining",
        "VUSA Shorted",
        "Unknown Error"};

    return (error_code < 8) ? error_descriptions[error_code] : error_descriptions[8];
}

// Get timer mode description
const char* get_timer_mode_description(uint8_t timer_mode) {
    static const char* timer_mode_descriptions[] = {"No Timer", "Safe", "Self Destroy", "Unknown"};

    return (timer_mode < 3) ? timer_mode_descriptions[timer_mode] : timer_mode_descriptions[3];
}

// Get board state description
const char* get_board_state_description(uint8_t board_state) {
    static const char* board_state_descriptions[] = {"Init", "Disarmed", "Charging", "Armed", "Boom", "Unknown"};

    return (board_state < 5) ? board_state_descriptions[board_state] : board_state_descriptions[5];
}

// Get incrementing sequence number
uint8_t get_sequence_number() { return sequence_number++; }
// [NOTE] ?? �?????� ??????? ??????? ??? ?????????? ? ?????, ??? ?? ?????????????? ??? ?????????? ??????.
// MAVLink CRC calculation using exact MCRF4XX algorithm from MAVLink library
uint16_t calculate_mavlink_crc(const uint8_t* data, uint8_t len, uint8_t msg_id) {
    uint16_t crc = 0xFFFF;

    // Process the packet data using exact MCRF4XX CRC16 algorithm from MAVLink library
    for (int i = 0; i < len; i++) {
        uint8_t tmp = data[i] ^ (uint8_t)(crc & 0xFF);
        tmp ^= (tmp << 4);
        crc = (crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4);
    }

    // Add CRC extra bytes based on message ID
    uint8_t crc_extra = 0;
    switch (msg_id) {
        case 0:  // HEARTBEAT
            crc_extra = 50;
            break;
        case 74:  // VFR_HUD
            crc_extra = 20;
            break;
        case 76:  // COMMAND_LONG
            crc_extra = 155;
            break;
        case 77:  // COMMAND_ACK
            crc_extra = 143;
            break;
        default:
            crc_extra = 0;
            break;
    }

    // Add the CRC extra byte using exact MCRF4XX algorithm from MAVLink library
    if (crc_extra != 0) {
        uint8_t tmp = crc_extra ^ (uint8_t)(crc & 0xFF);
        tmp ^= (tmp << 4);
        crc = (crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4);
    }

    return crc;
}

// ======================= SETUP FUNCTIONS =======================
// ESP32 UART2 setup - GPIO 16 (RX2), GPIO 17 (TX2)
void setup_uart2() {
    Serial2.begin(MAVLINK_UART_BAUD_RATE, SERIAL_8N1, MAVLINK_ESP32_UART2_RX_PIN, MAVLINK_ESP32_UART2_TX_PIN);
    Serial.println("[INIT] UART2 initialized (GPIO16=RX, GPIO17=TX, 9600 baud)");
}

// ESP32 GPIO setup for state reading inputs
void setup_state_inputs() {
    pinMode(MAVLINK_ESP32_ARMING_PIN, INPUT_PULLUP);        // GPIO32 - ARMING state with pull-up (inverted logic: LOW=ARMED, HIGH=DISARMED)
    pinMode(MAVLINK_ESP32_ARM_PIN, INPUT_PULLUP);           // GPIO4 - ARM state with pull-up
    pinMode(MAVLINK_ESP32_PREARM_PIN, INPUT_PULLUP);        // GPIO15 - PREARM state with pull-up
    pinMode(MAVLINK_ESP32_IGNITION_PIN, INPUT_PULLUP);      // GPIO0 - IGNITION state with pull-up
    pinMode(MAVLINK_ESP32_VFR_BUTTON_PIN, INPUT_PULLUP);    // GPIO25 - VFR button with pull-up
    pinMode(MAVLINK_ESP32_FLIGHT_STATE_PIN, INPUT_PULLUP);  // GPIO27 - FLIGHT state with pull-up (LOW=FLYING, HIGH=LANDED/default)

    Serial.println("[INIT] State inputs configured:");
    Serial.println("  GPIO32 (ARMING state - base_mode, inverted: LOW=ARMED) - INPUT_PULLUP");
    Serial.println("  GPIO4 (ARM state) - INPUT_PULLUP");
    Serial.println("  GPIO15 (PREARM state) - INPUT_PULLUP");
    Serial.println("  GPIO0 (IGNITION state) - INPUT_PULLUP");
    Serial.println("  GPIO25 (VFR button) - INPUT_PULLUP");
    Serial.println("  GPIO27 (FLIGHT state: HIGH=FLYING, LOW=LANDED) - INPUT_PULLUP");
}

// Initialize message timing
void init_message_timing() {
    msg_timing.base_time = millis();
    msg_timing.heartbeat_last_sent = 0;
    msg_timing.vfr_hud_last_sent = MAVLINK_VFR_HUD_OFFSET_MS;  // Offset VFR_HUD by 500ms

    Serial.println("[INIT] Message timing initialized (HEARTBEAT: 1Hz, VFR_HUD: 1Hz offset +500ms)");
}

// ======================= INPUT PROCESSING =======================
// Read ARMING state from GPIO32 (with pull-up: HIGH=DISARMED, LOW=ARMED) - encoded in HEARTBEAT base_mode
mavlink_arming_state_t read_arming_state() {
    bool pin32 = digitalRead(MAVLINK_ESP32_ARMING_PIN);
    return pin32 ? MAVLINK_ARMING_STATE_DISARMED : MAVLINK_ARMING_STATE_ARMED;  // Inverted logic: LOW=ARMED, HIGH=DISARMED
}

// Read ARM state from GPIO4 (with pull-up: LOW=DISARMED, HIGH=ARMED)
mavlink_arm_state_t read_arm_state() {
    bool pin4 = digitalRead(MAVLINK_ESP32_ARM_PIN);
    return pin4 ? MAVLINK_ARM_STATE_ARMED : MAVLINK_ARM_STATE_DISARMED;  // Normal logic: HIGH=ARMED, LOW=DISARMED
}

// Read PREARM state from GPIO15 (with pull-up: LOW=ENABLED, HIGH=DISABLED)
mavlink_prearm_state_t read_prearm_state() {
    bool pin15 = digitalRead(MAVLINK_ESP32_PREARM_PIN);
    return pin15 ? MAVLINK_PREARM_STATE_DISABLED : MAVLINK_PREARM_STATE_ENABLED;  // Inverted logic with pull-up
}

// Read IGNITION state from GPIO0 (with pull-up: LOW=ON, HIGH=OFF)
mavlink_ignition_state_t read_ignition_state() {
    bool pin0 = digitalRead(MAVLINK_ESP32_IGNITION_PIN);
    return pin0 ? MAVLINK_IGNITION_STATE_OFF : MAVLINK_IGNITION_STATE_ON;  // Inverted logic with pull-up
}

// Read VFR button state from GPIO25 (with pull-up: LOW=PRESSED, HIGH=NOT_PRESSED)
mavlink_vfr_button_state_t read_vfr_button_state() {
    bool pin25 = digitalRead(MAVLINK_ESP32_VFR_BUTTON_PIN);
    return pin25 ? MAVLINK_VFR_BUTTON_NOT_PRESSED : MAVLINK_VFR_BUTTON_PRESSED;  // Inverted logic with pull-up
}

// Read FLIGHT state from GPIO27 (with pull-up: LOW=FLYING, HIGH=LANDED/default)
mavlink_flight_state_t read_flight_state() {
    bool pin35 = digitalRead(MAVLINK_ESP32_FLIGHT_STATE_PIN);
    return pin35 ? MAVLINK_FLIGHT_STATE_LANDED : MAVLINK_FLIGHT_STATE_FLYING;  // Inverted logic: LOW=FLYING, HIGH=LANDED
}

// ======================= MAVLINK MESSAGE GENERATION =======================
// Send Autopilot HEARTBEAT with ARMING state encoded in base_mode
void send_autopilot_heartbeat() {
    uint8_t heartbeat_packet[21] = {0};  // Mavlink v2.0 header + 9 bytes payload + CRC

    // Read current ARMING state from GPIO23
    mavlink_arming_state_t arming_state = read_arming_state();

    // Header (10 bytes for v2.0)
    heartbeat_packet[0] = 0xFD;  // Mavlink v2.0 magic
    heartbeat_packet[1] = 9;     // Payload length
    heartbeat_packet[2] = 0;     // Incompat flags
    heartbeat_packet[3] = 0;     // Compat flags
    heartbeat_packet[4] = get_sequence_number();
    heartbeat_packet[5] = MAVLINK_SYSTEM_ID_INITBOARD;  // We're autopilot speaking TO InitBoard
    heartbeat_packet[6] = MAVLINK_COMP_ID_INITBOARD;
    heartbeat_packet[7] = 0;  // HEARTBEAT message ID (low byte)
    heartbeat_packet[8] = 0;  // HEARTBEAT message ID (mid byte)
    heartbeat_packet[9] = 0;  // HEARTBEAT message ID (high byte)

    // Payload (9 bytes)
    heartbeat_packet[10] = 2;  // MAV_TYPE_QUADROTOR
    heartbeat_packet[11] = 0;  // MAV_AUTOPILOT_GENERIC

    // Base mode - encode ARMING state in bit 7 (MAV_MODE_FLAG_SAFETY_ARMED)
    uint8_t base_mode = 1;  // MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
    if (arming_state == MAVLINK_ARMING_STATE_ARMED) {
        base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;  // Set bit 7 when ARMED
    }
    heartbeat_packet[12] = base_mode;

    // Custom mode (4 bytes) - reserved (ARM/PREARM states now sent via commands)
    heartbeat_packet[13] = 0;  // Reserved
    heartbeat_packet[14] = 0;  // Reserved
    heartbeat_packet[15] = 0;  // Reserved
    heartbeat_packet[16] = 0;  // Reserved

    heartbeat_packet[17] = 4;  // MAV_STATE_ACTIVE
    heartbeat_packet[18] = 3;  // Mavlink version

    // Calculate and add CRC16
    // v2 CRC: header ??? magic (9 ????) + payload (len=9) + crc_extra(HEARTBEAT)
    uint16_t crc_ours = mavlink2_crc_x25(&heartbeat_packet[1], 9, &heartbeat_packet[10], 9,
        mav_crc_extra_lookup_minimal(0));  // msgid=0
    // ??????/�????????� ?????????? � ???? ??? ?????/??????????
    uint16_t crc_old = calculate_mavlink_crc(&heartbeat_packet[1], 18, 0);
    heartbeat_packet[19] = (uint8_t)(crc_ours & 0xFF);
    heartbeat_packet[20] = (uint8_t)(crc_ours >> 8);

    // Send via UART2
    Serial2.write(heartbeat_packet, 21);
}

// Send VFR_HUD message with simulated flight data
void send_vfr_hud() {
    uint8_t vfr_hud_packet[32] = {0};  // Mavlink v2.0 header + 20 bytes payload + CRC

    // Header (10 bytes for Mavlink v2.0)
    vfr_hud_packet[0] = 0xFD;                   // Magic byte
    vfr_hud_packet[1] = 20;                     // Payload length (VFR_HUD = 20 bytes)
    vfr_hud_packet[2] = 0;                      // Incompat flags
    vfr_hud_packet[3] = 0;                      // Compat flags
    vfr_hud_packet[4] = get_sequence_number();  // Sequence
    vfr_hud_packet[5] = 1;                      // System ID (autopilot)
    vfr_hud_packet[6] = 1;                      // Component ID (autopilot)
    vfr_hud_packet[7] = 74;                     // Message ID (VFR_HUD = 74)
    vfr_hud_packet[8] = 0;                      // Message ID (high byte)
    vfr_hud_packet[9] = 0;                      // Message ID (highest byte)

    // Payload (20 bytes) - VFR_HUD data
    // Airspeed (float, 4 bytes)
    memcpy(&vfr_hud_packet[10], &vfr_hud_data.airspeed, 4);

    // Groundspeed (float, 4 bytes)
    memcpy(&vfr_hud_packet[14], &vfr_hud_data.groundspeed, 4);

    // Heading (int16_t, 2 bytes)
    vfr_hud_packet[18] = vfr_hud_data.heading & 0xFF;
    vfr_hud_packet[19] = (vfr_hud_data.heading >> 8) & 0xFF;

    // Throttle (uint16_t, 2 bytes)
    vfr_hud_packet[20] = vfr_hud_data.throttle & 0xFF;
    vfr_hud_packet[21] = (vfr_hud_data.throttle >> 8) & 0xFF;

    // Altitude (float, 4 bytes)
    memcpy(&vfr_hud_packet[22], &vfr_hud_data.altitude, 4);

    // Climb rate (float, 4 bytes)
    memcpy(&vfr_hud_packet[26], &vfr_hud_data.climb_rate, 4);

    // Calculate and add CRC16
    // v2 CRC: header ??? magic (9) + payload (20) + crc_extra(VFR_HUD)
    uint16_t crc_ours = mavlink2_crc_x25(&vfr_hud_packet[1], 9, &vfr_hud_packet[10], 20,
        mav_crc_extra_lookup_minimal(74));  // msgid=74
    // ??????/�????????� � ??? ??????????
    uint16_t crc_old = calculate_mavlink_crc(&vfr_hud_packet[1], 29, 74);
    vfr_hud_packet[30] = (uint8_t)(crc_ours & 0xFF);
    vfr_hud_packet[31] = (uint8_t)(crc_ours >> 8);

    // Send via UART2
    Serial2.write(vfr_hud_packet, 32);
}

// Update VFR_HUD simulation data with dynamic values
void update_vfr_hud_simulation() {
    unsigned long current_time = millis();
    float time_sec = (current_time - msg_timing.base_time) / 1000.0f;

    // Check current VFR button state
    mavlink_vfr_button_state_t button_state = read_vfr_button_state();

    if (button_state == MAVLINK_VFR_BUTTON_PRESSED) {
        // Button pressed: airspeed = 20 m/s, altitude = 150m (fixed values)
        vfr_hud_data.airspeed = 20.0f;
        vfr_hud_data.groundspeed = 19.5f;  // Slightly lower groundspeed due to wind
        vfr_hud_data.altitude = 150.0f;
        vfr_hud_data.climb_rate = 0.0f;  // Stable altitude
    } else {
        // Button not pressed: airspeed < 17 m/s, altitude < 100m
        vfr_hud_data.airspeed = 15.0f + 2.0f * sin(time_sec * 0.1f);                             // 13-17 m/s
        vfr_hud_data.groundspeed = vfr_hud_data.airspeed - 0.5f + 1.0f * sin(time_sec * 0.05f);  // Wind effect
        vfr_hud_data.altitude = 80.0f + 15.0f * sin(time_sec * 0.03f);                           // 65-95m
        vfr_hud_data.climb_rate = 1.0f * cos(time_sec * 0.04f);                                  // -1 to +1 m/s
    }

    // Common parameters (not affected by button state)
    vfr_hud_data.heading = (int16_t)(180 + 30 * sin(time_sec * 0.02f)) % 360;  // 150-210 degrees
    vfr_hud_data.throttle = (uint16_t)(75 + 15 * sin(time_sec * 0.08f));       // 60-90%
}

// ======================= MAVLINK MESSAGE PROCESSING =======================
// Process received HEARTBEAT from InitBoard
void debug_mavlink_rx(uint8_t* packet, uint8_t len) {
    // Serial.printf("[MAVLINK_RX] Received packet: ");
    // for (int i = 0; i < len; i++) {
    //     Serial.printf("%02X ", packet[i]);
    // }
    // Serial.println();

    // Decode message type if it's a known message
    if (len >= 10) {
        uint32_t msg_id = packet[7] | (packet[8] << 8) | (packet[9] << 16);
        switch (msg_id) {
            case 0:
                // Serial.println("[MAVLINK_RX] Message type: HEARTBEAT");
                // Debug: Show packet length and system ID
                // Serial.printf("[DEBUG] Packet length: %d, System ID: 0x%02X (expected: 0x%02X)\n",
                //              len, packet[5], MAVLINK_SYSTEM_ID_INITBOARD);

                // Decode custom mode if HEARTBEAT from InitBoard
                if (len >= 21 && packet[5] == MAVLINK_SYSTEM_ID_INITBOARD) {
                    uint32_t custom_mode_raw = packet[13] | (packet[14] << 8) | (packet[15] << 16) | (packet[16] << 24);

                    // Decode bitfield (matching mavlink_custom_mode_t structure exactly)
                    uint16_t timer_sec = custom_mode_raw & 0x0FFF;                 // Bits 0-11 (12 bits)
                    uint8_t timer_mode = (custom_mode_raw >> 12) & 0x03;           // Bits 12-13 (2 bits)
                    uint8_t fc_control_present = (custom_mode_raw >> 14) & 0x01;   // Bit 14 (1 bit)
                    uint8_t fuse_present = (custom_mode_raw >> 15) & 0x01;         // Bit 15 (1 bit)
                    uint8_t board_state = (custom_mode_raw >> 16) & 0x07;          // Bits 16-18 (3 bits)
                    uint8_t battery_level = (custom_mode_raw >> 19) & 0x0F;        // Bits 19-22 (4 bits)
                    uint8_t error_code = (custom_mode_raw >> 23) & 0x0F;           // Bits 23-26 (4 bits)
                    uint8_t is_ignition_done = (custom_mode_raw >> 27) & 0x01;     // Bit 27 (1 bit)
                    uint8_t prearm_flag = (custom_mode_raw >> 28) & 0x01;          // Bit 28 (1 bit)
                    uint8_t speed_altitude_flag = (custom_mode_raw >> 29) & 0x01;  // Bit 29 (1 bit)
                    uint8_t is_flying = (custom_mode_raw >> 30) & 0x01;            // Bit 30 (1 bit)

                    Serial.printf(
                        "  Timer: %d sec, Timer Mode: %s, FC: %s, Fuse: %d, State: %s, Battery: %d%%, Error: %s, Ignition: %s, Prearm: %s, SpeedAlt: %s, "
                        "Flying: %s\n",
                        timer_sec,
                        get_timer_mode_description(timer_mode),
                        fc_control_present ? "Conn" : "Discon",
                        fuse_present,
                        get_board_state_description(board_state),
                        battery_level * 10,
                        get_error_description(error_code),
                        is_ignition_done ? "Done" : "Not Done",
                        prearm_flag ? "Enabled" : "Disabled",
                        speed_altitude_flag ? "Ena" : "Dis",
                        is_flying ? "IN AIR" : "LANDED");
                } else {
                    // Serial.println("[DEBUG] HEARTBEAT decode skipped - length or system ID mismatch");
                }
                break;
            case 77:
                // Serial.println("[MAVLINK_RX] Message type: COMMAND_ACK");
                if (len >= 13) {
                    uint16_t command = packet[10] | (packet[11] << 8);
                    uint8_t result = packet[12];
                    Serial.printf("  Command: %d, Result: %d\n", command, result);
                }
                break;
            default:
                // Serial.printf("[MAVLINK_RX] Message type: Unknown (ID=%d)\n", msg_id);
                break;
        }
    }
}

// Send IGNITION COMMAND_LONG message to InitBoard
void send_ignition_command(void) {
    send_custom_command(MAVLINK_CMD_IGNITION, 0);  // Send IGNITION command with command_data = 0
}

// Send custom command with specified type and data
void send_custom_command(uint8_t command_type, uint8_t command_data) {
    uint8_t command_packet[45] = {0};  // Mavlink v2.0 header + 33 bytes payload + CRC

    // Header (10 bytes for v2.0)
    command_packet[0] = 0xFD;  // Mavlink v2.0 magic
    command_packet[1] = 33;    // Payload length for COMMAND_LONG
    command_packet[2] = 0;     // Incompat flags
    command_packet[3] = 0;     // Compat flags
    command_packet[4] = get_sequence_number();
    command_packet[5] = MAVLINK_SYSTEM_ID_INITBOARD;  // Source system (we are autopilot)
    command_packet[6] = MAVLINK_COMP_ID_INITBOARD;    // Source component
    command_packet[7] = 0x4C;                         // COMMAND_LONG message ID (low byte)
    command_packet[8] = 0x00;                         // COMMAND_LONG message ID (mid byte)
    command_packet[9] = 0x00;                         // COMMAND_LONG message ID (high byte)

    // Payload (33 bytes for COMMAND_LONG)
    // param1 (4 bytes) - Custom command structure
    command_packet[10] = command_type;  // Custom command type (MAVLINK_CMD_IGNITION = 1)
    command_packet[11] = command_data;  // Command data (additional parameters)
    command_packet[12] = 0;             // Custom param2 (reserved)
    command_packet[13] = 0;             // Custom param3 (reserved)

    // param2-param7 (24 bytes) - zeros (not used)
    for (int i = 14; i < 38; i++) {
        command_packet[i] = 0;
    }

    // command (2 bytes) - MAV_CMD_USER_1 (31010)
    command_packet[38] = 31010 & 0xFF;         // Command low byte (0x22)
    command_packet[39] = (31010 >> 8) & 0xFF;  // Command high byte (0x79)

    // target_system and target_component
    command_packet[40] = MAVLINK_SYSTEM_ID_INITBOARD;  // target_system
    command_packet[41] = MAVLINK_COMP_ID_INITBOARD;    // target_component

    // confirmation (1 byte)
    command_packet[42] = 0;  // No confirmation required

    // Calculate and add CRC16 (??? ?????? + ??? ?????????? ?? �??????�)
    // v2 CRC: header ??? magic (9) + payload (33) + crc_extra(COMMAND_LONG)
    uint16_t crc_ours = mavlink2_crc_x25(&command_packet[1], 9, &command_packet[10], 33,
        mav_crc_extra_lookup_minimal(76));  // msgid=76
    uint16_t crc_old = calculate_mavlink_crc(&command_packet[1], 42, 76);
    command_packet[43] = (uint8_t)(crc_ours & 0xFF);
    command_packet[44] = (uint8_t)(crc_ours >> 8);

    // Send via UART2
    Serial2.write(command_packet, 45);

    Serial.printf("[TX] Custom command sent: type=%u, data=%u\n", command_type, command_data);
}

// Send EXTENDED_SYS_STATE message with flight state information
void send_extended_sys_state(uint8_t landed_state) {
    uint8_t extended_sys_state_packet[22] = {0};  // Mavlink v2.0 header + 2 bytes payload + CRC

    // Header (10 bytes for Mavlink v2.0)
    extended_sys_state_packet[0] = 0xFD;                                             // Magic byte
    extended_sys_state_packet[1] = 2;                                                // Payload length (EXTENDED_SYS_STATE = 2 bytes)
    extended_sys_state_packet[2] = 0;                                                // Incompat flags
    extended_sys_state_packet[3] = 0;                                                // Compat flags
    extended_sys_state_packet[4] = get_sequence_number();                            // Sequence
    extended_sys_state_packet[5] = MAVLINK_SYSTEM_ID_INITBOARD;                      // System ID (autopilot)
    extended_sys_state_packet[6] = MAVLINK_COMP_ID_INITBOARD;                        // Component ID (autopilot)
    extended_sys_state_packet[7] = MAVLINK_MSG_ID_EXTENDED_SYS_STATE & 0xFF;         // Message ID low byte (245 = 0xF5)
    extended_sys_state_packet[8] = (MAVLINK_MSG_ID_EXTENDED_SYS_STATE >> 8) & 0xFF;  // Message ID mid byte
    extended_sys_state_packet[9] = 0;                                                // Message ID high byte

    // Payload (2 bytes) - EXTENDED_SYS_STATE data
    extended_sys_state_packet[10] = MAV_VTOL_STATE_UNDEFINED;  // vtol_state: not VTOL
    extended_sys_state_packet[11] = landed_state;              // landed_state: 1=ON_GROUND, 2=IN_AIR

    // Calculate and add CRC16
    // v2 CRC: header ??? magic (9) + payload (2) + crc_extra(EXTENDED_SYS_STATE)
    uint16_t crc_ours = mavlink2_crc_x25(&extended_sys_state_packet[1],
        9,
        &extended_sys_state_packet[10],
        2,
        mav_crc_extra_lookup_minimal(MAVLINK_MSG_ID_EXTENDED_SYS_STATE));  // msgid=245
    extended_sys_state_packet[12] = (uint8_t)(crc_ours & 0xFF);
    extended_sys_state_packet[13] = (uint8_t)(crc_ours >> 8);

    // Send via UART2
    Serial2.write(extended_sys_state_packet, 14);

    const char* state_str = (landed_state == MAV_LANDED_STATE_ON_GROUND) ? "ON_GROUND" : (landed_state == MAV_LANDED_STATE_IN_AIR) ? "IN_AIR" : "UNKNOWN";
    Serial.printf("[TX] EXTENDED_SYS_STATE sent: landed_state=%s (%u)\n", state_str, landed_state);
}

// ======================= PERIODIC MESSAGE PROCESSING =======================
// Process periodic autopilot HEARTBEAT transmission (1Hz)
void process_periodic_heartbeat() {
    unsigned long current_time = millis();
    unsigned long elapsed = current_time - msg_timing.base_time;

    // Send HEARTBEAT every 1000ms
    if (elapsed - msg_timing.heartbeat_last_sent >= MAVLINK_HEARTBEAT_INTERVAL_MS) {
        send_autopilot_heartbeat();
        msg_timing.heartbeat_last_sent = elapsed;

        // Read current states for debug output
        mavlink_arm_state_t arm_state = read_arm_state();
        mavlink_prearm_state_t prearm_state = read_prearm_state();

        // Debug GPIO raw values
        bool gpio4_raw = digitalRead(MAVLINK_ESP32_ARM_PIN);
        bool gpio15_raw = digitalRead(MAVLINK_ESP32_PREARM_PIN);

        // Serial.printf("[TX] Autopilot HEARTBEAT: ARM=%s (GPIO4=%d), PREARM=%s (GPIO15=%d)\n",
        //               arm_state == MAVLINK_ARM_STATE_ARMED ? "ARMED" : "DISARMED", gpio4_raw,
        //               prearm_state == MAVLINK_PREARM_STATE_ENABLED ? "ENABLED" : "DISABLED", gpio15_raw);
    }
}

// Process periodic VFR_HUD transmission (1Hz, offset by 500ms from HEARTBEAT)
void process_periodic_vfr_hud() {
    unsigned long current_time = millis();
    unsigned long elapsed = current_time - msg_timing.base_time;

    // Send VFR_HUD every 1000ms, offset by 500ms from HEARTBEAT
    if (elapsed - msg_timing.vfr_hud_last_sent >= MAVLINK_VFR_HUD_INTERVAL_MS) {
        update_vfr_hud_simulation();  // Update dynamic flight data
        send_vfr_hud();
        msg_timing.vfr_hud_last_sent = elapsed;

        // Debug output for VFR_HUD transmission
        // Serial.printf("[VFR_HUD_TX] AS=%.1f, GS=%.1f, HDG=%d, THR=%d%%, ALT=%.1fm, CLB=%.1fm/s\n",
        //     vfr_hud_data.airspeed,
        //     vfr_hud_data.groundspeed,
        //     vfr_hud_data.heading,
        //     vfr_hud_data.throttle,
        //     vfr_hud_data.altitude,
        //     vfr_hud_data.climb_rate);
    }
}

// Process input changes with debounce
void process_input_states() {
    unsigned long current_time = millis();

    // Process ARMING state - update HEARTBEAT base_mode on change
    mavlink_arming_state_t new_arming_state = read_arming_state();
    if (new_arming_state != current_arming_state) {
        if (current_time - last_arming_change >= MAVLINK_ESP32_DEBOUNCE_MS) {
            current_arming_state = new_arming_state;
            last_arming_change = current_time;

            // Debug output for ARMING state change
            if (current_arming_state == MAVLINK_ARMING_STATE_ARMED) {
                Serial.println("[ARMING] GPIO23 state: ARMED (LOW) → base_mode |= MAV_MODE_FLAG_SAFETY_ARMED");
            } else {
                Serial.println("[ARMING] GPIO23 state: DISARMED (HIGH) → base_mode without SAFETY_ARMED flag");
            }
        }
    }

    // Process ARM state - send CHARGE when armed, DISCHARGE when disarmed
    mavlink_arm_state_t new_arm_state = read_arm_state();
    if (new_arm_state != current_arm_state) {
        if (current_time - last_arm_change >= MAVLINK_ESP32_DEBOUNCE_MS) {
            current_arm_state = new_arm_state;
            last_arm_change = current_time;
            debug_arm_state_change(current_arm_state);

            // Send appropriate command based on ARM state change
            if (current_arm_state == MAVLINK_ARM_STATE_ARMED) {
                // ARM transition: 0→1 → send CHARGE command (param1[0] = 2)
                Serial.print("[MAVLINK] ARM state: ARMED → sending CHARGE command\n");
                send_custom_command(2, 0);  // 2 = MAVLINK_CMD_CHARGE
            } else {
                // DISARM transition: 1→0 → send DISCHARGE command (param1[0] = 3)
                Serial.print("[MAVLINK] ARM state: DISARMED → sending DISCHARGE command\n");
                send_custom_command(3, 0);  // 3 = MAVLINK_CMD_DISCHARGE
            }
        }
    }

    // Process PREARM state - send PREARM_ENABLE when enabled, PREARM_DISABLE when disabled
    mavlink_prearm_state_t new_prearm_state = read_prearm_state();
    if (new_prearm_state != current_prearm_state) {
        if (current_time - last_prearm_change >= MAVLINK_ESP32_DEBOUNCE_MS) {
            current_prearm_state = new_prearm_state;
            last_prearm_change = current_time;
            debug_prearm_state_change(current_prearm_state);

            // Send appropriate command based on PREARM state change
            if (current_prearm_state == MAVLINK_PREARM_STATE_ENABLED) {
                // PREARM transition: 0→1 → send PREARM_ENABLE command (param1[0] = 4)
                Serial.print("[MAVLINK] PREARM state: ENABLED → sending PREARM_ENABLE command\n");
                send_custom_command(4, 0);  // 4 = MAVLINK_CMD_PREARM_ENABLE
            } else {
                // PREARM transition: 1→0 → send PREARM_DISABLE command (param1[0] = 5)
                Serial.print("[MAVLINK] PREARM state: DISABLED → sending PREARM_DISABLE command\n");
                send_custom_command(5, 0);  // 5 = MAVLINK_CMD_PREARM_DISABLE
            }
        }
    }

    // Process IGNITION state - send IGNITION command on state change to ON
    mavlink_ignition_state_t new_ignition_state = read_ignition_state();
    if (new_ignition_state != current_ignition_state) {
        if (current_time - last_ignition_change >= MAVLINK_ESP32_DEBOUNCE_MS) {
            current_ignition_state = new_ignition_state;
            last_ignition_change = current_time;
            debug_ignition_state_change(current_ignition_state);

            // Send IGNITION command when state changes to ON
            if (current_ignition_state == MAVLINK_IGNITION_STATE_ON) {
                send_ignition_command();
            }
        }
    }

    // Process VFR button state
    mavlink_vfr_button_state_t new_vfr_button_state = read_vfr_button_state();
    if (new_vfr_button_state != current_vfr_button_state) {
        if (current_time - last_vfr_button_change >= MAVLINK_ESP32_DEBOUNCE_MS) {
            current_vfr_button_state = new_vfr_button_state;
            last_vfr_button_change = current_time;
            debug_vfr_button_state_change(current_vfr_button_state);
        }
    }

    // Process FLIGHT state - send FLIGHT_STATE command on change
    mavlink_flight_state_t new_flight_state = read_flight_state();
    if (new_flight_state != current_flight_state) {
        if (current_time - last_flight_state_change >= MAVLINK_ESP32_DEBOUNCE_MS) {
            current_flight_state = new_flight_state;
            last_flight_state_change = current_time;

            // Send EXTENDED_SYS_STATE message with flight state info
            if (current_flight_state == MAVLINK_FLIGHT_STATE_FLYING) {
                Serial.println("[FLIGHT] GPIO27 state: FLYING → sending EXTENDED_SYS_STATE (IN_AIR)");
                send_extended_sys_state(MAV_LANDED_STATE_IN_AIR);  // Send IN_AIR state
            } else {
                Serial.println("[FLIGHT] GPIO27 state: LANDED → sending EXTENDED_SYS_STATE (ON_GROUND)");
                send_extended_sys_state(MAV_LANDED_STATE_ON_GROUND);  // Send ON_GROUND state
            }
        }
    }
}

// Process incoming UART2 data
void process_uart2_incoming() {
    static uint8_t rx_buffer[256];
    static uint8_t rx_index = 0;

    while (Serial2.available()) {
        uint8_t byte = Serial2.read();
        rx_buffer[rx_index++] = byte;

        // Simple packet detection (look for Mavlink magic byte)
        if (byte == 0xFD && rx_index > 1) {
            // Process previous packet if we have one
            if (rx_index > 1) {
                debug_mavlink_rx(rx_buffer, rx_index - 1);
            }
            // Start new packet
            rx_buffer[0] = 0xFD;
            rx_index = 1;
        }

        // Prevent buffer overflow
        if (rx_index >= sizeof(rx_buffer)) {
            rx_index = 0;
        }
    }
}

// ======================= MAIN FUNCTIONS =======================
void setup() {
    // Initialize debug serial
    Serial.begin(MAVLINK_DEBUG_BAUD_RATE);
    delay(1000);

    Serial.println("======================================");
    Serial.println("ESP32 Autopilot GPIO State Monitor");
    Serial.println("Reads ARM/PREARM states for InitBoard");
    Serial.println("======================================");

    // Initialize all subsystems
    setup_uart2();
    setup_state_inputs();
    init_message_timing();

    // Read initial states
    current_arm_state = read_arm_state();
    current_prearm_state = read_prearm_state();
    current_ignition_state = read_ignition_state();
    current_vfr_button_state = read_vfr_button_state();

    // Debug initial GPIO values
    bool gpio4_initial = digitalRead(MAVLINK_ESP32_ARM_PIN);
    bool gpio15_initial = digitalRead(MAVLINK_ESP32_PREARM_PIN);
    bool gpio0_initial = digitalRead(MAVLINK_ESP32_IGNITION_PIN);
    bool gpio25_initial = digitalRead(MAVLINK_ESP32_VFR_BUTTON_PIN);

    Serial.printf("[INIT] Initial GPIO values: GPIO4=%d, GPIO15=%d, GPIO0=%d, GPIO25=%d\n", gpio4_initial, gpio15_initial, gpio0_initial, gpio25_initial);
    Serial.printf("[INIT] Initial states: ARM=%s, PREARM=%s, IGNITION=%s, VFR_BUTTON=%s\n",
        current_arm_state == MAVLINK_ARM_STATE_ARMED ? "ARMED" : "DISARMED",
        current_prearm_state == MAVLINK_PREARM_STATE_ENABLED ? "ENABLED" : "DISABLED",
        current_ignition_state == MAVLINK_IGNITION_STATE_ON ? "ON" : "OFF",
        current_vfr_button_state == MAVLINK_VFR_BUTTON_PRESSED ? "PRESSED" : "NOT_PRESSED");

    Serial.println("[INIT] Initialization complete!");
    Serial.println("======================================");
}

void loop() {
    // Send periodic autopilot HEARTBEAT (1Hz) with ARM/PREARM states
    process_periodic_heartbeat();

    // Process periodic VFR_HUD transmission
    process_periodic_vfr_hud();

    // Process input states (ARM and PREARM monitoring)
    process_input_states();

    // Process incoming UART2 data (InitBoard HEARTBEAT messages)
    process_uart2_incoming();

    // Small delay to prevent overwhelming the system
    delay(10);
}

// ======================= DEBUG FUNCTIONS =======================
// Debug output for ARM state changes
void debug_arm_state_change(mavlink_arm_state_t new_state) {
    Serial.printf("[GPIO] ARM state changed: %s (GPIO4=%d)\n", new_state == MAVLINK_ARM_STATE_ARMED ? "ARMED" : "DISARMED", digitalRead(MAVLINK_ESP32_ARM_PIN));
}

// Debug output for PREARM state changes
void debug_prearm_state_change(mavlink_prearm_state_t new_state) {
    Serial.printf("[GPIO] PREARM state changed: %s (GPIO15=%d)\n",
        new_state == MAVLINK_PREARM_STATE_ENABLED ? "ENABLED" : "DISABLED",
        digitalRead(MAVLINK_ESP32_PREARM_PIN));
}

// Debug output for IGNITION state changes
void debug_ignition_state_change(mavlink_ignition_state_t new_state) {
    Serial.printf(
        "[GPIO] IGNITION state changed: %s (GPIO0=%d)\n", new_state == MAVLINK_IGNITION_STATE_ON ? "ON" : "OFF", digitalRead(MAVLINK_ESP32_IGNITION_PIN));
}

// Debug output for VFR button state changes
void debug_vfr_button_state_change(mavlink_vfr_button_state_t new_state) {
    Serial.printf("[GPIO] VFR_BUTTON state changed: %s (GPIO25=%d) - VFR_HUD mode: %s\n",
        new_state == MAVLINK_VFR_BUTTON_PRESSED ? "PRESSED" : "NOT_PRESSED",
        digitalRead(MAVLINK_ESP32_VFR_BUTTON_PIN),
        new_state == MAVLINK_VFR_BUTTON_PRESSED ? "High altitude/speed" : "Low altitude/speed");
}
