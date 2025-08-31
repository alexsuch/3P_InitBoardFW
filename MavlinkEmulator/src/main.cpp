#include <Arduino.h>
#include <HardwareSerial.h>

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
#define MAVLINK_ESP32_ARM_PIN 4           // ARM state pin (0=DISARMED, 1=ARMED)
#define MAVLINK_ESP32_PREARM_PIN 15       // PREARM state pin (0=PREARM_DISABLED, 1=PREARM_ENABLED)
#define MAVLINK_ESP32_IGNITION_PIN 0      // IGNITION state pin (0=OFF, 1=IGNITION)
#define MAVLINK_ESP32_DEBOUNCE_MS 300

// Message Timing Configuration
#define MAVLINK_HEARTBEAT_INTERVAL_MS 1000

// Command Configuration
#define MAVLINK_CMD_IGNITION 1            // Only command that autopilot can send

// ======================= TYPE DEFINITIONS =======================
// ARM and PREARM states (read from GPIO pins) - matching STM32G0 enum definitions
typedef enum {
    MAVLINK_ARM_STATE_DISARMED = 0,     // GPIO4=0 (MAVLINK_AUTOPILOT_ARM_DISARMED)
    MAVLINK_ARM_STATE_ARMED = 1         // GPIO4=1 (MAVLINK_AUTOPILOT_ARM_ARMED)
} mavlink_arm_state_t;

typedef enum {
    MAVLINK_PREARM_STATE_DISABLED = 0,  // GPIO15=0 (MAVLINK_AUTOPILOT_PREARM_DISABLED)
    MAVLINK_PREARM_STATE_ENABLED = 1    // GPIO15=1 (MAVLINK_AUTOPILOT_PREARM_ENABLED)
} mavlink_prearm_state_t;

typedef enum {
    MAVLINK_IGNITION_STATE_OFF = 0,     // GPIO2=0 (IGNITION_OFF)
    MAVLINK_IGNITION_STATE_ON = 1       // GPIO2=1 (IGNITION_ON)
} mavlink_ignition_state_t;

// Message timing state
typedef struct {
    unsigned long heartbeat_last_sent;
    unsigned long base_time;
} mavlink_timing_t;

// ======================= GLOBAL VARIABLES =======================
// State variables
mavlink_arm_state_t current_arm_state = MAVLINK_ARM_STATE_DISARMED;
mavlink_prearm_state_t current_prearm_state = MAVLINK_PREARM_STATE_DISABLED;
mavlink_ignition_state_t current_ignition_state = MAVLINK_IGNITION_STATE_OFF;
unsigned long last_arm_change = 0;
unsigned long last_prearm_change = 0;
unsigned long last_ignition_change = 0;

// Timing
mavlink_timing_t msg_timing = {0};
uint8_t sequence_number = 0;

// ======================= FUNCTION DECLARATIONS =======================
void debug_mavlink_rx(uint8_t* packet, uint8_t len);
void debug_arm_state_change(mavlink_arm_state_t new_state);
void debug_prearm_state_change(mavlink_prearm_state_t new_state);
void debug_ignition_state_change(mavlink_ignition_state_t new_state);
void send_ignition_command(void);

// ======================= HELPER FUNCTIONS =======================
// ESP32 helper function for error description
const char* get_error_description(uint8_t error_code) {
    static const char* error_descriptions[] = {
        "No Error", "Strong Alarm", "Battery Low", "Fuse Incorrect", 
        "Unexpected Ignition", "Unexpected Arm", "Unexpected Mining", 
        "VUSA Shorted", "Unknown Error"
    };
    
    return (error_code < 8) ? error_descriptions[error_code] : error_descriptions[8];
}

// Get incrementing sequence number
uint8_t get_sequence_number() {
    return sequence_number++;
}

// Simplified CRC calculation for Mavlink
uint16_t calculate_mavlink_crc(const uint8_t* data, uint8_t len) {
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

// ======================= SETUP FUNCTIONS =======================
// ESP32 UART2 setup - GPIO 16 (RX2), GPIO 17 (TX2)
void setup_uart2() {
    Serial2.begin(MAVLINK_UART_BAUD_RATE, SERIAL_8N1, 
                  MAVLINK_ESP32_UART2_RX_PIN, MAVLINK_ESP32_UART2_TX_PIN);
    Serial.println("[INIT] UART2 initialized (GPIO16=RX, GPIO17=TX, 9600 baud)");
}

// ESP32 GPIO setup for state reading inputs
void setup_state_inputs() {
    pinMode(MAVLINK_ESP32_ARM_PIN, INPUT_PULLUP);        // GPIO4 - ARM state with pull-up
    pinMode(MAVLINK_ESP32_PREARM_PIN, INPUT_PULLUP);     // GPIO15 - PREARM state with pull-up
    pinMode(MAVLINK_ESP32_IGNITION_PIN, INPUT_PULLUP);   // GPIO2 - IGNITION state with pull-up
    
    Serial.println("[INIT] State inputs configured:");
    Serial.println("  GPIO4 (ARM state) - INPUT_PULLUP");
    Serial.println("  GPIO15 (PREARM state) - INPUT_PULLUP");
    Serial.println("  GPIO0 (IGNITION state) - INPUT_PULLUP");
}

// Initialize message timing
void init_message_timing() {
    msg_timing.base_time = millis();
    msg_timing.heartbeat_last_sent = 0;
    
    Serial.println("[INIT] Message timing initialized (HEARTBEAT: 1Hz)");
}

// ======================= INPUT PROCESSING =======================
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

// ======================= MAVLINK MESSAGE GENERATION =======================
// Send Autopilot HEARTBEAT with ARM/PREARM states to InitBoard
void send_autopilot_heartbeat() {
    uint8_t heartbeat_packet[21] = {0}; // Mavlink v2.0 header + 9 bytes payload + CRC
    
    // Read current ARM/PREARM states from GPIO
    mavlink_arm_state_t arm_state = read_arm_state();
    mavlink_prearm_state_t prearm_state = read_prearm_state();
    
    // Encode ARM/PREARM states in 32-bit custom mode
    uint32_t custom_mode = 0;
    custom_mode |= (arm_state & 0x0F);          // Bits 0-3: ARM state
    custom_mode |= ((prearm_state & 0x0F) << 4); // Bits 4-7: PREARM state
    // Bits 8-31: Reserved for future use
    
    // Header (10 bytes for v2.0)
    heartbeat_packet[0] = 0xFD;  // Mavlink v2.0 magic
    heartbeat_packet[1] = 9;     // Payload length
    heartbeat_packet[2] = 0;     // Incompat flags
    heartbeat_packet[3] = 0;     // Compat flags
    heartbeat_packet[4] = get_sequence_number();
    heartbeat_packet[5] = MAVLINK_SYSTEM_ID_INITBOARD;  // We're autopilot speaking TO InitBoard
    heartbeat_packet[6] = MAVLINK_COMP_ID_INITBOARD;
    heartbeat_packet[7] = 0;     // HEARTBEAT message ID (low byte)
    heartbeat_packet[8] = 0;     // HEARTBEAT message ID (mid byte)
    heartbeat_packet[9] = 0;     // HEARTBEAT message ID (high byte)
    
    // Payload (9 bytes)
    heartbeat_packet[10] = 2;    // MAV_TYPE_QUADROTOR
    heartbeat_packet[11] = 0;    // MAV_AUTOPILOT_GENERIC
    heartbeat_packet[12] = 1;    // Base mode (MAV_MODE_FLAG_CUSTOM_MODE_ENABLED)
    
    // Custom mode (4 bytes) - ARM/PREARM states from GPIO
    heartbeat_packet[13] = custom_mode & 0xFF;         // ARM/PREARM states
    heartbeat_packet[14] = (custom_mode >> 8) & 0xFF;  // Reserved
    heartbeat_packet[15] = (custom_mode >> 16) & 0xFF; // Reserved
    heartbeat_packet[16] = (custom_mode >> 24) & 0xFF; // Reserved
    
    heartbeat_packet[17] = 4;    // MAV_STATE_ACTIVE
    heartbeat_packet[18] = 3;    // Mavlink version
    
    // Calculate and add CRC16
    uint16_t crc = calculate_mavlink_crc(heartbeat_packet, 19);
    heartbeat_packet[19] = crc & 0xFF;
    heartbeat_packet[20] = (crc >> 8) & 0xFF;
    
    // Send via UART2
    Serial2.write(heartbeat_packet, 21);
}

// ======================= MAVLINK MESSAGE PROCESSING =======================
// Process received HEARTBEAT from InitBoard
void debug_mavlink_rx(uint8_t* packet, uint8_t len) {
    Serial.printf("[MAVLINK_RX] Received packet: ");
    for (int i = 0; i < len; i++) {
        Serial.printf("%02X ", packet[i]);
    }
    Serial.println();
    
    // Decode message type if it's a known message
    if (len >= 10) {
        uint32_t msg_id = packet[7] | (packet[8] << 8) | (packet[9] << 16);
        switch (msg_id) {
            case 0:
                Serial.println("[MAVLINK_RX] Message type: HEARTBEAT");
                // Debug: Show packet length and system ID
                Serial.printf("[DEBUG] Packet length: %d, System ID: 0x%02X (expected: 0x%02X)\n", 
                             len, packet[5], MAVLINK_SYSTEM_ID_INITBOARD);
                
                // Decode custom mode if HEARTBEAT from InitBoard
                if (len >= 21 && packet[5] == MAVLINK_SYSTEM_ID_INITBOARD) {
                    uint32_t custom_mode_raw = packet[13] | (packet[14] << 8) | 
                                              (packet[15] << 16) | (packet[16] << 24);
                    
                    // Decode bitfield
                    uint16_t timer_sec = custom_mode_raw & 0x3FFF;           // Bits 0-13
                    uint8_t timer_mode = (custom_mode_raw >> 14) & 0x03;     // Bits 14-15
                    uint8_t fuse_present = (custom_mode_raw >> 16) & 0x01;   // Bit 16
                    uint8_t board_state = (custom_mode_raw >> 17) & 0x07;    // Bits 17-19
                    uint8_t battery_level = (custom_mode_raw >> 20) & 0x0F;  // Bits 20-23
                    uint8_t error_code = (custom_mode_raw >> 24) & 0x0F;     // Bits 24-27
                    
                    Serial.printf("  Timer: %d sec, Mode: %d, Fuse: %d, State: %d, Battery: %d%%, Error: %s\n",
                                 timer_sec, timer_mode, fuse_present, board_state,
                                 battery_level * 10, get_error_description(error_code));
                } else {
                    Serial.println("[DEBUG] HEARTBEAT decode skipped - length or system ID mismatch");
                }
                break;
            case 77:
                Serial.println("[MAVLINK_RX] Message type: COMMAND_ACK");
                if (len >= 13) {
                    uint16_t command = packet[10] | (packet[11] << 8);
                    uint8_t result = packet[12];
                    Serial.printf("  Command: %d, Result: %d\n", command, result);
                }
                break;
            default:
                Serial.printf("[MAVLINK_RX] Message type: Unknown (ID=%d)\n", msg_id);
                break;
        }
    }
}

// Send IGNITION COMMAND_LONG message to InitBoard
void send_ignition_command(void) {
    uint8_t command_packet[33] = {0}; // Mavlink v2.0 header + 23 bytes payload + CRC
    
    // Header (10 bytes for v2.0)
    command_packet[0] = 0xFD;  // Mavlink v2.0 magic
    command_packet[1] = 23;    // Payload length for COMMAND_LONG
    command_packet[2] = 0;     // Incompat flags
    command_packet[3] = 0;     // Compat flags
    command_packet[4] = get_sequence_number();
    command_packet[5] = MAVLINK_SYSTEM_ID_INITBOARD;  // Target system
    command_packet[6] = MAVLINK_COMP_ID_INITBOARD;    // Target component
    command_packet[7] = 0x4C;  // COMMAND_LONG message ID (low byte)
    command_packet[8] = 0x00;  // COMMAND_LONG message ID (mid byte)
    command_packet[9] = 0x00;  // COMMAND_LONG message ID (high byte)
    
    // Payload (23 bytes for COMMAND_LONG)
    // param1 (4 bytes) - IGNITION command
    uint32_t ignition_param = MAVLINK_CMD_IGNITION;
    command_packet[10] = ignition_param & 0xFF;
    command_packet[11] = (ignition_param >> 8) & 0xFF;
    command_packet[12] = (ignition_param >> 16) & 0xFF;
    command_packet[13] = (ignition_param >> 24) & 0xFF;
    
    // param2-param7 (4 bytes each) - zeros
    for(int i = 14; i < 30; i++) {
        command_packet[i] = 0;
    }
    
    // command (2 bytes) - MAV_CMD_USER_1
    command_packet[30] = 0x22;  // MAV_CMD_USER_1 low byte (0x7922)
    command_packet[31] = 0x79;  // MAV_CMD_USER_1 high byte
    
    // target_system and target_component
    command_packet[32] = MAVLINK_SYSTEM_ID_INITBOARD;  // target_system
    
    // Calculate and add CRC16
    uint16_t crc = calculate_mavlink_crc(command_packet, 31);
    command_packet[31] = crc & 0xFF;
    command_packet[32] = (crc >> 8) & 0xFF;
    
    // Send via UART2
    Serial2.write(command_packet, 33);
    
    Serial.println("[TX] IGNITION COMMAND_LONG sent to InitBoard");
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
        
        Serial.printf("[TX] Autopilot HEARTBEAT: ARM=%s (GPIO4=%d), PREARM=%s (GPIO15=%d)\n",
                      arm_state == MAVLINK_ARM_STATE_ARMED ? "ARMED" : "DISARMED", gpio4_raw,
                      prearm_state == MAVLINK_PREARM_STATE_ENABLED ? "ENABLED" : "DISABLED", gpio15_raw);
    }
}

// Process input changes with debounce
void process_input_states() {
    unsigned long current_time = millis();
    
    // Process ARM state
    mavlink_arm_state_t new_arm_state = read_arm_state();
    if (new_arm_state != current_arm_state) {
        if (current_time - last_arm_change >= MAVLINK_ESP32_DEBOUNCE_MS) {
            current_arm_state = new_arm_state;
            last_arm_change = current_time;
            debug_arm_state_change(current_arm_state);
        }
    }
    
    // Process PREARM state  
    mavlink_prearm_state_t new_prearm_state = read_prearm_state();
    if (new_prearm_state != current_prearm_state) {
        if (current_time - last_prearm_change >= MAVLINK_ESP32_DEBOUNCE_MS) {
            current_prearm_state = new_prearm_state;
            last_prearm_change = current_time;
            debug_prearm_state_change(current_prearm_state);
        }
    }
    
    // Process IGNITION state
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
    
    // Debug initial GPIO values
    bool gpio4_initial = digitalRead(MAVLINK_ESP32_ARM_PIN);
    bool gpio15_initial = digitalRead(MAVLINK_ESP32_PREARM_PIN);
    bool gpio0_initial = digitalRead(MAVLINK_ESP32_IGNITION_PIN);
    
    Serial.printf("[INIT] Initial GPIO values: GPIO4=%d, GPIO15=%d, GPIO0=%d\n", 
                  gpio4_initial, gpio15_initial, gpio0_initial);
    Serial.printf("[INIT] Initial states: ARM=%s, PREARM=%s, IGNITION=%s\n",
                  current_arm_state == MAVLINK_ARM_STATE_ARMED ? "ARMED" : "DISARMED",
                  current_prearm_state == MAVLINK_PREARM_STATE_ENABLED ? "ENABLED" : "DISABLED",
                  current_ignition_state == MAVLINK_IGNITION_STATE_ON ? "ON" : "OFF");
    
    Serial.println("[INIT] Initialization complete!");
    Serial.println("======================================");
}

void loop() {
    // Send periodic autopilot HEARTBEAT (1Hz) with ARM/PREARM states
    process_periodic_heartbeat();
    
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
    Serial.printf("[GPIO] ARM state changed: %s (GPIO4=%d)\n",
                  new_state == MAVLINK_ARM_STATE_ARMED ? "ARMED" : "DISARMED",
                  digitalRead(MAVLINK_ESP32_ARM_PIN));
}

// Debug output for PREARM state changes
void debug_prearm_state_change(mavlink_prearm_state_t new_state) {
    Serial.printf("[GPIO] PREARM state changed: %s (GPIO15=%d)\n",
                  new_state == MAVLINK_PREARM_STATE_ENABLED ? "ENABLED" : "DISABLED",
                  digitalRead(MAVLINK_ESP32_PREARM_PIN));
}

// Debug output for IGNITION state changes
void debug_ignition_state_change(mavlink_ignition_state_t new_state) {
    Serial.printf("[GPIO] IGNITION state changed: %s (GPIO0=%d)\n",
                  new_state == MAVLINK_IGNITION_STATE_ON ? "ON" : "OFF",
                  digitalRead(MAVLINK_ESP32_IGNITION_PIN));
}

