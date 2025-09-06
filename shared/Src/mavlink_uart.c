/*
 * mavlink_uart.c - STM32G0 Mavlink UART Communication Module
 * 
 * This module handles Mavlink v2.0 communication via UART for the STM32G0-based InitBoard.
 * It sends InitBoard HEARTBEAT messages with ARM/PREARM state encoding and processes
 * IGNITION commands from autopilot.
 * 
 * Features:
 * - Integer-only operations (no floating point)
 * - 8-bit custom mode encoding (ARM + PREARM states)
 * - IGNITION command processing
 * - UART interrupt-driven communication
 * - Autopilot connection monitoring
 */

#include "main.h"
#include "mavlink_uart.h"
#include "prj_config.h"
#include "app.h"
#include "timer.h"
#include "solution_wrapper.h"
#include <string.h>

#if (CONTROL_MODE == MAVLINK_V2_CTRL_SUPP)
// ======================= GLOBAL VARIABLES =======================
static mavlink_state_t mavlink_state = {0};
static uint8_t mavlink_tx_buffer[MAVLINK_MAX_MESSAGE_SIZE];
static uint8_t mavlink_rx_buffer[MAVLINK_MAX_MESSAGE_SIZE];
static uint8_t mavlink_sequence_number = 0;

// ======================= STATIC FUNCTION DECLARATIONS =======================
static void Mavlink_SendPacket(uint8_t* packet, uint16_t length);
static uint16_t Mavlink_CalculateCrc(const uint8_t* data, uint8_t len);
static uint8_t Mavlink_GetSequenceNumber(void);
static void Mavlink_ProcessAutopilotHeartbeat(const uint8_t* payload);
static void Mavlink_ProcessVfrHud(const uint8_t* payload);
static void Mavlink_ProcessCommandLong(const uint8_t* payload);
static void Mavlink_SendCommandAck(uint16_t command, uint8_t result);
static mavlink_custom_mode_t Mavlink_EncodeCustomMode(void);
static void Mavlink_SendInitBoardHeartbeat(void);
static void Mavlink_ProcessReceivedMessage(void);

// VFR_HUD conversion functions (integer math only - no FPU)
static uint16_t Mavlink_FloatToIntCmS(const uint8_t* float_bytes);
static int32_t Mavlink_FloatToIntCm(const uint8_t* float_bytes);
static int16_t Mavlink_FloatToIntCmS_Signed(const uint8_t* float_bytes);

// Timer callback functions
static void Mavlink_InitBoardHeartbeatTimerCallback(uint8_t tmr_id);
static void Mavlink_AutopilotConnectionTimeoutCallback(uint8_t tmr_id);

// ======================= INITIALIZATION =======================
/**
 * @brief Initialize Mavlink UART communication
 * @param system_cbk Callback function for system events
 * @param system_info Pointer to system state structure (will be read when sending InitBoard HEARTBEAT)
 */
void Mavlink_Init(app_cbk_fn system_cbk, init_board_system_info_t* system_info) {
    // Initialize state structure
    memset(&mavlink_state, 0, sizeof(mavlink_state_t));
    
    // Set default values
    mavlink_state.system_id = MAVLINK_SYSTEM_ID_INITBOARD;
    mavlink_state.component_id = MAVLINK_COMP_ID_INITBOARD;
    mavlink_state.connected = 0;
    mavlink_state.rx_state = MAVLINK_RX_IDLE;
    mavlink_state.rx_index = 0;
    mavlink_state.system_callback = system_cbk;
    mavlink_state.system_info = system_info;  // Store pointer to system state
    mavlink_state.initboard_heartbeat_send_flag = 0;
    mavlink_state.autopilot_connection_timeout_flag = 0;
    
    // Initialize autopilot ARM/PREARM states to unknown
    mavlink_state.autopilot_arm_state = 0xFF;      // Unknown state initially
    mavlink_state.autopilot_prearm_state = 0xFF;   // Unknown state initially
    
    // Initialize VFR_HUD data
    memset(&mavlink_state.vfr_hud_data, 0, sizeof(mavlink_vfr_hud_data_t));
    mavlink_state.vfr_hud_data.heading_deg = -1;   // Unknown heading initially
    
    // Initialize buffers
    memset(mavlink_tx_buffer, 0, sizeof(mavlink_tx_buffer));
    memset(mavlink_rx_buffer, 0, sizeof(mavlink_rx_buffer));
    
    mavlink_sequence_number = 0;
    
    // Start periodic InitBoard heartbeat timer
    Timer_Start(MAVLINK_INITBOARD_HEARTBEAT_TMR, MAVLINK_INITBOARD_HEARTBEAT_INTERVAL_MS, Mavlink_InitBoardHeartbeatTimerCallback);
}

// ======================= INITBOARD HEARTBEAT FUNCTIONS =======================
/**
 * @brief Send InitBoard HEARTBEAT message with custom mode bitfield
 */
static void Mavlink_SendInitBoardHeartbeat(void) {
    uint8_t packet[21] = {0};  // Mavlink v2.0 header + 9 bytes payload + CRC
    mavlink_custom_mode_t custom_mode = Mavlink_EncodeCustomMode();
    
    // Header (10 bytes for Mavlink v2.0)
    packet[MAVLINK_V2_MAGIC_INDEX] = MAVLINK_V2_MAGIC;                    // Magic byte
    packet[MAVLINK_V2_PAYLOAD_LEN_INDEX] = HEARTBEAT_PAYLOAD_SIZE;        // Payload length
    packet[MAVLINK_V2_INCOMPAT_FLAGS_INDEX] = 0;                         // Incompat flags
    packet[MAVLINK_V2_COMPAT_FLAGS_INDEX] = 0;                           // Compat flags
    packet[MAVLINK_V2_SEQUENCE_INDEX] = Mavlink_GetSequenceNumber();     // Sequence
    packet[MAVLINK_V2_SYSTEM_ID_INDEX] = mavlink_state.system_id;        // System ID
    packet[MAVLINK_V2_COMPONENT_ID_INDEX] = mavlink_state.component_id;  // Component ID
    packet[MAVLINK_V2_MSG_ID_LOW_INDEX] = MAVLINK_MSG_ID_HEARTBEAT & 0xFF;     // Message ID (low byte)
    packet[MAVLINK_V2_MSG_ID_MID_INDEX] = (MAVLINK_MSG_ID_HEARTBEAT >> 8) & 0xFF;  // Message ID (mid byte)
    packet[MAVLINK_V2_MSG_ID_HIGH_INDEX] = (MAVLINK_MSG_ID_HEARTBEAT >> 16) & 0xFF; // Message ID (high byte)
    
    // Payload (9 bytes)
    packet[MAVLINK_V2_PAYLOAD_START_INDEX + HEARTBEAT_TYPE_INDEX] = MAV_TYPE_GENERIC;                   // Type
    packet[MAVLINK_V2_PAYLOAD_START_INDEX + HEARTBEAT_AUTOPILOT_INDEX] = MAV_AUTOPILOT_INVALID;        // Autopilot
    packet[MAVLINK_V2_PAYLOAD_START_INDEX + HEARTBEAT_BASE_MODE_INDEX] = 0;                            // Base mode
    
    // Custom mode (4 bytes) - InitBoard system state bitfield
    packet[MAVLINK_V2_PAYLOAD_START_INDEX + HEARTBEAT_CUSTOM_MODE_INDEX + 0] = custom_mode.raw & 0xFF;
    packet[MAVLINK_V2_PAYLOAD_START_INDEX + HEARTBEAT_CUSTOM_MODE_INDEX + 1] = (custom_mode.raw >> 8) & 0xFF;
    packet[MAVLINK_V2_PAYLOAD_START_INDEX + HEARTBEAT_CUSTOM_MODE_INDEX + 2] = (custom_mode.raw >> 16) & 0xFF;
    packet[MAVLINK_V2_PAYLOAD_START_INDEX + HEARTBEAT_CUSTOM_MODE_INDEX + 3] = (custom_mode.raw >> 24) & 0xFF;
    
    packet[MAVLINK_V2_PAYLOAD_START_INDEX + HEARTBEAT_SYSTEM_STATUS_INDEX] = MAV_STATE_ACTIVE;         // System status
    packet[MAVLINK_V2_PAYLOAD_START_INDEX + HEARTBEAT_MAVLINK_VERSION_INDEX] = MAVLINK_VERSION;       // Mavlink version
    
    // Calculate and add CRC16
    uint16_t crc = Mavlink_CalculateCrc(packet, MAVLINK_V2_HEADER_SIZE + HEARTBEAT_PAYLOAD_SIZE);
    packet[MAVLINK_V2_HEADER_SIZE + HEARTBEAT_PAYLOAD_SIZE] = crc & 0xFF;
    packet[MAVLINK_V2_HEADER_SIZE + HEARTBEAT_PAYLOAD_SIZE + 1] = (crc >> 8) & 0xFF;
    
    // Send packet
    Mavlink_SendPacket(packet, 21);
}

/**
 * @brief Encode current system state into custom mode bitfield
 * @return mavlink_custom_mode_t - Encoded custom mode
 */
static mavlink_custom_mode_t Mavlink_EncodeCustomMode(void) {
    mavlink_custom_mode_t custom_mode = {0};
    
    // Check if system state pointer is valid
    if (mavlink_state.system_info == NULL) {
        return custom_mode;  // Return zero-filled structure if no data
    }
    
    // Call callback to refresh system_info structure on the upper layer
    if (mavlink_state.system_callback) {
        mavlink_state.system_callback(SYSTEM_EVT_INIT_DONE, 0);
    }
    
    // Get current system state from the structure
    init_board_system_info_t* state = mavlink_state.system_info;
    
    // Encode bitfield (matching updated REQUIREMENTS.md specification)
    custom_mode.bitfield.timer_sec = state->timer_seconds;           // 14 bits (0-13) - auto-truncated by compiler
    custom_mode.bitfield.timer_mode = state->timer_mode;             // 2 bits (14-15) - auto-truncated by compiler
    custom_mode.bitfield.fc_control_present = state->fc_control_present; // 1 bit (16) - autopilot connection status
    custom_mode.bitfield.fuse_present = state->fuse_present;         // 1 bit (17) - auto-truncated by compiler
    custom_mode.bitfield.board_state = state->board_state;           // 3 bits (18-20) - auto-truncated by compiler
    custom_mode.bitfield.battery_level = state->battery_level;       // 4 bits (21-24) - auto-truncated by compiler
    custom_mode.bitfield.error_code = state->error_code;             // 4 bits (25-28) - auto-truncated by compiler
    custom_mode.bitfield.is_ignition_done = state->is_ignition_done; // 1 bit (29) - auto-truncated by compiler
    custom_mode.bitfield.reserved = 0;                               // 2 bits (30-31)
    
    return custom_mode;
}

// ======================= MESSAGE PROCESSING =======================
/**
 * @brief Process received UART byte (call from UART interrupt)
 * @param byte Received byte
 */
void Mavlink_UartRxByte(uint8_t byte) {
    switch (mavlink_state.rx_state) //TODO OSAV REplace with DMA approach
    {
        case MAVLINK_RX_IDLE:
            if (byte == MAVLINK_V2_MAGIC) {
                mavlink_rx_buffer[0] = byte;
                mavlink_state.rx_index = 1;
                mavlink_state.rx_state = MAVLINK_RX_HEADER;
            }
            break;
            
        case MAVLINK_RX_HEADER:
            mavlink_rx_buffer[mavlink_state.rx_index++] = byte;
            
            // Check if we have complete header
            if (mavlink_state.rx_index >= MAVLINK_V2_HEADER_SIZE) {
                mavlink_state.payload_length = mavlink_rx_buffer[MAVLINK_V2_PAYLOAD_LEN_INDEX];
                mavlink_state.expected_length = MAVLINK_V2_HEADER_SIZE + mavlink_state.payload_length + MAVLINK_V2_CRC_SIZE;
                
                // Validate payload length
                if (mavlink_state.payload_length <= MAVLINK_MAX_PAYLOAD_SIZE) {
                    mavlink_state.rx_state = MAVLINK_RX_PAYLOAD;
                } else {
                    // Invalid payload length - check if current byte could be start of new message
                    if (byte == MAVLINK_V2_MAGIC) {
                        // Start new message
                        mavlink_rx_buffer[0] = byte;
                        mavlink_state.rx_index = 1;
                        mavlink_state.rx_state = MAVLINK_RX_HEADER;
                    } else {
                        // Reset to search for new message
                        mavlink_state.rx_state = MAVLINK_RX_IDLE;
                        mavlink_state.rx_index = 0;
                    }
                }
            }
            break;
            
        case MAVLINK_RX_PAYLOAD:
            mavlink_rx_buffer[mavlink_state.rx_index++] = byte;
            
            // Check if we have complete message
            if (mavlink_state.rx_index >= mavlink_state.expected_length) {
                Mavlink_ProcessReceivedMessage();
                mavlink_state.rx_state = MAVLINK_RX_IDLE;
                mavlink_state.rx_index = 0;
            }
            
            // Prevent buffer overflow
            if (mavlink_state.rx_index >= MAVLINK_MAX_MESSAGE_SIZE) {
                // Buffer overflow - check if current byte could be start of new message
                if (byte == MAVLINK_V2_MAGIC) {
                    // Start new message
                    mavlink_rx_buffer[0] = byte;
                    mavlink_state.rx_index = 1;
                    mavlink_state.rx_state = MAVLINK_RX_HEADER;
                } else {
                    // Reset to search for new message
                    mavlink_state.rx_state = MAVLINK_RX_IDLE;
                    mavlink_state.rx_index = 0;
                }
            }
            break;
    }
}

/**
 * @brief Process complete received Mavlink message
 */
static void Mavlink_ProcessReceivedMessage(void) {
    // Verify CRC
    uint16_t received_crc = mavlink_rx_buffer[mavlink_state.expected_length - 2] |
                           (mavlink_rx_buffer[mavlink_state.expected_length - 1] << 8);
    uint16_t calculated_crc = Mavlink_CalculateCrc(mavlink_rx_buffer, mavlink_state.expected_length - 2);
    
    if (received_crc != calculated_crc) {
        return;  // Invalid CRC, discard message
    }
    
    // Extract message ID
    uint32_t msg_id = mavlink_rx_buffer[MAVLINK_V2_MSG_ID_LOW_INDEX] | 
                     (mavlink_rx_buffer[MAVLINK_V2_MSG_ID_MID_INDEX] << 8) | 
                     (mavlink_rx_buffer[MAVLINK_V2_MSG_ID_HIGH_INDEX] << 16);
    
    // Extract source system and component (not used for filtering per requirements)
    uint8_t source_system = mavlink_rx_buffer[MAVLINK_V2_SYSTEM_ID_INDEX];
    uint8_t source_component = mavlink_rx_buffer[MAVLINK_V2_COMPONENT_ID_INDEX];
    
    // Update connection status for any incoming message (autopilot connection)
    if (!mavlink_state.connected) {
        mavlink_state.connected = 1;
        // Notify application about autopilot connection
        if (mavlink_state.system_callback) {
            mavlink_state.system_callback(SYSTEM_EVT_READY, MAVLINK_EVT_AUTOPILOT_CONNECTED);
        }
    }
    
    // Reset autopilot connection timeout timer for ANY received message
    Timer_Start(MAVLINK_AUTOPILOT_CONNECTION_TIMEOUT_TMR, MAVLINK_CONNECTION_TIMEOUT_MS, Mavlink_AutopilotConnectionTimeoutCallback);
    
    // Process message based on ID
    const uint8_t* payload = &mavlink_rx_buffer[MAVLINK_V2_PAYLOAD_START_INDEX];
    
    switch (msg_id) {
        case MAVLINK_MSG_ID_HEARTBEAT:
            Mavlink_ProcessAutopilotHeartbeat(payload);
            break;
            
        case MAVLINK_MSG_ID_VFR_HUD:
            Mavlink_ProcessVfrHud(payload);
            break;
            
        case MAVLINK_MSG_ID_COMMAND_LONG:
            Mavlink_ProcessCommandLong(payload);
            break;
            
        default:
            // Unknown message, ignore
            break;
    }
}

/**
 * @brief Process Autopilot HEARTBEAT message
 * @param payload Message payload
 */
static void Mavlink_ProcessAutopilotHeartbeat(const uint8_t* payload) {
    // Extract type and autopilot for logging/debugging if needed
    uint8_t type = payload[HEARTBEAT_TYPE_INDEX];
    uint8_t autopilot = payload[HEARTBEAT_AUTOPILOT_INDEX];
    
    // Extract custom mode (4 bytes) - contains ARM/PREARM states from ESP32 GPIO
    uint32_t custom_mode_raw = payload[HEARTBEAT_CUSTOM_MODE_INDEX] | 
                              (payload[HEARTBEAT_CUSTOM_MODE_INDEX + 1] << 8) | 
                              (payload[HEARTBEAT_CUSTOM_MODE_INDEX + 2] << 16) | 
                              (payload[HEARTBEAT_CUSTOM_MODE_INDEX + 3] << 24);
    
    // Decode ARM/PREARM states from 32-bit custom mode
    mavlink_autopilot_states_t autopilot_states;
    autopilot_states.raw = custom_mode_raw;
    
    uint32_t new_arm_state = autopilot_states.bitfield.arm_state;        // Bits 0-3 - auto-extracted by compiler
    uint32_t new_prearm_state = autopilot_states.bitfield.prearm_state;  // Bits 4-7 - auto-extracted by compiler
    
    // Check for ARM state changes and notify application with specific events
    if (new_arm_state != mavlink_state.autopilot_arm_state) {
        mavlink_state.autopilot_arm_state = new_arm_state;
        if (mavlink_state.system_callback) {
            if (new_arm_state == MAVLINK_AUTOPILOT_ARM_ARMED) {
                mavlink_state.system_callback(SYSTEM_EVT_READY, MAVLINK_EVT_AUTOPILOT_ARMED);
            } else if (new_arm_state == MAVLINK_AUTOPILOT_ARM_DISARMED) {
                mavlink_state.system_callback(SYSTEM_EVT_READY, MAVLINK_EVT_AUTOPILOT_DISARMED);
            }
        }
    }
    
    // Check for PREARM state changes and notify application with specific events
    if (new_prearm_state != mavlink_state.autopilot_prearm_state) {
        mavlink_state.autopilot_prearm_state = new_prearm_state;
        if (mavlink_state.system_callback) {
            if (new_prearm_state == MAVLINK_AUTOPILOT_PREARM_ENABLED) {
                mavlink_state.system_callback(SYSTEM_EVT_READY, MAVLINK_EVT_AUTOPILOT_PREARM_ENABLED);
            } else if (new_prearm_state == MAVLINK_AUTOPILOT_PREARM_DISABLED) {
                mavlink_state.system_callback(SYSTEM_EVT_READY, MAVLINK_EVT_AUTOPILOT_PREARM_DISABLED);
            }
        }
    }
    
    // Autopilot connection timeout timer is already restarted in Mavlink_ProcessReceivedMessage()
    // when any message from autopilot is received, including this heartbeat
    
    // Notify application about autopilot HEARTBEAT received
    if (mavlink_state.system_callback) {
        mavlink_state.system_callback(SYSTEM_EVT_READY, MAVLINK_EVT_AUTOPILOT_HEARTBEAT);
    }
}

/**
 * @brief Process VFR_HUD message from autopilot
 * @param payload Message payload (20 bytes)
 */
static void Mavlink_ProcessVfrHud(const uint8_t* payload) {
    // Extract VFR_HUD data using integer conversion (no FPU on STM32G0)
    mavlink_state.vfr_hud_data.airspeed_cm_s = Mavlink_FloatToIntCmS(&payload[VFR_HUD_AIRSPEED_INDEX]);
    mavlink_state.vfr_hud_data.groundspeed_cm_s = Mavlink_FloatToIntCmS(&payload[VFR_HUD_GROUNDSPEED_INDEX]);
    mavlink_state.vfr_hud_data.altitude_cm = Mavlink_FloatToIntCm(&payload[VFR_HUD_ALT_INDEX]);
    mavlink_state.vfr_hud_data.climb_rate_cm_s = Mavlink_FloatToIntCmS_Signed(&payload[VFR_HUD_CLIMB_INDEX]);
    
    // Extract heading (int16_t)
    mavlink_state.vfr_hud_data.heading_deg = payload[VFR_HUD_HEADING_INDEX] | (payload[VFR_HUD_HEADING_INDEX + 1] << 8);
    
    // Extract throttle (uint16_t)
    mavlink_state.vfr_hud_data.throttle_percent = payload[VFR_HUD_THROTTLE_INDEX] | (payload[VFR_HUD_THROTTLE_INDEX + 1] << 8);
    
    // Notify application about VFR_HUD data received
    if (mavlink_state.system_callback) {
        mavlink_state.system_callback(SYSTEM_EVT_READY, MAVLINK_EVT_VFR_HUD_RECEIVED);
    }
}

/**
 * @brief Process COMMAND_LONG message
 * @param payload Message payload (33 bytes)
 */
static void Mavlink_ProcessCommandLong(const uint8_t* payload) {
    // Extract main command ID (bytes 28-29) - should be MAV_CMD_USER_1 for custom commands
    uint16_t command_id = payload[28] | (payload[29] << 8);
    
    // Extract target system and component  
    uint8_t target_system = payload[30];
    uint8_t target_component = payload[31];
    
    // Check if command is for us (ignore system/component ID checks as per requirements)
    uint8_t result = MAV_RESULT_UNSUPPORTED;
    
    // Process custom commands via MAV_CMD_USER_1
    if (command_id == 31010) {  // MAV_CMD_USER_1
        // Extract custom command type from param1 first byte
        uint8_t custom_command_type = payload[0];  // First byte of param1 - command type
        uint8_t command_data = payload[1];         // Second byte of param1 - command data
        uint8_t custom_param2 = payload[2];        // Third byte of param1 (reserved)
        uint8_t custom_param3 = payload[3];        // Fourth byte of param1 (reserved)
        
        // Process custom command based on type
        switch (custom_command_type) {
            case MAVLINK_CMD_IGNITION:  // 1 - IGNITION command
                // Process IGNITION command via callback
                // command_data can contain ignition type, delay, or other parameters
                if (mavlink_state.system_callback) {
                    mavlink_state.system_callback(SYSTEM_EVT_READY, MAVLINK_EVT_COMMAND_IGNITION);
                    result = MAV_RESULT_ACCEPTED;
                }
                break;
                
            // Future custom commands can be added here:
            // case 2:  // Custom command type 2
            //     // command_data specific to command type 2
            //     break;
            // case 3:  // Custom command type 3
            //     // command_data specific to command type 3
            //     break;
                
            default:
                result = MAV_RESULT_UNSUPPORTED;
                break;
        }
    }
    
    // Send acknowledgment with the main command_id (MAV_CMD_USER_1)
    Mavlink_SendCommandAck(command_id, result);
}

/**
 * @brief Send COMMAND_ACK message
 * @param command Command ID that was processed
 * @param result Result code
 */
static void Mavlink_SendCommandAck(uint16_t command, uint8_t result) {
    uint8_t packet[15] = {0};  // Mavlink v2.0 header + 3 bytes payload + CRC
    
    // Header (10 bytes)
    packet[MAVLINK_V2_MAGIC_INDEX] = MAVLINK_V2_MAGIC;
    packet[MAVLINK_V2_PAYLOAD_LEN_INDEX] = COMMAND_ACK_PAYLOAD_SIZE;                 // Payload length
    packet[MAVLINK_V2_INCOMPAT_FLAGS_INDEX] = 0;                                     // Incompat flags
    packet[MAVLINK_V2_COMPAT_FLAGS_INDEX] = 0;                                       // Compat flags
    packet[MAVLINK_V2_SEQUENCE_INDEX] = Mavlink_GetSequenceNumber();                 // Sequence
    packet[MAVLINK_V2_SYSTEM_ID_INDEX] = mavlink_state.system_id;                    // System ID
    packet[MAVLINK_V2_COMPONENT_ID_INDEX] = mavlink_state.component_id;              // Component ID
    packet[MAVLINK_V2_MSG_ID_LOW_INDEX] = MAVLINK_MSG_ID_COMMAND_ACK & 0xFF;         // Message ID (low byte)
    packet[MAVLINK_V2_MSG_ID_MID_INDEX] = (MAVLINK_MSG_ID_COMMAND_ACK >> 8) & 0xFF; // Message ID (mid byte)
    packet[MAVLINK_V2_MSG_ID_HIGH_INDEX] = (MAVLINK_MSG_ID_COMMAND_ACK >> 16) & 0xFF; // Message ID (high byte)
    
    // Payload (3 bytes)
    packet[MAVLINK_V2_PAYLOAD_START_INDEX + COMMAND_ACK_COMMAND_INDEX] = command & 0xFF;        // Command (low byte)
    packet[MAVLINK_V2_PAYLOAD_START_INDEX + COMMAND_ACK_COMMAND_INDEX + 1] = (command >> 8) & 0xFF; // Command (high byte)
    packet[MAVLINK_V2_PAYLOAD_START_INDEX + COMMAND_ACK_RESULT_INDEX] = result;                 // Result
    
    // Calculate and add CRC16
    uint16_t crc = Mavlink_CalculateCrc(packet, MAVLINK_V2_HEADER_SIZE + COMMAND_ACK_PAYLOAD_SIZE);
    packet[MAVLINK_V2_HEADER_SIZE + COMMAND_ACK_PAYLOAD_SIZE] = crc & 0xFF;
    packet[MAVLINK_V2_HEADER_SIZE + COMMAND_ACK_PAYLOAD_SIZE + 1] = (crc >> 8) & 0xFF;
    
    // Send packet
    Mavlink_SendPacket(packet, 15);
}

// ======================= PERIODIC FUNCTIONS =======================
/**
 * @brief Main periodic processing function (call from main loop)
 */
void Mavlink_Process(void) {
    // Check InitBoard heartbeat send flag (set by timer callback)
    if (mavlink_state.initboard_heartbeat_send_flag) {
        mavlink_state.initboard_heartbeat_send_flag = 0;
        Mavlink_SendInitBoardHeartbeat();
    }
    
    // Check autopilot connection timeout flag (set by timer callback)
    if (mavlink_state.autopilot_connection_timeout_flag) {
        mavlink_state.autopilot_connection_timeout_flag = 0;
        if (mavlink_state.connected) {
            mavlink_state.connected = 0;
            // Notify application about autopilot disconnection via callback
            if (mavlink_state.system_callback) {
                mavlink_state.system_callback(SYSTEM_EVT_READY, MAVLINK_EVT_AUTOPILOT_DISCONNECTED);
            }
        }
    }
}

// ======================= TIMER CALLBACKS =======================
/**
 * @brief InitBoard heartbeat timer callback - called every 1000ms
 * @param tmr_id Timer ID
 */
static void Mavlink_InitBoardHeartbeatTimerCallback(uint8_t tmr_id) {
    // Set flag to send InitBoard heartbeat in main processing function
    mavlink_state.initboard_heartbeat_send_flag = 1;
    
    // Restart timer for next InitBoard heartbeat
    Timer_Start(MAVLINK_INITBOARD_HEARTBEAT_TMR, MAVLINK_INITBOARD_HEARTBEAT_INTERVAL_MS, Mavlink_InitBoardHeartbeatTimerCallback);
}

/**
 * @brief Autopilot connection timeout callback - called after 3000ms of no autopilot messages
 * @param tmr_id Timer ID
 */
static void Mavlink_AutopilotConnectionTimeoutCallback(uint8_t tmr_id) {
    // Set flag to handle autopilot connection timeout in main processing function
    mavlink_state.autopilot_connection_timeout_flag = 1;
    // Note: Timer is not restarted - it will be restarted when we receive next autopilot message
}

// ======================= PUBLIC GETTER FUNCTIONS =======================
/**
 * @brief Get autopilot ARM state (from last received HEARTBEAT)
 * @return uint8_t ARM state (0=DISARMED, 1=ARMED, 0xFF=unknown)
 */
uint8_t Mavlink_GetAutopilotArmState(void) {
    return mavlink_state.autopilot_arm_state;
}

/**
 * @brief Get autopilot PREARM state (from last received HEARTBEAT)
 * @return uint8_t PREARM state (0=PREARM_DISABLED, 1=PREARM_ENABLED, 0xFF=unknown)
 */
uint8_t Mavlink_GetAutopilotPrearmState(void) {
    return mavlink_state.autopilot_prearm_state;
}

/**
 * @brief Get last received VFR_HUD data
 * @return const mavlink_vfr_hud_data_t* Pointer to VFR_HUD data structure
 */
const mavlink_vfr_hud_data_t* Mavlink_GetVfrHudData(void) {
    return &mavlink_state.vfr_hud_data;
}

// ======================= UTILITY FUNCTIONS =======================
/**
 * @brief Send packet via UART
 * @param packet Packet data
 * @param length Packet length
 */
static void Mavlink_SendPacket(uint8_t* packet, uint16_t length) {
    if (length > MAVLINK_MAX_MESSAGE_SIZE || length > 255) return;
    
    // Copy to TX buffer
    memcpy(mavlink_tx_buffer, packet, length);
    
    // Send using public UART function
    bool success = UartSendData(mavlink_tx_buffer, (uint8_t)length);
    
    // Since we don't have TX complete callback, we assume transmission is successful
    // and don't use mavlink_tx_busy flag for blocking
    (void)success;  // Suppress unused variable warning
}

/**
 * @brief Calculate Mavlink CRC16
 * @param data Data to calculate CRC for
 * @param len Data length
 * @return uint16_t Calculated CRC
 */
static uint16_t Mavlink_CalculateCrc(const uint8_t* data, uint8_t len) {
    uint16_t crc = 0xFFFF;
    
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i] << 8;
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    
    return crc;
}

/**
 * @brief Get next sequence number
 * @return uint8_t Sequence number
 */
static uint8_t Mavlink_GetSequenceNumber(void) {
    return mavlink_sequence_number++;
}

// ======================= VFR_HUD CONVERSION FUNCTIONS =======================
/**
 * @brief Convert float bytes to cm/s (integer, positive values only)
 * @param float_bytes Pointer to 4-byte float in little-endian format
 * @return uint16_t Value in cm/s
 */
static uint16_t Mavlink_FloatToIntCmS(const uint8_t* float_bytes) {
    // Simple conversion assuming reasonable aviation speeds (0-500 m/s)
    // For accurate conversion, implement IEEE 754 float parsing
    // This is a simplified approximation for demo purposes
    
    union {
        uint32_t i;
        float f;
    } converter;
    
    converter.i = float_bytes[0] | (float_bytes[1] << 8) | (float_bytes[2] << 16) | (float_bytes[3] << 24);
    
    // Convert m/s to cm/s (multiply by 100) with bounds checking
    if (converter.f < 0.0f) return 0;
    if (converter.f > 500.0f) return 50000;  // Max 500 m/s = 50000 cm/s
    
    return (uint16_t)(converter.f * 100.0f);
}

/**
 * @brief Convert float bytes to cm (integer, signed values)
 * @param float_bytes Pointer to 4-byte float in little-endian format
 * @return int32_t Value in cm
 */
static int32_t Mavlink_FloatToIntCm(const uint8_t* float_bytes) {
    union {
        uint32_t i;
        float f;
    } converter;
    
    converter.i = float_bytes[0] | (float_bytes[1] << 8) | (float_bytes[2] << 16) | (float_bytes[3] << 24);
    
    // Convert m to cm (multiply by 100) with bounds checking
    if (converter.f < -100000.0f) return -10000000;  // Min -100km
    if (converter.f > 100000.0f) return 10000000;    // Max 100km
    
    return (int32_t)(converter.f * 100.0f);
}

/**
 * @brief Convert float bytes to cm/s (integer, signed values for climb rate)
 * @param float_bytes Pointer to 4-byte float in little-endian format
 * @return int16_t Value in cm/s
 */
static int16_t Mavlink_FloatToIntCmS_Signed(const uint8_t* float_bytes) {
    union {
        uint32_t i;
        float f;
    } converter;
    
    converter.i = float_bytes[0] | (float_bytes[1] << 8) | (float_bytes[2] << 16) | (float_bytes[3] << 24);
    
    // Convert m/s to cm/s (multiply by 100) with bounds checking
    if (converter.f < -100.0f) return -10000;  // Min -100 m/s = -10000 cm/s
    if (converter.f > 100.0f) return 10000;    // Max 100 m/s = 10000 cm/s
    
    return (int16_t)(converter.f * 100.0f);
}
#endif /*  MAVLINK_V2_CTRL_SUPP */