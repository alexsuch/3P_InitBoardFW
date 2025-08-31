/*
 * mavlink_uart.c - STM32G0 Mavlink UART Communication Module
 * 
 * This module handles Mavlink v2.0 communication via UART for the STM32G0-based InitBoard.
 * It processes incoming GCS commands and sends InitBoard HEARTBEAT messages with custom mode bitfield.
 * 
 * Features:
 * - Integer-only operations (no floating point)
 * - Custom mode bitfield encoding (32-bit)
 * - Command processing and acknowledgment
 * - UART interrupt-driven communication
 * - VFR_HUD message processing with float-to-integer conversion
 */

#include "solution_wrapper.h"
#include "mavlink_uart.h"
#include "prj_config.h"
#include "app.h"
#include "timer.h"
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
static void Mavlink_ProcessGcsHeartbeat(const uint8_t* payload);
static void Mavlink_ProcessVfrHud(const uint8_t* payload);
static void Mavlink_ProcessCommandLong(const uint8_t* payload);
static void Mavlink_SendCommandAck(uint16_t command, uint8_t result);
static mavlink_custom_mode_t Mavlink_EncodeCustomMode(void);
static void Mavlink_DecodeVfrHudFloats(const uint8_t* float_data, mavlink_vfr_hud_data_t* hud_data);
static void Mavlink_SendInitBoardHeartbeat(void);
static void Mavlink_ProcessReceivedMessage(void);

// Timer callback functions
static void Mavlink_InitBoardHeartbeatTimerCallback(uint8_t tmr_id);
static void Mavlink_GcsConnectionTimeoutCallback(uint8_t tmr_id);

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
    mavlink_state.component_id = MAVLINK_COMP_ID_USER1;
    mavlink_state.connected = 0;
    mavlink_state.rx_state = MAVLINK_RX_IDLE;
    mavlink_state.rx_index = 0;
    mavlink_state.system_callback = system_cbk;
    mavlink_state.system_info = system_info;  // Store pointer to system state
    mavlink_state.initboard_heartbeat_send_flag = 0;
    mavlink_state.gcs_connection_timeout_flag = 0;
    
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
    packet[0] = MAVLINK_V2_MAGIC;                    // Magic byte
    packet[1] = 9;                                   // Payload length
    packet[2] = 0;                                   // Incompat flags
    packet[3] = 0;                                   // Compat flags
    packet[4] = Mavlink_GetSequenceNumber();         // Sequence
    packet[5] = mavlink_state.system_id;             // System ID
    packet[6] = mavlink_state.component_id;          // Component ID
    packet[7] = MAVLINK_MSG_ID_HEARTBEAT & 0xFF;     // Message ID (low byte)
    packet[8] = (MAVLINK_MSG_ID_HEARTBEAT >> 8) & 0xFF;  // Message ID (mid byte)
    packet[9] = (MAVLINK_MSG_ID_HEARTBEAT >> 16) & 0xFF; // Message ID (high byte)
    
    // Payload (9 bytes)
    packet[10] = MAV_TYPE_GENERIC;                   // Type
    packet[11] = MAV_AUTOPILOT_INVALID;              // Autopilot
    packet[12] = 0;                                  // Base mode
    
    // Custom mode (4 bytes) - our bitfield structure
    packet[13] = custom_mode.raw & 0xFF;
    packet[14] = (custom_mode.raw >> 8) & 0xFF;
    packet[15] = (custom_mode.raw >> 16) & 0xFF;
    packet[16] = (custom_mode.raw >> 24) & 0xFF;
    
    packet[17] = MAV_STATE_ACTIVE;                   // System status
    packet[18] = MAVLINK_VERSION;                    // Mavlink version
    
    // Calculate and add CRC16
    uint16_t crc = Mavlink_CalculateCrc(packet, 19);
    packet[19] = crc & 0xFF;
    packet[20] = (crc >> 8) & 0xFF;
    
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
    
    // Encode bitfield (matching REQUIREMENTS.md specification)
    custom_mode.bitfield.timer_sec = state->timer_seconds & 0x3FFF;           // 14 bits (0-13)
    custom_mode.bitfield.timer_mode = state->timer_mode & 0x03;               // 2 bits (14-15)
    custom_mode.bitfield.fuse_present = state->fuse_present & 0x01;           // 1 bit (16)
    custom_mode.bitfield.board_state = state->board_state & 0x07;             // 3 bits (17-19)
    custom_mode.bitfield.battery_level = state->battery_level & 0x0F;         // 4 bits (20-23)
    custom_mode.bitfield.error_code = state->error_code & 0x0F;               // 4 bits (24-27)
    custom_mode.bitfield.reserved = 0;                                        // 4 bits (28-31)
    
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
            
            // Check if we have complete header (10 bytes)
            if (mavlink_state.rx_index >= 10) {
                mavlink_state.payload_length = mavlink_rx_buffer[1];
                mavlink_state.expected_length = 10 + mavlink_state.payload_length + 2; // Header + Payload + CRC
                
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
    uint32_t msg_id = mavlink_rx_buffer[7] | 
                     (mavlink_rx_buffer[8] << 8) | 
                     (mavlink_rx_buffer[9] << 16);
    
    // Extract source system and component
    uint8_t source_system = mavlink_rx_buffer[5];
    //uint8_t source_component = mavlink_rx_buffer[6];
    
    // Update connection status if message from GCS
    if (source_system == MAVLINK_SYSTEM_ID_GCS) {
        if (!mavlink_state.connected) {
            mavlink_state.connected = 1;
            // Notify application about GCS connection
            if (mavlink_state.system_callback) {
                mavlink_state.system_callback(SYSTEM_EVT_READY, MAVLINK_EVT_GCS_CONNECTED);
            }
        }
        
        // Reset GCS connection timeout timer every time we receive ANY message from GCS
        Timer_Start(MAVLINK_GCS_CONNECTION_TIMEOUT_TMR, MAVLINK_CONNECTION_TIMEOUT_MS, Mavlink_GcsConnectionTimeoutCallback);
    }
    
    // Process message based on ID
    const uint8_t* payload = &mavlink_rx_buffer[10];
    
    switch (msg_id) {
        case MAVLINK_MSG_ID_HEARTBEAT:
        	//HAL_GPIO_TogglePin(LED_YELLOW_OUT_GPIO_Port, LED_YELLOW_OUT_Pin);
            Mavlink_ProcessGcsHeartbeat(payload);
            break;
            
        case MAVLINK_MSG_ID_VFR_HUD:
        	//HAL_GPIO_TogglePin(LED_YELLOW_OUT_GPIO_Port, LED_YELLOW_OUT_Pin);
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
 * @brief Process GCS HEARTBEAT message
 * @param payload Message payload
 */
static void Mavlink_ProcessGcsHeartbeat(const uint8_t* payload) {
    // Extract type and autopilot for logging/debugging if needed
    uint8_t type = payload[0];
    uint8_t autopilot = payload[1];
    uint8_t base_mode = payload[2];
    
    // Extract custom mode (4 bytes at offset 3-6) - ARM/PREARM states from autopilot
    uint32_t custom_mode_raw = payload[3] | (payload[4] << 8) | (payload[5] << 16) | (payload[6] << 24);
    
    // Decode ARM/PREARM states from autopilot custom mode
    uint8_t arm_state = custom_mode_raw & 0x0F;          // Bits 0-3: ARM state
    uint8_t prearm_state = (custom_mode_raw >> 4) & 0x0F; // Bits 4-7: PREARM state
    
    // TODO: Process ARM/PREARM states if needed by application
    // Could call system callback with these states
    
    // GCS connection timeout timer is already restarted in Mavlink_ProcessReceivedMessage()
    // when any message from GCS is received, including this heartbeat
    
    // Could add additional GCS heartbeat-specific processing here if needed
}

/**
 * @brief Process VFR_HUD message
 * @param payload Message payload (20 bytes)
 */
static void Mavlink_ProcessVfrHud(const uint8_t* payload) {
    mavlink_vfr_hud_data_t hud_data = {0};
    
    // Decode float values to integer format
    Mavlink_DecodeVfrHudFloats(payload, &hud_data);
    
    // Notify application via callback with VFR_HUD data
    if (mavlink_state.system_callback) {
        mavlink_state.system_callback(SYSTEM_EVT_READY, MAVLINK_EVT_VFR_HUD_RECEIVED);
        // Note: hud_data can be accessed through a global variable or separate callback parameter
        // For now, we just notify that VFR_HUD was received
    }
}

/**
 * @brief Decode VFR_HUD float data to integer format
 * @param float_data Raw float data from payload
 * @param hud_data Output integer data structure
 */
static void Mavlink_DecodeVfrHudFloats(const uint8_t* float_data, mavlink_vfr_hud_data_t* hud_data) {
    // Use union for float-to-bytes conversion
    union {
        float f;
        uint8_t bytes[4];
    } float_converter;
    
    // Airspeed (bytes 0-3)
    memcpy(float_converter.bytes, &float_data[0], 4);
    hud_data->airspeed_cm_s = (int32_t)(float_converter.f * 100);  // Convert to cm/s
    
    // Groundspeed (bytes 4-7)
    memcpy(float_converter.bytes, &float_data[4], 4);
    hud_data->groundspeed_cm_s = (int32_t)(float_converter.f * 100);  // Convert to cm/s
    
    // Altitude (bytes 8-11)
    memcpy(float_converter.bytes, &float_data[8], 4);
    hud_data->altitude_cm = (int32_t)(float_converter.f * 100);  // Convert to cm
    
    // Climb rate (bytes 12-15)
    memcpy(float_converter.bytes, &float_data[12], 4);
    hud_data->climb_rate_cm_s = (int32_t)(float_converter.f * 100);  // Convert to cm/s
    
    // Heading (bytes 16-17)
    memcpy(&hud_data->heading_deg, &float_data[16], 2);
    
    // Throttle (bytes 18-19)
    memcpy(&hud_data->throttle_percent, &float_data[18], 2);
}

/**
 * @brief Process COMMAND_LONG message
 * @param payload Message payload (33 bytes)
 */
static void Mavlink_ProcessCommandLong(const uint8_t* payload) {
    // Extract command ID (bytes 28-29)
    uint16_t command_id = payload[28] | (payload[29] << 8);
    
    // Extract target system and component
    uint8_t target_system = payload[30];
    uint8_t target_component = payload[31];
    
    // Check if command is for us
    if (target_system != mavlink_state.system_id || 
        target_component != mavlink_state.component_id) {
        return;  // Not for us
    }
    
    uint8_t result = MAV_RESULT_UNSUPPORTED;
    
    // Process specific commands
    if (command_id == MAV_CMD_USER_1) {
        // Extract param1 (contains our custom command)
        union {
            float f;
            uint8_t bytes[4];
        } param1_converter;
        
        memcpy(param1_converter.bytes, &payload[0], 4);
        uint32_t custom_command = *(uint32_t*)&param1_converter.f;  // Union conversion
        
        // Process custom command via callback
        mavlink_event_t event;
        uint8_t command_valid = 1;
        
        switch (custom_command) {
            case MAVLINK_CMD_DISARM:
                event = MAVLINK_EVT_COMMAND_DISARM;
                break;
                
            case MAVLINK_CMD_ARM:
                event = MAVLINK_EVT_COMMAND_ARM;
                break;
                
            case MAVLINK_CMD_IGNITION:
                event = MAVLINK_EVT_COMMAND_IGNITION;
                break;
                
            case MAVLINK_CMD_PREARM_ENABLED:
                event = MAVLINK_EVT_COMMAND_PREARM_ENABLED;
                break;
                
            case MAVLINK_CMD_PREARM_DISABLED:
                event = MAVLINK_EVT_COMMAND_PREARM_DISABLED;
                break;
                
            default:
                command_valid = 0;
                result = MAV_RESULT_UNSUPPORTED;
                break;
        }
        
        if (command_valid && mavlink_state.system_callback) {
            // Call callback to notify application
            mavlink_state.system_callback(SYSTEM_EVT_READY, event);
            result = MAV_RESULT_ACCEPTED;  // Assume command will be processed successfully
        }
    }
    
    // Send acknowledgment
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
    packet[0] = MAVLINK_V2_MAGIC;
    packet[1] = 3;                                   // Payload length
    packet[2] = 0;                                   // Incompat flags
    packet[3] = 0;                                   // Compat flags
    packet[4] = Mavlink_GetSequenceNumber();         // Sequence
    packet[5] = mavlink_state.system_id;             // System ID
    packet[6] = mavlink_state.component_id;          // Component ID
    packet[7] = MAVLINK_MSG_ID_COMMAND_ACK & 0xFF;   // Message ID (low byte)
    packet[8] = (MAVLINK_MSG_ID_COMMAND_ACK >> 8) & 0xFF;  // Message ID (mid byte)
    packet[9] = (MAVLINK_MSG_ID_COMMAND_ACK >> 16) & 0xFF; // Message ID (high byte)
    
    // Payload (3 bytes)
    packet[10] = command & 0xFF;        // Command (low byte)
    packet[11] = (command >> 8) & 0xFF; // Command (high byte)
    packet[12] = result;                // Result
    
    // Calculate and add CRC16
    uint16_t crc = Mavlink_CalculateCrc(packet, 13);
    packet[13] = crc & 0xFF;
    packet[14] = (crc >> 8) & 0xFF;
    
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
    
    // Check GCS connection timeout flag (set by timer callback)
    if (mavlink_state.gcs_connection_timeout_flag) {
        mavlink_state.gcs_connection_timeout_flag = 0;
        if (mavlink_state.connected) {
            mavlink_state.connected = 0;
            // Notify application about GCS disconnection via callback
            if (mavlink_state.system_callback) {
                mavlink_state.system_callback(SYSTEM_EVT_READY, MAVLINK_EVT_GCS_DISCONNECTED);
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
 * @brief GCS connection timeout callback - called after 3000ms of no GCS messages
 * @param tmr_id Timer ID
 */
static void Mavlink_GcsConnectionTimeoutCallback(uint8_t tmr_id) {
    // Set flag to handle GCS connection timeout in main processing function
    mavlink_state.gcs_connection_timeout_flag = 1;
    // Note: Timer is not restarted - it will be restarted when we receive next GCS message
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

#endif 
