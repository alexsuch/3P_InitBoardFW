#pragma once

#include <init_brd.h>
#include <stdbool.h>
#include <stdint.h>

// MSP Protocol feature toggles
#define MSP_V2_ENABLE 0  // Set to 1 to enable MSP v2 support

#if MSP_V2_ENABLE
// MSP v2 Protocol constants
#define MSP_V2_HEADER_SIZE 5
#define MSP_V2_CRC_SIZE 2
#endif  // MSP_V2_ENABLE

#define MSP_V2_MAX_PAYLOAD_SIZE 255

// MSP command IDs
#define MSP_SET_NAME 11

// MSP_SET_NAME specific limits (16 characters for compact display)
#define MSP_SET_NAME_MAX_LENGTH 16

// OSD Message formatting constants
#define OSD_MAX_MESSAGE_LENGTH 16       // Maximum OSD message length (chars)
#define OSD_MIN_BUFFER_SIZE 17          // Minimum buffer size (16 + null terminator)
#define OSD_TIMER_MAX_VALUE 999         // Maximum timer value (3 digits)
#define OSD_TIMER_DIGITS 3              // Number of digits for timer display
#define OSD_TIMER_STRING_LENGTH 6       // Timer string length "S:030s" = 6 chars
#define OSD_MIN_SPACES_FOR_ALIGNMENT 1  // Minimum spaces needed for alignment
#define OSD_BOOM_MESSAGE_LENGTH 7       // "BOOM!!!" length
#define OSD_ERROR_PREFIX_LENGTH 5       // "ERR: " prefix length
#define OSD_ERROR_BLINK_SYMBOL "_"      // Symbol used for error blinking effect

// OSD Message LUT (Lookup Table) indices
typedef enum {
    OSD_MSG_BOOM = 0,
    OSD_MSG_ERR_LOW_BATTERY,
    OSD_MSG_ERR_CHEKA_FAIL,
    OSD_MSG_ERR_SERVO_FAIL,
    OSD_MSG_ERR_VUSA_FAIL,
    OSD_MSG_ERR_UNKNOWN,
    OSD_MSG_ERR_FALLBACK,
    OSD_MSG_DISARM,
    OSD_MSG_SAFE_TMR,
    OSD_MSG_CHARGE,
    OSD_MSG_ARMED,
    OSD_MSG_MINING,
    OSD_MSG_CHEKA,
    OSD_MSG_INIT,
    OSD_MSG_BATTERY_PREFIX,
    OSD_MSG_BATTERY_SUFFIX,
    OSD_MSG_TIMER_SAFE_PREFIX,
    OSD_MSG_TIMER_SAFE_SUFFIX,
    OSD_MSG_TIMER_DESTROY_PREFIX,
    OSD_MSG_TIMER_DESTROY_SUFFIX,
    OSD_MSG_TIMER_MINING_PREFIX,
    OSD_MSG_TIMER_MINING_SUFFIX,
    OSD_MSG_LB_DEST,
    OSD_MSG_TIMER_SEPARATOR,
    OSD_MSG_COUNT  // Must be last - used for bounds checking
} osd_message_index_t;

// OSD Message LUT - preformatted strings
extern const char* const OSD_MESSAGE_LUT[OSD_MSG_COUNT];

#if MSP_V2_ENABLE
// MSP v2 packet structure (internal use only)
typedef struct {
    uint8_t header[MSP_V2_HEADER_SIZE];  // $M< + direction + size
    uint8_t payload[MSP_V2_MAX_PAYLOAD_SIZE];
    uint16_t payload_size;
    uint16_t crc;
} msp_v2_packet_t;
#endif  // MSP_V2_ENABLE

typedef struct {
    bool update_status;
    bool error_blink_state;  // Flag for error message blinking (true = show error, false = show empty)

    uint8_t tx_buffer[64];
    uint16_t packet_size;

    // Callback function and system state pointer
    app_ext_cbk_fn system_callback;
    init_board_system_info_t* system_info;  // Pointer to system state data
} mspOsdStatus_t;

#if MSP_V2_ENABLE
/**
 * @brief Encode MSP v2 packet with bomb status information and return ready-to-send buffer
 *
 * @param armed 1 if armed, 0 if disarmed
 * @param safeTime Safe time in seconds (if > 0, shows safe mode)
 * @param selfDestroyTime Self-destruct time in minutes (if safeTime = 0)
 * @param buffer Output buffer for the complete MSP packet
 * @param buffer_size Size of the output buffer
 * @param packet_size Output parameter for the actual packet size
 * @return true if encoding successful, false otherwise
 * @note The generated status string must not exceed 16 characters (MSP_SET_NAME limit)
 */
bool msp_osd_encode_bomb_status_v2(uint8_t armed, uint16_t safeTime, uint16_t selfDestroyTime, uint8_t* buffer, uint16_t buffer_size, uint16_t* packet_size);

/**
 * @brief Calculate MSP v2 CRC16 checksum
 *
 * @param data Data to calculate CRC for
 * @param length Length of data
 * @return CRC16 checksum
 */
uint16_t msp_osd_calculate_crc16_v2(const uint8_t* data, uint16_t length);

/**
 * @brief Validate MSP v2 packet structure and CRC
 *
 * @param packet Complete MSP v2 packet buffer
 * @param packet_size Size of the packet
 * @return true if packet is valid, false otherwise
 */
bool msp_osd_validate_packet_v2(const uint8_t* packet, uint16_t packet_size);
#endif  // MSP_V2_ENABLE

/**
 * @brief Calculate MSP v1 XOR checksum (for compatibility with Reefwing-MSP)
 *
 * @param data Data to calculate checksum for
 * @param length Length of data
 * @return XOR checksum
 */
uint8_t msp_osd_calculate_xor_checksum_v1(const uint8_t* data, uint16_t length);

/**
 * @brief Encode MSP v1 packet with OSD status message based on system_info
 *
 * This function creates MSP v1 packets using system_info data according to
 * OSD_Message_Rules.md formatting guidelines. It automatically formats
 * messages based on system state, errors, and active timers.
 *
 * @param system_info Pointer to system information structure
 * @param buffer Output buffer for the complete MSP packet
 * @param buffer_size Size of the output buffer
 * @param packet_size Output parameter for the actual packet size
 * @return true if encoding successful, false otherwise
 */
bool msp_osd_encode_bomb_status_v1(const init_board_system_info_t* system_info, uint8_t* buffer, uint16_t buffer_size, uint16_t* packet_size);

/**
 * @brief OSD MSP task to handle periodic updates
 */
void OsdMsp_Task(void);
/**
 * @brief Initialize OSD MSP module
 * @param system_cbk Callback function to get system state
 * @param system_info Pointer to system state structure
 */
void OsdMsp_Init(app_ext_cbk_fn system_cbk, init_board_system_info_t* system_info);