#include "msp_osd.h"
#include "main.h"
#include "prj_config.h"
#include "app.h"
#include "timer.h"
#include "solution_wrapper.h"
#include <string.h>

// OSD Message LUT - preformatted strings (defined in header)
const char* const OSD_MESSAGE_LUT[] = {
    [OSD_MSG_BOOM] = "BOOM!!!",
    [OSD_MSG_ERR_LOW_BATTERY] = "ERR: LOW BATTERY",
    [OSD_MSG_ERR_CHEKA_FAIL] = "ERR: CHEKA FAIL",
    [OSD_MSG_ERR_SERVO_FAIL] = "ERR: SERVO FAIL",
    [OSD_MSG_ERR_VUSA_FAIL] = "ERR: VUSA FAIL",
    [OSD_MSG_ERR_UNKNOWN] = "ERR: UNKNOWN",
    [OSD_MSG_ERR_FALLBACK] = "ERR: ERROR",
    [OSD_MSG_DISARM] = "DISARM",
    [OSD_MSG_SAFE_TMR] = "SAFE TMR",
    [OSD_MSG_CHARGE] = "CHARGING",
    [OSD_MSG_ARMED] = "ARMED",
    [OSD_MSG_MINING] = "!MINE ON",
    [OSD_MSG_CHEKA] = "CHEKA ON",
    [OSD_MSG_INIT] = "INIT",
    [OSD_MSG_BATTERY_PREFIX] = "B:",
    [OSD_MSG_BATTERY_SUFFIX] = "%",
    [OSD_MSG_TIMER_SAFE_PREFIX] = "S:",
    [OSD_MSG_TIMER_SAFE_SUFFIX] = "s",
    [OSD_MSG_TIMER_DESTROY_PREFIX] = "D:",
    [OSD_MSG_TIMER_DESTROY_SUFFIX] = "m",
    [OSD_MSG_TIMER_MINING_PREFIX] = "M:",
    [OSD_MSG_TIMER_MINING_SUFFIX] = "s",
    [OSD_MSG_LB_DEST] = "LB DEST",
    [OSD_MSG_TIMER_SEPARATOR] = " "
};

static mspOsdStatus_t mspOsdStatus = {0};

// Versions for 32
//  BOMB:DISARMED SAFE:1000 sec. M:1
//  BOMB:ARMED    DESTR:255 min. M:0
// Version for 16
//  -----------------
//  B:DISA S:1000s. |
//  B:ARM  D:255m.  |
//  -----------------
/**
 * @brief Manual string length calculation
 */
static uint16_t msp_strlen(const char *str) {
    uint16_t len = 0;
    while (str[len] != '\0') {
        len++;
    }
    return len;
}

/**
 * @brief Manual string copy
 */
static void msp_strcpy(char *dest, const char *src) {
    uint16_t i = 0;
    while (src[i] != '\0') {
        dest[i] = src[i];
        i++;
    }
    dest[i] = '\0';
}

/**
 * @brief Manual string concatenation
 */
static void msp_strcat(char *dest, const char *src) {
    uint16_t dest_len = msp_strlen(dest);
    uint16_t i = 0;
    while (src[i] != '\0') {
        dest[dest_len + i] = src[i];
        i++;
    }
    dest[dest_len + i] = '\0';
}

/**
 * @brief Manual memory set
 */
static void msp_memset(void *ptr, uint8_t value, uint16_t size) {
    uint8_t *p = (uint8_t *)ptr;
    for (uint16_t i = 0; i < size; i++) {
        p[i] = value;
    }
}

#if MSP_V2_ENABLE
/**
 * @brief Manual memory copy
 */
static void msp_memcpy(void *dest, const void *src, uint16_t size) {
    uint8_t *d = (uint8_t *)dest;
    const uint8_t *s = (const uint8_t *)src;
    for (uint16_t i = 0; i < size; i++) {
        d[i] = s[i];
    }
}
#endif

#if MSP_V2_ENABLE
/**
 * @brief Convert number to string
 */
static void msp_itoa(uint16_t num, char *str) {
    if (num == 0) {
        str[0] = '0';
        str[1] = '\0';
        return;
    }

    char temp[16];
    uint16_t i = 0;
    while (num > 0) {
        temp[i] = '0' + (num % 10);
        num /= 10;
        i++;
    }

    // Reverse the string
    for (uint16_t j = 0; j < i; j++) {
        str[j] = temp[i - 1 - j];
    }
    str[i] = '\0';
}
#endif

/**
 * @brief Convert number to string with leading zeros (3 digits)
 * Automatically limits number to OSD_TIMER_MAX_VALUE (999)
 */
static void msp_itoa_3digits(uint16_t num, char *str) {
    // Ensure timer value doesn't exceed maximum
    if (num > OSD_TIMER_MAX_VALUE) {
        num = OSD_TIMER_MAX_VALUE;
    }
    
    str[0] = '0' + (num / 100);
    str[1] = '0' + ((num / 10) % 10);
    str[2] = '0' + (num % 10);
    str[3] = '\0';
}



/**
 * @brief Get OSD message from LUT with bounds checking
 * @param index Message index from osd_message_index_t enum
 * @return Pointer to message string or fallback message if index invalid
 */
static const char* get_osd_message(osd_message_index_t index) {
    if (index >= OSD_MSG_COUNT) {
        return OSD_MESSAGE_LUT[OSD_MSG_ERR_FALLBACK];  // Fallback for invalid index
    }
    return OSD_MESSAGE_LUT[index];
}

/**
 * @brief Format timer string with automatic minutes/seconds selection
 * @param timer_str Output buffer for formatted timer string
 * @param timer_seconds Input timer value in seconds
 * @param prefix_index LUT index for timer prefix (e.g., OSD_MSG_TIMER_DESTROY_PREFIX)
 * @param minutes_suffix_index LUT index for minutes suffix (e.g., OSD_MSG_TIMER_DESTROY_SUFFIX)
 * @param seconds_suffix_index LUT index for seconds suffix (e.g., OSD_MSG_TIMER_SAFE_SUFFIX)
 * @return true if formatting successful, false on error
 */
static bool format_timer_string(char* timer_str, uint16_t timer_seconds, 
                               osd_message_index_t prefix_index,
                               osd_message_index_t minutes_suffix_index,
                               osd_message_index_t seconds_suffix_index) {
    if (!timer_str) {
        return false;
    }
    
    // Start with prefix
    msp_strcpy(timer_str, get_osd_message(prefix_index));
    
    if (timer_seconds < 60) {
        // Менше 1 хвилини - показуємо секунди
        uint16_t seconds = timer_seconds;
        if (seconds > OSD_TIMER_MAX_VALUE) {
            seconds = OSD_TIMER_MAX_VALUE;
        }
        char time_str[OSD_TIMER_DIGITS + 1];
        msp_itoa_3digits(seconds, time_str);
        msp_strcat(timer_str, time_str);
        msp_strcat(timer_str, get_osd_message(seconds_suffix_index)); // "s" для секунд
    } else {
        // 1+ хвилина - показуємо хвилини
        uint16_t minutes = timer_seconds / 60;
        if (minutes > OSD_TIMER_MAX_VALUE) {
            minutes = OSD_TIMER_MAX_VALUE;
        }
        char time_str[OSD_TIMER_DIGITS + 1];
        msp_itoa_3digits(minutes, time_str);
        msp_strcat(timer_str, time_str);
        msp_strcat(timer_str, get_osd_message(minutes_suffix_index)); // "m" для хвилин
    }
    
    return true;
}

/**
 * @brief Validate OSD message length and content
 * @param message Message to validate
 * @return true if message meets OSD requirements, false otherwise
 */
static bool validate_osd_message(const char* message) {
    if (!message) {
        return false;
    }
    
    uint16_t len = msp_strlen(message);
    
    // Check length constraint (allow empty strings for blinking effect)
    if (len > OSD_MAX_MESSAGE_LENGTH) {
        return false;
    }
    
    // If empty string, it's valid (for blinking effect)
    if (len == 0) {
        return true;
    }
    
    // Check for valid printable ASCII characters (0x20-0x7E)
    for (uint16_t i = 0; i < len; i++) {
        if (message[i] < 0x20 || message[i] > 0x7E) {
            return false;
        }
    }
    
    return true;
}

/**
 * @brief Format OSD message based on system_info according to OSD_Message_Rules.md
 * 
 * @param system_info Pointer to system information structure
 * @param output_buffer Output buffer for formatted message (min 17 bytes)
 * @param buffer_size Size of output buffer
 * @param error_blink_state Error blinking state (true = show error, false = show empty for blinking effect)
 * @return true if formatting successful, false on error
 */
static bool format_osd_message_from_system_info(const init_board_system_info_t* system_info, 
                                               char* output_buffer, 
                                               uint16_t buffer_size,
                                               bool error_blink_state) {
    // Input validation with proper constants
    if (!system_info || !output_buffer || buffer_size < OSD_MIN_BUFFER_SIZE) {
        return false;
    }
    
    // Clear output buffer
    msp_memset(output_buffer, 0, buffer_size);
    
    // ПРІОРИТЕТ 0: Помилки - найвищий пріоритет з блиманням
    // Блимання помилок: показуємо помилку тільки якщо error_blink_state == true
    if (system_info->error_code != ERR_CODE_NO_ERROR) {
        // Якщо блимання вимкнено (error_blink_state == false), показуємо символ блимання
        if (!error_blink_state) {
            msp_strcpy(output_buffer, OSD_ERROR_BLINK_SYMBOL);  // Символ для ефекту блимання помилок
            return true;
        }
        const char* error_message = NULL;
        
        switch (system_info->error_code) {
            case ERR_CODE_BATTERY_LOW:
                error_message = get_osd_message(OSD_MSG_ERR_LOW_BATTERY);  // 16 символів
                break;
                
            case ERR_CODE_FUSE_INCORRECT_STATE:
                error_message = get_osd_message(OSD_MSG_ERR_CHEKA_FAIL);   // 15 символів
                break;
                
            case ERR_CODE_UNEXPECTED_IGNITION:
            case ERR_CODE_UNEXPECTED_ARM:
            case ERR_CODE_UNEXPECTED_MINING:
                error_message = get_osd_message(OSD_MSG_ERR_SERVO_FAIL);   // 15 символів
                break;
                
            case ERR_CODE_UNEXPECTED_VUSA_SHORTED:
                error_message = get_osd_message(OSD_MSG_ERR_VUSA_FAIL);    // 14 символів
                break;
                
            default:
                error_message = get_osd_message(OSD_MSG_ERR_UNKNOWN);      // 12 символів
                break;
        }
        
        // Перевірка довжини помилки перед копіюванням
        uint16_t error_len = msp_strlen(error_message);
        if (error_len > OSD_MAX_MESSAGE_LENGTH) {
            // Якщо помилка занадто довга, скорочуємо
            msp_strcpy(output_buffer, get_osd_message(OSD_MSG_ERR_FALLBACK));  // 10 символів - fallback
        } else {
            msp_strcpy(output_buffer, error_message);
        }
        
        // Фінальна перевірка довжини
        if (msp_strlen(output_buffer) > OSD_MAX_MESSAGE_LENGTH) {
            return false;
        }
        return true;
    }
    
    // ПРІОРИТЕТ 1: CHEKA - другий пріоритет з відсотками батареї
    if (system_info->fuse_present != 0) {
        // Формування повідомлення: "CHEKA      B:xx%" (16 символів)
        const char* cheka_msg = get_osd_message(OSD_MSG_CHEKA);  // "CHEKA" = 5 символів
        uint16_t cheka_len = msp_strlen(cheka_msg);
        
        // Форматування батареї: "B:xx%" (4-5 символів)
        char battery_str[8];  // "B:100%\0"
        msp_strcpy(battery_str, get_osd_message(OSD_MSG_BATTERY_PREFIX));  // "B:"
        
        // Конвертація battery_level (0-10) в відсотки (0-100%)
        uint16_t battery_percent = system_info->battery_level * 10;
        if (battery_percent > 100) {
            battery_percent = 100;
        }
        
        // Додаємо відсотки
        char percent_str[4];  // "100\0"
        if (battery_percent == 100) {
            msp_strcpy(percent_str, "100");
        } else if (battery_percent >= 10) {
            percent_str[0] = '0' + (battery_percent / 10);
            percent_str[1] = '0' + (battery_percent % 10);
            percent_str[2] = '\0';
        } else {
            percent_str[0] = '0' + battery_percent;
            percent_str[1] = '\0';
        }
        
        msp_strcat(battery_str, percent_str);
        msp_strcat(battery_str, get_osd_message(OSD_MSG_BATTERY_SUFFIX));  // "%"
        
        // Розрахунок пробілів для вирівнювання до 16 символів
        uint16_t battery_len = msp_strlen(battery_str);
        uint16_t total_content_len = cheka_len + battery_len;
        
        if (total_content_len < OSD_MAX_MESSAGE_LENGTH) {
            uint16_t spaces_needed = OSD_MAX_MESSAGE_LENGTH - total_content_len;
            
            // Формування: "CHEKA      B:xx%"
            msp_strcpy(output_buffer, cheka_msg);
            
            // Додаємо пробіли
            for (uint16_t i = 0; i < spaces_needed; i++) {
                msp_strcat(output_buffer, get_osd_message(OSD_MSG_TIMER_SEPARATOR));
            }
            
            // Додаємо батарею
            msp_strcat(output_buffer, battery_str);
            
            // Перевірка фінальної довжини
            if (msp_strlen(output_buffer) == OSD_MAX_MESSAGE_LENGTH) {
                return true;
            }
        }
        
        // Fallback - просто CHEKA без батареї
        msp_strcpy(output_buffer, cheka_msg);
        return true;
    }
    
    // ПРІОРИТЕТ 2: BOOM!!! - третій пріоритет (7 символів)
    if (system_info->board_state == BOARD_STATE_BOOM) {
        msp_strcpy(output_buffer, get_osd_message(OSD_MSG_BOOM));
        
        // Внутрішня перевірка довжини
        if (msp_strlen(output_buffer) > OSD_MAX_MESSAGE_LENGTH) {
            return false;  // Should never happen, but safety check
        }
        return true;
    }
    
    // ПРІОРИТЕТ 3: Нормальні стани з таймерами (використовуємо board_state)
    const char* board_state_message = NULL;
    
    switch (system_info->board_state) {
        case BOARD_STATE_DISCHARGED:
            // Перевіряємо чи активний сейф таймер
            if (system_info->timer_mode == TIMER_MODE_SAFE && system_info->timer_seconds > 0) {
                board_state_message = get_osd_message(OSD_MSG_SAFE_TMR);
            } else {
                board_state_message = get_osd_message(OSD_MSG_DISARM);
            }
            break;
        case BOARD_STATE_CHARGING:
            board_state_message = get_osd_message(OSD_MSG_CHARGE);
            break;
        case BOARD_STATE_CHARGED:
            board_state_message = get_osd_message(OSD_MSG_ARMED);
            break;
        case BOARD_STATE_MINING:
            board_state_message = get_osd_message(OSD_MSG_MINING);
            break;
        default:
            break;
    }
    
    // Спеціальна логіка для MINING з низьким зарядом батареї
    if ((system_info->board_state == BOARD_STATE_MINING) && (system_info->low_pwr_self_dest_allowed != 0)) {
        // Формуємо повідомлення: "!MINE ON LB DEST" (16 символів)
        uint16_t base_len = msp_strlen(board_state_message);  // "!MINE ON" = 8 символів
        const char* lb_dest_msg = get_osd_message(OSD_MSG_LB_DEST);  // "LB DEST" = 7 символів
        uint16_t lb_dest_len = msp_strlen(lb_dest_msg);
        
        // Перевіряємо чи поміститься повідомлення (8 + 1 пробіл + 7 = 16)
        // Використовуємо точно 16 символів: "!MINE ON LB DEST"
        if (base_len + 1 + lb_dest_len <= OSD_MAX_MESSAGE_LENGTH) {
            msp_strcpy(output_buffer, board_state_message);
            msp_strcat(output_buffer, get_osd_message(OSD_MSG_TIMER_SEPARATOR));  // пробіл
            msp_strcat(output_buffer, lb_dest_msg);
            
            // Фінальна перевірка довжини
            uint16_t final_len = msp_strlen(output_buffer);
            if (final_len <= OSD_MAX_MESSAGE_LENGTH) {
                return true;
            }
        }
        
        // Якщо не поміщається, повертаємся до простого стану
        msp_strcpy(output_buffer, board_state_message);
        return true;
    }
    
    // Перевірка активного таймера з system_info
    // CHARGING стан не показує таймери
    bool has_active_timer = (system_info->board_state != BOARD_STATE_CHARGING &&
                            system_info->timer_mode != TIMER_MODE_NONE && 
                            system_info->timer_seconds > 0);
    
    if (has_active_timer) {
        // Комбінувати стан + таймер з system_info
        char timer_str[OSD_TIMER_STRING_LENGTH + 2];  // "S:030s\0" - 6 символів + safety
        bool timer_formatted = false;
        
        // Формування таймера (завжди OSD_TIMER_STRING_LENGTH символів з 3-значним числом)
        switch (system_info->timer_mode) {
            case TIMER_MODE_SAFE:
                // Safe timer - показуємо тільки секунди без префіксу, вирівнюємо по правому краю як інші таймери
                timer_formatted = format_timer_string(timer_str, system_info->timer_seconds,
                                                     OSD_MSG_TIMER_SAFE_PREFIX,
                                                     OSD_MSG_TIMER_DESTROY_SUFFIX,
                                                     OSD_MSG_TIMER_SAFE_SUFFIX);
                // Видаляємо префікс "S:" для SAFE таймера, залишаємо суфікс "s" для правильного вирівнювання
                if (timer_formatted && timer_str[0] == 'S' && timer_str[1] == ':') {
                    // Витягуємо число та суфікс (030s) з рядка "S:030s"
                    char temp_str[5];  // "030s\0"
                    msp_strcpy(temp_str, &timer_str[2]);  // копіюємо "030s"
                    
                    // Формуємо фінальний рядок: "  030s" (2 пробіли + число + суфікс для вирівнювання чисел з D: та M:)
                    msp_strcpy(timer_str, "  ");      // 2 пробіли замість префікса "S:"
                    msp_strcat(timer_str, temp_str);  // додаємо "030s"
                }
                break;
                
            case TIMER_MODE_SELF_DESTROY:
                // Self destroy timer - показуємо секунди якщо < 60s, інакше хвилини
                timer_formatted = format_timer_string(timer_str, system_info->timer_seconds,
                                                     OSD_MSG_TIMER_DESTROY_PREFIX,
                                                     OSD_MSG_TIMER_DESTROY_SUFFIX,
                                                     OSD_MSG_TIMER_SAFE_SUFFIX);
                break;
                
            case TIMER_MODE_MINING:
                // Mining timer - таймер активації мінного режиму, показуємо з префіксом "M:"
                timer_formatted = format_timer_string(timer_str, system_info->timer_seconds,
                                                     OSD_MSG_TIMER_MINING_PREFIX,
                                                     OSD_MSG_TIMER_DESTROY_SUFFIX,
                                                     OSD_MSG_TIMER_MINING_SUFFIX);
                break;
                
            default:
                // Немає активного таймера
                has_active_timer = false;
                timer_formatted = false;
                break;
        }
        
        // Перевірка довжини сформованого таймера
        if (timer_formatted && msp_strlen(timer_str) != OSD_TIMER_STRING_LENGTH) {
            has_active_timer = false;  // Невірна довжина таймера
        }
        
        if (has_active_timer) {
            // Розрахунок пробілів для вирівнювання (OSD_MAX_MESSAGE_LENGTH символів загалом)
            uint16_t base_len = msp_strlen(board_state_message);
            uint16_t timer_len = msp_strlen(timer_str);  // Повинно бути OSD_TIMER_STRING_LENGTH символів
            
            // Перевірка що таймер має правильну довжину
            if (timer_len != OSD_TIMER_STRING_LENGTH) {
                has_active_timer = false;  // Невірна довжина таймера
            } else {
                // Розрахунок необхідних пробілів
                if (base_len + timer_len < OSD_MAX_MESSAGE_LENGTH) {
                    uint16_t spaces_needed = OSD_MAX_MESSAGE_LENGTH - base_len - timer_len;
                    
                    // Перевірка що є мінімум один пробіл для вирівнювання
                    if (spaces_needed >= OSD_MIN_SPACES_FOR_ALIGNMENT) {
                        // Формування комбінованого повідомлення: "ARMED     S:030s"
                        msp_strcpy(output_buffer, board_state_message);
                        
                        // Додаємо пробіли для вирівнювання
                        for (uint16_t i = 0; i < spaces_needed; i++) {
                            msp_strcat(output_buffer, get_osd_message(OSD_MSG_TIMER_SEPARATOR));
                        }
                        
                        // Додаємо таймер
                        msp_strcat(output_buffer, timer_str);
                        
                        // Фінальна перевірка довжини
                        uint16_t final_len = msp_strlen(output_buffer);
                        if (final_len == OSD_MAX_MESSAGE_LENGTH) {
                            return true;
                        } else {
                            // Якщо довжина не точно 16 символів, повертаємось до простого стану
                            has_active_timer = false;
                        }
                    }
                }
            }
        }
    }
    
    // ПРІОРИТЕТ 4: Тільки стан без таймера
    msp_strcpy(output_buffer, board_state_message);
    
    // Фінальна перевірка довжини для простого стану
    uint16_t final_len = msp_strlen(output_buffer);
    if (final_len > OSD_MAX_MESSAGE_LENGTH) {
        // Якщо стан занадто довгий, скорочуємо до INIT
        msp_strcpy(output_buffer, get_osd_message(OSD_MSG_INIT));
        final_len = msp_strlen(output_buffer);
    }
    
    return (final_len <= OSD_MAX_MESSAGE_LENGTH);
}

#if MSP_V2_ENABLE
// MSP v2 header constants
#define MSP_V2_HEADER_START '$'
#define MSP_V2_HEADER_M 'M'
#define MSP_V2_HEADER_DIRECTION '<'  // To flight controller
#define MSP_V2_HEADER_SIZE_BYTE 3    // Position of size byte in header

/**
 * @brief Calculate MSP v2 CRC16-CCITT checksum
 *
 * MSP v2 protocol uses CRC16-CCITT with polynomial 0x1021
 * Initial value: 0x0000 (not 0xFFFF)
 * This implementation matches the standard CRC16-CCITT algorithm
 */
uint16_t msp_osd_calculate_crc16_v2(const uint8_t *data, uint16_t length) {
    uint16_t crc = 0x0000;  // MSP v2 uses 0x0000 as initial value

    for (uint16_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0x8408;  // CRC16-CCITT polynomial (reversed)
            } else {
                crc = crc >> 1;
            }
        }
    }

    return crc;
}

/**
 * @brief Encode MSP v2 packet with bomb status information and return ready-to-send buffer
 */
bool msp_osd_encode_bomb_status_v2(uint8_t armed, uint16_t safeTime, uint16_t selfDestroyTime, uint8_t *buffer, uint16_t buffer_size, uint16_t *packet_size) {
    if (!buffer || !packet_size) {
        return false;
    }

    // Build the status string with 16-character limit format
    // Format: "B:DISA S:1000s." or "B:ARM  D:255m."
    char status_string[32];
    const char *armed_str = armed ? "ARM" : "DISA";

    // Start building the status string
    msp_strcpy(status_string, "B:");
    msp_strcat(status_string, armed_str);

    if (safeTime > 0) {
        // Add extra space if armed (3 chars) to align with DISA (4 chars)
        if (armed) {
            msp_strcat(status_string, " ");
        }
        msp_strcat(status_string, " S:");
        char time_str[8];
        msp_itoa(safeTime, time_str);
        msp_strcat(status_string, time_str);
        msp_strcat(status_string, "s.");
    } else {
        // Add extra space if armed (3 chars) to align with DISA (4 chars)
        if (armed) {
            msp_strcat(status_string, " ");
        }
        msp_strcat(status_string, " D:");
        char time_str[8];
        msp_itoa(selfDestroyTime, time_str);
        msp_strcat(status_string, time_str);
        msp_strcat(status_string, "m.");
    }

    // Calculate payload size
    uint16_t payload_len = msp_strlen(status_string);
    if (payload_len >= MSP_V2_MAX_PAYLOAD_SIZE) {
        return false;
    }

    // Check MSP_SET_NAME limit (32 characters max)
    if (payload_len > MSP_SET_NAME_MAX_LENGTH) {
        return false;
    }

    // Calculate total packet size
    uint16_t total_size = MSP_V2_HEADER_SIZE + payload_len + MSP_V2_CRC_SIZE;
    if (total_size > buffer_size) {
        return false;
    }

    // Build complete MSP v2 packet in buffer
    uint16_t pos = 0;

    // Header: $M< + size + command
    buffer[pos++] = MSP_V2_HEADER_START;      // '$'
    buffer[pos++] = MSP_V2_HEADER_M;          // 'M'
    buffer[pos++] = MSP_V2_HEADER_DIRECTION;  // '<' (to flight controller)
    buffer[pos++] = payload_len;              // Payload size
    buffer[pos++] = MSP_SET_NAME;             // Command ID

    // Payload
    for (uint16_t i = 0; i < payload_len; i++) {
        buffer[pos++] = status_string[i];
    }

    // Calculate CRC for the data part (size + command + payload)
    // MSP v2 CRC covers: size + command + payload (not the header $M<)
    uint8_t crc_data[MSP_V2_MAX_PAYLOAD_SIZE + 2];
    crc_data[0] = payload_len;
    crc_data[1] = MSP_SET_NAME;
    for (uint16_t i = 0; i < payload_len; i++) {
        crc_data[2 + i] = status_string[i];
    }

    uint16_t crc = msp_osd_calculate_crc16_v2(crc_data, payload_len + 2);

    // CRC (little endian)
    buffer[pos++] = crc & 0xFF;
    buffer[pos++] = (crc >> 8) & 0xFF;

    *packet_size = pos;

    return true;
}

/**
 * @brief Validate MSP v2 packet structure and CRC
 *
 * @param packet Complete MSP v2 packet buffer
 * @param packet_size Size of the packet
 * @return true if packet is valid, false otherwise
 */
bool msp_osd_validate_packet_v2(const uint8_t *packet, uint16_t packet_size) {
    if (!packet || packet_size < MSP_V2_HEADER_SIZE + MSP_V2_CRC_SIZE) {
        return false;
    }

    // Check header
    if (packet[0] != '$' || packet[1] != 'M' || packet[2] != '<') {
        return false;
    }

    uint8_t payload_size = packet[3];
    uint8_t command = packet[4];

    // Validate packet size
    if (packet_size != MSP_V2_HEADER_SIZE + payload_size + MSP_V2_CRC_SIZE) {
        return false;
    }

    // Calculate expected CRC
    uint8_t crc_data[MSP_V2_MAX_PAYLOAD_SIZE + 2];
    crc_data[0] = payload_size;
    crc_data[1] = command;
    for (uint16_t i = 0; i < payload_size; i++) {
        crc_data[2 + i] = packet[5 + i];
    }

    uint16_t expected_crc = msp_osd_calculate_crc16_v2(crc_data, payload_size + 2);
    uint16_t actual_crc = packet[5 + payload_size] | (packet[6 + payload_size] << 8);

    return expected_crc == actual_crc;
}
#endif // MSP_V2_ENABLE

/**
 * @brief Calculate MSP v1 XOR checksum (for compatibility with Reefwing-MSP)
 *
 * This function provides compatibility with MSP v1 implementations
 * like the Reefwing-MSP library which uses simple XOR checksum
 *
 * @param data Data to calculate checksum for
 * @param length Length of data
 * @return XOR checksum
 */
uint8_t msp_osd_calculate_xor_checksum_v1(const uint8_t *data, uint16_t length) {
    uint8_t checksum = 0;
    for (uint16_t i = 0; i < length; i++) {
        checksum ^= data[i];
    }
    return checksum;
}

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
bool msp_osd_encode_bomb_status_v1(const init_board_system_info_t* system_info, uint8_t *buffer, uint16_t buffer_size, uint16_t *packet_size) {
    if (!system_info || !buffer || !packet_size) {
        return false;
    }

    // Format OSD message based on system_info using new formatting rules
    char status_string[OSD_MAX_MESSAGE_LENGTH + 16];  // Extra space for safety
    if (!format_osd_message_from_system_info(system_info, status_string, sizeof(status_string), mspOsdStatus.error_blink_state)) {
        return false;
    }
    
    // Validate formatted message
    if (!validate_osd_message(status_string)) {
        return false;
    }

    // Calculate payload size
    uint16_t payload_len = msp_strlen(status_string);
    if (payload_len >= MSP_V2_MAX_PAYLOAD_SIZE) {
        return false;
    }

    // Check MSP_SET_NAME limit (OSD_MAX_MESSAGE_LENGTH characters max according to OSD rules)
    if (payload_len > OSD_MAX_MESSAGE_LENGTH) {
        return false;
    }

    // Calculate total packet size (MSP v1: $M< + size + type + payload + checksum)
    uint16_t total_size = 3 + 1 + 1 + payload_len + 1;  // header + size + type + payload + checksum
    if (total_size > buffer_size) {
        return false;
    }

    // Build MSP v1 packet
    uint16_t pos = 0;

    // Header: $M<
    buffer[pos++] = '$';
    buffer[pos++] = 'M';
    buffer[pos++] = '<';
    buffer[pos++] = payload_len;   // Payload size
    buffer[pos++] = MSP_SET_NAME;  // Command ID

    // Payload (formatted OSD message)
    for (uint16_t i = 0; i < payload_len; i++) {
        buffer[pos++] = status_string[i];
    }

    // Calculate XOR checksum for MSP v1 (size + type + payload)
    uint8_t checksum = msp_osd_calculate_xor_checksum_v1(&buffer[3], payload_len + 2);
    buffer[pos++] = checksum;

    *packet_size = pos;

    return true;
}

static bool OsdMsp_UpdateStatus (void)
{
    bool ret = false;

    // Check if system_info is available
    if (!mspOsdStatus.system_info) {
        return false;
    }

    // Encode MSP packet using system_info data according to OSD_Message_Rules.md
    if (msp_osd_encode_bomb_status_v1(mspOsdStatus.system_info, mspOsdStatus.tx_buffer, sizeof(mspOsdStatus.tx_buffer), &mspOsdStatus.packet_size))
    {
        // Print to UART
        ret = UartSendData(mspOsdStatus.tx_buffer, mspOsdStatus.packet_size);
    }

    return ret;
}

static void OsdMsp_UpdateStatusCbk(uint8_t timer_id)
{
	(void)timer_id;
	mspOsdStatus.update_status = 1;
}

void OsdMsp_Init(app_ext_cbk_fn system_cbk, init_board_system_info_t* system_info)
{
    /*  Save callback and system info pointer */
    mspOsdStatus.system_callback = system_cbk;
    mspOsdStatus.system_info = system_info;
    
    // Initialize error blinking state (start with showing message)
    mspOsdStatus.error_blink_state = true;

    // OSD MSP initialization implementation
    Timer_Start(FLIGHT_PARAMS_UPDATE_TMR, FLIGHT_PARAMS_UPDATE_TMR_OSD_PERIOD_MS, OsdMsp_UpdateStatusCbk);
}

void OsdMsp_Task(void)
{
    // OSD MSP task implementation
    if (mspOsdStatus.update_status)
    {
        mspOsdStatus.update_status = 0;
        
        // Toggle error blinking state if there's an error
        if (mspOsdStatus.system_info && mspOsdStatus.system_info->error_code != ERR_CODE_NO_ERROR) {
            mspOsdStatus.error_blink_state = !mspOsdStatus.error_blink_state;
        } else {
            // No error - always show normal message
            mspOsdStatus.error_blink_state = true;
        }
        
        // Restart the timer for the next update
        Timer_Start(FLIGHT_PARAMS_UPDATE_TMR, FLIGHT_PARAMS_UPDATE_TMR_OSD_PERIOD_MS, OsdMsp_UpdateStatusCbk);

        // Trigger OSD update
        if (OsdMsp_UpdateStatus()) 
        {
            // Successfully updated OSD
            
        }
    }
}
