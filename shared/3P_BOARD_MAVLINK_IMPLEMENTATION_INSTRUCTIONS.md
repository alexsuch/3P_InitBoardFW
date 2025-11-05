# MAVLink UART Communication - Detailed Technical Documentation

## Зміст
1. [Основні макроси MAVLink та UART](#основні-макроси-mavlink-та-uart)
2. [HEARTBEAT з плати ініціації](#heartbeat-з-плати-ініціації)
3. [HEARTBEAT з автопілота](#heartbeat-з-автопілота)
4. [VFR_HUD повідомлення](#vfr_hud-повідомлення)
5. [Кастомна COMMAND_LONG команда](#кастомна-command_long-команда)

---

## Основні макроси MAVLink та UART

### Протокол конфігурація
```c
#define MAVLINK_V2_MAGIC                   0xFD    // 253 - Магічний байт MAVLink v2.0
#define MAVLINK_VERSION                    0x03    // 3 - Версія протоколу
#define MAVLINK_MAX_MESSAGE_SIZE           0x118   // 280 - Максимальний розмір повідомлення
#define MAVLINK_MAX_PAYLOAD_SIZE           0xFF    // 255 - Максимальний розмір payload
```

### Структура пакету MAVLink v2.0
```c
#define MAVLINK_V2_HEADER_SIZE             10      // Розмір заголовку в байтах
#define MAVLINK_V2_CRC_SIZE                2       // Розмір CRC в байтах
#define MAVLINK_V2_MIN_PACKET_SIZE         12      // Мінімальний розмір пакету (заголовок + CRC)
```

### Індекси байтів у заголовку MAVLink v2.0
```c
#define MAVLINK_V2_MAGIC_INDEX             0       // Індекс магічного байту
#define MAVLINK_V2_PAYLOAD_LEN_INDEX       1       // Індекс довжини payload
#define MAVLINK_V2_INCOMPAT_FLAGS_INDEX    2       // Індекс несумісних флагів
#define MAVLINK_V2_COMPAT_FLAGS_INDEX      3       // Індекс сумісних флагів
#define MAVLINK_V2_SEQUENCE_INDEX          4       // Індекс номера послідовності
#define MAVLINK_V2_SYSTEM_ID_INDEX         5       // Індекс ID системи
#define MAVLINK_V2_COMPONENT_ID_INDEX      6       // Індекс ID компонента
#define MAVLINK_V2_MSG_ID_LOW_INDEX        7       // Індекс молодшого байту ID повідомлення
#define MAVLINK_V2_MSG_ID_MID_INDEX        8       // Індекс середнього байту ID повідомлення
#define MAVLINK_V2_MSG_ID_HIGH_INDEX       9       // Індекс старшого байту ID повідомлення
#define MAVLINK_V2_PAYLOAD_START_INDEX     10      // Індекс початку payload
```

### ID систем та компонентів
```c
#define MAVLINK_SYSTEM_ID_INITBOARD        0x42    // 66 - ID системи плати ініціації
#define MAVLINK_COMP_ID_INITBOARD          0x42    // 66 - ID компонента плати ініціації
```

### ID повідомлень
```c
#define MAVLINK_MSG_ID_HEARTBEAT           0x00    // 0 - HEARTBEAT повідомлення
#define MAVLINK_MSG_ID_VFR_HUD             0x4A    // 74 - VFR_HUD повідомлення
#define MAVLINK_MSG_ID_COMMAND_LONG        0x4C    // 76 - COMMAND_LONG повідомлення
#define MAVLINK_MSG_ID_COMMAND_ACK         0x4D    // 77 - COMMAND_ACK повідомлення
```

### MAV типи та стани
```c
#define MAV_TYPE_GENERIC                   0x00    // 0 - Загальний тип
#define MAV_AUTOPILOT_INVALID              0x08    // 8 - Невалідний автопілот
#define MAV_STATE_ACTIVE                   0x04    // 4 - Активний стан
```

### Коди результатів MAV
```c
#define MAV_RESULT_ACCEPTED                0x00    // 0 - Команда прийнята
#define MAV_RESULT_TEMPORARILY_REJECTED    0x01    // 1 - Команда тимчасово відхилена
#define MAV_RESULT_DENIED                  0x02    // 2 - Команда відхилена
#define MAV_RESULT_UNSUPPORTED             0x03    // 3 - Команда не підтримується
#define MAV_RESULT_FAILED                  0x04    // 4 - Команда не виконана
#define MAV_RESULT_IN_PROGRESS             0x05    // 5 - Команда виконується
#define MAV_RESULT_CANCELLED               0x06    // 6 - Команда скасована
```

---

## HEARTBEAT з плати ініціації

### Макроси індексів payload HEARTBEAT
```c
#define HEARTBEAT_TYPE_INDEX               0       // MAV тип
#define HEARTBEAT_AUTOPILOT_INDEX          1       // Тип автопілота
#define HEARTBEAT_BASE_MODE_INDEX          2       // Базовий режим
#define HEARTBEAT_CUSTOM_MODE_INDEX        3       // Кастомний режим (4 байти)
#define HEARTBEAT_SYSTEM_STATUS_INDEX      7       // Статус системи
#define HEARTBEAT_MAVLINK_VERSION_INDEX    8       // Версія MAVLink
#define HEARTBEAT_PAYLOAD_SIZE             9       // Розмір payload HEARTBEAT
```

### Структура Custom Mode битового поля (32 біти)
```c
typedef union {
    uint32_t raw;
    struct {
        uint32_t timer_sec : 14;           // Біти 0-13: Секунди таймера (0-16383)
        uint32_t timer_mode : 2;           // Біти 14-15: Режим таймера (timer_mode_t)
        uint32_t fc_control_present : 1;   // Біт 16: Наявність з'єднання з автопілотом
        uint32_t fuse_present : 1;         // Біт 17: Наявність запобіжника
        uint32_t board_state : 3;          // Біти 18-20: Стан плати (board_state_t)
        uint32_t battery_level : 4;        // Біти 21-24: Рівень батареї (0-10), в одиницях 10% (3=30%, 10=100%)
        uint32_t error_code : 4;           // Біти 25-28: Код помилки (err_code_t)
        uint32_t is_ignition_done : 1;     // Біт 29: Прапорець завершення підпалу
        uint32_t reserved : 2;             // Біти 30-31: Зарезервовано
    } bitfield;
} mavlink_custom_mode_t;
```

### Енумерації для timer_mode (timer_mode_t з init_brd.h)
```c
typedef enum {
    TIMER_MODE_NONE = 0x00,         // Немає активного таймера
    TIMER_MODE_SAFE = 0x01,         // Активний безпечний таймер
    TIMER_MODE_SELF_DESTROY = 0x02  // Активний таймер самознищення
} timer_mode_t;
```

### Енумерації для board_state (board_state_t з init_brd.h)
```c
typedef enum {
    BOARD_STATE_INIT = 0x00,        // Ініціалізація
    BOARD_STATE_DISCHARGED = 0x01,  // Розряджено
    BOARD_STATE_CHARGING = 0x02,    // Зарядка
    BOARD_STATE_CHARGED = 0x03,     // Заряджено
    BOARD_STATE_BOOM = 0x04         // Вибух
} board_state_t;
```

### Енумерації для error_code (err_code_t з init_brd.h)
```c
typedef enum {
    ERR_CODE_NO_ERROR = 0,                      // Немає помилки
    ERR_CODE_ACCELEROMETER_FAIL,                // Помилка акселерометра
    ERR_CODE_BATTERY_LOW,                       // Низький заряд батареї
    ERR_CODE_FUSE_INCORRECT_STATE,              // Некоректний стан запобіжника
    ERR_CODE_UNEXPECTED_IGNITION,               // Неочікуваний підпал
    ERR_CODE_UNEXPECTED_ARM,                    // Неочікуване озброєння
    ERR_CODE_UNEXPECTED_MINING,                 // Неочікувана активація мінування
    ERR_CODE_UNEXPECTED_VUSA_SHORTED,           // Неочікуване замикання VUSA
    ERR_CODE_MAX_ERROR                          // Максимальний код помилки
} err_code_t;
```

### Структура даних системного стану
```c
typedef struct {
    uint16_t timer_seconds;      // Секунди таймера (0-16383)
    uint8_t timer_mode;          // Режим таймера (timer_mode_t)
    uint8_t fuse_present;        // Наявність запобіжника (0-1)
    uint8_t board_state;         // Стан плати (board_state_t)
    uint8_t battery_level;       // Рівень батареї (0-10), в одиницях 10% (3=30%, 10=100%)
    uint8_t error_code;          // Код помилки (err_code_t)
    uint8_t is_ignition_done;    // Прапорець завершення підпалу (0-1)
    uint8_t fc_control_present;  // Статус з'єднання з FC (0-1)
} init_board_system_info_t;
```

### Приклад побудови HEARTBEAT від InitBoard
```c
// Кодування custom mode
mavlink_custom_mode_t custom_mode = {0};
custom_mode.bitfield.timer_sec = system_info->timer_seconds;           // 0-16383
custom_mode.bitfield.timer_mode = system_info->timer_mode;             // timer_mode_t
custom_mode.bitfield.fc_control_present = system_info->fc_control_present; // 0-1
custom_mode.bitfield.fuse_present = system_info->fuse_present;         // 0-1
custom_mode.bitfield.board_state = system_info->board_state;           // board_state_t
custom_mode.bitfield.battery_level = system_info->battery_level;       // 0-10 (в одиницях 10%)
custom_mode.bitfield.error_code = system_info->error_code;             // err_code_t
custom_mode.bitfield.is_ignition_done = system_info->is_ignition_done; // 0-1

// Payload HEARTBEAT
packet[HEARTBEAT_TYPE_INDEX] = MAV_TYPE_GENERIC;
packet[HEARTBEAT_AUTOPILOT_INDEX] = MAV_AUTOPILOT_INVALID;
packet[HEARTBEAT_BASE_MODE_INDEX] = 0;
packet[HEARTBEAT_CUSTOM_MODE_INDEX + 0] = custom_mode.raw & 0xFF;
packet[HEARTBEAT_CUSTOM_MODE_INDEX + 1] = (custom_mode.raw >> 8) & 0xFF;
packet[HEARTBEAT_CUSTOM_MODE_INDEX + 2] = (custom_mode.raw >> 16) & 0xFF;
packet[HEARTBEAT_CUSTOM_MODE_INDEX + 3] = (custom_mode.raw >> 24) & 0xFF;
packet[HEARTBEAT_SYSTEM_STATUS_INDEX] = MAV_STATE_ACTIVE;
packet[HEARTBEAT_MAVLINK_VERSION_INDEX] = MAVLINK_VERSION;
```

### Тайминг передачі InitBoard HEARTBEAT
```c
#define MAVLINK_INITBOARD_HEARTBEAT_INTERVAL_MS    1000    // 1000мс = 1Гц
```
---

## HEARTBEAT з автопілота

Автопілот (ESP32) надсилає свої HEARTBEAT повідомлення з **custom_mode** полем, в якому закодована інформація для плати ініціації. Це поле містить стани ARM та PREARM.

### Структура стану автопілота (32-бітне поле)
```c
typedef union {
    uint32_t raw;
    struct {
        uint32_t arm_state : 4;      // Біти 0-3: Стан озброєння (MAVLINK_AUTOPILOT_ARM_*)
        uint32_t prearm_state : 4;   // Біти 4-7: Стан пре-озброєння (MAVLINK_AUTOPILOT_PREARM_*)
        uint32_t reserved : 24;      // Біти 8-31: Зарезервовано для майбутніх розширень
    } bitfield;
} mavlink_autopilot_states_t;
```

### Енумерації станів автопілота
```c
// Стани ARM
#define MAVLINK_AUTOPILOT_ARM_DISARMED     0       // Роззброєно
#define MAVLINK_AUTOPILOT_ARM_ARMED        1       // Озброєно

// Стани PREARM
#define MAVLINK_AUTOPILOT_PREARM_DISABLED  0       // PREARM відключено
#define MAVLINK_AUTOPILOT_PREARM_ENABLED   1       // PREARM включено
```

### Тайминг з'єднання з автопілотом
```c
#define MAVLINK_CONNECTION_TIMEOUT_MS              3000    // 3 секунди тайм-аут
```

---

## VFR_HUD повідомлення

### Макроси індексів payload VFR_HUD
```c
#define VFR_HUD_AIRSPEED_INDEX             0       // Швидкість повітря (4 байти float)
#define VFR_HUD_GROUNDSPEED_INDEX          4       // Швидкість відносно землі (4 байти float)
#define VFR_HUD_HEADING_INDEX              8       // Курс (2 байти int16_t)
#define VFR_HUD_THROTTLE_INDEX             10      // Газ (2 байти uint16_t)
#define VFR_HUD_ALT_INDEX                  12      // Висота (4 байти float)
#define VFR_HUD_CLIMB_INDEX                16      // Швидкість набору висоти (4 байти float)
#define VFR_HUD_PAYLOAD_SIZE               20      // Розмір payload VFR_HUD
```

### Функції конвертації float в integer (без FPU)
```c
// Конвертація float байтів в см/с (позитивні значення)
static uint16_t Mavlink_FloatToIntCmS(const uint8_t* float_bytes);

// Конвертація float байтів в см (знакові значення)
static int32_t Mavlink_FloatToIntCm(const uint8_t* float_bytes);

// Конвертація float байтів в см/с (знакові значення для швидкості набору)
static int16_t Mavlink_FloatToIntCmS_Signed(const uint8_t* float_bytes);
```

### Приклад симуляції VFR_HUD на ESP32
```c
// ESP32 симуляція польотних даних
typedef struct {
    float airspeed;     // м/с
    float groundspeed;  // м/с
    int16_t heading;    // градуси
    uint16_t throttle;  // відсотки
    float altitude;     // метри
    float climb_rate;   // м/с
} vfr_hud_simulation_t;

// Оновлення симуляційних даних
void update_vfr_hud_simulation() {
    static vfr_hud_simulation_t sim_data = {
        .airspeed = 15.5f,      // 15.5 м/с
        .groundspeed = 14.2f,   // 14.2 м/с
        .heading = 285,         // 285°
        .throttle = 75,         // 75%
        .altitude = 150.0f,     // 150м
        .climb_rate = 2.1f      // 2.1 м/с вгору
    };
    
    // Динамічне оновлення симуляційних даних
    sim_data.heading = (sim_data.heading + 1) % 360;
    sim_data.altitude += sim_data.climb_rate * 0.001f; // Оновлення кожну секунду
    
    // Надсилання VFR_HUD повідомлення
    send_vfr_hud(&sim_data);
}
```

### Тайминг VFR_HUD
```c
// VFR_HUD надсилається з ESP32 з частотою 1Гц зі зміщенням 500мс після HEARTBEAT
#define VFR_HUD_INTERVAL_MS                1000    // 1 секунда
```

---

## Кастомна COMMAND_LONG команда

### Макроси кастомних команд
```c
#define MAVLINK_CMD_IGNITION               0x01    // 1 - Кастомна команда підпалу
#define MAV_CMD_USER_1                     31010   // MAVLink стандартна користувацька команда 1
```

### Структура COMMAND_LONG payload (33 байти)
```c
// Повна структура COMMAND_LONG повідомлення (33 байти payload)
typedef struct {
    float param1;           // Байти 0-3:   Параметр 1 (float) - ДЛЯ КАСТОМНИХ КОМАНД
    float param2;           // Байти 4-7:   Параметр 2 (float) - не використовується
    float param3;           // Байти 8-11:  Параметр 3 (float) - не використовується  
    float param4;           // Байти 12-15: Параметр 4 (float) - не використовується
    float param5;           // Байти 16-19: Параметр 5 (float) - не використовується
    float param6;           // Байти 20-23: Параметр 6 (float) - не використовується
    float param7;           // Байти 24-27: Параметр 7 (float) - не використовується
    uint16_t command;       // Байти 28-29: ID команди (MAV_CMD_USER_1 = 31010)
    uint8_t target_system;  // Байт 30:     Цільова система (MAVLINK_SYSTEM_ID_INITBOARD = 0x42)
    uint8_t target_component; // Байт 31:   Цільовий компонент (MAVLINK_COMP_ID_INITBOARD = 0x42)
    uint8_t confirmation;   // Байт 32:     Підтвердження (0)
} __attribute__((packed)) mavlink_command_long_t;

// Індекси байтів у payload COMMAND_LONG (відносно початку payload)
#define COMMAND_LONG_PARAM1_INDEX          0       // param1 (4 байти float) - КАСТОМНІ ДАНІ
#define COMMAND_LONG_PARAM2_INDEX          4       // param2 (4 байти float) - не використовується
#define COMMAND_LONG_PARAM3_INDEX          8       // param3 (4 байти float) - не використовується
#define COMMAND_LONG_PARAM4_INDEX          12      // param4 (4 байти float) - не використовується
#define COMMAND_LONG_PARAM5_INDEX          16      // param5 (4 байти float) - не використовується
#define COMMAND_LONG_PARAM6_INDEX          20      // param6 (4 байти float) - не використовується
#define COMMAND_LONG_PARAM7_INDEX          24      // param7 (4 байти float) - не використовується
#define COMMAND_LONG_COMMAND_INDEX         28      // command (2 байти uint16_t) - MAV_CMD_USER_1
#define COMMAND_LONG_TARGET_SYSTEM_INDEX   30      // target_system (1 байт uint8_t)
#define COMMAND_LONG_TARGET_COMPONENT_INDEX 31     // target_component (1 байт uint8_t)
#define COMMAND_LONG_CONFIRMATION_INDEX    32      // confirmation (1 байт uint8_t)
#define COMMAND_LONG_PAYLOAD_SIZE          33      // Розмір payload COMMAND_LONG
```

### Структура param1 поля (перші 4 байти) для кастомних команд
```c
// param1 (байти 0-3 в COMMAND_LONG) розбивається на байти для кастомних команд:
// payload[0] = param1[0] = custom_command_type (MAVLINK_CMD_IGNITION = 1, майбутні = 2,3,тощо)
// payload[1] = param1[1] = command_data (додаткові параметри для конкретного типу команди)
// payload[2] = param1[2] = зарезервовано для майбутнього використання
// payload[3] = param1[3] = зарезервовано для майбутнього використання

typedef union {
    float param1_float;     // Як float значення для стандартного MAVLink API
    struct {
        uint8_t custom_command_type;  // Байт 0: Тип кастомної команди (1=IGNITION)
        uint8_t command_data;         // Байт 1: Дані команди (специфічні для типу)
        uint8_t reserved1;            // Байт 2: Зарезервовано
        uint8_t reserved2;            // Байт 3: Зарезервовано
    } custom_cmd;
    uint32_t raw;           // Як 32-бітне значення для прямого доступу
} mavlink_param1_t;
```

### Відношення mavlink_param1_t до COMMAND_LONG структури
```c
// У COMMAND_LONG структурі:
// mavlink_command_long_t.param1 == mavlink_param1_t.param1_float
// 
// При парсингу payload на STM32:
// payload[0-3] відповідає mavlink_param1_t.custom_cmd структурі:
//   payload[0] = mavlink_param1_t.custom_cmd.custom_command_type
//   payload[1] = mavlink_param1_t.custom_cmd.command_data  
//   payload[2] = mavlink_param1_t.custom_cmd.reserved1
//   payload[3] = mavlink_param1_t.custom_cmd.reserved2
```

### Кастомні команди через MAV_CMD_USER_1
```
Протокол кастомних команд:
1. ESP32 надсилає COMMAND_LONG з command = MAV_CMD_USER_1 (31010)
2. У param1 закодовані кастомні дані:
   - param1[0] = custom_command_type (1=IGNITION, 2=майбутня команда, тощо)
   - param1[1] = command_data (специфічні параметри для кожного типу)
   - param1[2-3] = зарезервовано
3. STM32 перевіряє command == 31010, потім парсить param1[0] для типу команди
4. STM32 відповідає COMMAND_ACK з тим же command ID (31010) та результатом
```

### Події команд
```c
#define MAVLINK_EVT_COMMAND_IGNITION       0x01    // Команда підпалу отримана
```

### Макроси COMMAND_ACK payload
```c
#define COMMAND_ACK_COMMAND_INDEX          0       // ID команди (2 байти)
#define COMMAND_ACK_RESULT_INDEX           2       // Код результату
#define COMMAND_ACK_PAYLOAD_SIZE           3       // Розмір payload COMMAND_ACK
```

### Обробка COMMAND_LONG на STM32
```c
static void Mavlink_ProcessCommandLong(const uint8_t* payload) {
    // 1. Витягування основного ID команди з payload[28-29]
    uint16_t command_id = payload[COMMAND_LONG_COMMAND_INDEX] | 
                         (payload[COMMAND_LONG_COMMAND_INDEX + 1] << 8);
    
    // 2. Витягування target system та component (для перевірки адресації)
    uint8_t target_system = payload[COMMAND_LONG_TARGET_SYSTEM_INDEX];
    uint8_t target_component = payload[COMMAND_LONG_TARGET_COMPONENT_INDEX];
    
    uint8_t result = MAV_RESULT_UNSUPPORTED;
    
    // 3. Перевірка чи це наша кастомна команда MAV_CMD_USER_1
    if (command_id == MAV_CMD_USER_1) {  // 31010
        
        // 4. Парсинг param1 як кастомних даних (payload[0-3])
        mavlink_param1_t param1_data;
        param1_data.custom_cmd.custom_command_type = payload[COMMAND_LONG_PARAM1_INDEX + 0];  // payload[0]
        param1_data.custom_cmd.command_data = payload[COMMAND_LONG_PARAM1_INDEX + 1];         // payload[1]
        param1_data.custom_cmd.reserved1 = payload[COMMAND_LONG_PARAM1_INDEX + 2];            // payload[2]
        param1_data.custom_cmd.reserved2 = payload[COMMAND_LONG_PARAM1_INDEX + 3];            // payload[3]
        
        // 5. Обробка кастомної команди за типом
        switch (param1_data.custom_cmd.custom_command_type) {
            case MAVLINK_CMD_IGNITION:  // 1 - Команда ПІДПАЛУ
                // Обробка команди ПІДПАЛУ через callback
                // param1_data.custom_cmd.command_data містить додаткові параметри підпалу
                if (mavlink_state.system_callback) {
                    mavlink_state.system_callback(SYSTEM_EVT_READY, MAVLINK_EVT_COMMAND_IGNITION);
                    result = MAV_RESULT_ACCEPTED;
                }
                break;
                
            // Майбутні кастомні команди:
            // case 2:  // Тип кастомної команди 2
            //     // param1_data.custom_cmd.command_data специфічний для команди типу 2
            //     result = process_custom_command_2(param1_data.custom_cmd.command_data);
            //     break;
                
            default:
                result = MAV_RESULT_UNSUPPORTED;
                break;
        }
    }
    
    // 6. Надсилання підтвердження з основним command_id (MAV_CMD_USER_1 = 31010)
    Mavlink_SendCommandAck(command_id, result);
}
```

### Надсилання COMMAND_ACK
```c
static void Mavlink_SendCommandAck(uint16_t command, uint8_t result) {
    uint8_t packet[15] = {0};  // Mavlink v2.0 заголовок + 3 байти payload + CRC
    
    // Заголовок (10 байтів)
    packet[MAVLINK_V2_MAGIC_INDEX] = MAVLINK_V2_MAGIC;
    packet[MAVLINK_V2_PAYLOAD_LEN_INDEX] = COMMAND_ACK_PAYLOAD_SIZE;
    packet[MAVLINK_V2_MSG_ID_LOW_INDEX] = MAVLINK_MSG_ID_COMMAND_ACK & 0xFF;
    packet[MAVLINK_V2_MSG_ID_MID_INDEX] = (MAVLINK_MSG_ID_COMMAND_ACK >> 8) & 0xFF;
    packet[MAVLINK_V2_MSG_ID_HIGH_INDEX] = (MAVLINK_MSG_ID_COMMAND_ACK >> 16) & 0xFF;
    
    // Payload (3 байти)
    packet[MAVLINK_V2_PAYLOAD_START_INDEX + COMMAND_ACK_COMMAND_INDEX] = command & 0xFF;
    packet[MAVLINK_V2_PAYLOAD_START_INDEX + COMMAND_ACK_COMMAND_INDEX + 1] = (command >> 8) & 0xFF;
    packet[MAVLINK_V2_PAYLOAD_START_INDEX + COMMAND_ACK_RESULT_INDEX] = result;
    
    // Розрахунок та додавання CRC16
    uint16_t crc = Mavlink_CalculateCrc(packet, MAVLINK_V2_HEADER_SIZE + COMMAND_ACK_PAYLOAD_SIZE);
    packet[MAVLINK_V2_HEADER_SIZE + COMMAND_ACK_PAYLOAD_SIZE] = crc & 0xFF;
    packet[MAVLINK_V2_HEADER_SIZE + COMMAND_ACK_PAYLOAD_SIZE + 1] = (crc >> 8) & 0xFF;
    
    Mavlink_SendPacket(packet, 15);
}
```

### Приклад надсилання IGNITION команди з ESP32
```c
// ESP32 надсилання команди підпалу через MAV_CMD_USER_1
void send_ignition_command() {
    mavlink_message_t msg;
    
    // Підготовка param1 з кастомними даними
    mavlink_param1_t param1_data = {0};
    param1_data.custom_cmd.custom_command_type = MAVLINK_CMD_IGNITION;  // 1 - тип команди
    param1_data.custom_cmd.command_data = 0;                            // 0 - стандартний підпал
    param1_data.custom_cmd.reserved1 = 0;                               // зарезервовано
    param1_data.custom_cmd.reserved2 = 0;                               // зарезервовано
    
    // Пакування COMMAND_LONG повідомлення
    mavlink_msg_command_long_pack(
        SYSTEM_ID_ESP32, COMP_ID_ESP32,     // source system & component
        &msg,
        MAVLINK_SYSTEM_ID_INITBOARD,        // target system (0x42)
        MAVLINK_COMP_ID_INITBOARD,          // target component (0x42)
        MAV_CMD_USER_1,                     // command = 31010 (основна команда)
        0,                                  // confirmation
        param1_data.param1_float,           // param1 (кастомні дані як float)
        0, 0, 0, 0, 0, 0                   // param2-7 (не використовуються)
    );
    
    // Надсилання через UART2
    send_mavlink_message(&msg);
}

// Приклад майбутньої кастомної команди
void send_custom_command_2(uint8_t custom_data) {
    mavlink_message_t msg;
    
    mavlink_param1_t param1_data = {0};
    param1_data.custom_cmd.custom_command_type = 2;           // тип команди 2
    param1_data.custom_cmd.command_data = custom_data;       // специфічні дані
    
    mavlink_msg_command_long_pack(
        SYSTEM_ID_ESP32, COMP_ID_ESP32, &msg,
        MAVLINK_SYSTEM_ID_INITBOARD, MAVLINK_COMP_ID_INITBOARD,
        MAV_CMD_USER_1, 0,
        param1_data.param1_float,       // param1 з новими кастомними даними
        0, 0, 0, 0, 0, 0
    );
    
    send_mavlink_message(&msg);
}
```
