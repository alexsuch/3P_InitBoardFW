# OSD Message Rules and Implementation Guide

## Overview
This document describes the OSD (On-Screen Display) message formatting rules and implementation for the 3P InitBoard system using MSP (MultiWii Serial Protocol).

## Message Structure

### Basic Constraints
- **Maximum Length**: 16 characters per message
- **Character Set**: Printable ASCII (0x20-0x7E)
- **Protocol**: MSP v1 with XOR checksum
- **Update Rate**: Configurable via `FLIGHT_PARAMS_UPDATE_TMR_OSD_PERIOD_MS`

### Message Priority System
Messages are displayed according to priority levels:

1. **PRIORITY 1: BOOM** (Highest Priority)
   - Message: `"BOOM!!!"`
   - Length: 7 characters
   - Condition: `board_state == BOARD_STATE_BOOM`
   - No timer information displayed

2. **PRIORITY 2: Error Messages**
   - No timer information displayed
   - Condition: `error_code != ERR_CODE_NO_ERROR`
   
   | Error Code | Message | Length |
   |------------|---------|--------|
   | `ERR_CODE_BATTERY_LOW` | `"ERR: LOW BATTERY"` | 16 chars |
   | `ERR_CODE_FUSE_INCORRECT_STATE` | `"ERR: CHEKA FAIL"` | 15 chars |
   | `ERR_CODE_UNEXPECTED_IGNITION`<br>`ERR_CODE_UNEXPECTED_ARM`<br>`ERR_CODE_UNEXPECTED_MINING` | `"ERR: SERVO FAIL"` | 15 chars |
   | `ERR_CODE_UNEXPECTED_VUSA_SHORTED` | `"ERR: VUSA FAIL"` | 14 chars |
   | Other errors | `"ERR: UNKNOWN"` | 12 chars |
   | Fallback | `"ERR: ERROR"` | 10 chars |

3. **PRIORITY 3: Normal States with Timers**
   - Format: `"STATE     TIMER"`
   - Total length: 16 characters exactly
   
4. **PRIORITY 4: Normal States without Timers**
   - Simple state display only

## Board States

### Standard States
| State | Message | Length | Description |
|-------|---------|--------|-------------|
| `BOARD_STATE_DISCHARGED` | `"DISARM"` | 6 chars | System discharged |
| `BOARD_STATE_CHARGING` | `"CHARGING"` | 8 chars | Battery charging |
| `BOARD_STATE_CHARGED` | `"ARMED"` | 5 chars | System charged |
| `BOARD_STATE_MINING` | `"!MINE ACT"` | 9 chars | Mining mode active |
| `BOARD_STATE_INIT` | `"INIT"` | 4 chars | Initialization/unknown |

### Mining State Special Cases

#### Standard Mining Display
- **Without Timer**: `"!MINE ACT"`
- **With Timer**: `"!MINE ACT  M:030s"` or `"!MINE ACT   M:005m"`

#### Low Battery Self-Destruct Mode
- **Condition**: `board_state == BOARD_STATE_MINING` AND `low_pwr_self_dest_allowed == true`
- **Message**: `"!MINE ACT LB DEST"`
- **Length**: 16 characters exactly
- **Behavior**: Timer information is **NOT displayed** regardless of active timers
- **Purpose**: Indicates mining mode with low battery self-destruct enabled

## Timer Display Rules

### Timer Modes
| Mode | Prefix | Time Display | Suffix | Example |
|------|--------|--------------|--------|---------|
| `TIMER_MODE_SAFE` | `"S:"` | Always seconds | `"s"` | `"S:030s"` |
| `TIMER_MODE_SELF_DESTROY` | `"D:"` | Seconds if <60s<br>Minutes if ≥60s | `"s"` or `"m"` | `"D:030s"` or `"D:005m"` |
| `TIMER_MODE_MINING` | `"M:"` | Seconds if <60s<br>Minutes if ≥60s | `"s"` or `"m"` | `"M:030s"` or `"M:005m"` |

### Timer Format Specifications
- **Fixed Length**: 6 characters always (`"X:YYYz"`)
- **Number Format**: 3 digits with leading zeros
- **Maximum Value**: 999 (seconds or minutes)
- **Overflow Handling**: Values >999 are capped to 999

### Timer Priority System
When multiple timers are active, priority determines which timer is displayed in `sys_info`:

1. **Mining Timer** (Highest Priority)
2. **Self-Destroy Timer**
3. **Safe Timer**

## Message Formatting Examples

### Complete 16-Character Messages
```
"ARMED     S:030s"  // Armed with 30-second safe timer
"!MINE ACT  M:030s"  // Mining with 30-second mining timer
"!MINE ACT   M:005m" // Mining with 5-minute mining timer
"!MINE ACT LB DEST"  // Mining with low battery self-destruct
"CHARGING   D:015m"  // Charging with 15-minute self-destroy timer
```

### State-Only Messages (No Active Timer)
```
"DISARM"     // 6 characters + 10 spaces (invisible)
"ARMED"      // 5 characters + 11 spaces (invisible)
"!MINE ACT"  // 9 characters + 7 spaces (invisible)
"CHARGING"   // 8 characters + 8 spaces (invisible)
```

## Implementation Details

### LUT (Lookup Table) System
All messages are stored in a centralized lookup table (`OSD_MESSAGE_LUT`) for maintainability:

```c
typedef enum {
    OSD_MSG_BOOM = 0,
    OSD_MSG_ERR_LOW_BATTERY,
    // ... error messages
    OSD_MSG_DISARM,
    OSD_MSG_CHARGE,
    OSD_MSG_ARMED,
    OSD_MSG_MINING,
    OSD_MSG_INIT,
    // ... timer prefixes/suffixes
    OSD_MSG_LB_DEST,
    OSD_MSG_TIMER_SEPARATOR,
    OSD_MSG_COUNT
} osd_message_index_t;
```

### Key Functions
- `format_osd_message_from_system_info()`: Main formatting function
- `format_timer_string()`: Universal timer formatting with minutes/seconds logic
- `get_osd_message()`: Safe LUT access with bounds checking
- `validate_osd_message()`: Message validation

### System Integration
- **Data Source**: `init_board_system_info_t` structure
- **Timer Conflicts**: Resolved by priority system in application layer
- **Real-time Updates**: Timer values updated every second via tick callbacks

## Protocol Details

### MSP v1 Packet Structure
```
Header: $M< (3 bytes)
Size: payload_length (1 byte)
Command: MSP_SET_NAME (1 byte)
Payload: OSD message (1-16 bytes)
Checksum: XOR checksum (1 byte)
```

### Validation Rules
1. Message length: 1-16 characters
2. Character validation: 0x20-0x7E (printable ASCII)
3. Timer format validation: Exactly 6 characters for timer strings
4. Total message validation: Exactly 16 characters for combined messages

## Configuration Constants

### Timing
- `FLIGHT_PARAMS_UPDATE_TMR_OSD_PERIOD_MS`: OSD update interval
- `ONE_SECOND_TICK_TMR_PERIOD_MS`: Timer tick interval (1000ms)

### Formatting
- `OSD_MAX_MESSAGE_LENGTH`: 16 characters
- `OSD_TIMER_STRING_LENGTH`: 6 characters
- `OSD_TIMER_DIGITS`: 3 digits
- `OSD_TIMER_MAX_VALUE`: 999
- `OSD_MIN_SPACES_FOR_ALIGNMENT`: 1 space minimum

## Change History

### Latest Updates
1. **Mining Timer Enhancement**: Added minutes/seconds logic matching Self-Destroy timer
2. **Board State Addition**: Added `BOARD_STATE_MINING` (value 0x05)
3. **Low Battery Integration**: Added `low_pwr_self_dest_allowed` to `init_board_system_info_t`
4. **Mining LB DEST Logic**: Special case for mining with low battery self-destruct
5. **Timer Refactoring**: Universal `format_timer_string()` function for code reuse
6. **Enum Reordering**: Moved `BOARD_STATE_MINING` after `BOARD_STATE_BOOM` for compatibility

### Migration Notes
- `low_pwr_self_dest_allowed` moved from `system_status_t` to `init_board_system_info_t`
- All `sysStatus.low_pwr_self_dest_allowed` references updated to `sysStatus.sys_info.low_pwr_self_dest_allowed`
- `BOARD_STATE_BOOM` value changed from 0x04 to 0x04, `BOARD_STATE_MINING` added as 0x05

## Testing Scenarios

### Basic States
- Verify all board states display correctly
- Confirm 16-character limit enforcement
- Test error message priorities

### Timer Display
- Test timer format consistency (6 characters)
- Verify minutes/seconds switching at 60-second boundary  
- Test timer priority conflicts (Mining > Self-Destroy > Safe)

### Mining Special Cases
- Test standard mining display with and without timers
- Verify "LB DEST" display when `low_pwr_self_dest_allowed = true`
- Confirm timer suppression in LB DEST mode

### Edge Cases
- Maximum timer values (999 seconds/minutes)
- Timer overflow handling
- Empty/invalid system_info handling
- Character set validation