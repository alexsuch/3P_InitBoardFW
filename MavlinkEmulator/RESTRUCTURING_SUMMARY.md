# MavlinkEmulator Logic Restructuring Summary

## Overview
Restructured the MavlinkEmulator command architecture from **passive state encoding** in HEARTBEAT messages to **discrete command sending** based on GPIO state changes.

## Changes Applied

### 1. Updated `send_autopilot_heartbeat()` - Line 250
**Previous Behavior:**
- Read ARM/PREARM states from GPIO pins (PA4, PA15)
- Encode states into 32-bit custom_mode field:
  - Bits 0-3: ARM state value
  - Bits 4-7: PREARM state value
- Send HEARTBEAT with encoded state information

**New Behavior:**
- Removed all GPIO state reading from this function
- Set custom_mode to all zeros (0x00000000) as reserved/unused
- HEARTBEAT now only used for connection keep-alive signaling
- State changes are communicated via discrete commands instead

**Rationale:**
- HEARTBEAT should contain autopilot state summary, not control state encoding
- State transitions require immediate action, not periodic HEARTBEAT sends
- Cleaner separation: HEARTBEAT for telemetry, COMMAND_LONG for control

### 2. Simplified `process_periodic_heartbeat()` - Line 497
**Current State:**
- Already simplified (ARM/PREARM state reading code is commented out)
- Sends HEARTBEAT every 1000ms for connection keep-alive
- Debug output disabled

**Status:** ✅ No changes needed - already in correct state

### 3. Enhanced `process_input_states()` - Line 543
**Previous Behavior:**
- Detected state changes with debounce (300ms)
- Called debug functions to print state changes
- **Did NOT send any commands** - only IGNITION sent commands

**New Behavior:**
- Detects ARM state transitions → sends commands:
  - ARM: 0→1 (DISARMED→ARMED) → `send_custom_command(2, 0)` (MAVLINK_CMD_CHARGE)
  - ARM: 1→0 (ARMED→DISARMED) → `send_custom_command(3, 0)` (MAVLINK_CMD_DISCHARGE)

- Detects PREARM state transitions → sends commands:
  - PREARM: 0→1 (DISABLED→ENABLED) → `send_custom_command(4, 0)` (MAVLINK_CMD_PREARM_ENABLE)
  - PREARM: 1→0 (ENABLED→DISABLED) → `send_custom_command(5, 0)` (MAVLINK_CMD_PREARM_DISABLE)

- IGNITION state transitions → unchanged:
  - IGNITION: 0→1 (OFF→ON) → `send_ignition_command()` (MAVLINK_CMD_IGNITION = 1)

- All transitions debounced (300ms) before sending command

**Rationale:**
- State changes require immediate command transmission, not passive encoding
- Matches STM32G431CBT6 command processor expectations (Mavlink_ProcessCommandLong)
- Enables real-time state synchronization with debounce protection

## Command Structure

All commands sent via `send_custom_command(uint8_t command_type, uint8_t command_data)`:

```c
// Command IDs (mapped in param1[0])
#define MAVLINK_CMD_IGNITION           1   // Ignition command
#define MAVLINK_CMD_CHARGE             2   // ARM transition → CHARGE battery
#define MAVLINK_CMD_DISCHARGE          3   // DISARM transition → DISCHARGE battery
#define MAVLINK_CMD_PREARM_ENABLE      4   // PREARM enable command
#define MAVLINK_CMD_PREARM_DISABLE     5   // PREARM disable command

// MAVLink Command Wrapper
#define MAV_CMD_USER_1                 31010  // Standard MAVLink user command
```

**Packet Format:**
- Message ID: COMMAND_LONG (76)
- param1[0]: custom_command_type (1-5 mapped above)
- param1[1]: custom_command_data (unused in current implementation, sent as 0)
- Command: MAV_CMD_USER_1 (31010)
- Target: MAVLINK_SYSTEM_ID_INITBOARD (0x42)

## Integration with STM32 (Reference)

The STM32G431CBT6 side processes commands in `Mavlink_ProcessCommandLong()`:

```c
// STM32 Command Handler (mavlink_uart.c)
case MAVLINK_CMD_CHARGE:           // 2
    system_callback(SYSTEM_EVT_READY, MAVLINK_EVT_AUTOPILOT_CHARGE, NULL);
    break;
case MAVLINK_CMD_DISCHARGE:        // 3
    system_callback(SYSTEM_EVT_READY, MAVLINK_EVT_AUTOPILOT_DISCHARGE, NULL);
    break;
case MAVLINK_CMD_PREARM_ENABLE:    // 4
    system_callback(..., MAVLINK_EVT_AUTOPILOT_PREARM_ENABLE, ...);
    break;
case MAVLINK_CMD_PREARM_DISABLE:   // 5
    system_callback(..., MAVLINK_EVT_AUTOPILOT_PREARM_DISABLE, ...);
    break;
```

The STM32 firmware now receives and processes these discrete commands immediately upon state change.

## Testing Recommendations

1. **GPIO Simulation:**
   - Toggle ARM pin (GPIO4) and verify CHARGE/DISCHARGE commands transmitted
   - Toggle PREARM pin (GPIO15) and verify PREARM_ENABLE/PREARM_DISABLE commands transmitted
   - Oscilloscope validation on MAIN_UART (PA3 RX)

2. **Debounce Verification:**
   - Rapid GPIO changes should not spam commands
   - Single transitions after debounce period should trigger exactly one command

3. **STM32 Response:**
   - Verify STM32 callback events triggered on command reception
   - Monitor board state changes after command processing
   - Check for proper COMMAND_ACK responses (if implemented)

4. **Scope Verification:**
   - Monitor PA2/PA3 (MAIN_UART lines)
   - Expected: COMMAND_LONG packet (45 bytes) transmitted within 300ms after state change
   - Check packet structure: magic=0xFD, payload_len=33, msg_id=76, crc_correct

## Files Modified

- **main.cpp** (MavlinkEmulator):
  - `send_autopilot_heartbeat()` - Removed ARM/PREARM encoding
  - `process_input_states()` - Added command transmission logic

## Build Status
✅ Ready for compilation and testing on ESP32

## Notes
- All command IDs and values match STM32 enum definitions in `shared/Inc/mavlink_uart.h`
- CRC calculation uses MAVLink v2.0 X.25 algorithm with message-specific crc_extra
- Backward compatibility: HEARTBEAT still sent for connection keep-alive, just without state encoding
