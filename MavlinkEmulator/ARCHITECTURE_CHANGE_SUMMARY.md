# Architecture Change Summary
## InitBoard ↔ Autopilot Communication System

### MAJOR ARCHITECTURAL CHANGES

#### 1. **Communication Paradigm Shift**
- **Before:** STM32G0 InitBoard ↔ Ground Control Station (GCS)
- **After:** STM32G0 InitBoard ↔ ESP32 Autopilot

#### 2. **Role Transformation**
- **InitBoard (STM32G0):** Changed from "drone controller" to "status reporter"
  - **Sends:** Status updates via HEARTBEAT (ignition, mining, ARM states, error codes)
  - **Receives:** Commands from autopilot (currently only IGNITION)
  
- **ESP32:** Changed from "GCS Emulator" to "Autopilot Emulator"
  - **Sends:** GPIO states via HEARTBEAT + IGNITION commands
  - **Receives:** InitBoard status updates

#### 3. **System ID & Component ID Changes**
- **Before:** Configurable system IDs, different component IDs
- **After:** Synchronized System ID 0x42 for both devices
  - InitBoard Component ID: 1
  - Autopilot Component ID: 2

#### 4. **Command Set Simplification**
- **Before:** Multiple commands (DISARM, ARM, PREARM_ENABLED, PREARM_DISABLED, IGNITION)
- **After:** Single command (IGNITION) triggered by GPIO0 LOW

#### 5. **Event System Update**
- **Before:** GCS-centric events (GCS_CONNECTED, GCS_DISCONNECTED, etc.)
- **After:** Autopilot-centric events:
  ```c
  MAVLINK_EVT_AUTOPILOT_CONNECTED
  MAVLINK_EVT_AUTOPILOT_DISCONNECTED  
  MAVLINK_EVT_AUTOPILOT_HEARTBEAT
  MAVLINK_EVT_AUTOPILOT_ARMED
  MAVLINK_EVT_AUTOPILOT_DISARMED
  MAVLINK_EVT_AUTOPILOT_PREARM_ENABLED
  MAVLINK_EVT_AUTOPILOT_PREARM_DISABLED
  MAVLINK_EVT_COMMAND_IGNITION
  ```

### TECHNICAL IMPLEMENTATION CHANGES

#### 6. **STM32G0 Mavlink Module (mavlink_uart.c/.h)**
- **New Functions:**
  - `Mavlink_ProcessAutopilotHeartbeat()` - replaced GCS heartbeat processing
  - `Mavlink_GetAutopilotArmState()` - access autopilot ARM state
  - `Mavlink_GetAutopilotPrearmState()` - access autopilot PREARM state

- **Updated Structures:**
  ```c
  typedef struct {
      uint8_t arm_state;      // GPIO1 state (0=HIGH, 1=LOW)
      uint8_t prearm_state;   // GPIO2 state (0=HIGH, 1=LOW) 
      uint8_t ignition_state; // GPIO0 state (0=HIGH, 1=LOW)
  } mavlink_autopilot_states_t;
  ```

- **New Enums:**
  ```c
  typedef enum {
      MAVLINK_TIMER_MODE_NONE = 0x00,
      MAVLINK_TIMER_MODE_SAFE = 0x01,
      MAVLINK_TIMER_MODE_DESTROY = 0x02
  } mavlink_timer_mode_t;

  typedef enum {
      MAVLINK_BOARD_STATE_NO_FC = 0x00,
      MAVLINK_BOARD_STATE_HAS_FC = 0x01,
      MAVLINK_BOARD_STATE_ARMED = 0x02
  } mavlink_board_state_t;

  typedef enum {
      MAVLINK_AUTOPILOT_ARM_STATE_DISARMED = 0x00,
      MAVLINK_AUTOPILOT_ARM_STATE_ARMED = 0x01
  } mavlink_autopilot_arm_state_t;
  ```

#### 7. **ESP32 Autopilot Emulator Changes**
- **GPIO Monitoring:** Real-time monitoring of GPIO 0/1/2 states
- **Command Generation:** IGNITION command triggered when GPIO0 goes LOW
- **State Encoding:** GPIO states encoded in HEARTBEAT custom mode bitfield
- **Debug Output:** Comprehensive logging of autopilot state changes

### MESSAGE FLOW ARCHITECTURE

#### 8. **Bidirectional HEARTBEAT Communication**

**InitBoard → Autopilot (1Hz):**
```
Custom Mode Bitfield:
├── [0] IGNITION_PIN_STATE (0 = HIGH, 1 = LOW)
├── [1] MINING_STATUS_BIT  (0 = LOW, 1 = HIGH) 
├── [2] ARM_STATUS_BIT     (0 = LOW, 1 = HIGH)
├── [3] IS_ARMED_BIT       (0 = DISARMED, 1 = ARMED)
├── [4] NEED_IGNITION_BIT  (0 = NO, 1 = YES)
├── [5-7] ERROR_CODE_LOW   (Error code bits 0-2)
└── [8-10] ERROR_CODE_HIGH (Error code bits 3-5)
```

**Autopilot → InitBoard (1Hz):**
```
Custom Mode Bitfield:
├── [0] ARM_GPIO_STATE     (GPIO1: 0 = HIGH, 1 = LOW)
├── [1] PREARM_GPIO_STATE  (GPIO2: 0 = HIGH, 1 = LOW)
├── [2] IGNITION_GPIO_STATE (GPIO0: 0 = HIGH, 1 = LOW)
└── [3-31] Reserved
```

#### 9. **Command Processing**
- **COMMAND_LONG (IGNITION):** Sent when GPIO0 goes LOW on autopilot
- **COMMAND_ACK:** Response from InitBoard confirming command reception

### UPDATED DOCUMENTATION

#### 10. **Files Updated to Reflect New Architecture**
- ✅ **REQUIREMENTS_UPDATED.md** - Complete rewrite for autopilot architecture
- ✅ **INTEGRATION_GUIDE_FINAL.md** - New comprehensive autopilot integration guide
- ✅ **REQUIREMENTS.md** - Updated original requirements file
- ✅ **MAVLINK_INTEGRATION_UPDATED.md** - Updated integration guide
- ✅ **ARCHITECTURE_CHANGE_SUMMARY.md** - This summary document

### IMPACT ANALYSIS

#### 11. **Benefits of New Architecture**
- **Simplified Command Set:** Single IGNITION command vs. multiple GCS commands
- **Real-time GPIO Monitoring:** Direct hardware state feedback via HEARTBEAT
- **Autonomous Operation:** Autopilot drives InitBoard actions vs. manual GCS control
- **Synchronized System IDs:** Cleaner communication protocol
- **Event-driven Architecture:** Better separation of concerns

#### 12. **Integration Points**
- **Hardware:** GPIO 0/1/2 monitoring on ESP32 autopilot
- **Software:** Updated STM32G0 event handlers for autopilot events
- **Protocol:** Mavlink v2.0 HEARTBEAT and COMMAND_LONG messages
- **Timing:** 1Hz periodic communication with timeout detection

### NEXT STEPS
1. **Validate Implementation:** Test GPIO monitoring and command generation
2. **Performance Testing:** Verify 1Hz timing and timeout handling
3. **Integration Testing:** End-to-end autopilot ↔ InitBoard communication
4. **Documentation Review:** Ensure all files reflect new architecture

---
**Date:** Current
**Status:** Architecture transformation complete, documentation updated
**Author:** GitHub Copilot
