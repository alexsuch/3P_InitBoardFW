# Self-Destruction Timer Redesign Summary

## Overview
Redesigned the self-destruction timer architecture to use 1-second callback pattern with decremental counter, similar to the safe timer implementation.

## Changes Made

### 1. Structure Updates (`shared/Inc/init_brd.h`)
- Added `uint16_t self_destroy_tmr_tick` to `system_status_t` structure
- Added `APP_TASK_SELF_DESTROY_TMR_TICK_CBK` to app task enum

### 2. Timer Implementation (`shared/Src/app.c`)

#### New Functions:
- `App_SelfDestroyTmrTick(uint8_t timer_id)` - Timer callback that sets task flag
- `App_SelfDestroyTmrTickCbk(void)` - Task callback that decrements counter

#### Modified Functions:
- `App_SelfDestroyTimerStart()` - Now initializes counter and starts 1-second timer
- `App_SelfDestroyTimerStop()` - Now resets the counter variable

### 3. Timer Architecture

#### Old Approach:
```c
Timer_Start(SELF_DESTRUCTION_TMR, (timeout_in_ms), App_SelfDestructionInitTmrCbk);
```

#### New Approach:
```c
sysStatus.self_destroy_tmr_tick = timeout_in_seconds;
Timer_Start(SELF_DESTRUCTION_TMR, ONE_SECOND_TICK_TMR_PERIOD_MS, App_SelfDestroyTmrTick);
```

### 4. Timer Behavior

1. **Initialization**: Counter set to desired timeout in seconds
2. **Execution**: Timer fires every 1 second
3. **Countdown**: Counter decremented each second
4. **Completion**: When counter reaches 0, `App_SelfDestroyInit()` is called
5. **Continuation**: Timer automatically restarts for next second if counter > 0

### 5. Benefits

- **Consistent Architecture**: Same pattern as safe timer
- **Better Precision**: 1-second granularity instead of single long timeout
- **Real-time Updates**: `sysStatus.sys_info.timer_seconds` updated every second
- **Easier Debugging**: Can inspect countdown in real-time
- **Pause/Resume Ready**: Architecture supports future pause functionality

### 6. Test Configurations

The implementation respects existing test macros:
- `TEST_SELF_DESTROY_ONLY_MODE`: 20 seconds countdown
- `TEST_SELF_DESTROY_MINING_MODE`: 30 seconds countdown  
- Normal mode: Uses `sysStatus.config->selfDestroyTimeoutMin * 60` seconds
- Control lost mode: Uses `SELF_DESTRUCTION_TMR_LOST_PERIOD_MINUTES * 60` seconds

### 7. Memory Usage
- Added 2 bytes to `system_status_t` structure for the counter
- Minimal code size increase for new functions

## Verification
- Code compiles successfully without errors
- All existing timer modes preserved
- Integration with sys_info timer display maintained
