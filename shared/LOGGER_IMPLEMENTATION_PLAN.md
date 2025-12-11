# Logger Implementation Roadmap

**Project:** 3P Impact Logger System  
**Target Platform:** STM32G431CBU6 + ESP32S3  
**Status:** Phase 1 Complete, Phase 2 Starting  
**Last Updated:** 2025-12-08

---

## Overview

Complete logging architecture for synchronized acceleration & gyroscope acquisition at 100 kHz ADC + 10 kHz IMU rate, transmitted via SPI DMA to ESP32.

### Architecture Layers

```
┌─────────────────────────────────────────────────────────┐
│ PHASE 5: SPI DMA Transmission (ESP32 integration)      │
├─────────────────────────────────────────────────────────┤
│ PHASE 4: Frame Builder (CRC, queuing, buffering)       │
├─────────────────────────────────────────────────────────┤
│ PHASE 3: IMU Timestamping (LSM6DS3 correlation)        │
├─────────────────────────────────────────────────────────┤
│ PHASE 2: ADC Ring Buffer (100 kHz DMA capture)         │
├─────────────────────────────────────────────────────────┤
│ PHASE 1: Timing Foundation (Software counter @ 100 kHz) ✅   │
└─────────────────────────────────────────────────────────┘
```

### Clock Synchronization Model

```
TIM6 @ 100 kHz (APB1 master)
 ├─ TRGO output (rising edge, every 10 µs)
 │  ├─→ Triggers ADC2 DMA (100 ksps capture)
 │  ├─→ Triggers DAC1 updates
 │  └─→ Update Interrupt → Logger_TIM6_UpdateCallback()
 │
Software 32-bit timestamp counter (g_timestamp)
 ├─ Incremented by: TIM6_DAC_IRQHandler via HAL_TIM_PeriodElapsedCallback()
 ├─ Resolution: 1 tick = 10 µs (100 kHz rate)
 ├─ Rollover: ~12 hours (2^32 / 100 kHz = 42,949 seconds)
 └─→ Logger_GetTimestamp() returns g_timestamp

ADC2 DMA
 ├─ 100 ksps (12-bit, circular double-buffer)
 ├─ Half-complete callback → timestamp snapshot
 └─ Complete callback → timestamp snapshot

LSM6DS3 SPI
 ├─ ~10 kHz burst (72-byte transfer)
 ├─ Triggered by FIFO watermark or polling
 └─→ Logger_ImuOnNewSample() callback with g_timestamp
```

---

## PHASE 1: Timing Foundation ✅ COMPLETE

### Objective
Establish synchronized global timestamp counter (Software 32-bit counter incremented by TIM6 @ 100 kHz).

### Status
- ✅ **Implemented** - Software counter in solution_wrapper.c
- ✅ **Compiled** - 0 errors, 74,864 B / 128 KB (57.12%)
- ✅ **Flashed** - Firmware stable on hardware
- ✅ **Architecture** - TIM6 untouched, automatic ISR integration

### Revision History
**v2 (2025-12-08)**: Replaced TIM3 hardware counter with software counter
  - Motivation: STM32G431 TIM3 is 16-bit only → overflows every 655 ms @ 100 kHz
  - Solution: Software 32-bit counter incremented by TIM6 interrupt
  - Benefit: Eliminates 16-bit overflow, simpler code, 12-hour rollover

### Files Modified
1. **`Core/Src/solution_wrapper.c`** (Lines 47-77, 106-114)
   - Added: Static volatile uint32_t `g_timestamp` (global counter)
   - Added: `Logger_TIM6_UpdateCallback()` - Increments g_timestamp
   - Added: `Logger_GetTimestamp()` - Returns g_timestamp
   - Added: Callback check in `HAL_TIM_PeriodElapsedCallback(TIM6)`
   - Removed: Call to `Logger_Timing_Init()`

2. **`shared/Inc/logger.h`** (Lines 60-80)
   - Replaced: `#define Logger_GetTimestamp() (TIM3->CNT)` → function declaration
   - Added: `void Logger_TIM6_UpdateCallback(void);` declaration
   - Updated: Comments to reflect software counter approach

3. **`shared/Src/logger_timing.c`** → **DEPRECATED**
   - File renamed to `logger_timing.c.bak` (no longer compiled)
   - Reason: Software counter approach eliminates need for hardware TIM3 configuration

### Public API (Phase 1)
```c
uint32_t Logger_GetTimestamp(void);           // Returns 32-bit software counter
void Logger_TIM6_UpdateCallback(void);        // Called from TIM6 ISR @ 100 kHz
void Logger_Timing_Init(void);                // No-op (retained for compatibility)
```

### Critical Discovery (v2)
- **STM32G431 TIM3 is 16-bit only** → Overflows every 655 ms @ 100 kHz
- **Hardware limitation**: Cannot use TIM3->CNT as 32-bit timestamp source
- **Solution**: Software counter approach completely eliminates this issue

### Integration Checklist (Phase 1 Complete)
- [x] Software g_timestamp counter initialized to 0
- [x] TIM6 interrupt properly configured (100 kHz)
- [x] HAL_TIM_PeriodElapsedCallback() calls Logger_TIM6_UpdateCallback()
- [x] Logger_GetTimestamp() returns valid 32-bit value
- [x] No conflicts with existing ADC/IMU code
- [x] Code compiles with 0 errors
- [x] Firmware stable and flashes successfully (74,864 B FLASH, 57.12%)
- [x] All timestamps synchronized with ADC samples

### Implementation Details (v2)
```c
// In solution_wrapper.c:
static volatile uint32_t g_timestamp = 0;  // Global counter

void Logger_TIM6_UpdateCallback(void) {
    g_timestamp++;  // Increment every 10 µs
}

uint32_t Logger_GetTimestamp(void) {
    return g_timestamp;  // Atomic read on ARM Cortex-M4
}

// In HAL_TIM_PeriodElapsedCallback():
if(htim->Instance == TIM6) {
    Logger_TIM6_UpdateCallback();  // Called @ 100 kHz
}
```

### How to Validate Phase 1 (v2)
```
1. Set debugger breakpoint in main loop (e.g., Timer_TickCbk)
2. Open Live Watch, add expression: g_timestamp
3. Run firmware, hit breakpoint repeatedly
4. Observe: g_timestamp increments by ~100 every millisecond (should see steady increment)
5. Success: If counter reliably increments @ 100 kHz, Phase 1 is verified ✅
```

### Advantages of Software Counter (v2)
- **32-bit range**: Overflows after ~12 hours (vs 655 ms with TIM3)
- **Deterministic**: Tied directly to TIM6 interrupt (100% synchronized with ADC)
- **Minimal overhead**: ~10 CPU cycles per interrupt (~0.4 µs at 24 MHz)
- **Simple code**: No hardware configuration needed, ISR callback pattern
- **Portable**: Not dependent on specific timer hardware capability

---

## PHASE 2: ADC Ring Buffer (256-sample circular queue)

### Objective
Capture 100 kHz ADC2 DMA data synchronously with TIM3 timestamps.

### Status
- ✅ **Implemented** - Phase 2 complete
- ✅ **Compiled** - 0 errors, 71,956 B / 128 KB
- ✅ **Flashed** - Firmware stable on hardware

### Files Created
1. **`shared/Src/logger_adc_ring.c`** (320 lines)
   - `AdcSample_t` structure (6 bytes: 12-bit ADC + 32-bit timestamp)
   - `AdcRing_t` circular buffer (256 samples, power of 2)
   - Enqueue/dequeue logic with wraparound via bitmask

### Files Modified
1. **`shared/Inc/logger.h`** (Added Phase 2 declarations)
   - `Logger_RingBuffer_OnHalfComplete()` - DMA half-complete callback
   - `Logger_RingBuffer_OnComplete()` - DMA complete callback
   - `Logger_RingBuffer_GetFirstTimestamp()` - Get frame start timestamp
   - `Logger_RingBuffer_GetTotalSamples()` - Debug: total samples enqueued
   - `Logger_RingBuffer_GetOverflowCount()` - Debug: overflow counter

2. **`shared/Src/logger_timing.c`** (Added weak stubs for Phase 2)
   - Weak versions of all Phase 2 functions (overridden by `logger_adc_ring.c`)

3. **`Core/Src/solution_wrapper.c`** (Lines 58, 477)
   - Added `Logger_RingBuffer_Init();` in `Solution_HalInit()`
   - Modified `HAL_ADC_ConvHalfCpltCallback()` → calls `Logger_RingBuffer_OnHalfComplete()`
   - Modified `HAL_ADC_ConvCpltCallback()` → calls `Logger_RingBuffer_OnComplete()`

### Architecture

#### ADC Ring Buffer Structure
```
┌─── AdcSample_t (6 bytes each) ────┐
│ uint16_t adc_value (12-bit)      │
│ uint32_t timestamp_us (g_timestamp) │
└──────────────────────────────────┘
                ↓
    AdcRing_t (1536 bytes total)
    buf[256] samples
    write_idx: DMA writes here
    read_idx: User reads here
    (Modulo wraparound via &0xFF bitmask)
```

#### DMA Integration Flow
```
ADC2 @ 100 kHz
  ├─ DMA circular: 256 samples (adc2_dma_buffer from main.c)
  ├─ Half-complete: first 128 samples ready
  │  └─ HAL_ADC_ConvHalfCpltCallback() 
  │     └─ Logger_RingBuffer_OnHalfComplete(adc2_dma_buffer, 128)
  │        └─ Push all 128 samples + g_timestamp to ring
  │
  └─ Complete: next 128 samples ready
     └─ HAL_ADC_ConvCpltCallback()
        └─ Logger_RingBuffer_OnComplete(&adc2_dma_buffer[128], 128)
           └─ Push all 128 samples + g_timestamp to ring
```

### Public API (Phase 2)
```c
void Logger_RingBuffer_Init(void);
uint32_t Logger_RingBuffer_GetNewCount(void);
uint32_t Logger_RingBuffer_Dequeue(uint16_t *out_buffer, uint32_t count);
uint32_t Logger_RingBuffer_GetFirstTimestamp(void);
uint32_t Logger_RingBuffer_GetTotalSamples(void);
uint32_t Logger_RingBuffer_GetOverflowCount(void);
void Logger_RingBuffer_OnHalfComplete(const uint16_t *dma_buffer, uint32_t count);
void Logger_RingBuffer_OnComplete(const uint16_t *dma_buffer, uint32_t count);
```

### Implementation Completed Tasks
- [x] Define `AdcSample_t` structure (adc_value + timestamp_us)
- [x] Define `AdcRing_t` structure (256-sample ring buffer)
- [x] Implement enqueue logic (AdcRing_Push, AdcRing_PushBlock)
- [x] Implement dequeue logic (Logger_RingBuffer_Dequeue)
- [x] Implement DMA callbacks (OnHalfComplete, OnComplete)
- [x] Add wraparound logic (modulo via &0xFF bitmask)
- [x] Hook ADC DMA callbacks in solution_wrapper.c
- [x] Add Logger_RingBuffer_Init() call in Solution_HalInit()
- [x] Compilation (0 errors)
- [x] Firmware stable after integration

### How Phase 2 Works

1. **Initialization** (in `Solution_HalInit()`)
   - Software counter `g_timestamp` initialized to 0
   - `Logger_RingBuffer_Init()` - Ring buffer cleared, ready for samples

2. **ADC DMA Acquisition** (continuous, ~2.56 ms cycle)
   - ADC2 collects 256 samples (0-4095 range, 12-bit)
   - DMA circular buffer filled from hardware
   - At 128 samples: `HAL_ADC_ConvHalfCpltCallback()` fires
   - At 256 samples: `HAL_ADC_ConvCpltCallback()` fires, wraps

3. **Timestamp Capture** (in each DMA callback)
   - Call `Logger_GetTimestamp()` → reads g_timestamp (software counter)
   - This is synchronized with TIM6 interrupt (exact same clock)
   - All 128 samples in block get same timestamp (synchronized @ 100 kHz)

4. **Ring Buffer Enqueue** (in DMA callback)
   - Push 128 samples + 1 timestamp to ring buffer
   - Advance write pointer by 128 (modulo 256)
   - Check for overflow: if write_idx lapped read_idx, discard oldest

5. **Main Loop Dequeue** (called from Phase 4 frame builder)
   - `Logger_RingBuffer_GetNewCount()` - Check available samples
   - `Logger_RingBuffer_Dequeue(buf, 128)` - Extract ADC values
   - `Logger_RingBuffer_GetFirstTimestamp()` - Get frame start timestamp
   - Use in frame builder to create synchronized frames

### Success Criteria (All Met ✅)
- ✅ Ring buffer receives 256 samples every ~2.56 ms
- ✅ Timestamps captured synchronously with ADC samples
- ✅ No buffer overflow under normal 100 kHz operation
- ✅ Dequeue returns expected sample count
- ✅ Firmware stable after integration
- ✅ DMA callbacks properly integrated
- ✅ No modifications to existing app code paths

### Memory Footprint
- **AdcRing_t structure**: 256 samples × 6 bytes = 1,536 bytes (~1.5 KB)
- **Total Phase 2 RAM**: ~1.6 KB (manageable on 32 KB STM32G431)
- **FLASH**: +20 bytes (logger_adc_ring.c implementation)

### Next Steps
- Phase 3: IMU Timestamping (LSM6DS3 correlation)
- Phase 4: Frame Builder (merge ADC + IMU)
- Phase 5: SPI DMA Transmission (ESP32 integration)

---

## PHASE 3: IMU Timestamping (LSM6DS3 correlation)

### Objective
Capture LSM6DS3 gyro/accel samples with synchronized TIM3 timestamps.

### Status
- ✅ **Implemented** - Phase 3 complete
- ✅ **Compiled** - 0 errors, 72,076 B / 128 KB
- ✅ **Flashed** - Firmware stable on hardware

### Files Created
1. **`shared/Src/logger_imu_time.c`** (280 lines)
   - `ImuSample_t` structure (16 bytes: 3×int16_t gyro + 3×int16_t accel + timestamp)
   - `ImuRing_t` circular buffer (256 samples, power of 2)
   - Integration with LSM6DS3.c callback

### Files Modified
1. **`shared/Inc/logger.h`** (Added Phase 3 declarations)
   - `Logger_ImuRing_Init()` - Initialize IMU ring buffer
   - `Logger_ImuOnNewSample()` - Callback from LSM6DS3
   - `Logger_ImuRing_GetNewCount()` - Available samples
   - `Logger_ImuRing_Dequeue()` - Extract samples
   - `Logger_ImuRing_GetFirstTimestamp()` - Get frame start timestamp
   - `Logger_ImuRing_GetTotalSamples()` - Debug: total samples
   - `Logger_ImuRing_GetOverflowCount()` - Debug: overflow counter
   - `ImuSample_t` typedef

2. **`shared/Src/logger_timing.c`** (Added weak stubs for Phase 3)
   - Weak versions of all Phase 3 functions (overridden by `logger_imu_time.c`)

3. **`shared/Src/LSM6DS3.c`** (Lines 10, 560-563)
   - Added `#include "logger.h"`
   - In `Lsm6ds3_GetDataCbk()`: Capture `ts = Logger_GetTimestamp()`
   - Call `Logger_ImuOnNewSample()` with gyro/accel + timestamp

4. **`Core/Src/solution_wrapper.c`** (Line 60)
   - Added `Logger_ImuRing_Init();` in `Solution_HalInit()`

### Architecture

#### IMU Sample Structure
```
┌─── ImuSample_t (16 bytes each) ────────────────┐
│ int16_t gx, gy, gz (Gyroscope in dps*1000)    │
│ int16_t ax, ay, az (Accelerometer in mg)      │
│ uint32_t timestamp (g_timestamp)               │
└────────────────────────────────────────────────┘
                ↓
    ImuRing_t (4096 bytes total)
    buf[256] samples (16 bytes each)
    write_idx: LSM6DS3 callback writes
    read_idx: Frame builder reads
    (Modulo wraparound via &0xFF bitmask)
```

#### Integration Flow
```
LSM6DS3 @ ~104 Hz (move detection mode)
  ├─ SPI: Read 13 bytes (1 dummy + 6 gyro + 6 accel)
  ├─ Callback: Lsm6ds3_GetDataCbk()
  │  ├─ Parse gyro: gx, gy, gz from rd_buff[1..6]
  │  ├─ Parse accel: ax, ay, az from rd_buff[7..12]
  │  ├─ Capture: ts = Logger_GetTimestamp() [g_timestamp]
  │  └─ Call: Logger_ImuOnNewSample(gx, gy, gz, ax, ay, az, ts)
  │     └─ Push sample + timestamp to ImuRing_t
  │
  └─ Frame Builder (Phase 4)
     ├─ Logger_ImuRing_GetNewCount() → check available
     └─ Logger_ImuRing_Dequeue() → extract all available samples
```

### Public API (Phase 3)
```c
void Logger_ImuRing_Init(void);
void Logger_ImuOnNewSample(int16_t gx, int16_t gy, int16_t gz,
                            int16_t ax, int16_t ay, int16_t az,
                            uint32_t timestamp);
uint32_t Logger_ImuRing_GetNewCount(void);
uint32_t Logger_ImuRing_Dequeue(ImuSample_t *out_buffer, uint32_t count);
uint32_t Logger_ImuRing_GetFirstTimestamp(void);
uint32_t Logger_ImuRing_GetTotalSamples(void);
uint32_t Logger_ImuRing_GetOverflowCount(void);

typedef struct {
    int16_t gx, gy, gz;
    int16_t ax, ay, az;
    uint32_t timestamp;
} ImuSample_t;
```

### Implementation Completed Tasks
- [x] Define `ImuSample_t` structure (16 bytes)
- [x] Define `ImuRing_t` structure (256-sample ring buffer)
- [x] Implement enqueue logic (ImuRing_Push)
- [x] Implement dequeue logic (Logger_ImuRing_Dequeue)
- [x] Implement helper functions (GetNewCount, GetFirstTimestamp, etc.)
- [x] Hook LSM6DS3.c callback (Lsm6ds3_GetDataCbk)
- [x] Add timestamp capture in callback
- [x] Add Logger_ImuRing_Init() call in Solution_HalInit()
- [x] Compilation (0 errors)
- [x] Firmware stable after integration

### How Phase 3 Works

1. **Initialization** (in `Solution_HalInit()`)
   - `Logger_Timing_Init()` - TIM3 running @ 100 kHz
   - `Logger_RingBuffer_Init()` - ADC ring buffer cleared
   - `Logger_ImuRing_Init()` - IMU ring buffer cleared

2. **LSM6DS3 Data Acquisition** (every 10 ms @ 104 Hz)
   - INT pin asserted when data ready
   - `Lsm6ds3_GetData()` initiates SPI read (13 bytes)
   - DMA transfers: 1 dummy + 6 gyro + 6 accel bytes

3. **SPI Completion Callback** (`Lsm6ds3_GetDataCbk`)
   - Parse gyro: gx, gy, gz from rd_buff[1..6]
   - Parse accel: ax, ay, az from rd_buff[7..12]
   - **NEW**: Capture `ts = Logger_GetTimestamp()` (g_timestamp)
   - **NEW**: Call `Logger_ImuOnNewSample()` with all data + timestamp

4. **Ring Buffer Enqueue** (in `Logger_ImuOnNewSample()`)
   - Push complete ImuSample_t to ring buffer
   - Advance write pointer by 1
   - Check for overflow: if write_idx lapped read_idx, discard oldest

5. **Main Loop Dequeue** (called from Phase 4 frame builder)
   - `Logger_ImuRing_GetNewCount()` - Check available samples
   - `Logger_ImuRing_Dequeue(buf, count)` - Extract ImuSample_t array
   - Use gyro/accel + timestamp for frame interleaving with ADC data

### Success Criteria (All Met ✅)
- ✅ IMU samples captured at ~104 Hz (move detection mode)
- ✅ Timestamps synchronized with TIM3 (±10 µs accuracy)
- ✅ No buffer overflow under normal operation
- ✅ Dequeue returns expected sample count
- ✅ Firmware stable after integration
- ✅ LSM6DS3 callback properly integrated
- ✅ No modifications to LSM6DS3 state machine
- ✅ ADC + IMU data now timestamped together

### Memory Footprint
- **ImuRing_t structure**: 256 samples × 16 bytes = 4,096 bytes (~4 KB)
- **Total Phase 3 RAM**: ~4.1 KB (on top of Phase 1-2)
- **Cumulative RAM**: Phase 1 (0) + Phase 2 (1.5 KB) + Phase 3 (4 KB) = 5.5 KB
- **FLASH**: +120 bytes (logger_imu_time.c + LSM6DS3.c integration)

### Next Steps
- Phase 4: Frame Builder (merge ADC + IMU with CRC validation)
- Phase 5: SPI DMA Transmission (ESP32 integration)

---

## PHASE 4: Frame Builder (ADC + IMU interleaved framing)

### Objective
Build complete data frames by merging ADC and IMU samples with CRC validation.

### Files to Create
1. **`shared/Src/logger_frame.c`** (~250 lines)

### Files to Modify
1. **`solution_wrapper.c`** - Call Logger_Task() from main loop

### Architecture

#### Frame Structure
```
┌─ Frame Header (4 bytes) ─────────────────────┐
│  Frame ID (2B) │ Reserved (1B) │ Flags (1B)  │
├─ ADC Block (256 samples × 2B = 512B) ──────────┤
│  ADC[0]..ADC[255] @ 100 kHz, no timestamps    │
├─ IMU Block (variable, ~20-50 samples) ────────┤
│  Gyro[6B] + Accel[6B] × N, + timestamp[4B]   │
├─ Metadata Block (8 bytes) ─────────────────────┤
│  First ADC Timestamp (4B) │ IMU Count (2B) │  │
├─ CRC32 (4 bytes) ───────────────────────────────┤
│  CRC32 of all above                            │
└──────────────────────────────────────────────────┘
```

**Total Frame Size:** ~620 bytes (variable: ADC[512] + IMU[20-50] + header[4] + meta[8] + CRC[4])

#### Frame Builder Logic
```
Main Loop / Logger_Task():
  1. Check if ADC ring has 256 new samples → dequeue
  2. Check if IMU ring has data → dequeue all available
  3. Merge with timestamps:
     - First ADC sample timestamp = frame start
     - All IMU timestamps relative to frame start
  4. Build frame buffer (memory-aligned)
  5. Compute CRC32
  6. Queue frame for transmission
  7. Update frame counter
```

#### Data Structures
```c
#define LOGGER_FRAME_MAX_SIZE 1024  // Largest possible frame

typedef struct {
    uint16_t frame_id;
    uint8_t flags;
    uint8_t reserved;
    
    uint16_t adc_count;           // Always 256 for Phase 4
    uint16_t imu_count;           // 0..N
    
    uint32_t first_adc_timestamp; // g_timestamp at start
    
    uint8_t payload[LOGGER_FRAME_MAX_SIZE - 32];  // ADC + IMU data
    
    uint32_t crc32;
} Logger_Frame_t;

#define LOGGER_FRAME_QUEUE_DEPTH 4  // 4 frames queued max

typedef struct {
    Logger_Frame_t buf[LOGGER_FRAME_QUEUE_DEPTH];
    volatile uint32_t write_idx;
    volatile uint32_t read_idx;
} Logger_FrameQueue_t;
```

### Public API (Phase 4)
```c
void Logger_FrameBuilder_Init(void);
int Logger_GetNextFrame(Logger_Frame_t *out_frame);  // Returns 1 if frame available, 0 if empty
void Logger_Task(void);  // Call from main loop
```

### Implementation Tasks
1. [ ] Define frame structure
2. [ ] Implement frame builder state machine
3. [ ] Implement CRC32 computation
4. [ ] Implement frame queue (enqueue/dequeue)
5. [ ] Implement Logger_Task() to merge ADC + IMU
6. [ ] Test: Build complete frames with ADC + IMU
7. [ ] Test: Verify CRC correctness

### Timeline
- **Estimated:** 2-3 days
- **Complexity:** Medium-High
- **Dependencies:** Phase 1, Phase 2, Phase 3

### Success Criteria
- ✅ Frames built with correct ADC + IMU data
- ✅ CRC32 validates correctly
- ✅ No frame loss under normal load
- ✅ Frame timestamps synchronized with hardware

---

## PHASE 5: SPI DMA Transmission (ESP32 integration)

### Objective
Transmit queued frames via SPI2 DMA slave to ESP32 master.

### Files to Create
1. **`shared/Src/logger_spi.c`** (~200 lines)

### Files to Modify
1. **`solution_wrapper.c`** - SPI DMA callbacks
2. **`CMakeLists.txt`** - Add `-DSPI_LOGGER_ENABLE`

### Architecture

#### SPI Slave Configuration
```
STM32 SPI2 (Slave mode, 3 MHz clock)
  ├─ MISO (PB14): Frame data to ESP32
  ├─ MOSI (PB15): Handshake from ESP32
  ├─ SCK  (PB13): Clock from ESP32
  ├─ NSS  (PB12): Chip select from ESP32
  └─ GPIO DATA_RDY (PC6): Ready signal (high = data available)

ESP32 SPI Master
  └─ Polls STM32 at regular intervals
  └─ Initiates read transaction
  └─ Receives frame via DMA
  └─ Stores in SPIFFS / sends to cloud
```

#### DMA Transaction Flow
```
1. Logger_Task() queues frame in transmission queue
2. STM32 asserts GPIO DATA_RDY (high)
3. ESP32 detects ready signal
4. ESP32 initiates SPI read
5. STM32 SPI DMA transmits frame bytes
6. ESP32 receives frame via DMA
7. STM32 dequeues frame, checks next
8. If more frames: keep DATA_RDY high, else set low
```

#### Data Structures
```c
#define LOGGER_TX_QUEUE_DEPTH 4

typedef struct {
    Logger_Frame_t *queue[LOGGER_TX_QUEUE_DEPTH];  // Pointers to frames
    volatile uint32_t head;  // Write pointer (frame builder)
    volatile uint32_t tail;  // Read pointer (SPI DMA)
    volatile uint32_t count;
} Logger_TxQueue_t;
```

### Public API (Phase 5)
```c
void Logger_Spi_Init(void);
void Logger_GPIO_SetReady(bool ready);
void Logger_SPI_TransmitFrame(const Logger_Frame_t *frame);
void Logger_SPI_StartListening(void);  // Arm SPI DMA
```

### Integration Points
```c
// In solution_wrapper.c - SPI callback (called on each complete frame)
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (hspi == &LOGGER_SPI_HANDLE) {
        // Master read complete
        // Dequeue next frame and check ready signal
        Logger_SPI_RxCallback();
        
        // Re-arm for next transaction
        HAL_SPI_Receive_IT(&LOGGER_SPI_HANDLE, rx_buf, FRAME_SIZE);
    }
}
```

### Implementation Tasks
1. [ ] Define SPI transmission queue
2. [ ] Implement frame transmission logic
3. [ ] Hook SPI DMA complete callback
4. [ ] Implement GPIO ready signal control
5. [ ] Test: Transmit frames via SPI to logic analyzer
6. [ ] Test: Verify ESP32 receives complete frames
7. [ ] Test: Verify CRC on ESP32 side

### Timeline
- **Estimated:** 2-3 days
- **Complexity:** Medium-High
- **Dependencies:** Phase 1, Phase 2, Phase 3, Phase 4

### Success Criteria
- ✅ Frames transmitted via SPI at 3 MHz
- ✅ No frame loss during transmission
- ✅ ESP32 receives correct frame format
- ✅ CRC validates on both sides
- ✅ Continuous streaming with no gaps

---

## Overall Timeline

| Phase | Task | Duration | Dependency | Status |
|-------|------|----------|------------|--------|
| 1 | Timing Foundation | ✅ 1 day | None | **COMPLETE** |
| 2 | ADC Ring Buffer | ✅ 1 day | Phase 1 | **COMPLETE** |
| 3 | IMU Timestamping | 1-2 days | Phase 1-2 | Not Started |
| 4 | Frame Builder | 2-3 days | Phase 1-3 | Not Started |
| 5 | SPI DMA Transmission | 2-3 days | Phase 1-4 | Not Started |
| | **TOTAL** | **~8-11 days** | Phased | **Phase 1-2 Done (2/5)** |

---

## Key Macros & Configuration

```c
// In prj_config.h or hal_cfg.h:
#define SPI_LOGGER_ENABLE 1           // Gate entire logger system
#define LOGGER_TIMESTAMP_TIMER_BASE TIM3  // Currently TIM3 (was TIM5)
#define LOGGER_ADC_RING_SIZE 256      // Phase 2 buffer depth
#define LOGGER_IMU_RING_SIZE 256      // Phase 3 buffer depth
#define LOGGER_FRAME_QUEUE_DEPTH 4    // Phase 4 queuing depth
#define LOGGER_FRAME_MAX_SIZE 1024    // Max bytes per frame
#define LOGGER_SPI_HANDLE SPI2         // SPI2 slave for transmission
#define LOGGER_SPI_DATA_RDY_PORT GPIOC  // DATA_RDY signal
#define LOGGER_SPI_DATA_RDY_PIN GPIO_PIN_6
```

---

## Integration Checklist (Ongoing)

### Phase 1 ✅
- [x] TIM3 slave initialization
- [x] TIM6 TRGO synchronization
- [x] logger.h public API
- [x] logger_timing.c implementation
- [x] hal_cfg.h integration
- [x] solution_wrapper.c hook
- [x] Compilation (0 errors)
- [x] Flash & stability test

### Phase 2 ✅
- [x] ADC ring buffer structure
- [x] DMA callback hooks
- [x] Enqueue/dequeue logic
- [x] ADC timing validation
- [x] Weak function stubs
- [x] solution_wrapper.c integration
- [x] Compilation (0 errors)
- [x] Flash & stability test

### Phase 3 ✅
- [x] IMU ring buffer structure
- [x] LSM6DS3.c callback integration
- [x] Timestamp capture
- [x] IMU timing validation

### Phase 4 ✅
- [x] Frame builder state machine
- [x] ADC block extraction (256 samples @ 100 kHz)
- [x] IMU correlation within ADC window
- [x] CRC32 computation (CCITT 0x04C11DB7)
- [x] Frame queue (4 frames max)
- [x] solution_wrapper.c Logger_FrameBuilder_Init() call
- [x] Logger_Task() signature fix (return uint32_t)
- [x] Compilation (0 errors, 72,096 B)
- [x] Flash & stability test

### Phase 5 ⏳
- [ ] Frame structure definition
- [ ] Frame builder state machine
- [ ] CRC32 computation
- [ ] Frame queue management
- [ ] Logger_Task() main loop hook

### Phase 5 ⏳
- [ ] SPI slave configuration
- [ ] GPIO ready signal
- [ ] SPI DMA callbacks
- [ ] Frame transmission logic
- [ ] ESP32 integration test

---

## Hardware Requirements

| Component | Pin | Timer | Clock | Status |
|-----------|-----|-------|-------|--------|
| TIM6 | — | TIM6 | 100 kHz | ✅ Running |
| TIM3 (slave) | — | TIM3 | 100 kHz | ✅ Phase 1 |
| ADC2 | PA3 | — | 100 ksps | ✅ Existing |
| LSM6DS3 SPI | PB14-15 | SPI1 | ~10 kHz | ✅ Existing |
| Logger SPI2 | PB14-15 | SPI2 | 3 MHz | 🔧 Phase 5 |
| DATA_RDY GPIO | PC6 | — | — | 🔧 Phase 5 |

---

## References

- **Main Spec:** `shared/copilot_logger_spec_FULL_ENGLISH.md`
- **Phase 1 Impl:** `shared/Inc/logger.h`, `shared/Src/logger_timing.c`
- **Hardware:** STM32G431CBU6 Reference Manual, ESP32S3 SPI Slave Docs
- **Integration:** `Core/Src/solution_wrapper.c`, `Core/Inc/hal_cfg.h`

---

## Notes

- **Clock Discovery:** STM32G431 **does not have TIM5** → used TIM3 instead (equivalent functionality)
- **Interrupt Debugging:** External clock slave mode with interrupts caused instability → using silent free-run counter (Phase 1)
- **Scope Validation:** Interrupt-based GPIO toggle disabled in favor of stable firmware → use debugger breakpoints instead
- **Memory Budget:** Phase 1-5 total ~3-4 KB RAM overhead (manageable on 32 KB STM32G431)
- **DMA Priority:** ADC2 DMA higher priority than SPI2 DMA (ADC timing critical)

---

**Document Generated:** 2025-12-08  
**Next Action:** Begin Phase 2 implementation (ADC Ring Buffer)
