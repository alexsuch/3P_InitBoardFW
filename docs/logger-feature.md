# SPI Logger Feature — STM32G431 (IB_Wing) ↔ ESP32-S3 Logger

This document describes the **SPI logger** architecture in `3P_InitBoardFW`.

In this system:

- **STM32G431** samples **ADC2** and reads **LSM6DS3 IMU**, merges the data into **fixed-size frames**, and streams them over **SPI2 (slave)**.
- **ESP32‑S3** acts as **SPI master**, reads the frames, and writes them to an SD card as a `.dat` file.
- The **log converter** turns `.dat` files into CSV (`adc_data.csv`, `imu_data.csv`, etc.) without forcing “monotonic” timestamps.

---

## Binary Log Format (on SD card)

Each `.dat` file is a simple concatenation:

```text
[ 64B logger_config_t header ][ 852B logger_frame_t ][ 852B logger_frame_t ]...
```

### Header (`logger_config_t`, 64 bytes)

Written by ESP32 at the start of the file (copied from STM32 config response).

Key fields:

- `magic = 0xCAFE`, version `major.minor`
- ADC: `adc_sample_rate_khz`, `adc_block_size` (typically 100 kHz, 256)
- IMU config snapshot (presence flags, ODR, ranges, register snapshots)
- `reserved[0]` = checksum algorithm id (1=CRC8_SW, 2=SUM8, 3=CRC8_HW)

### Frame (`logger_frame_t` / `LogFrame_t`, 852 bytes)

Fixed layout (little-endian, packed):

|           Field | Size | Notes                                                                  |
| --------------: | ---: | ---------------------------------------------------------------------- |
|         `magic` |    2 | `0x5A5A`                                                               |
|         `n_imu` |    2 | number of valid IMU samples in `imu[]` (0..20)                         |
| `adc_timestamp` |    4 | timestamp of the **first** ADC sample in this 256-sample block (ticks) |
|      `adc[256]` |  512 | `int16_t`                                                              |
|       `imu[20]` |  320 | each sample is 12 raw bytes + `uint32_t timestamp` (ticks)             |
|   `mavlink_log` |   10 | MAVLink flags + basic telemetry (may be all zeros)                     |
|     `checksum8` |    1 | CRC8/SUM8 depending on build                                           |
|  `checksum_pad` |    1 | reserved (0)                                                           |

### Timestamps (ticks → microseconds)

Firmware timestamps are **ticks** at `config.adc_sample_rate_khz * 1000` Hz.
Example: 100 kHz ⇒ 1 tick = 10 µs.

The log converter converts ticks to `timestamp_us` using:

```text
timestamp_us = ticks * 1000 / adc_sample_rate_khz
```

---

## Build / Config Switches (STM32 side)

Main switch:

- `IB_Wing_STM32G431CBT6/Core/Inc/prj_config.h`: `SPI_LOGGER_ENABLE`
  - `0`: “standard” application mode
  - `1`: logger mode (SPI logger pipeline active)

Logger-related macros:

- `ADC_SAMPLING_FREQ_KHZ` (default 100)
- `ADC_DMA_BUFFER_SIZE` (default 512 samples)
- `ADC_DMA_HALF_SIZE` (default 256 samples)
- `LOGGER_ADC_BLOCK_SIZE` (typically 256)
- `LSM6DS3_SAMPLING_FREQ_HZ` (IMU ODR: 1666 / 3332 / 6664, etc.)
- `LOGGER_CHECKSUM_ALGO` (CRC8 / SUM8 / CRC8_HW)

---

## Configuration & Persistence

The STM32 firmware maintains the **source of truth** for the logger configuration.

### Getting Configuration (CMD 42)

The ESP32 requests the configuration at startup (CMD 42). The STM32 responds with its current settings (sampling rates, enabled sensors, version).

### Setting Configuration (CMD 45) — **Draft / TODO**

Currently, the `SET_CONFIG` (CMD 45) logic is **not fully implemented** on the STM32 side.

- **Current Behavior:** Configuration is hardcoded in `prj_config.h` and `Logger_Init()`. Any valid `logger_config_t` received acts as read-only state.
- **Persistence:** Since settings are compiled-in, they **reset to defaults** on every MCU reboot. There is currently no EEPROM/Flash storage for runtime configuration changes.
- **MAVLink Logging:** Enabled by default. To disable, `loggerStat.config.mavlink_logging_enabled` must be modified in firmware or via a future `SET_CONFIG` implementation.

---

## High-Level End-to-End Data Flow

```text
   (SPI1)                         (SPI2 slave)              (SPI3 master)            (SPI2 master)
IMU LSM6DS3 ----INT---->  STM32G431 Logger firmware  --RDY--> ESP32-S3 logger fw  --> SD card (.dat)
                      \      - ADC2 @100kHz DMA           /   - STM link task
                       \     - IMU read + timestamp      /    - SD writer task
                        \    - frame builder + checksum /
                         \   - SPI2 TX DMA frames       /
```

---

## STM32 Architecture (producer)

Main code areas:

- `shared/Src/logger_spi.c` — ADC buffer, IMU ring buffer, frame builder, SPI2 protocol
- `shared/Src/LSM6DS3.c` — IMU init + data acquisition callback into logger
- `IB_Wing_STM32G431CBT6/Core/Src/solution_wrapper.c` — HAL callbacks and timestamp source

### 1) Timestamp source (shared timebase)

The logger uses a hardware counter:

- **TIM7** runs as a free-running counter at `ADC_SAMPLING_FREQ_KHZ` (e.g., 100 kHz).
- `Logger_GetTimestamp()` extends TIM7 to 32-bit by tracking overflows (`UIF`).
- It is atomic under PRIMASK because it can be called from both ISR and task contexts.

This single timebase is used for:

- ADC block timestamps (`adc_timestamp`)
- IMU sample timestamps (`imu[i].timestamp`)
- MAVLink data association (stored in the same frame)

### 2) ADC2 sampling pipeline

```text
TIM6 TRGO @100kHz -> ADC2 -> DMA circular buffer (512 samples)
TIM7 @100kHz (free-running) -> `Logger_GetTimestamp()` timebase for both ADC/IMU
                        |
                        +-> HAL_ADC_ConvHalf/ConvCpltCallback
                              |
                              v
                        Logger_AdcBuffer_OnComplete(256 samples)
                        - copies 256 samples into queued storage
                        - computes ts_first = ts_end - (256-1)
                        - stores ts_first for this block
```

Internals:

- ADC block queue depth: `ADC_BLOCK_QUEUE_DEPTH = 4` (small backlog buffer).
- Overruns are counted if producer outruns consumer.

### 3) IMU sampling pipeline (LSM6DS3)

Current implementation uses **SPI1 DMA burst reads** gated by the IMU INT pin:

```text
LSM6DS3 INT (level) -> Lsm6ds3_Task() polling ReadAccIntGpio()
                         |
                         v
                   SpiGetAccData() (SPI1 TransmitReceive DMA, 13 bytes)
                         |
                         v
                   Lsm6ds3_GetDataCbk()
                         |
                         v
                   Logger_ImuOnNewSample(12 raw bytes)
                         |
                         v
                   IMU ring buffer (max 50 samples)
```

Important nuance:

- The IMU INT/DRDY is treated as a **level** signal in this code path (not EXTI edge-trigger).
- If INT stays HIGH, polling can trigger multiple reads before the IMU updates output registers, producing **duplicate raw samples**.
- To avoid that, the driver uses a **“re-arm” rule**: allow one read while INT is HIGH, and require INT to go LOW before allowing the next read.

### 4) Frame builder (ADC + IMU merge)

`Logger_Task()` runs in the main application loop and builds one frame per ADC block:

1. Wait for an ADC block (256 samples + `ts_first`) from the ADC buffer queue.
2. Pop IMU samples from the ring buffer:
   - ring buffer can hold up to 50, but the frame stores up to **20** IMU samples
3. Assemble `LogFrame_t`:
   - `magic`, `n_imu`, `adc_timestamp`, `adc[]`, `imu[]`, `mavlink_log`
4. Compute checksum **in slices** (multiple parts) to reduce worst-case blocking.
5. Enqueue frame into a small frame FIFO (`LOGGER_FRAME_QUEUE_SIZE = 10`).

### 5) SPI2 slave streaming to master (CMD 42 only)

Signals:

- SPI2 pins: `PB12` NSS, `PB13` SCK, `PB14` MISO, `PB15` MOSI
- Data-ready GPIO: `PB11` (`LOGGER_SPI_DATA_RDY_PIN`) driven by `Logger_GPIO_SetReady()`

Protocol:

1. On boot `Logger_Init()` arms `HAL_SPI_Receive_IT(..., 5)` to receive a 5-byte command packet.
2. Master sends 5 bytes; byte 0 is command.
3. Supported command:
   - `LOGGER_SPI_CMD_CONFIG = 42 (0x2A)` → read `logger_config_t` (64 bytes)
4. On command receive:
   - STM32 transmits `logger_config_t` via `HAL_SPI_Transmit_DMA` and raises DATA_RDY during TX.
5. After config TX completes:
   - STM32 starts acquisition by calling `Solution_LoggingStart()`
   - frames begin to be produced and streamed automatically via SPI2 TX DMA when available

---

## ESP32-S3 Architecture (consumer + storage)

Main code areas:

- `logger/main/modules/remote_accel_reader.c` — SPI master to STM32 (config + frames)
- `logger/main/modules/logger_module.c` — buffered binary writer to SD
- `logger/main/app_logic.c` — button handling + mode transitions
- `logger/include/target.h` — pin mapping + task core affinity + SPI clocks

### Task placement (dual-core)

Pinned task layout (see `logger/include/target.h`):

- Core 0:
  - `APP_LOGIC` (buttons/modes)
  - `REM_STM` (STM32 link reader)
- Core 1:
  - `LOG_WRITE` (SD writer)

This reduces contention between SPI ingest and SD writes.

### STM32 link reader (SPI master)

The ESP reads STM32 data via a dedicated SPI bus and a dedicated “INT/READY” GPIO:

- SPI host: `LINK_SPI_HOST` (typically SPI3)
- Clock: `LINK_SPI_FREQ_HZ` (currently 8 MHz)
- Ready/INT: `LINK_INT_GPIO` (GPIO interrupt on posedge)

Flow:

1. **Config handshake**
   - Send `CMD 42` (5 bytes: `{42,0,0,0,0}`)
   - Wait for `LINK_INT_GPIO` to assert (with timeout)
   - Read 64 bytes config
   - If config is missing key IMU fields right after reset, retry after short delay
   - If it fails repeatedly, reset STM32 and retry; after 5 attempts enter ERROR state
   - On success, pass config to `logger_module_set_stm_config()` (writes 64B header to file)
2. **Frame streaming (LOGGING mode only)**
   - When `LINK_INT_GPIO` is HIGH, read `logger_frame_t` (852 bytes)
   - Validate `magic` before writing
   - Push bytes into logger buffers (`logger_module_write_frame()`)

When not logging, the reader intentionally ignores streaming frames to keep SPI traffic minimal.

### SD writer (buffered binary logging)

ESP writes raw binary frames to SD using ping/pong buffers:

- Two buffers (default 16 KB each)
- A small queue of “chunks” to the writer task
- Writer task writes chunks to SD and periodically flushes/fsyncs

This keeps the SPI ingest path fast and avoids blocking on SD I/O.

---

## Diagnostics / Tooling

The converter produces:

- `config.csv` (decoded from 64B header)
- `frame_status.csv` (per-frame magic + checksum check)
- `timestamp_anomalies.csv` (ADC timestamp non-monotonic / delta mismatch)
- `imu_anomalies.csv` (IMU duplicates / gaps)
- `adc_data.csv`, `imu_data.csv`, `mavlink_events.csv`

---

## Known Pitfalls / Notes

- **IMU duplicates** can appear if IMU INT is treated as a level and polled too frequently. The driver uses a “re-arm” rule to avoid repeated reads while INT remains HIGH.
- **Startup gaps** in IMU data can happen right after reset while IMU is still initializing; first few frames may have `n_imu=0`.
- **Checksum validation**: frames include an 8-bit checksum; validation is typically done offline (converter).
- **Throughput**: at 100 kHz ADC and block 256, frame rate is ~390.6 fps; raw payload rate is ~333 kB/s (852 \* 390.6). SPI link frequency must be comfortably above this with margin.

---

## File Map (most relevant)

STM32:

- `shared/Inc/logger.h` — public logger structs (`logger_config_t`, `LogFrame_t`) and constants
- `shared/Src/logger_spi.c` — buffers + frame builder + SPI2 protocol callbacks
- `shared/Src/LSM6DS3.c` — IMU init + acquisition, `Logger_ImuOnNewSample()`
- `IB_Wing_STM32G431CBT6/Core/Src/solution_wrapper.c` — timestamp + HAL callbacks + `Solution_LoggingStart()`

ESP32-S3:

- `logger/main/modules/remote_accel_reader.c` — config handshake + frame reads
- `logger/main/modules/logger_module.c` — buffered SD writes (`.dat`)
- `logger/main/app_logic.c` — mode transitions (IDLE/LOGGING/ERROR)
- `logger/include/target.h` — pin mapping, SPI clocks, task core affinity
