# Logger Feature (SPI Logger) — IB_Wing_STM32G431CBT6

This document describes how the **SPI logger** works in `3P_InitBoardFW`, focusing on code under:
- `shared/`
- `IB_Wing_STM32G431CBT6/`

The logger collects:
- **ADC2 samples** (piezo/sensor input) at **100 kHz** via DMA
- **LSM6DS3 IMU raw samples** (gyro+accel) via SPI1 (timestamped)
- It **streams fixed-size frames to an external SPI master** (e.g., ESP32) over **SPI2 (slave)**

Note: despite the repository also containing MAVLink support, **MAVLink data is not part of the SPI logger stream in `SPI_LOGGER_ENABLE` mode** (details below).

---

## Build/Config Switches (Logger Mode)

Primary compile-time switch:
- `IB_Wing_STM32G431CBT6/Core/Inc/prj_config.h`: `SPI_LOGGER_ENABLE`
  - `0`: “standard” application mode (detonation / control features, MAVLink task etc.)
  - `1`: logger mode (SPI logger pipeline runs; other application flows are largely bypassed)

Logger-relevant configuration macros (same file):
- `ADC_SAMPLING_FREQ_KHZ` (default `100u`)
- `ADC_DMA_BUFFER_SIZE` (default `512u` samples)
- `ADC_DMA_HALF_SIZE` (default `256u` samples)
- `LOGGER_ADC_BLOCK_SIZE` (`ADC_DMA_HALF_SIZE`, i.e. 256 samples per block)
- `LSM6DS3_SAMPLING_FREQ_HZ` (IMU ODR; see `LSM6DS3_ODR_VALUE` mapping)

SPI pin/handle mapping:
- `IB_Wing_STM32G431CBT6/Core/Inc/hal_cfg.h`:
  - `LOGGER_SPI_HANDLE` → `hspi2` (SPI2 slave)
  - `LOGGER_SPI_DATA_RDY_PORT/PIN` → `GPIOB/PB11` (“data ready” GPIO to master)

---

## High-Level Data Flow

1. **Init**
   - `shared/Src/app.c`: `App_InitRun()` calls `Logger_Init()` when `SPI_LOGGER_ENABLE == 1u`.
   - `shared/Src/logger_spi.c`: `Logger_Init()` arms SPI2 RX to wait for a config request from the SPI master.

2. **SPI config handshake (one-time)**
   - Master sends **command 42** (`0x2A`) over SPI2.
   - Slave responds with `logger_config_t` (32 bytes).
   - After config is sent, the logger starts ADC/IMU acquisition (`Solution_LoggingStart()`).

3. **Acquisition**
   - **ADC2** continuously samples at 100 kHz via **TIM6 TRGO** + **DMA circular buffer**.
   - **LSM6DS3** delivers raw bursts (12 bytes: gyro+accel) and each is timestamped.

4. **Framing**
   - `Logger_Task()` (called from `App_Task()`) merges:
     - 256 ADC samples + timestamp
     - 0..N IMU samples (typically 0 or 1 per ADC block)
   - Frames are queued in a small ring queue (10 frames).

5. **SPI streaming**
   - When SPI is idle and a frame is available, the slave starts a **SPI2 TX DMA** transfer.
   - A **GPIO “data ready”** line is asserted high during a transfer and lowered on TX complete.

---

## Block Diagram

```text
                       +------------------+
                       |  SPI Master      |
                       |  (ESP32, etc.)   |
                       +---------+--------+
                                 |
                                 | SPI2 (NSS/SCK/MOSI/MISO)
                                 v
+----------------------+   +-------------------------+   +----------------------+
| STM32 SPI2 (slave)   |<->| Logger SPI protocol     |-->| SPI2 TX DMA          |
| hspi2                |   | shared/Src/logger_spi.c |   | + TX complete cbk    |
+----------+-----------+   +-----------+-------------+   +----------+-----------+
           ^                           |                            |
           | PB11 (DATA_RDY)           |                            v
           | GPIO high/low             |                     Master clocks out
           |                           |
           |                           v
           |                +----------------------+
           |                | Frame queue (x10)    |
           |                | LogFrame_t (842B)    |
           |                +----------+-----------+
           |                           ^
           |                           |
           |                           v
           |                +----------------------+
           |                | Frame builder task   |
           |                | Logger_Task()        |
           |                +----+------------+----+
           |                     |            |
           |                     |            |
           |                     v            v
           |            +------------+   +------------------+
           |            | ADC buffer |   | IMU ring buffer  |
           |            | 256 samples|   | up to 50 samples |
           |            +------+-----+   +--------+---------+
           |                   ^                  ^
           |                   |                  |
           |                   |                  |
           |   HAL_ADC DMA cbk |                  | LSM6DS3 read cbk
           |                   |                  |
           v                   |                  |
 +------------------+          |        +----------------------+
 | ADC2 + DMA       |          |        | LSM6DS3 (SPI1)       |
 | TIM6 TRGO @100kHz|----------+        | raw gyro+accel burst |
 +------------------+                   +----------------------+
```

Mermaid (optional):
```mermaid
flowchart TD
  subgraph Master[External module (e.g., ESP32)]
    M[SPI master]
  end

  subgraph STM32[STM32G431CBT6]
    SPI2[SPI2 slave (hspi2)]
    RDY[PB11 DATA_RDY GPIO]
    RX[HAL_SPI_Receive_IT (5B cmd)]
    TX[HAL_SPI_Transmit_DMA (config/frame)]
    TXC[HAL_SPI_TxCpltCallback]

    TIM6[TIM6 TRGO @ 100kHz]
    ADC2[ADC2 ext trig from TIM6]
    DMA[DMA circular buffer (512 samples)]
    ADCCB[HAL_ADC_ConvHalf/ConvCpltCallback]
    ADCBUF[g_adc_buffer (256 samples + timestamp)]

    IMU[LSM6DS3 driver (SPI1)]
    IMUCB[Lsm6ds3_GetDataCbk]
    IMUBUF[g_imu_buffer (<=50 samples)]

    TASK[Logger_Task()]
    Q[Frame queue (10)]
  end

  M -->|SPI2 clocks| SPI2
  SPI2 --> RX -->|cmd 0x2A| TX --> RDY
  TX --> TXC -->|RDY low| RDY
  TASK --> Q --> TX

  TIM6 --> ADC2 --> DMA --> ADCCB --> ADCBUF --> TASK
  IMU --> IMUCB --> IMUBUF --> TASK
```

---

## Data Collection Details

### Timestamping (100 kHz tick)

- `IB_Wing_STM32G431CBT6/Core/Src/solution_wrapper.c`:
  - `static volatile uint32_t g_timestamp`
  - `Logger_TIM6_UpdateCallback()` increments `g_timestamp`.
  - `HAL_TIM_PeriodElapsedCallback()` calls `Logger_TIM6_UpdateCallback()` when `htim->Instance == TIM6`.
  - `Logger_GetTimestamp()` returns `g_timestamp`.

TIM6 setup:
- `IB_Wing_STM32G431CBT6/Core/Src/solution_hal_cfg.c`: TIM6 period is derived from `ADC_SAMPLING_FREQ_KHZ`.
- `IB_Wing_STM32G431CBT6/Core/Src/main.c`: CubeMX init sets `htim6.Init.Period = 1679` (for 168 MHz / 100 kHz).

### ADC2 sampling (100 kHz, DMA circular)

Configuration:
- `IB_Wing_STM32G431CBT6/Core/Src/solution_hal_cfg.c`:
  - ADC2 external trigger: `ADC_EXTERNALTRIG_T6_TRGO` on rising edge
  - `DMAContinuousRequests = ENABLE`, `ContinuousConvMode = DISABLE` (triggered conversions)

Runtime:
- `IB_Wing_STM32G431CBT6/Core/Src/solution_wrapper.c`:
  - `adc2_dma_buffer[ADC_DMA_BUFFER_SIZE]` (512 samples)
  - `HAL_ADC_ConvHalfCpltCallback()` calls `Logger_AdcBuffer_OnComplete(adc2_dma_buffer, 256)`
  - `HAL_ADC_ConvCpltCallback()` calls `Logger_AdcBuffer_OnComplete(&adc2_dma_buffer[256], 256)`
- `shared/Src/logger_spi.c`:
  - `Logger_AdcBuffer_OnComplete()` copies 256 samples into `g_adc_buffer.samples[]` and sets:
    - `g_adc_buffer.block_timestamp = Logger_GetTimestamp()`
    - `g_adc_buffer.ready = 1`

Important nuance:
- The code comments describe `block_timestamp` as “timestamp of first sample”, but the implementation stores the timestamp **when the DMA callback runs** (i.e., roughly at the end of a 256-sample block). If the receiver needs “first-sample timestamp”, it should account for an offset of ~`LOGGER_ADC_BLOCK_SIZE` ticks.

### IMU raw samples (LSM6DS3)

IMU config publication (sent in `logger_config_t`):
- `shared/Src/LSM6DS3.c` builds an `imu_config_t` and calls:
  - `Logger_OnAccelerometerReady(&imu_cfg)` (logger stores it into `loggerStat.config.imu_config`)

Raw sample capture:
- `shared/Src/LSM6DS3.c` in `Lsm6ds3_GetDataCbk()`:
  - `Logger_ImuOnNewSample(&lsm6ds3_Stat.rd_buff[1])` (skips a dummy byte; passes 12 raw bytes)
- `shared/Src/logger_spi.c` in `Logger_ImuOnNewSample()`:
  - appends `{ data[12], timestamp=Logger_GetTimestamp() }` into `g_imu_buffer` (up to 50 samples)

Raw IMU byte layout (12 bytes):
- `[gx_lo,gx_hi, gy_lo,gy_hi, gz_lo,gz_hi, ax_lo,ax_hi, ay_lo,ay_hi, az_lo,az_hi]`

---

## Frame Format (`LogFrame_t`, 842 bytes)

Defined in `shared/Inc/logger.h` as a packed struct:

| Field | Size | Notes |
|------:|-----:|------|
| `magic` | 2 | `0x5A5A` |
| `n_imu` | 2 | number of valid IMU samples (intended 0..`LOGGER_IMU_BLOCK_SIZE` = 20) |
| `adc_timestamp` | 4 | timestamp for the ADC block (see nuance above) |
| `adc[256]` | 512 | 256 × `int16_t` |
| `imu[20]` | 320 | 20 × `ImuRawSample_t` (16 bytes each) |
| `crc16` | 2 | checksum field (see CRC note below) |

Endianness: STM32 is little-endian; the struct is packed, and the byte layout matches little-endian encoding of multi-byte fields.

### CRC note (implementation vs naming)

Although `LogFrame_t` has a `uint16_t crc16` field and `logger.h` documents “CRC16”, the actual implementation in `shared/Src/logger_spi.c` computes a **CRC-8** (table-based, polynomial `0x07`) and writes it into `frame->crc16` (upper byte will typically be `0`).

Receiver-side recommendation (based on current code): validate **CRC-8 over the frame bytes excluding the last byte** as implemented, or update firmware + receiver together if true CRC16 is desired.

---

## SPI Protocol (SPI2 Slave → Master)

### Signals / pins

- SPI2 pins (CubeMX / MSP init):
  - `PB12` NSS, `PB13` SCK, `PB14` MISO, `PB15` MOSI
- “Data ready” GPIO:
  - `PB11` (`LOGGER_SPI_DATA_RDY_PIN`) toggled by `Logger_GPIO_SetReady()`

### Startup handshake

1. STM32 boots, calls `Logger_Init()`:
   - arms `HAL_SPI_Receive_IT(..., 5)` to capture a 5-byte “command packet”
   - ensures DATA_RDY is low
2. Master asserts NSS and sends 5 bytes where the first byte is the command:
   - supported command: `LOGGER_SPI_CMD_CONFIG = 42 (0x2A)`
3. On RX complete:
   - `Logger_SPI_RxCallback()` sends `logger_config_t` (32 bytes) via `HAL_SPI_Transmit_DMA`
   - asserts DATA_RDY high during the transfer
4. On TX complete:
   - `Logger_SPI_TxCallback()` deasserts DATA_RDY and sets an internal “TX done” state

### Frame streaming

- After the config transmission is done, the logger starts acquisition via:
  - `Solution_LoggingStart()` (called from `Logger_DrainQueue()` when `config_sent` is observed)
- Frames are produced in `Logger_Task()` and immediately transmitted (DMA) whenever SPI is idle.

Operationally, the master should:
- wait for DATA_RDY to go high
- clock out exactly `sizeof(LogFrame_t)` bytes (842) for a frame
- DATA_RDY will go low when DMA finishes (TX complete callback)

---

## MAVLink Data (Not part of SPI logger stream)

The repository includes MAVLink parsing and application events under `shared/Src/mavlink_uart.c`, but when `SPI_LOGGER_ENABLE == 1u`:
- `shared/Src/app.c` bypasses MAVLink init and `Mavlink_Process()` in the main task loop
- `LogFrame_t` contains only ADC and IMU payloads (no MAVLink/event fields)

If “MAVLink events over SPI” are required, it would need an explicit protocol extension (new frame type/fields or a parallel message channel).

---

## Files Involved (Logger Functionality)

### Shared (`shared/`)
- `shared/Inc/logger.h`: public structs and logger API (`LogFrame_t`, `logger_config_t`, callbacks)
- `shared/Src/logger_spi.c`: ADC/IMU buffers, frame builder + queue, SPI protocol callbacks
- `shared/Src/LSM6DS3.c`: IMU init/config extraction + raw sample callback into logger (`Logger_OnAccelerometerReady`, `Logger_ImuOnNewSample`)
- `shared/Inc/LSM6DS3.h`: IMU register definitions and driver API used by the logger build
- `shared/Src/app.c`: enables logger mode (`Logger_Init`, `Logger_Task`) when `SPI_LOGGER_ENABLE == 1u`

### Board (`IB_Wing_STM32G431CBT6/`)
- `IB_Wing_STM32G431CBT6/Core/Inc/prj_config.h`: `SPI_LOGGER_ENABLE`, ADC rate/buffer sizing, IMU ODR selection
- `IB_Wing_STM32G431CBT6/Core/Inc/hal_cfg.h`: logger SPI2 handle + DATA_RDY pin mapping
- `IB_Wing_STM32G431CBT6/Core/Src/solution_wrapper.c`: timestamp source, ADC DMA callbacks to logger, SPI2 wrapper callbacks (`HAL_SPI_*CpltCallback`)
- `IB_Wing_STM32G431CBT6/Core/Src/solution_hal_cfg.c`: TIM6/ADC2 trigger configuration, GPIO setup for DATA_RDY
- `IB_Wing_STM32G431CBT6/Core/Src/main.c`: CubeMX init for SPI2 slave, TIM6 base init, peripheral bring-up sequence
- `IB_Wing_STM32G431CBT6/Core/Src/stm32g4xx_hal_msp.c`: SPI2 DMA (TX) + SPI2 GPIO AF config, ADC DMA, NVIC
- `IB_Wing_STM32G431CBT6/Core/Src/stm32g4xx_it.c`: SPI2 IRQ handler, DMA2 Ch2 IRQ handler, TIM6 IRQ
- `IB_Wing_STM32G431CBT6/IB_Wing_STM32G431CBT6.ioc`: CubeMX configuration (SPI2 slave pins, DMA request)
- `IB_Wing_STM32G431CBT6/PINOUT.md`: pinout/interrupt mapping reference for logger mode

---

## Known Gaps / Things to Watch

- **CRC mismatch in naming**: implementation currently computes CRC-8 but stores into `crc16`.
- **IMU sample count not clamped**: frame payload is sized for `LOGGER_IMU_BLOCK_SIZE` (20), while the IMU buffer can hold up to 50. In normal timing (100 kHz ADC blocks vs IMU ODR), `n_imu` is typically very small; if IMU ODR increases significantly, `Logger_Task()` should clamp/copy at most 20 samples.
- **ADC timestamp meaning**: timestamp is captured at DMA callback time; adjust in the receiver if “first-sample timestamp” is needed.
- **DATA_RDY pin documentation drift**: comments mention PB9 in one place; code mapping in `hal_cfg.h` is PB11.

