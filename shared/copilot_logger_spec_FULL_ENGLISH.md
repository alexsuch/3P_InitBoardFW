# Copilot Logger Architecture Specification
### High‑Speed Impact Logger (STM32 Slave → ESP32 Master)

This document defines the **complete architecture**, **timing model**, **data flow**, **ring buffering**, **frame building**, and **SPI transfer logic** for the high‑speed impact logger.  
It is written **specifically for GitHub Copilot**, using explicit MUST/SHOULD rules to guarantee correct code generation.

Copilot must follow this specification exactly when generating or modifying code inside this project.

---

# 0. System Overview

The system consists of two MCUs:

- **STM32** — real‑time data acquisition, frame building, SPI slave.
- **ESP32** — logging and SD card writing, SPI master.

STM32 responsibilities:

1. Sample **ADC at 100 kHz** using DMA.  
2. Receive **IMU samples (~6.66 kHz)** from the IMU driver via callback.  
3. Assign precise timestamps using a **hardware global timeline**.  
4. Build **log frames** (256 ADC samples + attached IMU samples).  
5. Queue frames.  
6. Transfer frames to ESP32 via **SPI (slave, DMA)**.

The architecture is strictly **non‑blocking** and **ISR‑minimal**.

---

# 1. Global Timeline (Hard Requirement)

## 1.1 TIM6 Already Exists  
TIM6 generates an **Update Event @ 100 kHz**.  
It already triggers ADC and DAC.  
Copilot MUST NOT modify TIM6 configuration.

---

## 1.2 TIM2 = 32‑bit Global Timestamp Counter

Copilot MUST configure **TIM2 (or TIM5)** as follows:

- Mode: **Slave, External Clock Mode 1**  
- Trigger source: **ITR from TIM6 TRGO**  
- Behavior: **each TIM6 update increments TIM2 by 1**

Therefore:

```
TIM2->CNT == Global ADC Sample Index
```

This is the master timeline for **ALL** sensors.

Copilot MUST implement:

```c
void Logger_Timing_Init(void);
```

This function:

- Configures TIM2 correctly.  
- Resets TIM2->CNT = 0.  
- Starts TIM2.  

TIM2 overflow is allowed — offline PC processing will handle wraparound.

---

# 2. ADC Acquisition (100 kHz DMA)

ADC is triggered by TIM6 update.  
DMA uses a **double buffer**.  

Callbacks:

```c
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc);
```

These MUST call logger wrappers.

---

## 2.1 ADC Ring Buffer

Copilot MUST implement:

```c
#define ADC_RING_SIZE 4096  // must be power of two

typedef struct {
    uint16_t buf[ADC_RING_SIZE];
    volatile uint32_t write_idx;
    volatile uint32_t read_idx;
} AdcRing_t;

static AdcRing_t g_adc_ring;
```

Helper functions:

```c
static inline uint32_t AdcRing_Available(const AdcRing_t *r)
{
    return (r->write_idx - r->read_idx) & (ADC_RING_SIZE - 1);
}

static inline void AdcRing_PushBlock(AdcRing_t *r, const uint16_t *src, uint32_t n)
{
    for (uint32_t i = 0; i < n; i++) {
        r->buf[r->write_idx & (ADC_RING_SIZE - 1)] = src[i];
        r->write_idx++;
    }
}

static inline uint16_t AdcRing_Pop(AdcRing_t *r)
{
    uint16_t v = r->buf[r->read_idx & (ADC_RING_SIZE - 1)];
    r->read_idx++;
    return v;
}
```

### DMA Callback Wrappers

```c
void Logger_AdcDmaHalfCallback(void)
{
    const uint32_t half = ADC_DMA_BUF_SIZE / 2;
    AdcRing_PushBlock(&g_adc_ring, &adc_dma_buf[0], half);
}

void Logger_AdcDmaFullCallback(void)
{
    const uint32_t half = ADC_DMA_BUF_SIZE / 2;
    AdcRing_PushBlock(&g_adc_ring, &adc_dma_buf[half], half);
}
```

HAL integration:

```c
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc == &hadc1) Logger_AdcDmaHalfCallback();
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc == &hadc1) Logger_AdcDmaFullCallback();
}
```

---

# 3. IMU Acquisition (~6.66 kHz)

IMU driver provides raw 6‑axis sample via callback.  
Copilot MUST create:

```c
void Logger_ImuOnNewSample(const int16_t *raw);
```

This function MUST:

1. Read global timestamp:
   ```c
   uint32_t adc_idx = TIM2->CNT;
   ```
2. Push sample into IMU ring buffer.

---

## 3.1 IMU Ring Buffer

```c
#define IMU_RING_SIZE 1024

typedef struct {
    uint32_t adc_idx;
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
} ImuSample_t;

typedef struct {
    ImuSample_t buf[IMU_RING_SIZE];
    volatile uint32_t write_idx;
    volatile uint32_t read_idx;
} ImuRing_t;

static ImuRing_t g_imu_ring;
```

Callback implementation:

```c
void Logger_ImuOnNewSample(const int16_t *raw)
{
    ImuSample_t s;
    s.adc_idx = TIM2->CNT;

    s.ax = raw[0]; s.ay = raw[1]; s.az = raw[2];
    s.gx = raw[3]; s.gy = raw[4]; s.gz = raw[5];

    uint32_t wi = g_imu_ring.write_idx & (IMU_RING_SIZE - 1);
    g_imu_ring.buf[wi] = s;
    g_imu_ring.write_idx++;
}
```

---

# 4. Frame Format (Strict Layout)

Each frame contains:

- 256 ADC samples
- all IMU samples whose timestamp falls into that window
- CRC16

```c
#define N_ADC_PER_FRAME      256
#define MAX_IMU_PER_FRAME     20

typedef struct __attribute__((packed)) {
    uint16_t magic;            // 0xA55A
    uint16_t frame_id;

    uint32_t first_adc_index;
    uint16_t n_adc;
    uint16_t n_imu;

    int16_t adc[N_ADC_PER_FRAME];

    struct {
        uint32_t adc_idx;
        int16_t ax, ay, az;
        int16_t gx, gy, gz;
    } imu[MAX_IMU_PER_FRAME];

    uint16_t crc;
} LogFrame_t;
```

Copilot MUST implement:

```c
uint16_t Logger_Frame_CalcCrc(const LogFrame_t *f);
```

---

# 5. Frame Queue

```c
#define FRAME_QUEUE_SIZE 16

typedef struct {
    LogFrame_t frames[FRAME_QUEUE_SIZE];
    volatile uint8_t head;
    volatile uint8_t tail;
    volatile uint8_t count;
} FrameQueue_t;

static FrameQueue_t g_frame_queue;
```

Queue MUST be lock‑free.

---

# 6. Frame Builder Task (Main Loop)

Copilot MUST implement:

```c
void Logger_Builder_Task(void);
```

Builder MUST:

1. Check ADC availability:

```c
uint32_t available = AdcRing_Available(&g_adc_ring);
```

2. While enough for a frame:

   a. Initialize frame  
   b. Set metadata  
   c. Read 256 ADC samples  
   d. Attach IMU samples whose `adc_idx` falls in range  
   e. Compute CRC  
   f. Push into queue  

The builder MUST NOT call SPI functions.

---

# 7. SPI Transfer (STM32 Slave → ESP32 Master)

### Busy Flag:

```c
static volatile bool g_spi_busy = false;
```

### Data Ready GPIO:

```c
void Logger_SetDataReady(bool level);
```

### Start Transmission:

```c
void Logger_Spi_TryStartTx(void)
{
    if (g_spi_busy) return;
    if (g_frame_queue.count == 0) {
        Logger_SetDataReady(false);
        return;
    }

    LogFrame_t *f = FrameQueue_Front(&g_frame_queue);
    g_spi_busy = true;
    Logger_SetDataReady(true);

    HAL_SPI_TransmitReceive_DMA(&hspiX,
                                (uint8_t*)f,
                                dummy_rx_buffer,
                                sizeof(LogFrame_t));
}
```

### DMA Complete:

```c
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == &hspiX) {

        FrameQueue_Pop(&g_frame_queue);
        g_spi_busy = false;

        if (g_frame_queue.count == 0)
            Logger_SetDataReady(false);
        else
            Logger_Spi_TryStartTx();
    }
}
```

---

# 8. Main Loop

```c
while (1)
{
    Logger_Builder_Task();
    Logger_Spi_TryStartTx();
}
```

---

# 9. Public API Required

```c
void Logger_Init(void);
void Logger_Timing_Init(void);

void Logger_AdcDmaHalfCallback(void);
void Logger_AdcDmaFullCallback(void);

void Logger_ImuOnNewSample(const int16_t *raw);

void Logger_Builder_Task(void);
void Logger_Spi_TryStartTx(void);
```

---

# 10. Copilot MUST / MUST NOT Rules

### Copilot MUST:

- Use TIM2->CNT as the **only timestamp source**.
- Use DMA for SPI transfers.
- Keep all heavy work in main loop.
- Keep ISRs minimal.
- Avoid floating point in ISRs.
- Avoid dynamic allocation.
- Maintain struct layouts exactly.

### Copilot MUST NOT:

- Change TIM6 configuration.
- Use blocking SPI calls.
- Modify ADC/DAC timing.
- Place frame building inside ISRs.

---

# 11. Synchronization Guarantees

- TIM6 → triggers ADC at 100 kHz  
- TIM6 TRGO → clocks TIM2 → increments global ADC sample index  
- IMU callback timestamps each sample with TIM2->CNT  
- Frames gather ADC blocks + all IMU inside same window  

Thus offline reconstruction is perfectly aligned.

---

# END OF SPEC
