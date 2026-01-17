# Logger Performance Improvements (6 kHz IMU ODR) — IB_Wing_STM32G431CBT6

Goal: improve logger stability/performance when LSM6DS3 accelerometer/gyro runs at **~6.6 kHz** (`LSM6DS3_SAMPLING_FREQ_HZ = 6664`), where the current design shows “data packaging takes too much time” symptoms (missed samples, queue overflow, frame drops, lockups).

This document is based on the current implementation in:
- `shared/Src/logger_spi.c` (logger pipeline)
- `shared/Src/LSM6DS3.c` + `shared/Src/acc_proc.c` (IMU acquisition)
- `IB_Wing_STM32G431CBT6/Core/Src/solution_wrapper.c` (timestamp, ADC callbacks, SPI callbacks)
- `IB_Wing_STM32G431CBT6/Core/Inc/prj_config.h` (ODR / logger macros)

---

## Executive Summary (What’s Likely Going Wrong at 6 kHz)

At 6 kHz ODR, the system workload is dominated by **IMU sampling overhead** (SPI1 DMA transactions + callbacks + per-sample buffer writes), not by the “CRC” itself. The current logger has two major contributors:

1. **Correctness + time bomb:** IMU sample count handling can **overflow fixed-size frame-builder buffers**.
   - `ImuBuffer_t` can hold up to **50** samples.
   - `FrameBuilder_t.imu[]` and `LogFrame_t.imu[]` are sized to **`LOGGER_IMU_BLOCK_SIZE` (= 20)**.
   - Current code copies `imu_count` samples without clamping to 20.
   - Under interrupt load / slow main loop, `imu_count` can exceed 20 → memory corruption + unpredictable CPU time.

2. **O(n) per-sample IMU buffering in worst case:** `Logger_ImuOnNewSample()` uses `memmove()` when full.
   - If the main loop falls behind, the IMU buffer becomes full and every new IMU sample triggers a `memmove()` of ~49 samples.
   - At multi-kHz ODR this can spiral into “CPU stuck doing memory moves”.

Additionally, `Lsm6ds3_Task()` currently **polls** the data-ready GPIO and performs **one SPI read per loop**, which becomes fragile at high ODR if the main loop is starved by interrupts.

---

## Hot Path Inventory (Where Time Is Spent)

### 1) IMU ingestion path (high-frequency)

Pipeline:
`Lsm6ds3_Task()` → `SpiGetAccData(... DMA ...)` → `Lsm6ds3_GetDataCbk()` → `Logger_ImuOnNewSample()`

Key characteristics:
- One SPI burst per sample (13 bytes including dummy).
- DMA + HAL completion callbacks per sample.
- `Logger_ImuOnNewSample()` does: timestamp read + memcpy(12) + occasionally memmove(~800 bytes worst-case).

At ~6.6 kHz, per-sample software overhead matters more than raw SPI bandwidth.

### 2) Frame assembly path (per ADC block, ~390 Hz @ 100 kHz / 256)

`Logger_Task()` does (per frame):
- ADC: copies 256×int16 (`512 B`) into builder and then again into frame (extra copy).
- IMU: copies `imu_count × 16 B` into builder and then again into frame (extra copy).
- CRC: computes CRC-8 incrementally across multiple `Logger_Task()` states (adds latency if main loop isn’t running fast).
- Queue: copies a full frame again when dequeuing (`Logger_GetNextFrame()` memcpy into `loggerStat.tx_frame`).

This is not massive in raw bytes/sec, but at high interrupt load the **extra copies and multi-call state machine** increase the chance that the logger cannot “catch up” in time.

---

## Critical Fixes (Must Do First)

### A) Clamp IMU sample counts to frame capacity (correctness + performance)

Problem:
- `Logger_Task()` copies `imu_count` samples into `FrameBuilder_t.imu[LOGGER_IMU_BLOCK_SIZE]` without clamping.
- Any `imu_count > LOGGER_IMU_BLOCK_SIZE` is a buffer overflow.

Fix direction:
- Always clamp: `imu_to_copy = min(imu_count, LOGGER_IMU_BLOCK_SIZE)`.
- Prefer keeping the **most recent** samples (not the oldest) if truncating:
  - If IMU buffer has 0..50 samples and frame holds 20, use the last 20.

Impact:
- Removes corruption that can look like “packaging takes too long” (because the CPU is running in undefined behavior).

### B) Replace IMU “shift-on-overflow” with a real ring buffer (avoid memmove)

Problem:
- `Logger_ImuOnNewSample()` uses `memmove()` to keep newest 50 samples when full.
- At high ODR + slow consumer, this becomes O(n) per sample.

Fix direction:
- Implement IMU buffer as a circular ring:
  - `head` (write index), `tail` (read index), `count` (or derive count)
  - On overflow: overwrite oldest by advancing `tail`.
  - No `memmove()` ever.

Consumer API options:
- **Pop/copy up to N** samples into frame builder (recommended).
- Or provide “peek contiguous span(s)” (requires handling wrap-around).

Impact:
- Worst-case IMU buffering cost becomes O(1) per sample even if consumer is slow.

---

## Optimization Variants (Choose Based on Effort vs Gain)

### Variant 1 — “Minimal invasive” (fastest path to stable 6 kHz)

Changes:
1. Clamp IMU count to `LOGGER_IMU_BLOCK_SIZE` and copy most recent N.
2. Replace `memmove()` IMU buffer with ring buffer.
3. Add a tiny critical section (or lock-free snapshot) when copying IMU samples in `Logger_Task()`:
   - Today the producer (SPI callback) and consumer (main loop) access `g_imu_buffer` concurrently.
   - At high rate this can cause inconsistent count/data reads.

Why it helps:
- Stops buffer overflows.
- Eliminates the “death spiral” where the system spends most cycles shifting buffers.

Expected improvement:
- Very large stability improvement; CPU headroom increases sharply in the “consumer occasionally delayed” scenario.

Risks:
- Requires careful concurrency design (interrupt vs main loop).

### Variant 2 — “Best ROI” (reduce IMU transaction & interrupt rate with FIFO + watermark)

Observation:
At 6.6 kHz, even an efficient `Logger_ImuOnNewSample()` still executes ~6.6k times/sec, and SPI1 DMA completion callbacks also run at that frequency. That overhead can dominate CPU time.

Changes (conceptual):
1. Configure LSM6DS3 FIFO to collect samples internally.
2. Use FIFO watermark interrupt (or periodic read) to fetch **a burst** of samples.
3. Perform one SPI read per burst instead of per sample.

Example design goal:
- Read FIFO every ~2–5 ms and fetch ~15–30 samples per burst (matches ADC block cadence).

Why it helps:
- Collapses thousands of callbacks/sec into hundreds/sec.
- Reduces HAL/DMA overhead and contention.

Expected improvement:
- Often the biggest improvement at multi-kHz ODR, because it attacks the dominant overhead (interrupt + transaction count).

Risks:
- More work in the IMU driver (FIFO config, FIFO readout parsing).
- Needs clear policy for timestamping FIFO samples:
  - Option A: timestamp each sample as “read-time minus sample_index × dt” (approx).
  - Option B: use sensor timestamp features if available / feasible.

### Variant 3 — “Make frame build deterministic” (avoid multi-step CRC + extra copies)

Problem:
- Current `Logger_Task()` spreads CRC and queueing over multiple states (`BUILD` → `CRC_PART1` → …).
- Under high interrupt load, the main loop may not advance states quickly, increasing end-to-end latency.

Changes:
1. Compute CRC in **one pass** in the same call where the frame is assembled:
   - CRC-8 over ~841 bytes is cheap on a 168 MHz M4.
2. Remove intermediate builder copies:
   - Copy ADC directly into the target frame queue slot.
   - Copy IMU samples directly into the target frame queue slot.
3. Remove `Logger_GetNextFrame()` memcpy into `tx_frame`:
   - Transmit DMA directly from the queue slot.
   - Only advance `queue_read_idx` on `Logger_SPI_TxCallback()` (TX complete).

Why it helps:
- Reduces copies per frame and reduces “time-to-frame-ready”.
- Decreases pressure on the main loop (fewer Logger_Task invocations needed per frame).

Expected improvement:
- Moderate CPU reduction; improved determinism under load.

Risks:
- DMA source memory must remain valid until TX complete (requires “don’t overwrite queue slot until sent”).

### Variant 4 — “Acquisition scheduling” (stop polling data-ready)

Problem:
- `Lsm6ds3_Task()` polls `ReadAccIntGpio()` and performs one read per main-loop iteration.
- At 6 kHz, missing data-ready events becomes likely if the loop is delayed.

Changes:
- Move IMU trigger to an **EXTI interrupt** on DRDY.
  - ISR only sets a flag/counter; SPI read is started from a high-priority context or from main loop.
- Or pair with FIFO (preferred) so DRDY frequency doesn’t matter.

Why it helps:
- Ensures sample acquisition isn’t gated by “how fast the main loop spins”.

Expected improvement:
- Helps correctness; performance depends on how reads are handled.

Risks:
- Must be careful not to start a new SPI DMA read while the previous one is in flight.

### Variant 5 — HAL/LL and bus-level improvements (incremental)

Changes:
- Increase SPI1 clock to reduce transfer time (if board/sensor allows).
- Use LL drivers (or optimized HAL settings) to reduce per-transaction overhead.
- Tune NVIC priorities to avoid starvation patterns:
  - SPI1 DMA / SPI1 IRQ / SPI2 DMA / TIM6 should be set consciously.

Expected improvement:
- Small to moderate; useful after the big architectural issues are addressed.

---

## Recommended Implementation Order

1. **Correctness + stability first**
   - Clamp IMU counts to 20 everywhere.
   - Fix IMU buffer to ring buffer (no `memmove`).
   - Fix concurrency between producer (IMU callback) and consumer (`Logger_Task`).

2. **Reduce high-frequency overhead**
   - Implement LSM6DS3 FIFO + burst reads (watermark).

3. **Reduce per-frame overhead**
   - Remove extra copies (direct-to-frame + direct DMA from queue slot).
   - Compute CRC in one pass and queue immediately.

4. **Tune system**
   - IRQ priorities, SPI clocks, profiling.

---

## Profiling Plan (So We Don’t Guess)

To decide which variant is necessary, measure:

### What to measure
- `Logger_ImuOnNewSample()` time and call rate
- SPI1 DMA completion callback rate
- `Logger_Task()` duration per invocation
- Time from “ADC block ready” to “frame queued”
- Frame drop counters: `stats_frames_dropped`, any IMU overflow counters (add one)

### How to measure (typical on STM32)
- Enable DWT CYCCNT (cycle counter) and record max/avg cycles per function.
- Or toggle a GPIO around hot sections (scope/logic analyzer).
- Add counters printed over UART in a debug build (if available in logger mode).

Key metric:
- Main loop must have enough slack to:
  1) keep up with ADC block cadence (~2.56 ms per 256 samples)
  2) not fall into IMU-buffer overflow path

---

## Notes / Observations From Current Code

- Frame holds `LOGGER_IMU_BLOCK_SIZE = 20` IMU samples, while the IMU buffer can collect **50** (`ImuBuffer_t samples[50]`).
  - This mismatch exists because IMU sampling and frame building are **decoupled**: IMU samples arrive continuously (via `Logger_ImuOnNewSample()`), but IMU samples are only consumed/cleared when `Logger_Task()` reaches the “ADC block ready” path and copies IMU data into the frame builder.
  - At 100 kHz ADC with 256-sample blocks, the frame cadence is ~390 Hz (one frame every ~2.56 ms). At 6.6 kHz IMU ODR that is ~17 IMU samples per “ideal” frame window, which fits into 20 **only if** `Logger_Task()` runs frequently and keeps up.
  - If `Logger_Task()` is delayed (interrupt load, SPI traffic, multi-call state machine progress, etc.), IMU samples continue accumulating. At 6.6 kHz, a 50-sample buffer represents only ~7.5 ms of backlog; exceeding that pushes the code into its “buffer full” behavior (currently `memmove()`), and `imu_count` can exceed the 20-sample frame capacity unless explicitly clamped.
- CRC naming mismatch exists (CRC-8 computed, stored into `crc16` field). This is not the primary performance issue, but it matters for protocol correctness.

---

## Next Step

If you want, I can implement Variant 1 (clamp + ring buffer + safe pop) and then optionally Variant 3 (remove extra copies / direct DMA-from-queue), and we can re-test at `LSM6DS3_SAMPLING_FREQ_HZ = 6664` before tackling FIFO (Variant 2).
