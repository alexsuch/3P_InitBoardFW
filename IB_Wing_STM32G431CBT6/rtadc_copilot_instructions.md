# Realtime ADC processing architecture (STM32, no-RTOS) — instructions for Copilot

## Goal
Implement **real-time processing pipeline** for an STM32 project where:
- ADC runs at ~100 kHz using DMA
- We process data in *small blocks* (e.g. 16 samples) with **half-transfer** and **transfer-complete** DMA IRQs
- We maintain a **larger history ring buffer** (e.g. 512 samples) for “look-back” computations
- When some quick condition triggers (“suspicious event”), we **snapshot** the latest N samples (e.g. 64) into a **linear buffer** and run heavier analysis **outside ISR**
- ISR must be *very short* (no heavy memcpy loops, no CRC, no SPI logic, etc.)

This is **separate from the logger**; treat it as a realtime processing subsystem.

---

## Design constraints
1. **No RTOS**. Pure C.
2. ISR work must be **constant and minimal**.
3. ADC DMA callbacks occur every:
   - `BLOCK = 16` samples => full period = 160 µs at 100 kHz
   - half period = 80 µs
4. Ring buffer size must be **power of two** (256/512/1024) for fast modulo with mask.
5. Snapshot copying (e.g. 64 samples) is allowed in main loop or low-priority context; not in ISR.

---

## What Copilot must implement (deliverables)
Create new module files (or integrate into existing ones):

- `realtime_adc_proc.h`
- `realtime_adc_proc.c`

Provide:
1. **Initialization** function that sets buffer pointers, sizes, and resets state.
2. **DMA half/full callbacks** entry points called by the existing HAL callbacks.
3. **Ring buffer** implementation: push blocks, relative sample access (x[-k]), snapshot extract.
4. **Event trigger mechanism**: ISR sets a flag + snapshot anchor index; main loop consumes it.
5. A **poll function** called from main loop that performs snapshot extraction + analysis hook.

Do NOT change existing ADC configuration logic (timer triggers, DMA setup) except to route callbacks.

---

## Proposed constants
- ADC DMA block size: `ADC_DMA_BLOCK = 16` samples (`int16_t` each).
- History ring size: `HIST_N = 512` samples (can be 256 if RAM is tight).
- Snapshot window size: `SNAP_N = 64` samples (or 128 if needed).
- Use `int16_t` samples unless your ADC buffer is `uint16_t`; adapt accordingly.

---

## Data structures

### realtime_adc_proc_t
Copilot must implement a context structure similar to:

```c
#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifndef ADC_DMA_BLOCK
#define ADC_DMA_BLOCK 16u
#endif

#ifndef HIST_N
#define HIST_N 512u   // must be power of 2
#endif

#ifndef SNAP_N
#define SNAP_N 64u    // snapshot window size
#endif

typedef struct {
    // Pointer to the DMA buffer owned by ADC DMA (size ADC_DMA_BLOCK)
    volatile int16_t *adc_dma_buf;

    // History ring buffer
    int16_t hist[HIST_N];
    volatile uint16_t hist_w;     // write index (next write position)
    uint16_t hist_mask;           // HIST_N-1

    // Flags from ISR -> main
    volatile bool need_snapshot;
    volatile uint16_t snap_w_anchor;   // captured hist_w at trigger time

    // Latest snapshot linear buffer
    int16_t snap[SNAP_N];
    volatile bool snap_ready;

    // Optional counters for diagnostics
    volatile uint32_t isr_half_count;
    volatile uint32_t isr_full_count;
    volatile uint32_t snapshot_count;
} realtime_adc_proc_t;
```

---

## API to implement

```c
// Must be called once at boot
void rtadc_init(realtime_adc_proc_t *ctx, volatile int16_t *adc_dma_buf);

// Called from HAL ADC DMA callbacks:
void rtadc_on_dma_half(realtime_adc_proc_t *ctx);
void rtadc_on_dma_full(realtime_adc_proc_t *ctx);

// Called frequently from main loop:
void rtadc_poll(realtime_adc_proc_t *ctx);

// Optional helper accessors:
int16_t rtadc_get_rel(const realtime_adc_proc_t *ctx, uint16_t k_back); // x[-k_back]
bool rtadc_try_get_snapshot(realtime_adc_proc_t *ctx, int16_t *out, uint16_t n);
```

---

## ISR behavior (critical)
In `rtadc_on_dma_half()` and `rtadc_on_dma_full()`:
1. Identify source pointer:
   - Half: `src = &adc_dma_buf[0]` length 8
   - Full: `src = &adc_dma_buf[8]` length 8
2. Push these 8 samples into the history ring buffer.
3. Update tiny “quick features” if desired, but **keep it tiny**.
4. If a simple condition triggers (placeholder), set:
   - `ctx->need_snapshot = true;`
   - `ctx->snap_w_anchor = ctx->hist_w;`
   Do not copy snapshot in ISR.

### Placeholder trigger condition
Copilot must add a placeholder like:

```c
static inline bool rtadc_quick_trigger(int16_t s) {
    // TODO: user will replace this with real logic later
    return (s > 2000 || s < -2000);
}
```

If any sample in the pushed block triggers, set `need_snapshot`.

---

## Main loop behavior
In `rtadc_poll()`:
1. If `ctx->need_snapshot` and not already `ctx->snap_ready`:
   - capture `uint16_t w = ctx->snap_w_anchor;`
   - extract last `SNAP_N` samples ending at `(w-1)` into `ctx->snap[]`
   - set `ctx->snap_ready = true;`
   - clear `ctx->need_snapshot = false;`
   - increment `snapshot_count`
2. Call an analysis hook stub with the snapshot:

```c
__attribute__((weak)) void rtadc_on_snapshot_ready(const int16_t *win, uint16_t n) {
    (void)win; (void)n;
    // user will implement
}
```

So the user can implement analysis elsewhere without editing this module.

---

## Ring buffer helpers to implement

### Push block
Efficient push for small blocks (8 samples). Implementation must handle wrap:
- If `hist_w + n <= HIST_N`: single memcpy loop
- Else: two-part wrap copy

### Relative access x[-k]
Provide:

```c
int16_t_t rtadc_get_rel(const realtime_adc_proc_t *ctx, uint16_t k_back) {
    // k_back = 1 means previous sample
    uint16_t idx = (uint16_t)(ctx->hist_w - k_back) & ctx->hist_mask;
    return ctx->hist[idx];
}
```

*(Copilot: correct the typo `int16_t` in return type.)*

### Extract last N into linear buffer
`extract_last(ctx, w_anchor, out, N)` copies samples:
- start = `w_anchor - N`
- for i=0..N-1: out[i] = hist[(start+i) & mask]

Use fast two-chunk memcpy when possible.

---

## Interrupt safety notes (no-RTOS)
- `hist_w` is modified in ISR. In `rtadc_poll()`, read `snap_w_anchor` and `hist_w` as local copies.
- Snapshot extraction uses the captured `snap_w_anchor`. This prevents “moving target” issues.
- If you want full atomicity, you may briefly disable ADC DMA IRQ only around copying `SNAP_N` samples; but for 64 samples it’s typically fine to not disable. Prefer correctness: **use anchor index**.

---

## Integration points (HAL)
User will already have something like:

```c
static realtime_adc_proc_t g_rtadc;

void app_init(void) {
    rtadc_init(&g_rtadc, (volatile int16_t*)g_adc_dma_buf);
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) {
    (void)hadc;
    rtadc_on_dma_half(&g_rtadc);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    (void)hadc;
    rtadc_on_dma_full(&g_rtadc);
}
```

And in the main superloop:

```c
while (1) {
    rtadc_poll(&g_rtadc);
    // existing main loop tasks...
}
```

Copilot must not break existing logger pipeline.

---

## Performance expectations
- ISR: push 8 samples + small checks should be **well under 10–20 µs**
- Snapshot copy 64 samples (128 bytes) in main loop is trivial.
- Heavy analysis must not block the ISR cadence; keep it in poll or split into stages.

---

## Deliverable checklist
Copilot output must include:
- [ ] `realtime_adc_proc.h/.c` with complete implementation
- [ ] clear comments where user should plug real trigger and analysis logic
- [ ] no dynamic allocation
- [ ] ring sizes power-of-two enforced via `static_assert` when possible
- [ ] compiles as C (C99) in STM32 project

---

## Optional: Compile-time checks
Add:

```c
#if (HIST_N & (HIST_N - 1)) != 0
#error "HIST_N must be power of two"
#endif
```

And similar for SNAP_N if needed.

---

## Notes
- Do **not** add CRC, SPI, SD logging, or frame packing into this realtime module.
- This module is intended purely for “fast path” signal processing scaffolding.
