# STM32G431CBU6 Wing Example - Pinout and Connection Map

## Project Overview
**MCU**: STM32G431CBU6 (UFQFPN48 package)  
**System Clock**: 168 MHz (PLL: HSI 16 MHz ÷ 2 × 42)  
**Core**: ARM Cortex-M4  
**Flash**: 128 KB  
**RAM**: 32 KB  

---

## Analog Inputs and Outputs

| Pin   | Peripheral  | Function                    | Configuration                  |
|-------|-------------|-----------------------------|--------------------------------|
| **PA0** | ADC2_IN1    | Analog Input (ADC2 Channel 1) | Single-ended, 2.5 cycles sampling |
| **PA4** | DAC1_OUT1   | DAC Output                   | Trigger: TIM6_TRGO, DMA enabled |
| **PA6** | OPAMP2_VOUT | Operational Amplifier Output | Follower mode                  |
| **PA7** | OPAMP2_VINP | OPAMP2 Non-Inverting Input   | Follower mode (VINP0)          |

**Signal Chain**: DAC1 → PA4 → OPAMP2 (PA7 input) → PA6 output → ADC2 (PA0 via external connection)

---

## Digital Outputs (LEDs and Control)

| Pin   | Signal           | Function                      | Configuration          |
|-------|------------------|-------------------------------|------------------------|
| **PA5**  | EX_LED_OUT       | External LED                  | GPIO Output            |
| **PA8**  | TEST_1           | Test Output 1                 | GPIO Output            |
| **PA10** | CHARGE_EN_OUT    | Charge Enable Control         | GPIO Output            |
| **PA11** | TEST_2           | Test Output 2                 | GPIO Output            |
| **PB1**  | LED_YELLOW_OUT   | Yellow LED (Error Indicator)  | GPIO Output            |
| **PB2**  | LED_GREEN_OUT    | Green LED (Status Indicator)  | GPIO Output            |
| **PB12** | BOOM_LOW_SIDE_OUT_2 | Detonation Low Side 2      | GPIO Output            |
| **PB13** | BOOM_LOW_SIDE_OUT_1 | Detonation Low Side 1      | GPIO Output            |
| **PC6**  | SPI_DATA_RDY     | SPI Data Ready Signal         | GPIO Output, Pull-down |

---

## Digital Inputs

| Pin    | Signal   | Function               | Configuration     |
|--------|----------|------------------------|-------------------|
| **PA12** | FUSE_IN  | Fuse Input Detection   | GPIO Input        |
| **PB6**  | ACC_INT2 | Accelerometer INT2     | GPIO Input        |
| **PB7**  | ACC_INT1 | Accelerometer INT1     | GPIO Input        |

---

## Communication Interfaces

### LPUART1 (Main Communication - CubeMX Configured)
| Pin   | Signal      | Function          | Configuration     |
|-------|-------------|-------------------|-------------------|
| **PA2** | LPUART1_TX  | UART Transmit     | Asynchronous mode |
| **PA3** | LPUART1_RX  | UART Receive      | Asynchronous mode |

**Baud Rate**: Configurable  
**Clock**: 84 MHz (APB1)

### USART2 (Main UART - hal_cfg.h Definition)
| Pin   | Signal     | Function          | Configuration     |
|-------|------------|-------------------|-------------------|
| **PA2** | USART2_TX  | UART Transmit     | AF7_USART2        |
| **PA3** | USART2_RX  | UART Receive      | AF7_USART2        |

**Note**: Conflicts with LPUART1 configuration in CubeMX

### USART3 (VUSA UART - hal_cfg.h Definition)
| Pin    | Signal     | Function          | Configuration     |
|--------|------------|-------------------|-------------------|
| **PB10** | USART3_TX  | VUSA UART TX      | AF7_USART3        |
| **PB11** | USART3_RX  | VUSA UART RX      | AF7_USART3        |

**DMA**: DMA1_Channel4 for TX  
**Clock**: 84 MHz (APB1)

### SPI1 (Accelerometer - hal_cfg.h Definition)
| Pin   | Signal      | Function               | Configuration     |
|-------|-------------|------------------------|-------------------|
| **PB3** | SPI1_SCK    | SPI Clock              | AF5_SPI1          |
| **PB4** | SPI1_MISO   | Master In Slave Out    | AF5_SPI1          |
| **PB5** | SPI1_MOSI   | Master Out Slave In    | AF5_SPI1          |
| **PC11** | ACC_CS      | Chip Select (GPIO)     | GPIO Output       |

**Mode**: Master, Full Duplex  
**Device**: LSM6DS3 IMU (Accelerometer + Gyroscope)

### SPI2 (Logger Slave - CubeMX Configured)
| Pin    | Signal      | Function               | Configuration         |
|--------|-------------|------------------------|-----------------------|
| **PB12** | SPI2_NSS    | Chip Select (HW)       | Hard NSS Input (Slave) |
| **PB13** | SPI2_SCK    | SPI Clock              | Slave mode            |
| **PB14** | SPI2_MISO   | Master In Slave Out    | Slave mode            |
| **PB15** | SPI2_MOSI   | Master Out Slave In    | Slave mode            |

**Mode**: Slave, Full Duplex, 8-bit data  
**DMA**: DMA2_Channel2 for TX

---

## PWM Outputs (Timers)

### TIM1 - Pump PWM Control
| Pin    | Signal         | Function                    | Configuration     |
|--------|----------------|-----------------------------|-------------------|
| **PA9**  | TIM1_CH2       | Pump PWM Main Output        | AF6_TIM1          |
| **PB14** | TIM1_CH2N      | Pump PWM Complementary Output | AF6_TIM1        |

**Timer Base**: TIM1  
**Channel**: Channel 2  
**Clock**: 168 MHz (APB2 Timer)

### TIM15 - Detonation High-Side Switch PWM
| Pin    | Signal         | Function                    | Configuration     |
|--------|----------------|-----------------------------|-------------------|
| **PB15** | TIM15_CH2      | Detonation High-Side PWM    | AF1_TIM15         |

**Timer Base**: TIM15  
**Channel**: Channel 2  
**Clock**: 168 MHz (APB2 Timer)

---

## Timers Configuration

| Timer  | Function                          | Configuration                    | Frequency/Period |
|--------|-----------------------------------|----------------------------------|------------------|
| **TIM1**  | Pump PWM                          | Channel 2 (+ complementary)      | Configurable     |
| **TIM2**  | System Tick Timer                 | Base timer for system ticks      | Configurable     |
| **TIM3**  | Logger Timestamp Counter          | Slave to TIM6 @ 100 kHz          | 100 kHz          |
| **TIM6**  | ADC2/DAC1 Clock                   | Trigger output (TRGO UPDATE)     | Period=1679 → 100 kHz @ 168 MHz |
| **TIM15** | Detonation High-Side Switch PWM   | Channel 2                        | Configurable     |

**TIM6 Calculation**: 168 MHz ÷ (1679 + 1) = 100 kHz

---

## DMA Configuration

| DMA Channel      | Peripheral  | Direction         | Mode     | Priority | Data Width |
|------------------|-------------|-------------------|----------|----------|------------|
| **DMA1_Channel1** | DAC1_CH1    | Memory → Periph   | Circular | High     | 32-bit     |
| **DMA1_Channel2** | ADC2        | Periph → Memory   | Circular | High     | 16-bit     |
| **DMA1_Channel4** | USART3_TX   | Memory → Periph   | Normal   | Medium   | 8-bit      |
| **DMA2_Channel2** | SPI2_TX     | Memory → Periph   | Normal   | High     | 8-bit      |

---

## Clock Configuration

### System Clocks
- **HSI**: 16 MHz (internal oscillator)
- **PLL Source**: HSI ÷ 2 = 8 MHz
- **PLL Multiplier**: ×42 = 336 MHz (VCO)
- **PLLR**: ÷2 → **SYSCLK = 168 MHz**
- **AHB (HCLK)**: 168 MHz (÷1)
- **APB1**: 84 MHz (÷2) → APB1 Timers: 168 MHz
- **APB2**: 84 MHz (÷2) → APB2 Timers: 168 MHz

### Peripheral Clocks
| Peripheral    | Clock Source | Frequency  |
|---------------|--------------|------------|
| ADC1/ADC2     | SYSCLK       | 168 MHz    |
| DAC1          | SYSCLK       | 168 MHz    |
| TIM1, TIM15   | APB2 Timer   | 168 MHz    |
| TIM2, TIM3, TIM6 | APB1 Timer | 168 MHz    |
| USART2, USART3 | APB1        | 84 MHz     |
| LPUART1       | APB1         | 84 MHz     |
| SPI1, SPI2    | SYSCLK       | 168 MHz    |

---

## NVIC Interrupts

| IRQ                  | Priority | Function                    |
|----------------------|----------|-----------------------------|
| ADC1_2_IRQn          | 0        | ADC1/ADC2 conversion complete |
| DMA1_Channel1_IRQn   | 0        | DAC1 DMA transfer           |
| DMA1_Channel2_IRQn   | 0        | ADC2 DMA transfer           |
| DMA1_Channel4_IRQn   | 0        | USART3 TX DMA               |
| DMA2_Channel2_IRQn   | 0        | SPI2 TX DMA                 |
| SPI2_IRQn            | 0        | SPI2 events                 |
| TIM6_DAC_IRQn        | 0        | TIM6/DAC events             |
| USART2_IRQn          | 0        | USART2 events               |
| USART3_IRQn          | 0        | USART3 events               |
| SysTick_IRQn         | 15       | System tick timer (1ms)     |

---

## Pin Summary by Port

### Port A (GPIOA)
| Pin | Function           | Type   | Peripheral   |
|-----|--------------------|--------|--------------|
| PA0 | ADC2_IN1           | Analog | ADC2         |
| PA2 | LPUART1_TX / USART2_TX | Output | LPUART1 / USART2 |
| PA3 | LPUART1_RX / USART2_RX | Input  | LPUART1 / USART2 |
| PA4 | DAC1_OUT1          | Analog | DAC1         |
| PA5 | EX_LED_OUT         | Output | GPIO         |
| PA6 | OPAMP2_VOUT        | Analog | OPAMP2       |
| PA7 | OPAMP2_VINP        | Analog | OPAMP2       |
| PA8 | TEST_1             | Output | GPIO         |
| PA9 | TIM1_CH2           | Output | TIM1 PWM     |
| PA10 | CHARGE_EN_OUT     | Output | GPIO         |
| PA11 | TEST_2            | Output | GPIO         |
| PA12 | FUSE_IN           | Input  | GPIO         |

### Port B (GPIOB)
| Pin  | Function           | Type   | Peripheral   |
|------|--------------------|--------|--------------|
| PB1  | LED_YELLOW_OUT     | Output | GPIO         |
| PB2  | LED_GREEN_OUT      | Output | GPIO         |
| PB3  | SPI1_SCK           | I/O    | SPI1         |
| PB4  | SPI1_MISO          | Input  | SPI1         |
| PB5  | SPI1_MOSI          | Output | SPI1         |
| PB6  | ACC_INT2           | Input  | GPIO         |
| PB7  | ACC_INT1           | Input  | GPIO         |
| PB10 | USART3_TX          | Output | USART3       |
| PB11 | USART3_RX          | Input  | USART3       |
| PB12 | BOOM_LOW_SIDE_OUT_2 / SPI2_NSS | Output / Input | GPIO / SPI2 |
| PB13 | BOOM_LOW_SIDE_OUT_1 / SPI2_SCK | Output / Input | GPIO / SPI2 |
| PB14 | TIM1_CH2N / SPI2_MISO | Output / I/O | TIM1 / SPI2 |
| PB15 | TIM15_CH2 / SPI2_MOSI | Output / Input | TIM15 / SPI2 |

### Port C (GPIOC)
| Pin  | Function           | Type   | Peripheral   |
|------|--------------------|--------|--------------|
| PC6  | SPI_DATA_RDY       | Output | GPIO         |
| PC11 | ACC_CS             | Output | GPIO         |

---

## Configuration Conflicts and Notes

### ⚠️ CubeMX vs hal_cfg.h Discrepancies

1. **UART Configuration**:
   - **CubeMX**: Uses **LPUART1** on PA2/PA3
   - **hal_cfg.h**: Defines **USART2** as MAIN_UART on PA2/PA3
   - **Resolution Needed**: Choose one UART type and update both files

2. **Port B Pins (PB12-PB15)**:
   - **CubeMX**: Configured as **SPI2 Slave** pins
   - **hal_cfg.h**: Defined as **GPIO outputs** (BOOM, PWM)
   - **Conflict**: Cannot use same pins for both functions simultaneously
   - **Resolution Needed**: Decide primary function for each pin

3. **Missing Peripherals in CubeMX**:
   - TIM1 (Pump PWM)
   - TIM2 (System Tick)
   - TIM3 (Logger Timestamp)
   - TIM15 (Detonation PWM)
   - SPI1 (Accelerometer)
   - USART2/USART3
   - Multiple GPIO pins defined in hal_cfg.h

### 📝 Recommendations

1. **Synchronize CubeMX with hal_cfg.h**: Add missing peripherals to CubeMX project
2. **Resolve pin conflicts**: Clarify whether PB12-15 are used for SPI2 or GPIO/PWM
3. **Update UART configuration**: Choose between LPUART1 and USART2 for main communication
4. **Document external connections**: Specify how DAC output connects to ADC input
5. **Verify DMA channels**: Ensure no conflicts between peripherals sharing DMA resources

---

## External Components

### LSM6DS3 IMU (Accelerometer + Gyroscope)
- **Interface**: SPI1 (PB3-5, PC11)
- **Interrupts**: ACC_INT1 (PB7), ACC_INT2 (PB6)
- **Modes**: Hit detection (1.66-6.66 kHz), Move detection (104 Hz)
- **Ranges**: ±2g to ±16g (accel), 245-2000 dps (gyro)

### Signal Processing Chain
```
DAC1 (PA4) → OPAMP2_VINP (PA7) → OPAMP2_VOUT (PA6) → [External] → ADC2 (PA0)
```

---

## Memory Usage (Last Build)
- **Flash**: 54,252 bytes / 128 KB (41.39%)
- **RAM**: 14,888 bytes / 32 KB (45.43%)

---

*Generated: December 12, 2025*  
*Project: G431CBU6_Wing_Example*  
*Repository: 3P_InitBoardFW (branch: wing_431_test)*
