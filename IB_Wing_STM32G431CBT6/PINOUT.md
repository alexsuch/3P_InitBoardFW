# STM32G431CBT6 Wing - Pinout and Connection Map

## Project Overview
**MCU**: STM32G431CBT6 (LQFP48 package)  
**System Clock**: 168 MHz (PLL: HSI 16 MHz ÷ 2 × 42)  
**Core**: ARM Cortex-M4  
**Flash**: 128 KB  
**RAM**: 32 KB  

---

## Pin Configuration by Mode

### Mode 1: SPI_LOGGER_ENABLE = 0 (Standard Detonation Mode)
- **VUSA UART**: Enabled (PB10/PB11)
- **Logger SPI**: Disabled
- **Detonation Control**: Active
- **Boom/LED Pins**: Active

### Mode 2: SPI_LOGGER_ENABLE = 1 (Logger Mode)
- **VUSA UART**: Disabled
- **Logger SPI2**: Active (PB12-15 as SPI slave)
- **Detonation Control**: Disabled
- **Boom/LED Pins**: Disabled
- **Logger Ready Signal**: PB11 GPIO output

---

## Core Peripherals (Always Active)

### Timers
| Timer  | Use                  | Channel  | Pin         | Frequency      |
|--------|----------------------|----------|-------------|----------------|
| TIM2   | System Tick Timer    | -        | -           | 100 Hz (10ms)  |
| TIM6   | ADC/DAC Clock        | TRGO     | -           | 100 kHz        |
| TIM3   | Logger Timestamp     | Slave    | -           | 100 kHz        |

### ADC/DAC (Always Configured)
| Peripheral | Pin   | Function           | Trigger    | Mode           |
|------------|-------|-------------------|------------|----------------|
| ADC2_CH1   | PA0   | Analog Input (Piezo/Sensor) | TIM6_TRGO | Single-ended   |
| DAC1_CH1   | PA4   | DAC Output (Test Signal) | TIM6_TRGO | DMA circular   |

### OPAMP2 (Always Configured)
| Pin   | Function                | Configuration |
|-------|-------------------------|----------------|
| PA7   | OPAMP2 Non-Inverting Input | Follower mode |
| PA6   | OPAMP2 Output           | Follower mode  |

**Signal Chain**: DAC1 (PA4) → OPAMP2 follower (PA7→PA6) → Precision output

### Main UART (USART2)
| Pin   | Function    | Baud Rate  |
|-------|-------------|------------|
| PA2   | TX          | 9600       |
| PA3   | RX          | 9600       |

### Accelerometer SPI (SPI1)
| Pin    | Function           | Mode          |
|--------|-------------------|---------------|
| PB3    | SCK                | SPI Master    |
| PB4    | MISO               | SPI Master    |
| PB5    | MOSI               | SPI Master    |
| PC11   | ACC_CS (GPIO)      | GPIO Output   |
| PB6    | ACC_INT2           | GPIO Input    |
| PB7    | ACC_INT1           | GPIO Input    |

---

## Mode-Dependent Configuration

### SPI_LOGGER_ENABLE = 0: Detonation Mode

#### VUSA UART (USART3)
| Pin    | Function | Baud Rate  |
|--------|----------|------------|
| PB10   | TX       | 115200     |
| PB11   | RX       | 115200     |

#### GPIO Outputs
| Pin    | Signal                  | Function                    | Type          |
|--------|-------------------------|---------------------------|---------------|
| PA5    | EX_LED_OUT              | External LED                | GPIO Output   |
| PA8    | TEST_1                  | Test Output 1               | GPIO Output   |
| PA10   | CHARGE_EN_OUT           | Charge Enable Control       | GPIO Output   |
| PA11   | TEST_2                  | Test Output 2               | GPIO Output   |
| PB1    | LED_ERROR_OUT           | Red LED (Error)             | GPIO Output   |
| PB2    | LED_STATUS_OUT          | Green LED (Status)          | GPIO Output   |
| PB12   | BOOM_LOW_SIDE_OUT_2     | Detonation Low-Side Out 2   | GPIO Output   |
| PB13   | BOOM_LOW_SIDE_OUT_1     | Detonation Low-Side Out 1   | GPIO Output   |

#### PWM Outputs
| Pin    | Function            | Timer   | Frequency      |
|--------|-------------------|---------|----------------|
| PA9    | PUMP_PWM_MAIN      | TIM1_CH2 | 40 kHz        |
| PB14   | PUMP_PWM_COMPL     | TIM1_CH2N | 40 kHz       |
| PB15   | DETON_PWM          | TIM15_CH2 | 138 kHz      |

#### GPIO Inputs
| Pin    | Signal    | Function                |
|--------|-----------|------------------------|
| PA12   | FUSE_IN   | Fuse Detection          |

---

### SPI_LOGGER_ENABLE = 1: Logger Mode

#### Logger SPI2 (Slave Mode)
| Pin    | Signal      | Function          | Mode        |
|--------|------------|-------------------|-------------|
| PB12   | SPI2_NSS   | Chip Select       | Hardware NSS |
| PB13   | SPI2_SCK   | Clock             | Slave       |
| PB14   | SPI2_MISO  | Master In Slave Out | Slave      |
| PB15   | SPI2_MOSI  | Master Out Slave In | Slave     |

#### Logger Ready Signal
| Pin    | Signal              | Function                      | Type         |
|--------|-------------------|-------------------------------|--------------|
| PB11   | LOGGER_SPI_DATA_RDY | SPI Data Ready to Master     | GPIO Output  |

**Note**: 
- VUSA UART (PB10/PB11) is **not initialized** in Logger mode
- PB11 used exclusively for Logger_GPIO_SetReady() signal
- Detonation and LED pins are **not initialized**
- PUMP and DETONATION PWM timers are **not initialized**

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

### Peripheral Clock Summary
| Peripheral    | Clock Source | Frequency  |
|---------------|--------------|------------|
| ADC1/ADC2     | SYSCLK       | 168 MHz    |
| DAC1          | SYSCLK       | 168 MHz    |
| TIM1, TIM15   | APB2 Timer   | 168 MHz    |
| TIM2, TIM3, TIM6 | APB1 Timer | 168 MHz    |
| USART2        | APB1         | 84 MHz     |
| USART3 (VUSA) | APB1         | 84 MHz     |
| SPI1 (Accel)  | SYSCLK       | 168 MHz    |
| SPI2 (Logger) | SYSCLK       | 168 MHz    |

---

## DMA Configuration

| DMA Channel      | Peripheral  | Direction         | Mode     | Priority |
|------------------|-------------|-------------------|----------|----------|
| **DMA1_Channel1** | DAC1_CH1    | Memory → Periph   | Circular | High     |
| **DMA1_Channel2** | ADC2        | Periph → Memory   | Circular | High     |
| **DMA1_Channel3** | SPI1_TX     | Memory → Periph   | Normal   | Low      |
| **DMA1_Channel4** | USART3_TX   | Memory → Periph   | Circular | Low      |
| **DMA1_Channel5** | SPI1_RX     | Periph → Memory   | Normal   | Low      |
| **DMA2_Channel2** | SPI2_TX     | Memory → Periph   | Normal   | High     |

---

## NVIC Interrupt Priorities

| IRQ                   | Priority | Function                    |
|----------------------|----------|---------------------------|
| ADC1_2_IRQn          | 0        | ADC conversion complete    |
| DMA1_Channel1_IRQn   | 0        | DAC1 DMA                  |
| DMA1_Channel2_IRQn   | 0        | ADC2 DMA                  |
| DMA1_Channel3_IRQn   | 0        | SPI1 TX DMA               |
| DMA1_Channel4_IRQn   | 0        | USART3 TX DMA             |
| DMA1_Channel5_IRQn   | 0        | SPI1 RX DMA               |
| DMA2_Channel2_IRQn   | 0        | SPI2 TX DMA               |
| TIM2_IRQn            | 0        | System Tick Timer         |
| TIM6_DAC_IRQn        | 0        | ADC/DAC Clock Trigger     |
| USART2_IRQn          | 0        | Main UART                 |
| USART3_IRQn          | 0        | VUSA UART (mode 0 only)  |
| SPI1_IRQn            | 0        | Accelerometer SPI         |
| SPI2_IRQn            | 0        | Logger SPI (mode 1 only) |

---

## Port A (GPIOA) Summary

| Pin | Function           | Type    | Mode 0 | Mode 1 |
|-----|-------------------|---------|--------|--------|
| PA0 | ADC2_IN1          | Analog  | ✓      | ✓      |
| PA2 | USART2_TX         | Output  | ✓      | ✓      |
| PA3 | USART2_RX         | Input   | ✓      | ✓      |
| PA4 | DAC1_OUT1         | Analog  | ✓      | ✓      |
| PA5 | EX_LED_OUT        | Output  | ✓      | ✗      |
| PA6 | OPAMP2_VOUT       | Analog  | ✓      | ✓      |
| PA7 | OPAMP2_VINP       | Analog  | ✓      | ✓      |
| PA8 | TEST_1            | Output  | ✓      | ✗      |
| PA9 | TIM1_CH2 (PWM)    | Output  | ✓      | ✗      |
| PA10| CHARGE_EN_OUT     | Output  | ✓      | ✗      |
| PA11| TEST_2            | Output  | ✓      | ✗      |
| PA12| FUSE_IN           | Input   | ✓      | ✗      |

---

## Port B (GPIOB) Summary

| Pin  | Function           | Type    | Mode 0 | Mode 1 |
|------|-------------------|---------|--------|--------|
| PB1  | LED_ERROR_OUT     | Output  | ✓      | ✗      |
| PB2  | LED_STATUS_OUT    | Output  | ✓      | ✗      |
| PB3  | SPI1_SCK          | I/O     | ✓      | ✓      |
| PB4  | SPI1_MISO         | Input   | ✓      | ✓      |
| PB5  | SPI1_MOSI         | Output  | ✓      | ✓      |
| PB6  | ACC_INT2          | Input   | ✓      | ✓      |
| PB7  | ACC_INT1          | Input   | ✓      | ✓      |
| PB10 | USART3_TX (VUSA)  | Output  | ✓      | ✗      |
| PB11 | USART3_RX / LOGGER_RDY | - | ✓(UART) | ✓(GPIO)|
| PB12 | BOOM_OUT_2 / SPI2_NSS | - | ✓(GPIO) | ✓(SPI) |
| PB13 | BOOM_OUT_1 / SPI2_SCK | - | ✓(GPIO) | ✓(SPI) |
| PB14 | PWM_COMPL / SPI2_MISO | - | ✓(PWM) | ✓(SPI) |
| PB15 | DETON_PWM / SPI2_MOSI | - | ✓(PWM) | ✓(SPI) |

---

## Port C (GPIOC) Summary

| Pin  | Function           | Type    | Mode 0 | Mode 1 |
|------|-------------------|---------|--------|--------|
| PC11 | ACC_CS (GPIO)     | Output  | ✓      | ✓      |

---

## External Connections

### LSM6DS3 IMU (Accelerometer + Gyroscope)
- **Interface**: SPI1 (PB3-5, PC11)
- **Interrupts**: ACC_INT1 (PB7), ACC_INT2 (PB6)
- **Features**: 6-axis IMU (accel + gyro)
- **Supported Modes**: Hit detection, Move detection

### Signal Processing Chain
```
DAC1 (PA4) → OPAMP2 (PA7/PA6) → [External Cable] → ADC2 (PA0)
```

---

## Key Features

✅ **Configurable Dual-Mode Operation**
- Mode 0: Standard detonation with VUSA communication
- Mode 1: Logger mode with SPI slave data streaming

✅ **High-Speed Sampling**
- ADC2 @ 100 kHz synchronized with DAC
- DMA-based circular buffering

✅ **Flexible PWM Control**
- Pump PWM: 40 kHz (TIM1)
- Detonation PWM: 138 kHz (TIM15)

✅ **Dual SPI Interfaces**
- SPI1 Master: Accelerometer communication
- SPI2 Slave: Logger data stream (mode 1 only)

✅ **Integrated Signal Processing**
- OPAMP2 follower for low-impedance output
- Closed-loop DAC-OPAMP-ADC circuit

---

*Last Updated: January 6, 2026*  
*Project: IB_Wing_STM32G431CBT6*  
*Mode Status: Configurable via SPI_LOGGER_ENABLE*


