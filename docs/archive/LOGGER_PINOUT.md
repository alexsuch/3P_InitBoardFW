# Logger Wiring Pinout (STM32G431CBU6 Wing Example)

This pinout is for the Logger build (SPI logger enabled) in `G431CBU6_Wing_Example`. It covers the IMU (accelerometer), piezo ADC input, and SPI2 slave link to the external module.

## Accelerometer (LSM6DS3 over SPI1)
| MCU pin | Signal | Direction | Notes |
| --- | --- | --- | --- |
| PB3 | SPI1_SCK | Output | SPI clock to IMU |
| PB4 | SPI1_MISO | Input | Data from IMU |
| PB5 | SPI1_MOSI | Output | Data to IMU |
| PC11 | ACC_CS | Output | Chip select (GPIO) |
| PB7 | ACC_INT1 | Input | Interrupt 1 from IMU |
| PB6 | ACC_INT2 | Input | Interrupt 2 from IMU |

LSM6DS3 pin label hints (SPI mode):
- SCL/SPC -> SPI1_SCK (PB3)
- SDA/SDI/SDO -> SPI1_MOSI (PB5)
- SDO/SA0 -> SPI1_MISO (PB4)

## Piezo / ADC input (ADC2)
| MCU pin | Signal | Direction | Notes |
| --- | --- | --- | --- |
| PA0 | ADC2_IN1 | Analog in | Piezo/ADC input sampled by ADC2 DMA |

Optional test chain used in the G431 example:
`PA4 (DAC1_OUT1) -> PA7 (OPAMP2_VINP) -> PA6 (OPAMP2_VOUT) -> PA0 (ADC2_IN1)`

## Logger SPI link to external module (SPI2 slave)
| MCU pin | Signal | Direction | Notes |
| --- | --- | --- | --- |
| PB12 | SPI2_NSS | Input | Chip select from master (hardware NSS) |
| PB13 | SPI2_SCK | Input | Clock from master |
| PB14 | SPI2_MISO | Output | Data to master |
| PB15 | SPI2_MOSI | Input | Data from master |
| PC6 | LOGGER_SPI_DATA_RDY | Output | Data ready GPIO, active high |

## Notes and conflicts
- PB12..PB15 are shared with BOOM outputs/PWM in `G431CBU6_Wing_Example/PINOUT.md`. Logger build uses them for SPI2, so do not drive them as GPIO/PWM.
- `G431CBU6_Wing_Example/Core/Inc/hal_cfg.h` defines `ADC2_IN_PIN` as PA7, but CubeMX/MSP config wires ADC2 to PA0 (`G431CBU6_Wing_Example/Core/Src/stm32g4xx_hal_msp.c`). Use PA0 for piezo wiring unless the pin config is updated.
