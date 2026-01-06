# IB_Wing_STM32G431CBT6

Firmware for STM32G431CBT6 (LQFP48) wing board - initialization board for 3P project.

## Project Overview

**MCU**: STM32G431CBT6 (LQFP48 package, 128KB Flash, 32KB RAM)  
**Core**: ARM Cortex-M4 @ 168 MHz  
**Package**: LQFP48 (Low-Profile Quad Flat Package)  
**Toolchain**: ARM GCC + CMake + Ninja

This project is functionally identical to `G431CBU6_Wing_Example` but uses the LQFP48 package (CBT6) instead of UFQFPN48 (CBU6) for easier assembly and debugging.

## Features

- **Data Logging**: High-speed ADC sampling @ 100 kHz with SPI slave interface
- **Accelerometer**: LSM6DS3 IMU (accelerometer + gyroscope) via SPI1
- **MAVLink Control**: UART communication for flight controller integration
- **Signal Processing**: DAC output → OPAMP follower → ADC input test chain
- **PWM Control**: Pump control (TIM1) and detonation control (TIM15)
- **Status Indicators**: Green/Yellow LEDs and external LED output

## Project Structure

```
IB_Wing_STM32G431CBT6/
├── Core/
│   ├── Inc/              # Headers (main.h, hal_cfg.h, prj_config.h, etc.)
│   └── Src/              # Application code (main.c, solution_*.c, etc.)
├── Drivers/              # STM32 HAL drivers
├── cmake/                # CMake configuration files
│   └── stm32cubemx/      # STM32CubeMX integration
├── build/                # Build output directory
│   ├── Debug/            # Debug build artifacts
│   └── Release/          # Release build artifacts
├── .vscode/              # VS Code configuration
├── CMakeLists.txt        # Main CMake configuration
├── CMakePresets.json     # CMake build presets
├── PINOUT.md             # Detailed pinout documentation
├── fix_cubemx.ps1        # Post-CubeMX configuration script
└── CUBEMX_FIX_README.md  # Instructions for CubeMX workflow
```

## Shared Code

This project uses shared library code from `../shared/`:
- Application logic (`app.c`)
- Accelerometer drivers (`LIS2DH12.c`, `LSM6DS3.c`)
- MAVLink integration (`mavlink_uart.c`)
- MSP OSD support (`msp_osd.c`)
- SPI logger (`logger_spi.c`)
- Configuration (`app_config.c`)
- Utilities (`indication.c`, `timer.c`, `stick_ctrl.c`, `uart_configurator.c`)

## Build Instructions

### Prerequisites

- ARM GCC toolchain (`arm-none-eabi-gcc`)
- CMake (≥3.22)
- Ninja build system
- STM32CubeMX (for hardware configuration regeneration)
- ST-LINK utilities or STM32CubeProgrammer (for flashing)

### Configure and Build

1. **Configure CMake** (first time or after clean):
   ```powershell
   cmake --preset Debug
   ```

2. **Build the project**:
   ```powershell
   cmake --build build/Debug
   ```
   
   Or use VS Code task: `Ctrl+Shift+B` → **Build Debug + Generate HEX**

3. **Output files** are in `build/Debug/`:
   - `IB_Wing_STM32G431CBT6.elf` (debug symbols)
   - `IB_Wing_STM32G431CBT6.hex` (flash programming)
   - `IB_Wing_STM32G431CBT6.bin` (binary image)

### Flash Firmware

#### Using VS Code Tasks
- **Flash Debug HEX**: Flash existing HEX file via ST-LINK
- **Build and Flash**: Rebuild and flash in one step

#### Using Command Line
```powershell
STM32_Programmer_CLI -c port=SWD -w build/Debug/IB_Wing_STM32G431CBT6.hex -v -rst
```

### Clean Build
```powershell
cmake --build build/Debug --target clean
```

## Development Workflow

### After CubeMX Regeneration

STM32CubeMX doesn't know about manually configured SPI1 peripheral, so after each code regeneration run:

```powershell
.\fix_cubemx.ps1
```

This script:
1. Enables `HAL_SPI_MODULE_ENABLED` in `stm32g4xx_hal_conf.h`
2. Adds SPI driver files to CMake build

See [CUBEMX_FIX_README.md](CUBEMX_FIX_README.md) for details.

### Debugging

Launch configurations are available in `.vscode/launch.json`:
- **STM32 Debug (Cortex-Debug / ST-LINK)** - recommended
- **STM32 Debug (OpenOCD)** - alternative debugger

Press `F5` to build and start debugging.

## Configuration

### Build Configuration

Edit `Core/Inc/prj_config.h` to change:
- ADC sampling frequency
- Accelerometer type and ODR
- Feature enable/disable (DAC test, VBAT, VUSA, OSD, logger, etc.)
- Control mode (PWM / MAVLink)

### Hardware Mapping

All hardware pins and peripherals are mapped in:
- `Core/Inc/hal_cfg.h` - Hardware abstraction layer definitions
- `PINOUT.md` - Complete pinout documentation

## Key Differences from G431CBU6_Wing_Example

- **Package**: LQFP48 instead of UFQFPN48 (same pinout, different physical form)
- **Project Name**: Updated throughout CMake, tasks, and launch configurations
- All other functionality remains identical

## Memory Usage

Typical build (Debug):
- **Flash**: ~54 KB / 128 KB (42%)
- **RAM**: ~15 KB / 32 KB (45%)

## Documentation

- [PINOUT.md](PINOUT.md) - Detailed pin assignments and peripheral configuration
- [CUBEMX_FIX_README.md](CUBEMX_FIX_README.md) - Post-CubeMX workflow
- `../shared/3P_BOARD_MAVLINK_IMPLEMENTATION_INSTRUCTIONS.md` - MAVLink integration guide
- `../shared/OSD_MESSAGE_RULES.md` - OSD messaging protocol

## License

This project is part of the 3P_InitBoardFW repository.

---

*Project: IB_Wing_STM32G431CBT6*  
*Last Updated: January 1, 2026*
