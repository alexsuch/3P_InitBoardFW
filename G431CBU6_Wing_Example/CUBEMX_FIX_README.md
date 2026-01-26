# Post-CubeMX Configuration Fix

Після кожної регенерації коду через STM32CubeMX запустіть скрипт для відновлення custom конфігурації SPI:

## Windows (PowerShell)
```powershell
.\fix_cubemx.ps1
```

## Що виправляє скрипт:

1. **stm32g4xx_hal_conf.h** - Розкоментовує `HAL_SPI_MODULE_ENABLED`
2. **cmake/stm32cubemx/CMakeLists.txt** - Додає `stm32g4xx_hal_spi.c` та `stm32g4xx_hal_spi_ex.c`

## Чому це потрібно?

SPI1 ініціалізується повністю в `Core/Src/solution_hal_cfg.c` (не через CubeMX), тому:
- CubeMX не знає про використання SPI і вимикає його HAL модуль
- CubeMX не додає SPI драйвери в build

## Альтернатива (без скрипта)

Якщо не хочете використовувати скрипт, виправте вручну після CubeMX:

### 1. Core/Inc/stm32g4xx_hal_conf.h (рядок ~63)
```c
// Було:
/*#define HAL_SPI_MODULE_ENABLED   */

// Стало:
#define HAL_SPI_MODULE_ENABLED
```

### 2. cmake/stm32cubemx/CMakeLists.txt
Додайте після `stm32g4xx_hal_tim_ex.c`:
```cmake
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_spi.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_spi_ex.c
```
