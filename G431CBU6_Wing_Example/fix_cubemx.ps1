#!/usr/bin/env pwsh
# fix_cubemx.ps1
# Script to fix CubeMX-generated files for custom SPI initialization
# Run this script after CubeMX code generation

Write-Host "Fixing CubeMX-generated files..." -ForegroundColor Cyan

# Fix 1: Enable HAL_SPI_MODULE_ENABLED in stm32g4xx_hal_conf.h
$halConfFile = "Core/Inc/stm32g4xx_hal_conf.h"
Write-Host "  - Enabling HAL_SPI_MODULE_ENABLED in $halConfFile"
(Get-Content $halConfFile -Raw) -replace '/\*#define HAL_SPI_MODULE_ENABLED   \*/','#define HAL_SPI_MODULE_ENABLED' | Set-Content $halConfFile -NoNewline

# Fix 2: Add SPI drivers to CMakeLists.txt
$cmakeFile = "cmake/stm32cubemx/CMakeLists.txt"
Write-Host "  - Adding SPI drivers to $cmakeFile"

$content = Get-Content $cmakeFile -Raw

# Check if SPI drivers already added
if ($content -notmatch 'stm32g4xx_hal_spi\.c') {
    # Find the line with stm32g4xx_hal_tim_ex.c and add SPI drivers after it
    $content = $content -replace `
        '(stm32g4xx_hal_tim_ex\.c\r?\n\)',`
        ('stm32g4xx_hal_tim_ex.c' + "`n" + '    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_spi.c' + "`n" + '    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_spi_ex.c' + "`n" + ')')
    
    $content | Set-Content $cmakeFile -NoNewline
}

Write-Host "Done! Files fixed successfully." -ForegroundColor Green
Write-Host ""
Write-Host "Summary of changes:" -ForegroundColor Yellow
Write-Host "  1. Enabled HAL_SPI_MODULE_ENABLED in stm32g4xx_hal_conf.h"
Write-Host "  2. Added stm32g4xx_hal_spi.c and stm32g4xx_hal_spi_ex.c to CMakeLists.txt"
Write-Host ""
Write-Host "You can now build the project." -ForegroundColor Green
