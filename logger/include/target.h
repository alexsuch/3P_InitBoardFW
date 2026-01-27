#pragma once

#include "hal/ws2812.h"
#include "sdkconfig.h"

// This firmware receives binary frames from the STM32 over SPI and writes them to storage.
// Sensor acquisition and frame generation are performed on the STM32 side.
#define USE_S3_MINI_BOARD

#if !defined(CONFIG_IDF_TARGET_ESP32S3)
#error "This logger firmware supports ESP32-S3 only."
#endif

#define USE_FEATURE_BUTTON

// SPI Host Configuration
// Mapped per-target to available general-purpose SPI controllers.
// ESP32-S3 has SPI2_HOST and SPI3_HOST as general-purpose
#define SDCARD_SPI_HOST SPI2_HOST  // General-purpose SPI - safe to use with PSRAM

#if defined(USE_S3_MINI_BOARD)
#define LINK_SPI_HOST SPI3_HOST  // Shares SPI host/pins with the STM32 link
#else
#define LINK_SPI_HOST SPI2_HOST  // Alternative host option if SPI3 is unavailable
#endif
#define DAC_VREF_MV 3300

#define DEBUG_LOGGER_WRITES 1
#define DIAGNOSTIC_IN_LOG
#define DIAGNOSTIC_IN_LOG_PERIOD_SEC 30U

// STM32 link SPI clock (Hz). This is the master SCK used to read `logger_frame_t` from STM32.
// Note: Higher frequencies reduce backlog risk but require good signal integrity.
#ifndef LINK_SPI_FREQ_HZ
#define LINK_SPI_FREQ_HZ (8U * 1000U * 1000U)
#endif

#define LED_COLOR_ORDER 0


// SD card tuning (SPI via SDSPI host).
// These macros define the SPI clock for the SD card and basic FAT mount options.
// Increase SDCARD_SPI_FREQ_KHZ cautiously to explore the maximum sustainable rate.

#ifndef SDCARD_SPI_FREQ_KHZ
#define SDCARD_SPI_FREQ_KHZ 20000U
#endif

#ifndef SDCARD_MAX_FILES
#define SDCARD_MAX_FILES 1
#endif

#ifndef SDCARD_ALLOC_UNIT
#define SDCARD_ALLOC_UNIT (16 * 1024)
#endif

#if defined(CONFIG_IDF_TARGET_ESP32S3) && defined(USE_S3_MINI_BOARD)
// ESP32-S3 Mini Configuration
// Pin assignments using only GPIO pins 1-13 (GPIO 3 avoided - strapping pin)
// Configuration: 2 independent SPI buses, 1 interrupt, LED, external button, no UART
// These pins are safe because they are:
// - Not involved in bootstrapping (GPIO 3 is strapping pin - AVOIDED)
// - Not linked to flash memory or PSRAM (won't interfere with storage/memory)
// - Not dedicated to USB or JTAG (free for general use)
// - No special hardware connections (freely assignable)
// Note: GPIO 1 can conflict with UART0 TX, but UART functionality is not used

#define USE_USB_FILE_SERVER  // Use USB file server to download files from SD card (TinyUSB supported)
#define USE_RGB_LED
#define USE_LED_OUT

#define BUTTON_ENTER_GPIO 0
#define BUTTON_EXTERNAL_GPIO 12

#define LED_OUT_GPIO 11
#define RGB_LED_GPIO 48

#define STM_RESET_GPIO 3

#define LINK_INT_GPIO 10
#define LINK_SPI_CLK_GPIO 1
#define LINK_SPI_MISO_GPIO 2
#define LINK_SPI_MOSI_GPIO 4
#define LINK_SPI_CS_GPIO 5

// SD Card SPI Configuration (SPI2_HOST) - Second SPI bus (independent pins)
// Uses different pins from the STM32 link SPI to avoid conflicts
#define SD_MOUNT_PATH "/sdcard"
#define SD_SPI_CLK_GPIO 6
#define SD_SPI_MISO_GPIO 7
#define SD_SPI_MOSI_GPIO 8
#define SD_SPI_CS_GPIO 9

#define DAC_OUT_1_GPIO -1
#define DAC_OUT_2_GPIO -1

#endif  // USE_S3_MINI_BOARD

// Task Priority Definitions
#define TASK_PRIORITY_LED_BLINK 1
#define TASK_PRIORITY_USB_BOOT_CMD 1
#define TASK_PRIORITY_APP_LOGIC 2
#define TASK_PRIORITY_IO_MANAGER 3
#define TASK_PRIORITY_LOGGER 6
#define TASK_PRIORITY_STM_LINK 8
#if defined(USE_WIFI)
#define TASK_PRIORITY_WIFI_EVENT 4
#endif
#if defined(USE_WEB_FILE_SEREVER)
#define TASK_PRIORITY_WEB_SERVER 5
#endif

// Task core affinity configuration (0 or 1 on dual-core ESP32)
// Use tskNO_AFFINITY if you want the scheduler to choose.

#define CORE_APP_LOGIC 0
#define CORE_IO_MANAGER 1
#define CORE_STM_LINK_TASK 0
#define CORE_LOGGER_PROC 1
#define CORE_LOGGER_WRITE 1

// Task Stack Size Definitions (all values expressed in bytes for ESP-IDF)
// Optimized LED task stack - reduced from 3072 to 1536 bytes
// LED task only does simple GPIO operations and delays - no complex operations needed
// Stack usage reduced by removing expensive printf-style logging calls
#define TASK_STACK_SIZE_LED_BLINK (3072U)
// Note: uses stdio (`fgetc`) on the USB-CDC console, which needs more stack than a trivial task.
#define TASK_STACK_SIZE_USB_BOOT_CMD (8192U)
#define TASK_STACK_SIZE_APP_LOGIC (4096U)
#define TASK_STACK_SIZE_IO_MANAGER (3048U)
#define TASK_STACK_SIZE_LOGGER (4096U)
#define TASK_STACK_SIZE_STM_LINK (6000U)

#if defined(USE_WIFI)
#define TASK_STACK_SIZE_WIFI_EVENT (4096U)

// WiFi Credentials
#define WIFI_STA_SSID "902studio"
#define WIFI_STA_PASSWORD "Emc902NetWifiNew"
#define WIFI_AP_SSID "3PLogger"
#define WIFI_AP_PASSWORD "3PLogger"
#define WIFI_AP_CHANNEL 0

// WiFi Mode Configuration
// Enable one of the following WiFi modes:
// #define WIFI_MODE_STA_ONLY  // Station mode only
#define WIFI_MODE_AP_ONLY  // Access Point mode only
                           // #define WIFI_MODE_STA_AP  // Both Station and Access Point modes
#endif
#if defined(USE_WEB_FILE_SEREVER)
#define TASK_STACK_SIZE_WEB_SERVER (8192U)
#endif
