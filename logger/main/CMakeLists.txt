file(GLOB_RECURSE APP_SOURCES *.c *.cpp)
list(FILTER APP_SOURCES EXCLUDE REGEX ".*/test/.*")


get_filename_component(PROJ_ROOT "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)

set(APP_INCLUDE_DIRS
  "${CMAKE_CURRENT_LIST_DIR}"
)

if(EXISTS "${PROJ_ROOT}/include")
  list(APPEND APP_INCLUDE_DIRS "${PROJ_ROOT}/include")
endif()

# Build list of required components
set(COMPONENT_REQUIRES
    lwip
    fatfs
    hal-common
    hal-esp32
    vfs
    driver
    esp_timer
    esp_wifi
    esp_http_client
    esp_http_server
    esp_https_server
    esp_https_ota
    bootloader_support
    spi_flash
    app_update
    spiffs
    efuse
    bblanchon__arduinojson
)

# Only include esp_tinyusb for ESP32-S3 builds in this project.
if(IDF_TARGET STREQUAL "esp32s3")
    list(APPEND COMPONENT_REQUIRES esp_tinyusb)
endif()

idf_component_register(
  SRCS         ${APP_SOURCES}
  INCLUDE_DIRS ${APP_INCLUDE_DIRS}
  REQUIRES     ${COMPONENT_REQUIRES}
)
