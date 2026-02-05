cmake_minimum_required(VERSION 3.16.0)

if(DEFINED CMAKE_SCRIPT_MODE_FILE)
  return()
endif()

if(NOT CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  return()
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

list(APPEND EXTRA_COMPONENT_DIRS
    ${CMAKE_SOURCE_DIR}/components
    ${CMAKE_SOURCE_DIR}/include
)

#
# ---- Target-specific sdkconfig (forced) ----

if(NOT IDF_TARGET)
  set(IDF_TARGET "esp32")
endif()

# unset(SDKCONFIG CACHE)

# Get PlatformIO environment name if available
# Priority: CMake variable > Environment variable
if(DEFINED PIO_ENV_NAME)
  # Already set via CMake variable (e.g., from board_build.cmake_extra_args)
elseif(DEFINED ENV{PIOENV})
  set(PIO_ENV_NAME "$ENV{PIOENV}")
else()
  set(PIO_ENV_NAME "")
endif()

# Compose SDKCONFIG_DEFAULTS from environment-specific, target-specific and common defaults, if present
set(_SDK_DEFAULTS_LIST "")
set(_ENV_SDKCONFIG_FOUND FALSE)

# Priority 1: Environment-specific sdkconfig (e.g., sdkconfig.esp32s3_logger)
if(PIO_ENV_NAME)
  set(_ENV_SDKCONFIG "${CMAKE_SOURCE_DIR}/sdkconfig.${IDF_TARGET}_${PIO_ENV_NAME}")
  if(EXISTS "${_ENV_SDKCONFIG}")
    list(APPEND _SDK_DEFAULTS_LIST "${_ENV_SDKCONFIG}")
    set(_ENV_SDKCONFIG_FOUND TRUE)
    message(STATUS "Found environment-specific sdkconfig: ${_ENV_SDKCONFIG}")
  endif()
endif()

# Priority 2: Target-specific sdkconfig (e.g., sdkconfig.esp32s3) - only if no env-specific found
if(NOT _ENV_SDKCONFIG_FOUND)
  if(EXISTS "${CMAKE_SOURCE_DIR}/sdkconfig.${IDF_TARGET}")
    list(APPEND _SDK_DEFAULTS_LIST "${CMAKE_SOURCE_DIR}/sdkconfig.${IDF_TARGET}")
  endif()
endif()

# Priority 3: Common defaults
if(EXISTS "${CMAKE_SOURCE_DIR}/sdkconfig.defaults")
  list(APPEND _SDK_DEFAULTS_LIST "${CMAKE_SOURCE_DIR}/sdkconfig.defaults")
endif()
if(EXISTS "${CMAKE_SOURCE_DIR}/sdkconfig.defaults.${IDF_TARGET}")
  list(APPEND _SDK_DEFAULTS_LIST "${CMAKE_SOURCE_DIR}/sdkconfig.defaults.${IDF_TARGET}")
endif()
if(_SDK_DEFAULTS_LIST)
  set(SDKCONFIG_DEFAULTS "${_SDK_DEFAULTS_LIST}" CACHE STRING "Defaults for sdkconfig" FORCE)
endif()

if(DEFINED SDKCONFIG)
  message(STATUS "Using SDKCONFIG: ${SDKCONFIG} (IDF_TARGET=${IDF_TARGET}, PIO_ENV=${PIO_ENV_NAME})")
else()
  message(STATUS "Using SDKCONFIG_DEFAULTS: ${SDKCONFIG_DEFAULTS} (IDF_TARGET=${IDF_TARGET}, PIO_ENV=${PIO_ENV_NAME})")
endif()

project(3p-logger-next)
