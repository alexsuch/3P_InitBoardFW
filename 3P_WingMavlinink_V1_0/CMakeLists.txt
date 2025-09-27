cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Enable debug information for all configurations
set(CMAKE_C_FLAGS_DEBUG "-g3 -O0")
set(CMAKE_C_FLAGS_RELEASE "-g -O2")

# Set the project name
set(CMAKE_PROJECT_NAME 3P_WingMavlinink_V1_0)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Define paths to shared directories
set(SHARED_ROOT_DIR "${CMAKE_SOURCE_DIR}/../shared")
set(SHARED_SRC_DIR "${SHARED_ROOT_DIR}/Src")
set(SHARED_INC_DIR "${SHARED_ROOT_DIR}/Inc")

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Collect all shared source files from Src folder
file(GLOB_RECURSE SHARED_SOURCES
    "${SHARED_SRC_DIR}/*.c"
    "${SHARED_SRC_DIR}/*.cpp"
)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add shared source files
    ${SHARED_SOURCES}
    # Add user sources here
    Core/Src/solution_wrapper.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add shared include directory
    ${SHARED_INC_DIR}
    # Add shared source directory (for headers in Src folder)
    ${SHARED_SRC_DIR}
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
