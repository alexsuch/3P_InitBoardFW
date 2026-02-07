@echo off
echo Building Log Parser Wasm Module...

where emcc >nul 2>nul
if %ERRORLEVEL% neq 0 (
    echo ERROR: emcc not found. Please activate emsdk environment.
    exit /b 1
)

set EXPORTED_FUNCS="_parse_binary_log","_calculate_anomalies","_interpolate_to_master","_hello_world","_add","_malloc","_free"
set EXPORTED_RUNTIME="cwrap","getValue","setValue","HEAPF64","HEAPU8","HEAPU32"

call emcc log_parser.c -O3 ^
    -s MODULARIZE=1 ^
    -s SINGLE_FILE=1 ^
    -s EXPORT_NAME="createLogParserModule" ^
    -s EXPORTED_FUNCTIONS=[%EXPORTED_FUNCS%] ^
    -s EXPORTED_RUNTIME_METHODS=[%EXPORTED_RUNTIME%] ^
    -s ALLOW_MEMORY_GROWTH=1 ^
    -s INITIAL_MEMORY=33554432 ^
    -o ../log_parser.js

if %ERRORLEVEL% equ 0 (
    echo Build successful! Output in ../log_parser.js
) else (
    echo Build failed!
    exit /b 1
)
