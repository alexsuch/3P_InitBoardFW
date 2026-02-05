#ifndef ARDUINO_JSON_CONFIG_H
#define ARDUINO_JSON_CONFIG_H

// Configure ArduinoJson for ESP-IDF compatibility (non-Arduino environment)
// This solves the 'class String has no member named write' error
// Uses PsychicHttp's existing String class to avoid redefinition

// Disable Arduino dependencies
#define ARDUINOJSON_ENABLE_ARDUINO_STRING 0
#define ARDUINOJSON_ENABLE_ARDUINO_STREAM 0
#define ARDUINOJSON_ENABLE_ARDUINO_PRINT 0
#define ARDUINOJSON_ENABLE_PROGMEM 0

// Enable standard C++ features
#define ARDUINOJSON_ENABLE_STD_STRING 1
#define ARDUINOJSON_ENABLE_STD_STREAM 1

// Include standard C++ headers that ArduinoJson will use
#include <cctype>
#include <iostream>
#include <string>

// Note: We don't define String class here because PsychicHttp already provides it
// The String class is defined in lib/PsychiHttp/CustomString.h and included via PsychicCore.h

#endif  // ARDUINO_JSON_CONFIG_H