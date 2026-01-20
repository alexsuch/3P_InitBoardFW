#ifndef PsychicCore_h
#define PsychicCore_h
#include "CustomString.h"
#define PH_TAG "psychic"
#define PSYCHIC_USE_CJSON 0

// version numbers
#define PSYCHIC_HTTP_VERSION_MAJOR 1
#define PSYCHIC_HTTP_VERSION_MINOR 1
#define PSYCHIC_HTTP_VERSION_PATCH 0

#ifndef MAX_COOKIE_SIZE
#define MAX_COOKIE_SIZE 512
#endif

#ifndef FILE_CHUNK_SIZE
#define FILE_CHUNK_SIZE 4 * 1024  // Enhanced 4KB for better download performance
#endif

#ifndef STREAM_CHUNK_SIZE
#define STREAM_CHUNK_SIZE 1024
#endif

#ifndef MAX_UPLOAD_SIZE
#define MAX_UPLOAD_SIZE (2048 * 1024)  // 2MB
#endif

#ifndef MAX_REQUEST_BODY_SIZE
#define MAX_REQUEST_BODY_SIZE (16 * 1024)  // 16K
#endif

// WebSocket support can be disabled to save memory
#ifndef PSYCHIC_WEBSOCKET_ENABLED
#define PSYCHIC_WEBSOCKET_ENABLED 0  // Disabled by default for memory optimization
#endif

// ESP-IDF Configuration - Remove Arduino dependencies
// Configure ArduinoJson for non-Arduino use
#define ARDUINOJSON_ENABLE_ARDUINO_STRING 0
#define ARDUINOJSON_ENABLE_ARDUINO_STREAM 0
#define ARDUINOJSON_ENABLE_ARDUINO_PRINT 0
#define ARDUINOJSON_ENABLE_PROGMEM 0
#define ARDUINOJSON_ENABLE_STD_STRING 1
#define ARDUINOJSON_ENABLE_STD_STREAM 1

// === JSON backend selector ===
// 0 -> ArduinoJson (default), 1 -> cJSON
#ifndef PSYCHIC_USE_CJSON
#define PSYCHIC_USE_CJSON 0
#endif

// ESP-IDF includes
#include <esp_http_server.h>
#include <esp_log.h>
#include <esp_wifi.h>

// Standard C++ includes
#include <functional>
#include <list>
#include <map>
#include <string>

#include "esp_random.h"

// Custom utilities (ESP-IDF compatible)
#include "fs_utils.h"
#include "ip_utils.h"
#include "url_utils.h"
#include "wifi_utils.h"

#if PSYCHIC_USE_CJSON
#include <cJSON.h>
#else
// ArduinoJson (configured for ESP-IDF)
#include <ArduinoJson.h>
#endif

// File system aliases for compatibility
namespace fs {
using FS = PsychicHttp::ESPFileSystem;
using File = PsychicHttp::ESPFile;
}  // namespace fs

// IP Address alias for compatibility
using IPAddress = PsychicHttp::IPAddress;

// WiFi alias for compatibility
using WiFi = PsychicHttp::WiFiUtils;

// Forward declarations for ESP-IDF file system
struct _reent;
typedef struct _reent _reent_t;

enum HTTPAuthMethod { BASIC_AUTH, DIGEST_AUTH };

// URL encoding utilities are now in url_utils.h

class PsychicHttpServer;
class PsychicRequest;
class PsychicWebSocketRequest;
class PsychicClient;

// filter function definition
typedef std::function<bool(PsychicRequest *request)> PsychicRequestFilterFunction;

// client connect callback
typedef std::function<void(PsychicClient *client)> PsychicClientCallback;

// callback definitions
typedef std::function<esp_err_t(PsychicRequest *request)> PsychicHttpRequestCallback;
// NOTE: The JSON callback typedef is declared in PsychicJson.h now,
// because it depends on the selected JSON backend.
// Forward declaration for PsychicJsonRequestCallback (defined in PsychicJson.h)
#if PSYCHIC_USE_CJSON
typedef std::function<esp_err_t(PsychicRequest *, cJSON *)> PsychicJsonRequestCallback;
#else
typedef std::function<esp_err_t(PsychicRequest *, ArduinoJson::JsonVariant)> PsychicJsonRequestCallback;
#endif

struct HTTPHeader {
    char *field;
    char *value;
};

class DefaultHeaders {
    std::list<HTTPHeader> _headers;

   public:
    DefaultHeaders() {}

    void addHeader(const String &field, const String &value) { addHeader(field.c_str(), value.c_str()); }

    void addHeader(const char *field, const char *value) {
        HTTPHeader header;

        // these are just going to stick around forever.
        header.field = (char *)malloc(strlen(field) + 1);
        header.value = (char *)malloc(strlen(value) + 1);

        strlcpy(header.field, field, strlen(field) + 1);
        strlcpy(header.value, value, strlen(value) + 1);

        _headers.push_back(header);
    }

    const std::list<HTTPHeader> &getHeaders() { return _headers; }

    // delete the copy constructor, singleton class
    DefaultHeaders(DefaultHeaders const &) = delete;
    DefaultHeaders &operator=(DefaultHeaders const &) = delete;

    // single static class interface
    static DefaultHeaders &Instance() {
        static DefaultHeaders instance;
        return instance;
    }
};

#endif  // PsychicCore_h