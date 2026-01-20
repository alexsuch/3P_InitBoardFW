// PsychicJson.h
/*
  Async Response to use with ArduinoJson and AsyncWebServer
  Written by Andrew Melvin (SticilFace) with help from me-no-dev and BBlanchon.
  Ported to PsychicHttp by Zach Hoeken

*/
#ifndef PSYCHIC_JSON_H_
#define PSYCHIC_JSON_H_

#include "PsychicCore.h"
#if PSYCHIC_USE_CJSON
// cJSON backend
#include "cJSON.h"
#else
// ArduinoJson backend
#include <ArduinoJson.h>
#endif

#include "ChunkPrinter.h"
#include "PsychicRequest.h"
#include "PsychicWebHandler.h"

#if ARDUINOJSON_VERSION_MAJOR == 6
#define ARDUINOJSON_6_COMPATIBILITY
#ifndef DYNAMIC_JSON_DOCUMENT_SIZE
#define DYNAMIC_JSON_DOCUMENT_SIZE 4096
#endif
#endif

#ifndef JSON_BUFFER_SIZE
#define JSON_BUFFER_SIZE 4 * 1024
#endif

constexpr const char *JSON_MIMETYPE = "application/json";

// --------- Backend-dependent types ---------
#if PSYCHIC_USE_CJSON
using PsychicJsonRoot = cJSON *;  // root node
using PsychicJsonDoc = cJSON *;   // same for cJSON
using PsychicJsonError = int;     // 0 = OK, non-zero = parse error
                                  // PsychicJsonRequestCallback is now declared in PsychicCore.h
#else
#ifdef ARDUINOJSON_VERSION_MAJOR
#if ARDUINOJSON_VERSION_MAJOR == 6
#define ARDUINOJSON_6_COMPATIBILITY
#ifndef DYNAMIC_JSON_DOCUMENT_SIZE
#define DYNAMIC_JSON_DOCUMENT_SIZE 4096
#endif
#endif
#endif
using PsychicJsonRoot = ArduinoJson::JsonVariant;  // explicit namespace for IDF
#ifdef ARDUINOJSON_5_COMPATIBILITY
using PsychicJsonDoc = ArduinoJson::DynamicJsonBuffer;
#elif ARDUINOJSON_VERSION_MAJOR == 6
using PsychicJsonDoc = ArduinoJson::DynamicJsonDocument;
#else
using PsychicJsonDoc = ArduinoJson::JsonDocument;
#endif
using PsychicJsonError = ArduinoJson::DeserializationError;
// PsychicJsonRequestCallback is now declared in PsychicCore.h
#endif

/*
 * Json Response
 * */

class PsychicJsonResponse : public PsychicResponse {
   protected:
    // Backend-specific holders
    PsychicJsonDoc _jsonBuffer;
    PsychicJsonRoot _root;
    size_t _contentLength;

   public:
#if PSYCHIC_USE_CJSON
    // cJSON: no external buffer size needed
    PsychicJsonResponse(PsychicRequest *request, bool isArray = false);
#else
#ifdef ARDUINOJSON_5_COMPATIBILITY
    PsychicJsonResponse(PsychicRequest *request, bool isArray = false);
#elif ARDUINOJSON_VERSION_MAJOR == 6
    PsychicJsonResponse(PsychicRequest *request, bool isArray = false, size_t maxJsonBufferSize = DYNAMIC_JSON_DOCUMENT_SIZE);
#else
    PsychicJsonResponse(PsychicRequest *request, bool isArray = false);
#endif
#endif

    ~PsychicJsonResponse() {}

#if PSYCHIC_USE_CJSON
    PsychicJsonRoot getRoot();
#else
    ArduinoJson::JsonVariant &getRoot();
#endif
    size_t getLength();

    virtual esp_err_t send() override;
};

class PsychicJsonHandler : public PsychicWebHandler {
   protected:
    PsychicJsonRequestCallback _onRequest;
#if !PSYCHIC_USE_CJSON && (ARDUINOJSON_VERSION_MAJOR == 6)
    const size_t _maxJsonBufferSize = DYNAMIC_JSON_DOCUMENT_SIZE;
#endif

   public:
#if PSYCHIC_USE_CJSON
    PsychicJsonHandler() : _onRequest(NULL) {}
    PsychicJsonHandler(PsychicJsonRequestCallback onRequest) : _onRequest(onRequest) {}
#else
#ifdef ARDUINOJSON_5_COMPATIBILITY
    PsychicJsonHandler();
    PsychicJsonHandler(PsychicJsonRequestCallback onRequest);
#elif ARDUINOJSON_VERSION_MAJOR == 6
    PsychicJsonHandler(size_t maxJsonBufferSize = DYNAMIC_JSON_DOCUMENT_SIZE);
    PsychicJsonHandler(PsychicJsonRequestCallback onRequest, size_t maxJsonBufferSize = DYNAMIC_JSON_DOCUMENT_SIZE);
#else
    PsychicJsonHandler();
    PsychicJsonHandler(PsychicJsonRequestCallback onRequest);
#endif
#endif

    void onRequest(PsychicJsonRequestCallback fn);
    virtual esp_err_t handleRequest(PsychicRequest *request) override;
};

#endif