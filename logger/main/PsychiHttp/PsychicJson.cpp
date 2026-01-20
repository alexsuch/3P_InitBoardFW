#include "PsychicJson.h"

#if PSYCHIC_USE_CJSON
// ===================== cJSON backend =====================
PsychicJsonResponse::PsychicJsonResponse(PsychicRequest *request, bool isArray) : PsychicResponse(request), _jsonBuffer(nullptr), _root(nullptr) {
    setContentType(JSON_MIMETYPE);
    _root = isArray ? cJSON_CreateArray() : cJSON_CreateObject();
    _jsonBuffer = _root;  // alias, kept for symmetry
}

PsychicJsonRoot PsychicJsonResponse::getRoot() { return _root; }

size_t PsychicJsonResponse::getLength() {
    // cJSON has no measure API; serialize to get length, then free.
    char *tmp = cJSON_PrintUnformatted(_root);
    if (!tmp) return 0;
    size_t len = strlen(tmp);
    free(tmp);
    return len;
}

esp_err_t PsychicJsonResponse::send() {
    if (!_root) return ESP_FAIL;

    esp_err_t err = ESP_OK;
    size_t length = getLength();
    size_t buffer_size = (length < JSON_BUFFER_SIZE) ? (length + 1) : JSON_BUFFER_SIZE;

    char *buffer = (char *)malloc(buffer_size);
    if (!buffer) {
        httpd_resp_send_err(this->_request->request(), HTTPD_500_INTERNAL_SERVER_ERROR, "Unable to allocate memory.");
        cJSON_Delete(_root);
        _root = nullptr;
        return ESP_FAIL;
    }

    // serialize whole JSON into a single contiguous buffer first (limitation of cJSON)
    char *full = cJSON_PrintUnformatted(_root);
    if (!full) {
        httpd_resp_send_err(this->_request->request(), HTTPD_500_INTERNAL_SERVER_ERROR, "JSON serialization failed.");
        free(buffer);
        cJSON_Delete(_root);
        _root = nullptr;
        return ESP_FAIL;
    }

    if (length < JSON_BUFFER_SIZE) {
        this->setContent((const uint8_t *)full, length);
        this->setContentType(JSON_MIMETYPE);
        err = PsychicResponse::send();
    } else {
        // chunk it out using our ChunkPrinter
        ChunkPrinter dest(this, (uint8_t *)buffer, buffer_size);
        this->sendHeaders();
        size_t sent = 0;
        while (sent < length) {
            size_t block = std::min(buffer_size, length - sent);
            dest.write((const uint8_t *)(full + sent), block);
            sent += block;
        }
        dest.flush();
        err = this->finishChunking();
    }

    free(full);
    free(buffer);
    cJSON_Delete(_root);
    _root = nullptr;
    return err;
}

void PsychicJsonHandler::onRequest(PsychicJsonRequestCallback fn) { _onRequest = fn; }

esp_err_t PsychicJsonHandler::handleRequest(PsychicRequest *request) {
    // process basic stuff
    PsychicWebHandler::handleRequest(request);
    if (!_onRequest) return request->reply(500);

    // parse body with cJSON
    const String &body = request->body();
    cJSON *doc = cJSON_ParseWithLength(body.c_str(), body.length());
    if (!doc) return request->reply(400);

    esp_err_t rc = _onRequest(request, doc);
    cJSON_Delete(doc);
    return rc;
}

#else
// ===================== ArduinoJson backend (as before) =====================

#ifdef ARDUINOJSON_6_COMPATIBILITY
PsychicJsonResponse::PsychicJsonResponse(PsychicRequest *request, bool isArray, size_t maxJsonBufferSize)
    : PsychicResponse(request), _jsonBuffer(maxJsonBufferSize) {
    setContentType(JSON_MIMETYPE);
    if (isArray)
        _root = _jsonBuffer.createNestedArray();
    else
        _root = _jsonBuffer.createNestedObject();
}
#else
PsychicJsonResponse::PsychicJsonResponse(PsychicRequest *request, bool isArray) : PsychicResponse(request) {
    setContentType(JSON_MIMETYPE);
    if (isArray)
        _root = _jsonBuffer.add<ArduinoJson::JsonArray>();
    else
        _root = _jsonBuffer.add<ArduinoJson::JsonObject>();
}
#endif

PsychicJsonRoot &PsychicJsonResponse::getRoot() { return _root; }

size_t PsychicJsonResponse::getLength() { return ArduinoJson::measureJson(_root); }

esp_err_t PsychicJsonResponse::send() {
    esp_err_t err = ESP_OK;
    size_t length = getLength();
    size_t buffer_size;
    char *buffer;

    // how big of a buffer do we want?
    if (length < JSON_BUFFER_SIZE)
        buffer_size = length + 1;
    else
        buffer_size = JSON_BUFFER_SIZE;

    buffer = (char *)malloc(buffer_size);
    if (buffer == NULL) {
        httpd_resp_send_err(this->_request->request(), HTTPD_500_INTERNAL_SERVER_ERROR, "Unable to allocate memory.");
        return ESP_FAIL;
    }

    // send it in one shot or no?
    if (length < JSON_BUFFER_SIZE) {
        ArduinoJson::serializeJson(_root, buffer, buffer_size);

        this->setContent((uint8_t *)buffer, length);
        this->setContentType(JSON_MIMETYPE);

        err = PsychicResponse::send();
    } else {
        // helper class that acts as a stream to print chunked responses
        ChunkPrinter dest(this, (uint8_t *)buffer, buffer_size);

        // keep our headers
        this->sendHeaders();

        ArduinoJson::serializeJson(_root, dest);

        // send the last bits
        dest.flush();

        // done with our chunked response too
        err = this->finishChunking();
    }

    // let the buffer go
    free(buffer);

    return err;
}

#ifdef ARDUINOJSON_6_COMPATIBILITY
PsychicJsonHandler::PsychicJsonHandler(size_t maxJsonBufferSize) : _onRequest(NULL), _maxJsonBufferSize(maxJsonBufferSize) {};

PsychicJsonHandler::PsychicJsonHandler(PsychicJsonRequestCallback onRequest, size_t maxJsonBufferSize)
    : _onRequest(onRequest), _maxJsonBufferSize(maxJsonBufferSize) {}
#else
PsychicJsonHandler::PsychicJsonHandler() : _onRequest(NULL) {};

PsychicJsonHandler::PsychicJsonHandler(PsychicJsonRequestCallback onRequest) : _onRequest(onRequest) {}
#endif

void PsychicJsonHandler::onRequest(PsychicJsonRequestCallback fn) { _onRequest = fn; }

esp_err_t PsychicJsonHandler::handleRequest(PsychicRequest *request) {
    // process basic stuff
    PsychicWebHandler::handleRequest(request);

    if (_onRequest) {
#ifdef ARDUINOJSON_6_COMPATIBILITY
        ArduinoJson::DynamicJsonDocument jsonBuffer(this->_maxJsonBufferSize);
        ArduinoJson::DeserializationError error = deserializeJson(jsonBuffer, request->body());
        if (error) return request->reply(400);

        ArduinoJson::JsonVariant json = jsonBuffer.as<ArduinoJson::JsonVariant>();
#else
        ArduinoJson::JsonDocument jsonBuffer;
        ArduinoJson::DeserializationError error = deserializeJson(jsonBuffer, request->body());
        if (error) return request->reply(400);

        ArduinoJson::JsonVariant json = jsonBuffer.as<ArduinoJson::JsonVariant>();
#endif

        return _onRequest(request, json);
    } else
        return request->reply(500);
}

#endif  // PSYCHIC_USE_CJSON