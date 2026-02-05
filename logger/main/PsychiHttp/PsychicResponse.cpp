#include "PsychicResponse.h"

#include <algorithm>

#include "PsychicRequest.h"
#include "esp_random.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "http_status.h"
#include "sdkconfig.h"

// --- NEW: globals (configurable at runtime). Default = NO internal retries. ---
int PsychicResponse::s_chunkRetryAttempts = 0;  // 0 = no retries
uint32_t PsychicResponse::s_chunkRetryBaseDelayMs = 10;
uint32_t PsychicResponse::s_chunkRetryMaxDelayMs = 200;

void PsychicResponse::configureChunkRetry(int attempts, uint32_t baseDelayMs, uint32_t maxDelayMs) {
    if (attempts < 0) attempts = 0;
    s_chunkRetryAttempts = attempts;
    s_chunkRetryBaseDelayMs = baseDelayMs;
    s_chunkRetryMaxDelayMs = maxDelayMs;
}

PsychicResponse::PsychicResponse(PsychicRequest *request) : _request(request), _code(200), _status{}, _contentLength(0), _body(nullptr) {}

PsychicResponse::~PsychicResponse() {
    // clean up our header variables.  we have to do this on destruct since httpd_resp_send doesn't store copies
    for (HTTPHeader header : _headers) {
        free(header.field);
        free(header.value);
    }
    _headers.clear();
}

void PsychicResponse::addHeader(const char *field, const char *value) {
    if (field == NULL || value == NULL) {
        ESP_LOGE(PH_TAG, "addHeader: NULL parameter (field=%p, value=%p)", field, value);
        return;
    }

    // these get freed after send by the destructor
    HTTPHeader header;
    header.field = (char *)malloc(strlen(field) + 1);
    if (header.field == NULL) {
        ESP_LOGE(PH_TAG, "addHeader: malloc failed for field '%s'", field);
        return;
    }

    header.value = (char *)malloc(strlen(value) + 1);
    if (header.value == NULL) {
        ESP_LOGE(PH_TAG, "addHeader: malloc failed for value of field '%s'", field);
        free(header.field);
        return;
    }

    strlcpy(header.field, field, strlen(field) + 1);
    strlcpy(header.value, value, strlen(value) + 1);

    _headers.push_back(header);
}

void PsychicResponse::setCookie(const char *name, const char *value, unsigned long secondsFromNow, const char *extras) {
    time_t now = time(nullptr);

    String output;
    output = PsychicHttp::urlEncode(name) + "=" + PsychicHttp::urlEncode(value);

    // if current time isn't modern, default to using max age
    if (now < 1700000000) output += "; Max-Age=" + String(secondsFromNow);
    // otherwise, set an expiration date
    else {
        time_t expirationTimestamp = now + secondsFromNow;

        // Convert the expiration timestamp to a formatted string for the "expires" attribute
        struct tm *tmInfo = gmtime(&expirationTimestamp);
        char expires[30];
        strftime(expires, sizeof(expires), "%a, %d %b %Y %H:%M:%S GMT", tmInfo);
        output += "; Expires=" + String(expires);
    }

    // did we get any extras?
    if (extras && extras[0] != '\0') output += "; " + String(extras);

    // okay, add it in.
    addHeader("Set-Cookie", output.c_str());
}

void PsychicResponse::setCode(int code) { _code = code; }

void PsychicResponse::setContentType(const char *contentType) { httpd_resp_set_type(_request->request(), contentType); }

// WARNING: This method does NOT copy data - caller must guarantee lifetime
void PsychicResponse::setContent(const char *content) {
    _body = content;
    setContentLength(strlen(content));
}

// WARNING: This method does NOT copy data - caller must guarantee lifetime
void PsychicResponse::setContent(const uint8_t *content, size_t len) {
    _body = reinterpret_cast<const char *>(content);
    setContentLength(len);
}

const char *PsychicResponse::getContent() const { return _body; }

size_t PsychicResponse::getContentLength() const { return _contentLength; }

esp_err_t PsychicResponse::send() {
    // esp-idf makes you set the whole status.
    snprintf(_status, sizeof(_status), "%u %s", _code, http_status_reason(_code));
    httpd_resp_set_status(_request->request(), _status);

    // our headers too
    this->sendHeaders();

    // now send it off
    esp_err_t err = httpd_resp_send(_request->request(), getContent(), getContentLength());

    // did something happen?
    if (err != ESP_OK) ESP_LOGE(PH_TAG, "Send response failed (%s)", esp_err_to_name(err));

    return err;
}

void PsychicResponse::sendHeaders() {
    // get our global headers out of the way first
    for (HTTPHeader header : DefaultHeaders::Instance().getHeaders()) httpd_resp_set_hdr(_request->request(), header.field, header.value);

    // now do our individual headers
    for (HTTPHeader header : _headers) httpd_resp_set_hdr(this->_request->request(), header.field, header.value);

    // DO NOT RELEASE HEADERS HERE... released in the PsychicResponse destructor after they have been sent.
    // httpd_resp_set_hdr just passes on the pointer, but its needed after this call.
    // clean up our header variables after send
    // for (HTTPHeader header : _headers)
    // {
    //   free(header.field);
    //   free(header.value);
    // }
    // _headers.clear();
}

esp_err_t PsychicResponse::sendChunk(uint8_t *chunk, size_t chunksize) {
    // First, do ONE immediate attempt:
    esp_err_t err = httpd_resp_send_chunk(this->_request->request(), (char *)chunk, chunksize);
    if (err == ESP_OK) return ESP_OK;
    if (err != ESP_ERR_HTTPD_RESP_SEND || s_chunkRetryAttempts == 0) {
        // Fatal, or retry disabled -> return upward immediately (no internal backoff)
        return err;
    }

    // Otherwise: bounded short backoff/retries (configured at runtime)
    for (int attempt = 1; attempt <= s_chunkRetryAttempts; ++attempt) {
        uint32_t delay_ms = s_chunkRetryBaseDelayMs << (attempt - 1);
        if (delay_ms > s_chunkRetryMaxDelayMs) delay_ms = s_chunkRetryMaxDelayMs;
        delay_ms = std::min<uint32_t>(delay_ms + (esp_random() % 5U), s_chunkRetryMaxDelayMs);
        vTaskDelay(pdMS_TO_TICKS(delay_ms));

        err = httpd_resp_send_chunk(this->_request->request(), (char *)chunk, chunksize);
        if (err == ESP_OK) {
            if (attempt > 0) {
                ESP_LOGW(PH_TAG, "Chunk send recovered after %d retries (%zu bytes)", attempt, chunksize);
            }
            return ESP_OK;
        }
        if (err != ESP_ERR_HTTPD_RESP_SEND) return err;  // fatal -> bubble up
    }

    // still ESP_ERR_HTTPD_RESP_SEND after retries
    return err;
}

esp_err_t PsychicResponse::finishChunking() {
    /* Respond with an empty chunk to signal HTTP response completion */
    return httpd_resp_send_chunk(this->_request->request(), NULL, 0);
}