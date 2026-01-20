#ifndef PsychicResponse_h
#define PsychicResponse_h

#include <stdint.h>

#include "PsychicCore.h"
#include "time.h"

class PsychicRequest;

class PsychicResponse {
   protected:
    PsychicRequest *_request;

    int _code;
    char _status[60];
    std::list<HTTPHeader> _headers;
    int64_t _contentLength;
    const char *_body;

   public:
    PsychicResponse(PsychicRequest *request);
    virtual ~PsychicResponse();

    void setCode(int code);

    void setContentType(const char *contentType);
    void setContentLength(int64_t contentLength) { _contentLength = contentLength; }
    // (fixed) no bogus overload here
    void addHeader(const char *field, const char *value);

    void setCookie(const char *key, const char *value, unsigned long max_age = 60 * 60 * 24 * 30, const char *extras = "");

    // WARNING: setContent does NOT copy data - caller must guarantee lifetime
    void setContent(const char *content);
    // WARNING: setContent does NOT copy data - caller must guarantee lifetime
    void setContent(const uint8_t *content, size_t len);

    const char *getContent() const;
    size_t getContentLength() const;

    virtual esp_err_t send();
    void sendHeaders();
    esp_err_t sendChunk(uint8_t *chunk, size_t chunksize);
    esp_err_t finishChunking();

    // --- NEW: runtime configuration for sendChunk retry policy ---
    // attempts = number of retries after the first immediate try (0 = no retries).
    // baseDelayMs/maxDelayMs control the backoff if attempts > 0.
    static void configureChunkRetry(int attempts, uint32_t baseDelayMs = 10, uint32_t maxDelayMs = 200);

   private:
    static int s_chunkRetryAttempts;
    static uint32_t s_chunkRetryBaseDelayMs;
    static uint32_t s_chunkRetryMaxDelayMs;
};

#endif  // PsychicResponse_h