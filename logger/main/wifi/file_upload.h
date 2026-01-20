#ifndef DTRACK_WEB_SERVER_FILE_UPLOAD_H
#define DTRACK_WEB_SERVER_FILE_UPLOAD_H

#include <esp_err.h>
#include <esp_http_server.h>
#include <limits.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef FILE_CHUNK_SIZE
#define FILE_CHUNK_SIZE (2 * 1024)  // Reduced from 8KB to 2KB for memory-constrained systems
#endif

typedef struct FileUploadRequest FileUploadRequest;

typedef struct {
    size_t (*content_length)(void *ctx);
    size_t (*max_upload_size)(void *ctx);
    void (*load_params)(void *ctx);
    bool (*is_multipart)(void *ctx);
    const char *(*get_filename)(void *ctx);
    const char *(*get_header)(void *ctx, const char *name);
    httpd_req_t *(*get_httpd_req)(void *ctx);
    esp_err_t (*reply)(void *ctx, int code, const char *content_type, const char *content);
    esp_err_t (*reply_text)(void *ctx, const char *content);
    esp_err_t (*add_form_field)(void *ctx, const char *name, const char *value);
    esp_err_t (*add_file_field)(void *ctx, const char *name, const char *filename, size_t size);
} FileUploadRequestOps;

struct FileUploadRequest {
    void *ctx;
    const FileUploadRequestOps *ops;
    size_t current_file_size_hint;
    bool current_file_size_hint_valid;
};

static inline size_t file_upload_request_content_length(FileUploadRequest *request) {
    return (request && request->ops && request->ops->content_length) ? request->ops->content_length(request->ctx) : 0U;
}

static inline size_t file_upload_request_max_upload_size(FileUploadRequest *request) {
    return (request && request->ops && request->ops->max_upload_size) ? request->ops->max_upload_size(request->ctx) : SIZE_MAX;
}

static inline void file_upload_request_load_params(FileUploadRequest *request) {
    if (request && request->ops && request->ops->load_params) {
        request->ops->load_params(request->ctx);
    }
}

static inline bool file_upload_request_is_multipart(FileUploadRequest *request) {
    return (request && request->ops && request->ops->is_multipart) ? request->ops->is_multipart(request->ctx) : false;
}

static inline const char *file_upload_request_get_filename(FileUploadRequest *request) {
    return (request && request->ops && request->ops->get_filename) ? request->ops->get_filename(request->ctx) : "";
}

static inline const char *file_upload_request_get_header(FileUploadRequest *request, const char *name) {
    return (request && request->ops && request->ops->get_header) ? request->ops->get_header(request->ctx, name) : NULL;
}

static inline httpd_req_t *file_upload_request_get_httpd_req(FileUploadRequest *request) {
    return (request && request->ops && request->ops->get_httpd_req) ? request->ops->get_httpd_req(request->ctx) : NULL;
}

static inline esp_err_t file_upload_request_reply(FileUploadRequest *request, int code, const char *content_type, const char *content) {
    if (!request || !request->ops || !request->ops->reply) {
        return ESP_ERR_INVALID_STATE;
    }
    return request->ops->reply(request->ctx, code, content_type, content);
}

static inline esp_err_t file_upload_request_reply_text(FileUploadRequest *request, const char *content) {
    if (!request || !request->ops) {
        return ESP_ERR_INVALID_STATE;
    }
    if (request->ops->reply_text) {
        return request->ops->reply_text(request->ctx, content);
    }
    if (request->ops->reply) {
        return request->ops->reply(request->ctx, 200, "text/plain", content);
    }
    return ESP_ERR_INVALID_STATE;
}

static inline esp_err_t file_upload_request_add_form_field(FileUploadRequest *request, const char *name, const char *value) {
    if (request && request->ops && request->ops->add_form_field) {
        return request->ops->add_form_field(request->ctx, name, value);
    }
    return ESP_OK;
}

static inline esp_err_t file_upload_request_add_file_field(FileUploadRequest *request, const char *name, const char *filename, size_t size) {
    if (request && request->ops && request->ops->add_file_field) {
        return request->ops->add_file_field(request->ctx, name, filename, size);
    }
    return ESP_OK;
}

static inline void file_upload_request_clear_current_file_size_hint(FileUploadRequest *request) {
    if (!request) return;
    request->current_file_size_hint = 0U;
    request->current_file_size_hint_valid = false;
}

static inline void file_upload_request_set_current_file_size_hint(FileUploadRequest *request, size_t size, bool known) {
    if (!request) return;
    request->current_file_size_hint = size;
    request->current_file_size_hint_valid = known;
}

static inline bool file_upload_request_has_current_file_size_hint(const FileUploadRequest *request) {
    return request ? request->current_file_size_hint_valid : false;
}

static inline size_t file_upload_request_current_file_size_hint(const FileUploadRequest *request) {
    return (request && request->current_file_size_hint_valid) ? request->current_file_size_hint : 0U;
}

typedef esp_err_t (*FileUploadCallback)(FileUploadRequest *request, const char *filename, uint64_t index, const uint8_t *data, size_t len, bool final);

typedef esp_err_t (*FileUploadRequestCallback)(FileUploadRequest *request);

typedef enum {
    FILE_UPLOAD_EXPECT_BOUNDARY = 0,
    FILE_UPLOAD_PARSE_HEADERS,
    FILE_UPLOAD_WAIT_FOR_RETURN1,
    FILE_UPLOAD_EXPECT_FEED1,
    FILE_UPLOAD_EXPECT_DASH1,
    FILE_UPLOAD_EXPECT_DASH2,
    FILE_UPLOAD_BOUNDARY_OR_DATA,
    FILE_UPLOAD_DASH3_OR_RETURN2,
    FILE_UPLOAD_EXPECT_DASH3,
    FILE_UPLOAD_EXPECT_RETURN3,
    FILE_UPLOAD_EXPECT_FEED3,
    FILE_UPLOAD_EXPECT_FEED2,
    FILE_UPLOAD_PARSING_FINISHED,
    FILE_UPLOAD_PARSE_ERROR
} FileUploadMultipartState;

typedef struct {
    char *data;
    size_t length;
    size_t capacity;
} FileUploadMutableString;

typedef struct {
    FileUploadCallback upload_cb;
    FileUploadRequestCallback request_cb;
    FileUploadRequest *request;
    bool response_sent;
    size_t parsed_length;
    FileUploadMultipartState multipart_state;
    size_t boundary_position;
    size_t item_start_index;
    size_t item_size;
    uint8_t *item_buffer;
    size_t item_buffer_index;
    bool item_is_file;
    size_t item_declared_size;
    bool item_declared_size_known;
    esp_err_t last_error;
    FileUploadMutableString boundary;
    FileUploadMutableString temp;
    FileUploadMutableString item_name;
    FileUploadMutableString item_filename;
    FileUploadMutableString item_type;
    FileUploadMutableString item_value;
} FileUploadHandler;

void file_upload_handler_init(FileUploadHandler *handler);
void file_upload_handler_deinit(FileUploadHandler *handler);
bool file_upload_handler_can_handle(FileUploadHandler *handler, FileUploadRequest *request);
esp_err_t file_upload_handler_handle_request(FileUploadHandler *handler, FileUploadRequest *request);
FileUploadHandler *file_upload_handler_on_upload(FileUploadHandler *handler, FileUploadCallback cb);
FileUploadHandler *file_upload_handler_on_request(FileUploadHandler *handler, FileUploadRequestCallback cb);

#ifdef __cplusplus
}
#endif

#endif  // DTRACK_WEB_SERVER_FILE_UPLOAD_H
