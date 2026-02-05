#include "file_upload.h"

#include <ctype.h>
#include <errno.h>
#include <esp_log.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <target.h>

#define TAG "FileUpload"

// Limit for a single non-file form field value (defense-in-depth)
#ifndef MAX_FORM_FIELD_VALUE_BYTES
#define MAX_FORM_FIELD_VALUE_BYTES (32 * 1024)
#endif

static void mutable_string_reset(FileUploadMutableString *str) {
    if (!str) return;
    if (str->data) {
        str->data[0] = '\0';
    }
    str->length = 0;
}

static void mutable_string_free(FileUploadMutableString *str) {
    if (!str) return;
    free(str->data);
    str->data = NULL;
    str->length = 0;
    str->capacity = 0;
}

static bool mutable_string_reserve(FileUploadMutableString *str, size_t capacity) {
    if (!str) return false;
    if (capacity <= str->capacity) return true;

    char *new_data = (char *)realloc(str->data, capacity);
    if (!new_data) {
        return false;
    }

    str->data = new_data;
    str->capacity = capacity;
    return true;
}

static bool mutable_string_assign(FileUploadMutableString *str, const char *data, size_t len) {
    if (!str) return false;
    if (!mutable_string_reserve(str, len + 1)) return false;

    if (len > 0 && data) {
        memcpy(str->data, data, len);
    }

    str->data[len] = '\0';
    str->length = len;
    return true;
}

static bool mutable_string_append_char(FileUploadMutableString *str, char ch) {
    if (!str) return false;
    if (!mutable_string_reserve(str, str->length + 2)) return false;

    str->data[str->length++] = ch;
    str->data[str->length] = '\0';
    return true;
}

static const char *mutable_string_c_str(const FileUploadMutableString *str) { return (str && str->data) ? str->data : ""; }

static bool sanitize_filename_inplace(FileUploadMutableString *s) {
    if (!s || !s->data) return false;
    const char *p = s->data;
    if (strstr(p, "..") || strchr(p, '/') || strchr(p, '\\')) return false;
    for (const unsigned char *q = (const unsigned char *)p; *q; ++q) {
        if (*q < 0x20 || *q == 0x7F) return false;  // control chars
    }
    return true;
}

static void handler_reset_dynamic_state(FileUploadHandler *handler) {
    handler->parsed_length = 0;
    handler->multipart_state = FILE_UPLOAD_EXPECT_BOUNDARY;
    handler->boundary_position = 0;
    handler->item_start_index = 0;
    handler->item_size = 0;
    handler->item_buffer_index = 0;
    handler->item_is_file = false;
    handler->item_declared_size = 0;
    handler->item_declared_size_known = false;
    handler->last_error = ESP_OK;
    handler->response_sent = false;
    mutable_string_reset(&handler->temp);
    mutable_string_reset(&handler->item_name);
    mutable_string_reset(&handler->item_filename);
    mutable_string_reset(&handler->item_type);
    mutable_string_reset(&handler->item_value);
    if (handler->item_buffer) {
        free(handler->item_buffer);
        handler->item_buffer = NULL;
    }
}

void file_upload_handler_init(FileUploadHandler *handler) {
    if (!handler) return;
    memset(handler, 0, sizeof(*handler));
    handler->multipart_state = FILE_UPLOAD_EXPECT_BOUNDARY;
    handler->last_error = ESP_OK;
}

void file_upload_handler_deinit(FileUploadHandler *handler) {
    if (!handler) return;
    mutable_string_free(&handler->boundary);
    mutable_string_free(&handler->temp);
    mutable_string_free(&handler->item_name);
    mutable_string_free(&handler->item_filename);
    mutable_string_free(&handler->item_type);
    mutable_string_free(&handler->item_value);
    free(handler->item_buffer);
    handler->item_buffer = NULL;
}

FileUploadHandler *file_upload_handler_on_upload(FileUploadHandler *handler, FileUploadCallback cb) {
    if (!handler) return NULL;
    handler->upload_cb = cb;
    return handler;
}

FileUploadHandler *file_upload_handler_on_request(FileUploadHandler *handler, FileUploadRequestCallback cb) {
    if (!handler) return NULL;
    handler->request_cb = cb;
    return handler;
}

bool file_upload_handler_can_handle(FileUploadHandler *handler, FileUploadRequest *request) {
    (void)handler;
    (void)request;
    return true;
}

static esp_err_t send_size_error(FileUploadHandler *handler, FileUploadRequest *request, size_t max_size) {
    httpd_req_t *req = file_upload_request_get_httpd_req(request);
    char error[80];
    snprintf(error, sizeof(error), "File size must be less than %lu bytes!", (unsigned long)max_size);
    if (req) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, error);
    } else {
        file_upload_request_reply(request, 400, "text/plain", error);
    }
    handler->response_sent = true;
    handler->last_error = ESP_ERR_INVALID_SIZE;
    return ESP_FAIL;
}

static esp_err_t handle_upload_callback(FileUploadHandler *handler, const char *filename, uint64_t index, const uint8_t *data, size_t len, bool final) {
    if (!handler->upload_cb) {
        ESP_LOGE(TAG, "No upload callback specified");
        return ESP_FAIL;
    }
    return handler->upload_cb(handler->request, filename, index, data, len, final);
}

static esp_err_t basic_upload_handler(FileUploadHandler *handler, FileUploadRequest *request) {
    httpd_req_t *req = file_upload_request_get_httpd_req(request);
    if (!req) {
        ESP_LOGE(TAG, "Missing httpd request context");
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t *buf = (uint8_t *)malloc(FILE_CHUNK_SIZE);
    if (!buf) {
        ESP_LOGE(TAG, "Failed to allocate upload buffer");
        return ESP_ERR_NO_MEM;
    }

    size_t remaining = file_upload_request_content_length(request);
    bool length_known = (remaining > 0);
    file_upload_request_set_current_file_size_hint(request, remaining, length_known);
    size_t max_size = file_upload_request_max_upload_size(request);
    size_t total_received = 0;
    const char *filename = file_upload_request_get_filename(request);
    uint64_t index = 0;
    esp_err_t err = ESP_OK;

    bool final_sent = false;
    while (!length_known || remaining > 0) {
        size_t to_read = length_known ? (remaining > FILE_CHUNK_SIZE ? FILE_CHUNK_SIZE : remaining) : FILE_CHUNK_SIZE;
        if (!length_known && to_read == 0) {
            to_read = FILE_CHUNK_SIZE;
        }

        int received = httpd_req_recv(req, (char *)buf, to_read);
        if (received < 0) {
            if (received == HTTPD_SOCK_ERR_TIMEOUT) {
                continue;
            }
            ESP_LOGE(TAG, "Socket error during upload (%d)", received);
            err = ESP_FAIL;
            break;
        }

        if (received == 0) {
            if (length_known && remaining != 0) {
                ESP_LOGE(TAG, "Upload ended before receiving declared length");
                err = ESP_ERR_INVALID_RESPONSE;
            }
            break;
        }

        size_t chunk = (size_t)received;
        bool final = false;
        if (length_known) {
            if (chunk > remaining) {
                ESP_LOGE(TAG, "Received more data than expected");
                err = ESP_ERR_INVALID_RESPONSE;
                break;
            }
            remaining -= chunk;
            final = (remaining == 0);
        }

        if (max_size != SIZE_MAX) {
            size_t remaining_limit = (total_received < max_size) ? (max_size - total_received) : 0;
            if (chunk > remaining_limit) {
                ESP_LOGE(TAG, "Upload exceeded maximum size limit (%zu bytes)", max_size);
                err = send_size_error(handler, request, max_size);
                break;
            }
            total_received += chunk;
        }

        err = handle_upload_callback(handler, filename, index, buf, chunk, final);
        if (err != ESP_OK) {
            break;
        }
        if (final) {
            final_sent = true;
        }

        index += (uint64_t)chunk;
    }

    if (err == ESP_OK && !final_sent) {
        err = handle_upload_callback(handler, filename, index, buf, 0, true);
        if (err == ESP_OK) {
            final_sent = true;
        }
    }

    free(buf);
    file_upload_request_clear_current_file_size_hint(request);
    return err;
}

static bool ensure_item_buffer(FileUploadHandler *handler) {
    if (handler->item_buffer) return true;
    handler->item_buffer = (uint8_t *)malloc(FILE_CHUNK_SIZE);
    if (!handler->item_buffer) {
        ESP_LOGE(TAG, "Multipart: Failed to allocate buffer");
        handler->last_error = ESP_ERR_NO_MEM;
        handler->multipart_state = FILE_UPLOAD_PARSE_ERROR;
        return false;
    }
    handler->item_buffer_index = 0;
    return true;
}

static bool handle_upload_byte(FileUploadHandler *handler, uint8_t data, bool last) {
    if (!ensure_item_buffer(handler)) {
        return false;
    }

    handler->item_buffer[handler->item_buffer_index++] = data;
    if (last || handler->item_buffer_index == FILE_CHUNK_SIZE) {
        if (handler->upload_cb) {
            size_t chunk_len = handler->item_buffer_index;
            uint64_t chunk_index = handler->item_size - chunk_len;
            esp_err_t err =
                handler->upload_cb(handler->request, mutable_string_c_str(&handler->item_filename), chunk_index, handler->item_buffer, chunk_len, last);
            if (err != ESP_OK) {
                handler->last_error = err;
                handler->multipart_state = FILE_UPLOAD_PARSE_ERROR;
                return false;
            }
        }
        handler->item_buffer_index = 0;
    }
    return true;
}

static bool item_write_byte(FileUploadHandler *handler, uint8_t data, bool last) {
    handler->item_size++;
    if (handler->item_declared_size_known && handler->item_size > handler->item_declared_size) {
        ESP_LOGE(TAG, "Multipart: Received more data than declared for part '%s'", mutable_string_c_str(&handler->item_name));
        handler->last_error = ESP_ERR_INVALID_RESPONSE;
        handler->multipart_state = FILE_UPLOAD_PARSE_ERROR;
        return false;
    }
    if (handler->item_is_file) {
        return handle_upload_byte(handler, data, last);
    }
    if (handler->item_value.length >= MAX_FORM_FIELD_VALUE_BYTES) {
        ESP_LOGE(TAG, "Multipart: form field '%s' exceeds limit", mutable_string_c_str(&handler->item_name));
        handler->last_error = ESP_ERR_INVALID_SIZE;
        handler->multipart_state = FILE_UPLOAD_PARSE_ERROR;
        return false;
    }
    if (!mutable_string_append_char(&handler->item_value, (char)data)) {
        handler->last_error = ESP_ERR_NO_MEM;
        handler->multipart_state = FILE_UPLOAD_PARSE_ERROR;
        return false;
    }
    return true;
}

static void trim_token(const char **start, const char **end) {
    while (*start < *end && (isspace((int)**start) || **start == '"')) {
        (*start)++;
    }
    while (*end > *start && (isspace((int)*((*end) - 1)) || *((*end) - 1) == '"')) {
        (*end)--;
    }
}

static bool parse_content_disposition(FileUploadHandler *handler, const char *value) {
    const char *cursor = strchr(value, ';');
    if (!cursor) {
        return true;
    }

    cursor++;
    while (*cursor) {
        while (*cursor && isspace((int)*cursor)) cursor++;
        if (!*cursor) break;

        const char *next = strchr(cursor, ';');
        const char *end = next ? next : cursor + strlen(cursor);
        const char *eq = memchr(cursor, '=', end - cursor);
        if (eq) {
            const char *name_start = cursor;
            const char *name_end = eq;
            trim_token(&name_start, &name_end);

            const char *value_start = eq + 1;
            const char *value_end = end;
            trim_token(&value_start, &value_end);

            size_t name_len = (size_t)(name_end - name_start);
            size_t value_len = (size_t)(value_end - value_start);

            if (value_len > 0) {
                if (name_len == 4 && strncasecmp(name_start, "name", 4) == 0) {
                    if (!mutable_string_assign(&handler->item_name, value_start, value_len)) {
                        handler->last_error = ESP_ERR_NO_MEM;
                        handler->multipart_state = FILE_UPLOAD_PARSE_ERROR;
                        return false;
                    }
                } else if (name_len == 8 && strncasecmp(name_start, "filename", 8) == 0) {
                    if (!mutable_string_assign(&handler->item_filename, value_start, value_len)) {
                        handler->last_error = ESP_ERR_NO_MEM;
                        handler->multipart_state = FILE_UPLOAD_PARSE_ERROR;
                        return false;
                    }
                    if (!sanitize_filename_inplace(&handler->item_filename)) {
                        ESP_LOGE(TAG, "Multipart: unsafe filename");
                        handler->last_error = ESP_ERR_INVALID_ARG;
                        handler->multipart_state = FILE_UPLOAD_PARSE_ERROR;
                        return false;
                    }
                    handler->item_is_file = true;
                }
            }
        }

        if (!next) break;
        cursor = next + 1;
    }

    return true;
}

static bool parse_header_line(FileUploadHandler *handler) {
    const char *line = mutable_string_c_str(&handler->temp);
    size_t len = handler->temp.length;
    if (len == 0) {
        return true;
    }

    if (len >= 12 && strncasecmp(line, "Content-Type", 12) == 0) {
        const char *value = strchr(line, ':');
        if (!value) return true;
        value++;
        while (*value && isspace((int)*value)) value++;
        size_t value_len = strlen(value);
        while (value_len > 0 && isspace((int)value[value_len - 1])) value_len--;
        if (!mutable_string_assign(&handler->item_type, value, value_len)) {
            handler->last_error = ESP_ERR_NO_MEM;
            handler->multipart_state = FILE_UPLOAD_PARSE_ERROR;
            return false;
        }
        handler->item_is_file = true;
        return true;
    }

    if (len >= 14 && strncasecmp(line, "Content-Length", 14) == 0) {
        const char *value = strchr(line, ':');
        if (!value) return true;
        value++;
        while (*value && isspace((int)*value)) value++;
        errno = 0;
        char *endptr = NULL;
        unsigned long long parsed = strtoull(value, &endptr, 10);
        if (endptr != value) {
            if (errno == ERANGE || parsed > (unsigned long long)SIZE_MAX) {
                handler->item_declared_size = SIZE_MAX;
            } else {
                handler->item_declared_size = (size_t)parsed;
            }
            handler->item_declared_size_known = true;
        }
        return true;
    }

    if (len >= 19 && strncasecmp(line, "Content-Disposition", 19) == 0) {
        const char *value = strchr(line, ':');
        if (!value) return true;
        value++;
        while (*value && isspace((int)*value)) value++;
        return parse_content_disposition(handler, value);
    }

    return true;
}

static bool finish_file_item(FileUploadHandler *handler) {
    if (!handler->item_is_file) return true;
    if (handler->upload_cb && (handler->item_size > 0 || handler->item_buffer_index == 0)) {
        size_t chunk_len = handler->item_buffer_index;
        uint64_t chunk_index = handler->item_size;
        if (chunk_len > 0) {
            chunk_index -= chunk_len;
        }
        esp_err_t err = handler->upload_cb(handler->request, mutable_string_c_str(&handler->item_filename), chunk_index, handler->item_buffer, chunk_len, true);
        if (err != ESP_OK) {
            handler->last_error = err;
            handler->multipart_state = FILE_UPLOAD_PARSE_ERROR;
            return false;
        }
    }

    handler->item_buffer_index = 0;
    if (handler->request) {
        file_upload_request_clear_current_file_size_hint(handler->request);
    }
    handler->item_declared_size = 0;
    handler->item_declared_size_known = false;
    file_upload_request_add_file_field(handler->request, mutable_string_c_str(&handler->item_name), mutable_string_c_str(&handler->item_filename),
                                       handler->item_size);
    free(handler->item_buffer);
    handler->item_buffer = NULL;
    return true;
}

static bool parse_multipart_post_byte(FileUploadHandler *handler, uint8_t data, bool last) {
    if (handler->multipart_state == FILE_UPLOAD_PARSE_ERROR) {
        if (handler->item_buffer) {
            free(handler->item_buffer);
            handler->item_buffer = NULL;
        }
        return false;
    }

    if (handler->parsed_length == 0) {
        handler->multipart_state = FILE_UPLOAD_EXPECT_BOUNDARY;
        mutable_string_reset(&handler->temp);
        mutable_string_reset(&handler->item_name);
        mutable_string_reset(&handler->item_filename);
        mutable_string_reset(&handler->item_type);
    }

    switch (handler->multipart_state) {
        case FILE_UPLOAD_WAIT_FOR_RETURN1:
            if (data != '\r') {
                if (!item_write_byte(handler, data, last)) return false;
            } else {
                handler->multipart_state = FILE_UPLOAD_EXPECT_FEED1;
            }
            break;

        case FILE_UPLOAD_EXPECT_BOUNDARY: {
            size_t boundary_len = handler->boundary.length;
            if (handler->parsed_length < 2) {
                if (data != '-') {
                    ESP_LOGE(TAG, "Multipart: No boundary");
                    handler->multipart_state = FILE_UPLOAD_PARSE_ERROR;
                    handler->last_error = ESP_ERR_INVALID_STATE;
                    return false;
                }
            } else {
                size_t relative = handler->parsed_length - 2;
                if (relative < boundary_len && handler->boundary.data && handler->boundary.data[relative] != data) {
                    ESP_LOGE(TAG, "Multipart: Multipart malformed");
                    handler->multipart_state = FILE_UPLOAD_PARSE_ERROR;
                    handler->last_error = ESP_ERR_INVALID_RESPONSE;
                    return false;
                }
                if (relative == boundary_len && data != '\r') {
                    ESP_LOGE(TAG, "Multipart: Missing carriage return");
                    handler->multipart_state = FILE_UPLOAD_PARSE_ERROR;
                    handler->last_error = ESP_ERR_INVALID_RESPONSE;
                    return false;
                }
                if (relative == boundary_len + 1 && data != '\n') {
                    ESP_LOGE(TAG, "Multipart: Missing newline");
                    handler->multipart_state = FILE_UPLOAD_PARSE_ERROR;
                    handler->last_error = ESP_ERR_INVALID_RESPONSE;
                    return false;
                }
                if (relative == boundary_len + 1 && data == '\n') {
                    handler->multipart_state = FILE_UPLOAD_PARSE_HEADERS;
                    handler->item_is_file = false;
                    handler->item_declared_size = 0;
                    handler->item_declared_size_known = false;
                }
            }
            break;
        }

        case FILE_UPLOAD_PARSE_HEADERS:
            if ((char)data != '\r' && (char)data != '\n') {
                if (!mutable_string_append_char(&handler->temp, (char)data)) {
                    handler->last_error = ESP_ERR_NO_MEM;
                    handler->multipart_state = FILE_UPLOAD_PARSE_ERROR;
                    return false;
                }
            }
            if ((char)data == '\n') {
                if (handler->temp.length) {
                    if (!parse_header_line(handler)) return false;
                    mutable_string_reset(&handler->temp);
                } else {
                    handler->multipart_state = FILE_UPLOAD_WAIT_FOR_RETURN1;
                    handler->item_size = 0;
                    handler->item_start_index = handler->parsed_length;
                    mutable_string_reset(&handler->item_value);
                    if (handler->item_is_file) {
                        if (handler->item_buffer) {
                            free(handler->item_buffer);
                            handler->item_buffer = NULL;
                        }
                        if (!ensure_item_buffer(handler)) return false;
                        handler->item_buffer_index = 0;
                        if (handler->request) {
                            file_upload_request_set_current_file_size_hint(handler->request, handler->item_declared_size, handler->item_declared_size_known);
                        }
                    } else if (handler->request) {
                        file_upload_request_clear_current_file_size_hint(handler->request);
                    }
                }
            }
            break;

        case FILE_UPLOAD_EXPECT_FEED1:
            if (data != '\n') {
                handler->multipart_state = FILE_UPLOAD_WAIT_FOR_RETURN1;
                if (!item_write_byte(handler, '\r', last)) return false;
                return parse_multipart_post_byte(handler, data, last);
            } else {
                handler->multipart_state = FILE_UPLOAD_EXPECT_DASH1;
            }
            break;

        case FILE_UPLOAD_EXPECT_DASH1:
            if (data != '-') {
                handler->multipart_state = FILE_UPLOAD_WAIT_FOR_RETURN1;
                if (!item_write_byte(handler, '\r', last)) return false;
                if (!item_write_byte(handler, '\n', last)) return false;
                return parse_multipart_post_byte(handler, data, last);
            } else {
                handler->multipart_state = FILE_UPLOAD_EXPECT_DASH2;
            }
            break;

        case FILE_UPLOAD_EXPECT_DASH2:
            if (data != '-') {
                handler->multipart_state = FILE_UPLOAD_WAIT_FOR_RETURN1;
                if (!item_write_byte(handler, '\r', last)) return false;
                if (!item_write_byte(handler, '\n', last)) return false;
                if (!item_write_byte(handler, '-', last)) return false;
                return parse_multipart_post_byte(handler, data, last);
            } else {
                handler->multipart_state = FILE_UPLOAD_BOUNDARY_OR_DATA;
                handler->boundary_position = 0;
            }
            break;

        case FILE_UPLOAD_BOUNDARY_OR_DATA: {
            size_t boundary_len = handler->boundary.length;
            if (handler->boundary_position < boundary_len && handler->boundary.data && handler->boundary.data[handler->boundary_position] != data) {
                handler->multipart_state = FILE_UPLOAD_WAIT_FOR_RETURN1;
                if (!item_write_byte(handler, '\r', last)) return false;
                if (!item_write_byte(handler, '\n', last)) return false;
                if (!item_write_byte(handler, '-', last)) return false;
                if (!item_write_byte(handler, '-', last)) return false;
                for (size_t i = 0; i < handler->boundary_position; ++i) {
                    if (!item_write_byte(handler, (uint8_t)handler->boundary.data[i], last)) return false;
                }
                return parse_multipart_post_byte(handler, data, last);
            }
            if (handler->boundary_position + 1 == boundary_len) {
                handler->multipart_state = FILE_UPLOAD_DASH3_OR_RETURN2;
                if (!handler->item_is_file) {
                    handler->item_declared_size = 0;
                    handler->item_declared_size_known = false;
                    file_upload_request_add_form_field(handler->request, mutable_string_c_str(&handler->item_name), mutable_string_c_str(&handler->item_value));
                } else {
                    if (!finish_file_item(handler)) return false;
                }
            } else {
                handler->boundary_position++;
            }
            break;
        }

        case FILE_UPLOAD_DASH3_OR_RETURN2: {
            if (data == '\r') {
                handler->multipart_state = FILE_UPLOAD_EXPECT_FEED2;
            } else if (data == '-') {
                size_t total_length = file_upload_request_content_length(handler->request);
                if (total_length > 0) {
                    if (handler->parsed_length > total_length || (total_length - handler->parsed_length) != 4) {
                        ESP_LOGE(TAG, "Multipart: Unexpected end of data");
                        handler->multipart_state = FILE_UPLOAD_PARSE_ERROR;
                        handler->last_error = ESP_ERR_INVALID_RESPONSE;
                        return false;
                    }
                }
                handler->multipart_state = FILE_UPLOAD_EXPECT_DASH3;
            } else {
                handler->multipart_state = FILE_UPLOAD_WAIT_FOR_RETURN1;
                if (!item_write_byte(handler, '\r', last)) return false;
                if (!item_write_byte(handler, '\n', last)) return false;
                if (!item_write_byte(handler, '-', last)) return false;
                if (!item_write_byte(handler, '-', last)) return false;
                for (size_t i = 0; i < handler->boundary.length; ++i) {
                    if (!item_write_byte(handler, (uint8_t)handler->boundary.data[i], last)) return false;
                }
                if (!item_write_byte(handler, '\r', last)) return false;
                return parse_multipart_post_byte(handler, data, last);
            }
            break;
        }

        case FILE_UPLOAD_EXPECT_DASH3:
            if (data == '-') {
                handler->multipart_state = FILE_UPLOAD_EXPECT_RETURN3;
            } else {
                ESP_LOGE(TAG, "Multipart: Invalid closing boundary");
                handler->multipart_state = FILE_UPLOAD_PARSE_ERROR;
                handler->last_error = ESP_ERR_INVALID_RESPONSE;
                return false;
            }
            break;

        case FILE_UPLOAD_EXPECT_RETURN3:
            if (data == '\r') {
                handler->multipart_state = FILE_UPLOAD_EXPECT_FEED3;
            } else {
                ESP_LOGE(TAG, "Multipart: Invalid closing boundary");
                handler->multipart_state = FILE_UPLOAD_PARSE_ERROR;
                handler->last_error = ESP_ERR_INVALID_RESPONSE;
                return false;
            }
            break;

        case FILE_UPLOAD_EXPECT_FEED3:
            if (data == '\n') {
                handler->multipart_state = FILE_UPLOAD_PARSING_FINISHED;
            } else {
                ESP_LOGE(TAG, "Multipart: Invalid closing boundary");
                handler->multipart_state = FILE_UPLOAD_PARSE_ERROR;
                handler->last_error = ESP_ERR_INVALID_RESPONSE;
                return false;
            }
            break;

        case FILE_UPLOAD_EXPECT_FEED2:
            if (data == '\n') {
                handler->multipart_state = FILE_UPLOAD_PARSE_HEADERS;
                handler->item_is_file = false;
                mutable_string_reset(&handler->item_name);
                mutable_string_reset(&handler->item_filename);
                mutable_string_reset(&handler->item_type);
            } else {
                handler->multipart_state = FILE_UPLOAD_WAIT_FOR_RETURN1;
                if (!item_write_byte(handler, '\r', last)) return false;
                if (!item_write_byte(handler, '\n', last)) return false;
                if (!item_write_byte(handler, '-', last)) return false;
                if (!item_write_byte(handler, '-', last)) return false;
                for (size_t i = 0; i < handler->boundary.length; ++i) {
                    if (!item_write_byte(handler, (uint8_t)handler->boundary.data[i], last)) return false;
                }
                if (!item_write_byte(handler, '\r', last)) return false;
                return parse_multipart_post_byte(handler, data, last);
            }
            break;

        case FILE_UPLOAD_PARSING_FINISHED:
        case FILE_UPLOAD_PARSE_ERROR:
        default:
            break;
    }

    return handler->multipart_state != FILE_UPLOAD_PARSE_ERROR;
}

static esp_err_t multipart_upload_handler(FileUploadHandler *handler, FileUploadRequest *request) {
    const char *content_type = file_upload_request_get_header(request, "Content-Type");
    if (!content_type || strncmp(content_type, "multipart/", 10) != 0) {
        ESP_LOGE(TAG, "No multipart boundary found");
        esp_err_t reply_err = file_upload_request_reply(request, 400, "text/html", "No multipart boundary found.");
        if (reply_err == ESP_OK) {
            handler->response_sent = true;
            handler->last_error = ESP_ERR_INVALID_ARG;
            return ESP_FAIL;
        }
        handler->last_error = reply_err;
        return reply_err;
    }

    const char *boundary_key = strstr(content_type, "boundary=");
    if (!boundary_key) {
        ESP_LOGE(TAG, "Multipart boundary missing");
        esp_err_t reply_err = file_upload_request_reply(request, 400, "text/html", "No multipart boundary found.");
        if (reply_err == ESP_OK) {
            handler->response_sent = true;
            handler->last_error = ESP_ERR_INVALID_ARG;
            return ESP_FAIL;
        }
        handler->last_error = reply_err;
        return reply_err;
    }
    boundary_key += 9;
    const char *boundary_end = boundary_key;
    if (*boundary_key == '"') {
        boundary_key++;
        boundary_end = strchr(boundary_key, '"');
        if (!boundary_end) boundary_end = boundary_key + strlen(boundary_key);
    } else {
        while (*boundary_end && *boundary_end != ';' && !isspace((int)*boundary_end)) {
            boundary_end++;
        }
    }

    size_t boundary_len = (size_t)(boundary_end - boundary_key);
    if (boundary_len == 0 || boundary_len > 70) {
        ESP_LOGE(TAG, "Invalid multipart boundary length: %zu", boundary_len);
        esp_err_t reply_err = file_upload_request_reply(request, 400, "text/html", "Invalid multipart boundary.");
        if (reply_err == ESP_OK) {
            handler->response_sent = true;
            handler->last_error = ESP_ERR_INVALID_ARG;
        }
        return ESP_FAIL;
    }
    if (!mutable_string_assign(&handler->boundary, boundary_key, boundary_len)) {
        ESP_LOGE(TAG, "Failed to store boundary");
        return ESP_ERR_NO_MEM;
    }

    httpd_req_t *req = file_upload_request_get_httpd_req(request);
    if (!req) {
        ESP_LOGE(TAG, "Missing httpd request context");
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t *buf = (uint8_t *)malloc(FILE_CHUNK_SIZE);
    if (!buf) {
        ESP_LOGE(TAG, "Failed to allocate multipart buffer");
        return ESP_ERR_NO_MEM;
    }

    size_t remaining = file_upload_request_content_length(request);
    bool length_known = (remaining > 0);
    size_t max_size = file_upload_request_max_upload_size(request);
    esp_err_t err = ESP_OK;

    ESP_LOGI(TAG, "Multipart handler - remaining: %zu, length_known: %s, max_size: %zu", remaining, length_known ? "true" : "false", max_size);

    while (!length_known || remaining > 0) {
        size_t to_read = length_known ? (remaining > FILE_CHUNK_SIZE ? FILE_CHUNK_SIZE : remaining) : FILE_CHUNK_SIZE;
        if (!length_known && to_read == 0) {
            to_read = FILE_CHUNK_SIZE;
        }

        int received = httpd_req_recv(req, (char *)buf, to_read);
        if (received < 0) {
            if (received == HTTPD_SOCK_ERR_TIMEOUT) {
                continue;
            }
            ESP_LOGE(TAG, "Socket error during multipart upload (%d)", received);
            err = ESP_FAIL;
            break;
        }

        if (received == 0) {
            if (length_known && remaining != 0) {
                ESP_LOGE(TAG, "Multipart upload ended before receiving declared length - remaining: %zu", remaining);
                err = ESP_ERR_INVALID_RESPONSE;
            }
            break;
        }

        for (int i = 0; i < received; ++i) {
            if (max_size != SIZE_MAX) {
                if (handler->parsed_length >= max_size) {
                    ESP_LOGE(TAG, "Multipart upload exceeded maximum size limit (%zu bytes)", max_size);
                    err = send_size_error(handler, request, max_size);
                    handler->multipart_state = FILE_UPLOAD_PARSE_ERROR;
                    break;
                }
            }
            bool last = false;
            if (length_known) {
                if (remaining == 0) {
                    ESP_LOGE(TAG, "Multipart parser received more data than expected");
                    handler->multipart_state = FILE_UPLOAD_PARSE_ERROR;
                    handler->last_error = ESP_ERR_INVALID_RESPONSE;
                    err = ESP_ERR_INVALID_RESPONSE;
                    break;
                }
                remaining--;
                last = (remaining == 0);
            }
            if (err != ESP_OK) {
                break;
            }
            if (!parse_multipart_post_byte(handler, buf[i], last)) {
                err = (handler->last_error != ESP_OK) ? handler->last_error : ESP_FAIL;
                if (length_known) {
                    remaining = 0;
                }
                break;
            }
            handler->parsed_length++;
        }
        if (err != ESP_OK) {
            break;
        }
    }

    free(buf);
    if (handler->multipart_state == FILE_UPLOAD_PARSE_ERROR && err == ESP_OK) {
        err = (handler->last_error != ESP_OK) ? handler->last_error : ESP_FAIL;
    } else if (err == ESP_OK && handler->multipart_state != FILE_UPLOAD_PARSING_FINISHED) {
        ESP_LOGE(TAG, "Multipart: Stream ended without final boundary");
        err = ESP_ERR_INVALID_RESPONSE;
    }
    return err;
}

esp_err_t file_upload_handler_handle_request(FileUploadHandler *handler, FileUploadRequest *request) {
    if (!handler || !request) {
        return ESP_ERR_INVALID_ARG;
    }

    handler_reset_dynamic_state(handler);
    handler->request = request;
    file_upload_request_clear_current_file_size_hint(request);

    size_t content_length = file_upload_request_content_length(request);
    size_t max_size = file_upload_request_max_upload_size(request);
    if (max_size != SIZE_MAX && content_length > max_size) {
        ESP_LOGE(TAG, "File too large: %zu bytes", content_length);
        return send_size_error(handler, request, max_size);
    }

    file_upload_request_load_params(request);

    esp_err_t err = ESP_OK;
    if (file_upload_request_is_multipart(request)) {
        err = multipart_upload_handler(handler, request);
    } else {
        err = basic_upload_handler(handler, request);
    }

    if (err == ESP_OK) {
        if (handler->request_cb) {
            esp_err_t cb_err = handler->request_cb(request);
            if (cb_err != ESP_OK) {
                ESP_LOGE(TAG, "Request callback failed: %s", esp_err_to_name(cb_err));
                err = cb_err;
            }
        } else {
            err = file_upload_request_reply_text(request, "Upload Successful.");
            if (err == ESP_OK) {
                handler->response_sent = true;
            }
        }
    }

    if (err != ESP_OK && !handler->response_sent) {
        esp_err_t reply_err = file_upload_request_reply(request, 500, "text/html", "Error processing upload.");
        if (reply_err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to send error response: %s", esp_err_to_name(reply_err));
            err = reply_err;
        } else {
            handler->response_sent = true;
        }
    }

    return err;
}
