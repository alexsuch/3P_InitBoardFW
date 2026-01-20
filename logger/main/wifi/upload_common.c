#include "upload_common.h"

#include <ctype.h>
#include <errno.h>
#include <esp_log.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#define TAG "UPLOAD_COMMON"

// SD card root path - should match the one used in web_server_cpp.cpp
static const char *kSdRoot = "/sdcard";

bool upload_validate_csrf_token(const char *cookie_header, const char *form_token) {
    if (!cookie_header || !form_token) {
        ESP_LOGW(TAG, "CSRF validation failed: missing cookie or form token");
        return false;
    }

    // Find csrf_token in cookie header
    const char *needle = "csrf_token=";
    const char *pos = strstr(cookie_header, needle);
    if (!pos) {
        ESP_LOGW(TAG, "CSRF validation failed: csrf_token not found in cookie");
        return false;
    }

    pos += strlen(needle);
    const char *end = strchr(pos, ';');
    if (!end) {
        end = pos + strlen(pos);
    }

    size_t cookie_token_len = end - pos;
    size_t form_token_len = strlen(form_token);

    if (cookie_token_len != form_token_len) {
        ESP_LOGW(TAG, "CSRF validation failed: token length mismatch");
        return false;
    }

    if (strncmp(pos, form_token, cookie_token_len) != 0) {
        ESP_LOGW(TAG, "CSRF validation failed: token mismatch");
        return false;
    }

    ESP_LOGD(TAG, "CSRF validation successful");
    return true;
}

esp_err_t upload_validate_path(const char *path, char *out_relative, char *out_fs) {
    if (!path || !out_relative || !out_fs) {
        return ESP_ERR_INVALID_ARG;
    }

    // Simple path sanitization - normalize separators and remove dangerous patterns
    char normalized[512];
    size_t len = strlen(path);
    if (len >= sizeof(normalized)) {
        ESP_LOGW(TAG, "Path too long: %s", path);
        return ESP_ERR_INVALID_ARG;
    }

    // Start with root if not present
    size_t out_idx = 0;
    if (path[0] != '/') {
        normalized[out_idx++] = '/';
    }

    // Copy and normalize separators
    for (size_t i = 0; i < len && out_idx < sizeof(normalized) - 1; ++i) {
        if (path[i] == '\\') {
            normalized[out_idx++] = '/';
        } else {
            normalized[out_idx++] = path[i];
        }
    }
    normalized[out_idx] = '\0';

    // Simple check for dangerous patterns
    if (strstr(normalized, "..") != NULL) {
        ESP_LOGW(TAG, "Path contains '..': %s", path);
        return ESP_ERR_INVALID_ARG;
    }

    // Build filesystem path
    char fs_path[1024];  // Increased buffer size to prevent truncation
    int result = snprintf(fs_path, sizeof(fs_path), "%s%s", kSdRoot, normalized);
    if (result >= (int)sizeof(fs_path)) {
        ESP_LOGW(TAG, "Path too long after combining with root: %s", path);
        return ESP_ERR_INVALID_ARG;
    }

    if (!upload_is_within_root(fs_path)) {
        ESP_LOGW(TAG, "Path outside root: %s", path);
        return ESP_ERR_INVALID_ARG;
    }

    // Copy to output buffers
    strncpy(out_relative, normalized, 255);
    out_relative[255] = '\0';

    strncpy(out_fs, fs_path, 255);
    out_fs[255] = '\0';

    return ESP_OK;
}

esp_err_t upload_validate_directory(const char *fs_path) {
    if (!fs_path) {
        return ESP_ERR_INVALID_ARG;
    }

    struct stat st = {0};
    if (stat(fs_path, &st) != 0) {
        ESP_LOGW(TAG, "Directory not found: %s (%s)", fs_path, strerror(errno));
        return ESP_ERR_NOT_FOUND;
    }

    if (!S_ISDIR(st.st_mode)) {
        ESP_LOGW(TAG, "Path is not a directory: %s", fs_path);
        return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}

esp_err_t upload_sanitize_filename(const char *filename, char *out_safe) {
    if (!filename || !out_safe) {
        return ESP_ERR_INVALID_ARG;
    }

    if (strlen(filename) == 0) {
        strcpy(out_safe, "upload.bin");
        return ESP_OK;
    }

    // Sanitize filename - replace dangerous characters
    size_t len = strlen(filename);
    if (len >= 255) {
        ESP_LOGW(TAG, "Filename too long: %s", filename);
        return ESP_ERR_INVALID_ARG;
    }

    for (size_t i = 0; i < len; ++i) {
        unsigned char c = (unsigned char)filename[i];
        if (c < 0x20 || c == 0x7F || c == '"' || c == '\\' || c == '\r' || c == '\n' || c == ';' || c == '/') {
            out_safe[i] = '_';
        } else {
            out_safe[i] = c;
        }
    }
    out_safe[len] = '\0';

    // Additional security checks
    if (strstr(out_safe, "..") != NULL) {
        ESP_LOGW(TAG, "Unsafe filename detected: %s", filename);
        return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}

bool upload_check_file_exists(const char *fs_path) {
    if (!fs_path) {
        return false;
    }

    struct stat st = {0};
    return (stat(fs_path, &st) == 0 && S_ISREG(st.st_mode));
}

bool upload_is_within_root(const char *fs_path) {
    if (!fs_path) {
        return false;
    }

    const char *root = kSdRoot;
    size_t root_len = strlen(root);

    if (strcmp(fs_path, root) == 0) {
        return true;
    }

    if (strlen(fs_path) < root_len + 1) {
        return false;
    }

    return (strncmp(fs_path, root, root_len) == 0 && fs_path[root_len] == '/');
}
