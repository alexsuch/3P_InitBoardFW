#ifndef UPLOAD_COMMON_H
#define UPLOAD_COMMON_H

#include <esp_err.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Validate CSRF token from cookie and form data
 * @param cookie_header HTTP Cookie header value
 * @param form_token CSRF token from form data
 * @return true if tokens match and are valid, false otherwise
 */
bool upload_validate_csrf_token(const char *cookie_header, const char *form_token);

/**
 * @brief Validate and sanitize upload path
 * @param path Input path from request
 * @param out_relative Output buffer for sanitized relative path (must be at least 256 bytes)
 * @param out_fs Output buffer for filesystem path (must be at least 256 bytes)
 * @return ESP_OK if valid, ESP_ERR_INVALID_ARG if invalid
 */
esp_err_t upload_validate_path(const char *path, char *out_relative, char *out_fs);

/**
 * @brief Validate that directory exists and is accessible
 * @param fs_path Filesystem path to check
 * @return ESP_OK if directory exists, ESP_ERR_NOT_FOUND if not found, ESP_FAIL on error
 */
esp_err_t upload_validate_directory(const char *fs_path);

/**
 * @brief Sanitize filename to prevent path traversal and control characters
 * @param filename Input filename
 * @param out_safe Output buffer for sanitized filename (must be at least 256 bytes)
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if filename is invalid
 */
esp_err_t upload_sanitize_filename(const char *filename, char *out_safe);

/**
 * @brief Check if file already exists
 * @param fs_path Filesystem path to check
 * @return true if file exists, false otherwise
 */
bool upload_check_file_exists(const char *fs_path);

/**
 * @brief Check if path is within allowed root directory
 * @param fs_path Filesystem path to check
 * @return true if within root, false otherwise
 */
bool upload_is_within_root(const char *fs_path);

#ifdef __cplusplus
}
#endif

#endif  // UPLOAD_COMMON_H
