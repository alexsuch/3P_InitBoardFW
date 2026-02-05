#include <esp_log.h>
#include <hal/storage.h>

hal_err_t hal_storage_init(hal_storage_t *s, const char *name) {
    if (!s || !name) return HAL_ERR_INVALID_ARG;

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        err = nvs_flash_erase();
        if (err != ESP_OK) {
            ESP_LOGE("HAL_STORAGE", "nvs_flash_erase failed: %d", err);
            return HAL_ERR_FAILED;
        }
        err = nvs_flash_init();
    }
    if (err != ESP_OK) return HAL_ERR_FAILED;

    err = nvs_open(name, NVS_READWRITE, &s->handle);
    return err == ESP_OK ? HAL_ERR_NONE : HAL_ERR_FAILED;
}

bool hal_storage_get_blob(hal_storage_t *s, const char *key, void *buf, size_t *size) {
    if (!s || !s->handle || !key || !size) return false;

    esp_err_t err = nvs_get_blob(s->handle, key, buf, size);
    if (err == ESP_OK) {
        return true;
    }
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        return false;
    }
    // Log and return false instead of aborting
    ESP_LOGE("HAL_STORAGE", "get_blob failed: %d", err);
    return false;
}

hal_err_t hal_storage_get_blob2(hal_storage_t *s, const char *key, void *buf, size_t *size, bool *found) {
    if (!s || !s->handle || !key || !size) return HAL_ERR_INVALID_ARG;

    esp_err_t err = nvs_get_blob(s->handle, key, buf, size);
    if (err == ESP_OK) {
        if (found) *found = true;
        return HAL_ERR_NONE;
    }
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        if (found) *found = false;
        return HAL_ERR_NONE;  // Not found is not an error
    }

    // Log the actual error
    ESP_LOGE("HAL_STORAGE", "get_blob2 failed: %d", err);
    if (found) *found = false;
    return HAL_ERR_FAILED;
}

hal_err_t hal_storage_set_blob(hal_storage_t *s, const char *key, const void *buf, size_t size) {
    if (!s || !s->handle || !key) return HAL_ERR_INVALID_ARG;

    // size == 0 means erase operation
    if (size > 0) {
        if (!buf) return HAL_ERR_INVALID_ARG;
        esp_err_t err = nvs_set_blob(s->handle, key, buf, size);
        if (err != ESP_OK) {
            ESP_LOGE("HAL_STORAGE", "set_blob failed: %d", err);
            return HAL_ERR_FAILED;
        }
    } else {
        // Erase operation: size == 0 means erase the key
        esp_err_t err = nvs_erase_key(s->handle, key);
        if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
            ESP_LOGE("HAL_STORAGE", "erase failed: %d", err);
            return HAL_ERR_FAILED;
        }
    }

    // Commit the changes to NVS
    esp_err_t commit_err = nvs_commit(s->handle);
    if (commit_err != ESP_OK) {
        ESP_LOGE("HAL_STORAGE", "commit failed: %d", commit_err);
        return HAL_ERR_FAILED;
    }

    return HAL_ERR_NONE;
}

hal_err_t hal_storage_commit(hal_storage_t *s) {
    if (!s || !s->handle) return HAL_ERR_INVALID_ARG;

    esp_err_t err = nvs_commit(s->handle);
    if (err != ESP_OK) {
        ESP_LOGE("HAL_STORAGE", "commit failed: %d", err);
        return HAL_ERR_FAILED;
    }

    return HAL_ERR_NONE;
}

void hal_storage_deinit(hal_storage_t *s) {
    if (!s || !s->handle) return;
    nvs_close(s->handle);
    s->handle = 0;
}