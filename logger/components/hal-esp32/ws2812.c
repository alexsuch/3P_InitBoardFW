#include <driver/gpio.h>
#include <driver/rmt_encoder.h>
#include <driver/rmt_tx.h>
#include <esp_err.h>
#include <esp_log.h>
#include <esp_rom_sys.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <hal/ws2812.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"

// WS2812 timing parameters (in nanoseconds)
#define WS2812_T0H_NS 350
#define WS2812_T1H_NS 900
#define WS2812_T0L_NS 900
#define WS2812_T1L_NS 350
#define WS2812_RESET_US 50

// RMT configuration
// Use 20MHz so WS2812 timings (350ns, 900ns) are represented exactly in ticks

// ESP32-S3 and ESP32-C3 have 48 symbols per channel, ESP32 has 64
#if defined(CONFIG_IDF_TARGET_ESP32S3) || defined(CONFIG_IDF_TARGET_ESP32C3)
#define WS2812_RMT_MEM_BLOCK_SYMBOLS 48
#else
#define WS2812_RMT_MEM_BLOCK_SYMBOLS 64  // Number of symbols per block
#endif

// Static handles for RMT channel and encoder
static rmt_channel_handle_t ws2812_rmt_channel = NULL;
static rmt_encoder_handle_t ws2812_encoder = NULL;
static SemaphoreHandle_t ws2812_tx_done_sem = NULL;
static SemaphoreHandle_t ws2812_lock = NULL;
static gpio_num_t ws2812_gpio = -1;

// Memory pool for pixel buffers to reduce fragmentation
#define WS2812_MAX_LED_COUNT 16  // Maximum LEDs supported
#define WS2812_PIXEL_BUFFER_SIZE (WS2812_MAX_LED_COUNT * 3)
static uint8_t ws2812_pixel_pool[WS2812_PIXEL_BUFFER_SIZE];
static bool ws2812_pool_in_use = false;

// RMT TX done callback
static bool ws2812_tx_done_callback(rmt_channel_handle_t channel, const rmt_tx_done_event_data_t *edata, void *user_ctx) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (ws2812_tx_done_sem) {
        xSemaphoreGiveFromISR(ws2812_tx_done_sem, &xHigherPriorityTaskWoken);
    }
    return xHigherPriorityTaskWoken == pdTRUE;
}

hal_err_t hal_ws2812_open(hal_gpio_t gpio) {
    if (ws2812_rmt_channel) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!ws2812_lock) {
        ws2812_lock = xSemaphoreCreateMutex();
        if (!ws2812_lock) return ESP_ERR_NO_MEM;
    }
    ws2812_gpio = (gpio_num_t)gpio;

    // 1. Try to create RMT TX channel
    // ESP32-S3 has only 4 TX channels per group (vs 8 on ESP32)
    // If DMA fails or no channels available, try without DMA
    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .gpio_num = ws2812_gpio,
        .mem_block_symbols = WS2812_RMT_MEM_BLOCK_SYMBOLS,
        .resolution_hz = WS2812_RMT_RESOLUTION_HZ,
        .trans_queue_depth = 4,  // Better have a larger queue for DMA
        .flags.with_dma = true,  // Try to enable DMA
    };

    esp_err_t ret = rmt_new_tx_channel(&tx_chan_config, &ws2812_rmt_channel);

    // ESP32-S3 has only 1 DMA-capable TX channel (channel 3), so if buzzer uses it,
    // WS2812 will get ESP_ERR_NOT_FOUND or ESP_ERR_NO_RESOURCE
    // Catch all resource exhaustion errors and fall back to non-DMA
    if (ret != ESP_OK && tx_chan_config.flags.with_dma) {
        // DMA not available or channels exhausted, fall back to non-DMA mode
        ESP_LOGW("WS2812_HAL", "RMT DMA channel not available (err=%d, %s). Falling back to non-DMA mode.", ret, esp_err_to_name(ret));
        tx_chan_config.flags.with_dma = false;
        tx_chan_config.trans_queue_depth = 2;  // Can decrease queue
        ret = rmt_new_tx_channel(&tx_chan_config, &ws2812_rmt_channel);
        if (ret == ESP_OK) {
            ESP_LOGI("WS2812_HAL", "WS2812 initialized successfully in non-DMA mode");
        }
    }

    // After all attempts, check for final errors
    if (ret != ESP_OK) return ret;

    // 2. Create bytes encoder with WS2812 timing
    // TODO share frequency
    const int ns_per_tick = (1000000000 / WS2812_RMT_RESOLUTION_HZ);  // 50ns per tick at 20MHz
    // Round up to ensure we never go under the WS2812 spec timings
    const int t0h_ticks = (WS2812_T0H_NS + ns_per_tick - 1) / ns_per_tick;
    const int t0l_ticks = (WS2812_T0L_NS + ns_per_tick - 1) / ns_per_tick;
    const int t1h_ticks = (WS2812_T1H_NS + ns_per_tick - 1) / ns_per_tick;
    const int t1l_ticks = (WS2812_T1L_NS + ns_per_tick - 1) / ns_per_tick;

    rmt_bytes_encoder_config_t encoder_config = {
        .bit0 =
            {
                .duration0 = t0h_ticks,
                .level0 = 1,
                .duration1 = t0l_ticks,
                .level1 = 0,
            },
        .bit1 =
            {
                .duration0 = t1h_ticks,
                .level0 = 1,
                .duration1 = t1l_ticks,
                .level1 = 0,
            },
        .flags.msb_first = 1,
    };
    ret = rmt_new_bytes_encoder(&encoder_config, &ws2812_encoder);
    if (ret != ESP_OK) return ret;

    // 3. Register TX done callback
    rmt_tx_event_callbacks_t cbs = {
        .on_trans_done = ws2812_tx_done_callback,
    };
    ret = rmt_tx_register_event_callbacks(ws2812_rmt_channel, &cbs, NULL);
    if (ret != ESP_OK) return ret;

    // 4. Enable the channel
    ret = rmt_enable(ws2812_rmt_channel);
    if (ret != ESP_OK) return ret;

    return ESP_OK;
}

hal_err_t hal_ws2812_close(hal_gpio_t gpio) {
    (void)gpio;  // or validate: if (gpio != ws2812_gpio) return ESP_ERR_INVALID_ARG;
    if (ws2812_rmt_channel) {
        rmt_disable(ws2812_rmt_channel);
        rmt_del_channel(ws2812_rmt_channel);
        ws2812_rmt_channel = NULL;
    }
    if (ws2812_encoder) {
        rmt_del_encoder(ws2812_encoder);
        ws2812_encoder = NULL;
    }
    if (ws2812_tx_done_sem) {
        vSemaphoreDelete(ws2812_tx_done_sem);
        ws2812_tx_done_sem = NULL;
    }
    if (ws2812_lock) {
        vSemaphoreDelete(ws2812_lock);
        ws2812_lock = NULL;
    }
    ws2812_gpio = -1;
    return ESP_OK;
}

hal_err_t hal_ws2812_set_colors(hal_gpio_t gpio, hal_ws2812_color_order_e color_order, const hal_ws2812_color_t *colors, size_t count) {
    esp_err_t ret;
    if (!ws2812_rmt_channel || !ws2812_encoder) {
        return ESP_ERR_INVALID_STATE;
    }
    if (ws2812_gpio != (gpio_num_t)gpio) {
        return ESP_ERR_INVALID_ARG;
    }
    if (count > WS2812_MAX_LED_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }

    // Prepare the semaphore
    if (!ws2812_tx_done_sem) {
        ws2812_tx_done_sem = xSemaphoreCreateBinary();
    }
    if (!ws2812_tx_done_sem) return ESP_ERR_NO_MEM;
    // Serialize access (also protects pool_in_use)
    if (ws2812_lock) xSemaphoreTake(ws2812_lock, portMAX_DELAY);
    if (ws2812_pool_in_use) {
        if (ws2812_lock) xSemaphoreGive(ws2812_lock);
        return ESP_ERR_INVALID_STATE;
    }
    ws2812_pool_in_use = true;

    // Transmit the color data (convert to configured byte order expected by LED)
    // Each color is 3 bytes in the configured order
    size_t grb_len = count * 3;
    uint8_t *pix_bytes = ws2812_pixel_pool;
    for (size_t i = 0; i < count; i++) {
        const hal_ws2812_color_t *c = &colors[i];
        size_t base = i * 3;
        switch (color_order) {
            case HAL_WS2812_ORDER_GRB:
                pix_bytes[base + 0] = c->g;
                pix_bytes[base + 1] = c->r;
                pix_bytes[base + 2] = c->b;
                break;
            case HAL_WS2812_ORDER_RGB:
                pix_bytes[base + 0] = c->r;
                pix_bytes[base + 1] = c->g;
                pix_bytes[base + 2] = c->b;
                break;
            case HAL_WS2812_ORDER_BRG:
                pix_bytes[base + 0] = c->b;
                pix_bytes[base + 1] = c->r;
                pix_bytes[base + 2] = c->g;
                break;
            case HAL_WS2812_ORDER_GBR:
                pix_bytes[base + 0] = c->g;
                pix_bytes[base + 1] = c->b;
                pix_bytes[base + 2] = c->r;
                break;
            case HAL_WS2812_ORDER_RBG:
                pix_bytes[base + 0] = c->r;
                pix_bytes[base + 1] = c->b;
                pix_bytes[base + 2] = c->g;
                break;
            case HAL_WS2812_ORDER_BGR:
                pix_bytes[base + 0] = c->b;
                pix_bytes[base + 1] = c->g;
                pix_bytes[base + 2] = c->r;
                break;
            default:  // Fallback to GRB
                pix_bytes[base + 0] = c->g;
                pix_bytes[base + 1] = c->r;
                pix_bytes[base + 2] = c->b;
                break;
        }
    }

    rmt_transmit_config_t tx_config = {
        .loop_count = 0,
        .flags.eot_level = 0,
    };
    ret = rmt_transmit(ws2812_rmt_channel, ws2812_encoder, pix_bytes, grb_len, &tx_config);
    if (ret != ESP_OK) {
        ws2812_pool_in_use = false;
        if (ws2812_lock) xSemaphoreGive(ws2812_lock);
        return ret;
    }
    // Wait for transmission to complete
    if (xSemaphoreTake(ws2812_tx_done_sem, pdMS_TO_TICKS(100)) != pdTRUE) {
        ws2812_pool_in_use = false;
        if (ws2812_lock) xSemaphoreGive(ws2812_lock);
        return ESP_ERR_TIMEOUT;
    }
    // Line is kept low via eot_level=0; no direct GPIO poke needed
    esp_rom_delay_us(WS2812_RESET_US);

    // Release the memory pool
    ws2812_pool_in_use = false;
    if (ws2812_lock) xSemaphoreGive(ws2812_lock);
    return ESP_OK;
}