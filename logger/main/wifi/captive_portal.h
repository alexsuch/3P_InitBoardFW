#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "lwip/ip4_addr.h"

#ifdef __cplusplus
extern "C" {
#endif

struct udp_pcb;

typedef struct captive_portal_s {
    struct udp_pcb *dns_pcb;
    ip4_addr_t ap_ip;
    void *http_server;  // PsychicHttpServer instance
    bool running;
    bool enabled;
    bool http_endpoints_registered;
    uint32_t total_queries;
    uint32_t dropped_queries;
    uint32_t queries_since_log;
    uint64_t last_log_us;
    SemaphoreHandle_t lock;
} captive_portal_t;

void captive_portal_init(captive_portal_t *portal);
esp_err_t captive_portal_start(captive_portal_t *portal, const ip4_addr_t *ap_ip, void *http_server);
void captive_portal_stop(captive_portal_t *portal);
void captive_portal_set_enabled(captive_portal_t *portal, bool enabled);
bool captive_portal_is_running(const captive_portal_t *portal);

// Forward declaration for web server captive portal configuration
void web_server_configure_captive_portal(void *server, bool enabled);

#ifdef __cplusplus
}
#endif
