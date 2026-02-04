#include "captive_portal.h"

#include <string.h>

#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "log.h"
#include "lwip/def.h"
#include "lwip/inet.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"

#define CAPTIVE_PORTAL_DNS_PORT 53
#define CAPTIVE_PORTAL_LOG_INTERVAL_US (5ULL * 1000000ULL)
#define MAX_DNS_QUESTION 260  // Max QNAME is 255 bytes + 4 (QTYPE/QCLASS) => 259; pad a little

static const char *TAG = "CAPTIVE";

// Weak stub for when web server module is not compiled
// The real implementation is in web_server_cpp.cpp
__attribute__((weak)) void web_server_configure_captive_portal(void *server, bool enabled) {
    (void)server;
    (void)enabled;
    // No-op when web server module is not available
}

static bool portal_lock(captive_portal_t *portal) {
    if (!portal || !portal->lock) {
        return false;
    }
    return xSemaphoreTake(portal->lock, portMAX_DELAY) == pdTRUE;
}

static void portal_unlock(captive_portal_t *portal, bool locked) {
    if (portal && portal->lock && locked) {
        xSemaphoreGive(portal->lock);
    }
}

typedef struct __attribute__((packed)) {
    uint16_t id;
    uint16_t flags;
    uint16_t qdcount;
    uint16_t ancount;
    uint16_t nscount;
    uint16_t arcount;
} dns_header_t;

static bool copy_bytes(const struct pbuf *p, void *dst, size_t len, size_t offset) { return pbuf_copy_partial((struct pbuf *)p, dst, len, offset) == len; }

static bool extract_question(const struct pbuf *p, size_t offset, uint8_t *buffer, size_t buffer_len, size_t *question_len) {
    size_t idx = offset;
    size_t total = p->tot_len;
    size_t length = 0;

    while (idx < total) {
        uint8_t label_len = 0;
        if (!copy_bytes(p, &label_len, 1, idx)) {
            return false;
        }
        idx++;

        if (length >= buffer_len) {
            return false;
        }
        buffer[length++] = label_len;

        if (label_len == 0) {
            break;
        }

        if ((label_len & 0xC0U) == 0xC0U) {
            // Pointer compression terminates the name
            if (length >= buffer_len) {
                return false;
            }
            if (!copy_bytes(p, buffer + length, 1, idx)) {
                return false;
            }
            length++;
            idx++;
            break;
        }

        if (length + label_len > buffer_len) {
            return false;
        }
        if (!copy_bytes(p, buffer + length, label_len, idx)) {
            return false;
        }
        idx += label_len;
        length += label_len;
    }

    if (idx + 4 > total || length + 4 > buffer_len) {
        return false;
    }

    if (!copy_bytes(p, buffer + length, 4, idx)) {
        return false;
    }
    length += 4;

    *question_len = length;
    return true;
}

static void log_portal_metrics_locked(captive_portal_t *portal) {
    if (!portal) {
        return;
    }

    uint64_t now = esp_timer_get_time();
    if (portal->last_log_us == 0 || now - portal->last_log_us >= CAPTIVE_PORTAL_LOG_INTERVAL_US) {
        LOG_I(TAG, "Captive portal DNS stats: recent=%u total=%u dropped=%u", portal->queries_since_log, portal->total_queries, portal->dropped_queries);
        portal->queries_since_log = 0;
        portal->last_log_us = now;
    }
}

static void dns_request_handler(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port) {
    captive_portal_t *portal = (captive_portal_t *)arg;

    if (!portal || !p) {
        if (p) {
            pbuf_free(p);
        }
        return;
    }

    bool locked = portal_lock(portal);
    if (!locked) {
        LOG_E(TAG, "Failed to lock captive portal state");
        pbuf_free(p);
        return;
    }
    bool process = portal->running && portal->enabled;
    ip4_addr_t ap_ip_copy;
    ip4_addr_copy(ap_ip_copy, portal->ap_ip);
    portal_unlock(portal, locked);
    if (!process) {
        pbuf_free(p);
        return;
    }

    if (p->tot_len < sizeof(dns_header_t)) {
        // Brief lock to update counter
        locked = portal_lock(portal);
        if (locked) {
            portal->dropped_queries++;
            portal_unlock(portal, locked);
        }
        pbuf_free(p);
        return;
    }

    dns_header_t header = {0};
    if (!copy_bytes(p, &header, sizeof(header), 0)) {
        // Brief lock to update counter
        locked = portal_lock(portal);
        if (locked) {
            portal->dropped_queries++;
            portal_unlock(portal, locked);
        }
        pbuf_free(p);
        return;
    }

    uint16_t id = lwip_ntohs(header.id);
    uint16_t flags = lwip_ntohs(header.flags);
    uint16_t qdcount = lwip_ntohs(header.qdcount);

    if ((flags & 0x8000U) != 0 || qdcount == 0) {
        pbuf_free(p);
        return;
    }

    uint8_t question[MAX_DNS_QUESTION];
    size_t question_len = 0;
    if (!extract_question(p, sizeof(dns_header_t), question, sizeof(question), &question_len)) {
        // Brief lock to update counter
        locked = portal_lock(portal);
        if (locked) {
            portal->dropped_queries++;
            portal_unlock(portal, locked);
        }
        pbuf_free(p);
        return;
    }

    // Extract QTYPE/QCLASS (last 4 bytes of question)
    uint16_t qtype = (uint16_t)((question[question_len - 4] << 8) | question[question_len - 3]);
    uint16_t qclass = (uint16_t)((question[question_len - 2] << 8) | question[question_len - 1]);
    bool is_a_in = (qtype == 0x0001 /*A*/ && qclass == 0x0001 /*IN*/);
    size_t response_len = sizeof(dns_header_t) + question_len + (is_a_in ? 16 : 0);
    struct pbuf *response = pbuf_alloc(PBUF_TRANSPORT, response_len, PBUF_RAM);
    if (!response) {
        // Brief lock to update counter
        locked = portal_lock(portal);
        if (locked) {
            portal->dropped_queries++;
            portal_unlock(portal, locked);
        }
        pbuf_free(p);
        return;
    }

    ip4_addr_t target_ip;
    ip4_addr_copy(target_ip, ap_ip_copy);

    uint8_t *payload = (uint8_t *)response->payload;
    memset(payload, 0, response_len);

    dns_header_t resp_header = {0};
    resp_header.id = lwip_htons(id);
    // QR=1, echo client's RD flag, recursion not available (RA=0)
    uint16_t rd_flag = (flags & 0x0100U);
    resp_header.flags = lwip_htons(0x8000U | rd_flag);
    resp_header.qdcount = lwip_htons(1);
    resp_header.ancount = lwip_htons(is_a_in ? 1 : 0);
    memcpy(payload, &resp_header, sizeof(resp_header));

    memcpy(payload + sizeof(resp_header), question, question_len);

    size_t offset = sizeof(resp_header) + question_len;
    if (is_a_in) {
        payload[offset++] = 0xC0;
        payload[offset++] = 0x0C;  // Name pointer to question
        payload[offset++] = 0x00;
        payload[offset++] = 0x01;  // Type A
        payload[offset++] = 0x00;
        payload[offset++] = 0x01;  // Class IN
        payload[offset++] = 0x00;
        payload[offset++] = 0x00;
        payload[offset++] = 0x00;
        payload[offset++] = 0x3C;  // TTL 60s
        payload[offset++] = 0x00;
        payload[offset++] = 0x04;  // RDLENGTH
        payload[offset++] = ip4_addr1(&target_ip);
        payload[offset++] = ip4_addr2(&target_ip);
        payload[offset++] = ip4_addr3(&target_ip);
        payload[offset++] = ip4_addr4(&target_ip);
    }

    err_t err = udp_sendto(pcb, response, addr, port);
    pbuf_free(response);

    if (err != ERR_OK) {
        // Brief lock to update counter
        locked = portal_lock(portal);
        if (locked) {
            portal->dropped_queries++;
            portal_unlock(portal, locked);
        }
        LOG_W(TAG, "Failed to send DNS response: %d", (int)err);
        pbuf_free(p);
        return;
    }

    // Brief lock to update counters and log metrics
    locked = portal_lock(portal);
    if (locked) {
        portal->total_queries++;
        portal->queries_since_log++;
        log_portal_metrics_locked(portal);
        portal_unlock(portal, locked);
    }
    pbuf_free(p);
}

void captive_portal_init(captive_portal_t *portal) {
    if (!portal) {
        return;
    }

    if (portal->lock) {
        vSemaphoreDelete(portal->lock);
    }

    memset(portal, 0, sizeof(*portal));
    portal->lock = xSemaphoreCreateMutex();
    if (!portal->lock) {
        LOG_E(TAG, "Failed to create captive portal mutex");
        return;
    }
    portal->enabled = true;
    portal->http_endpoints_registered = false;
}

esp_err_t captive_portal_start(captive_portal_t *portal, const ip4_addr_t *ap_ip, void *http_server) {
    if (!portal || !ap_ip) {
        return ESP_ERR_INVALID_ARG;
    }

    if (portal->running) {
        bool locked = portal_lock(portal);
        if (!locked) {
            LOG_E(TAG, "Failed to lock captive portal state for update");
            return ESP_FAIL;
        }
        if (!ip4_addr_cmp(&portal->ap_ip, ap_ip)) {
            bool was_enabled = portal->enabled;
            portal->enabled = false;
            ip4_addr_copy(portal->ap_ip, *ap_ip);
            portal->enabled = was_enabled;
            LOG_I(TAG, "Captive portal IP updated to " IPSTR, IP2STR(&portal->ap_ip));
        }
        portal_unlock(portal, locked);
        return ESP_OK;
    }

    struct udp_pcb *pcb = udp_new_ip_type(IPADDR_TYPE_ANY);
    if (!pcb) {
        LOG_E(TAG, "Failed to allocate UDP PCB for captive portal DNS");
        return ESP_ERR_NO_MEM;
    }

    err_t bind_err = udp_bind(pcb, IP_ANY_TYPE, CAPTIVE_PORTAL_DNS_PORT);
    if (bind_err != ERR_OK) {
        LOG_E(TAG, "Failed to bind DNS server: %d", (int)bind_err);
        udp_remove(pcb);
        return ESP_FAIL;
    }

    bool locked = portal_lock(portal);
    if (!locked) {
        LOG_E(TAG, "Failed to lock captive portal state during start");
        udp_remove(pcb);
        return ESP_FAIL;
    }

    portal->dns_pcb = pcb;
    portal->http_server = http_server;
    ip4_addr_copy(portal->ap_ip, *ap_ip);
    portal->enabled = true;
    portal->running = true;
    portal->total_queries = 0;
    portal->dropped_queries = 0;
    portal->queries_since_log = 0;
    portal->last_log_us = esp_timer_get_time();
    if (http_server) {
        portal->http_endpoints_registered = true;
    }

    portal_unlock(portal, locked);

    udp_recv(pcb, dns_request_handler, portal);

    // Register HTTP endpoints if HTTP server is provided
    if (http_server) {
        LOG_I(TAG, "Registering captive portal HTTP endpoints");
        web_server_configure_captive_portal(http_server, true);
    } else {
        LOG_W(TAG, "No HTTP server provided, skipping HTTP endpoint registration");
    }

    LOG_I(TAG, "Captive portal DNS started on " IPSTR, IP2STR(ap_ip));
    return ESP_OK;
}

void captive_portal_stop(captive_portal_t *portal) {
    if (!portal) {
        return;
    }

    struct udp_pcb *pcb = NULL;
    bool was_running = false;

    bool locked = portal_lock(portal);
    if (!locked) {
        LOG_E(TAG, "Failed to lock captive portal state during stop");
        return;
    }

    if (portal->dns_pcb) {
        pcb = portal->dns_pcb;
        portal->dns_pcb = NULL;
    }

    was_running = portal->running;
    portal->running = false;
    portal->enabled = false;
    portal->queries_since_log = 0;
    if (portal->http_server && portal->http_endpoints_registered) {
        portal->http_endpoints_registered = false;
    }

    portal_unlock(portal, locked);

    // Disable HTTP endpoints if they were registered
    if (portal->http_server && portal->http_endpoints_registered) {
        LOG_I(TAG, "Disabling captive portal HTTP endpoints");
        web_server_configure_captive_portal(portal->http_server, false);
    }

    if (pcb) {
        udp_remove(pcb);
    }

    if (was_running) {
        LOG_I(TAG, "Captive portal DNS stopped");
    }
}

void captive_portal_set_enabled(captive_portal_t *portal, bool enabled) {
    if (!portal) {
        return;
    }

    bool locked = portal_lock(portal);
    if (!locked) {
        LOG_E(TAG, "Failed to lock captive portal state while setting enabled flag");
        return;
    }

    portal->enabled = enabled;

    // Update HTTP endpoints if server is available
    if (portal->http_server) {
        LOG_I(TAG, "Updating captive portal HTTP endpoints: %s", enabled ? "enabled" : "disabled");
        web_server_configure_captive_portal(portal->http_server, enabled);
    }

    portal_unlock(portal, locked);
}

bool captive_portal_is_running(const captive_portal_t *portal) {
    if (!portal) {
        return false;
    }
    return portal->running;
}
