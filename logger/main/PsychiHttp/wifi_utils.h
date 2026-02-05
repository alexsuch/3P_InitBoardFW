#ifndef WIFI_UTILS_H
#define WIFI_UTILS_H

#include "ip_utils.h"
#include <esp_wifi.h>
#include <esp_netif.h>

namespace PsychicHttp {

/**
 * WiFi utilities for ESP-IDF (replaces Arduino WiFi.h)
 */
class WiFiUtils {
public:
    /**
     * Get local IP address (replaces WiFi.localIP())
     * @return Local IP address
     */
    static IPAddress localIP();
    
    /**
     * Get soft AP IP address (replaces WiFi.softAPIP())
     * @return Soft AP IP address
     */
    static IPAddress softAPIP();
    
    /**
     * Check if WiFi is connected
     * @return true if connected
     */
    static bool isConnected();
    
    /**
     * Check if soft AP is running
     * @return true if soft AP is running
     */
    static bool isSoftAPRunning();
    
    /**
     * Get WiFi status
     * @return WiFi status
     */
    static wifi_ap_record_t* getAPInfo();
    
    /**
     * Get current WiFi mode
     * @return WiFi mode
     */
    static wifi_mode_t getMode();
};

} // namespace PsychicHttp

#endif // WIFI_UTILS_H 