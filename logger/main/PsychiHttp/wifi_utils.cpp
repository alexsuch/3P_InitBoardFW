#include "wifi_utils.h"
#include <esp_log.h>
#include <esp_netif.h>

namespace PsychicHttp {

IPAddress WiFiUtils::localIP() {
    esp_netif_t* netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (netif == nullptr) {
        return IPAddress(0, 0, 0, 0);
    }
    
    esp_netif_ip_info_t ip_info;
    if (esp_netif_get_ip_info(netif, &ip_info) != ESP_OK) {
        return IPAddress(0, 0, 0, 0);
    }
    
    return IPAddress(ip_info.ip.addr);
}

IPAddress WiFiUtils::softAPIP() {
    esp_netif_t* netif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
    if (netif == nullptr) {
        return IPAddress(0, 0, 0, 0);
    }
    
    esp_netif_ip_info_t ip_info;
    if (esp_netif_get_ip_info(netif, &ip_info) != ESP_OK) {
        return IPAddress(0, 0, 0, 0);
    }
    
    return IPAddress(ip_info.ip.addr);
}

bool WiFiUtils::isConnected() {
    wifi_ap_record_t ap_info;
    return esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK;
}

bool WiFiUtils::isSoftAPRunning() {
    wifi_mode_t mode;
    if (esp_wifi_get_mode(&mode) != ESP_OK) {
        return false;
    }
    return (mode == WIFI_MODE_APSTA || mode == WIFI_MODE_AP);
}

wifi_ap_record_t* WiFiUtils::getAPInfo() {
    static wifi_ap_record_t ap_info;
    if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
        return &ap_info;
    }
    return nullptr;
}

wifi_mode_t WiFiUtils::getMode() {
    wifi_mode_t mode;
    if (esp_wifi_get_mode(&mode) != ESP_OK) {
        return WIFI_MODE_NULL;
    }
    return mode;
}

} // namespace PsychicHttp 