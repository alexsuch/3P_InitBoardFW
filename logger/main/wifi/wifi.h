#pragma once

#include "target.h"  // Include target.h to get USE_WIFI definition

#if defined(USE_WIFI)

#include <stdbool.h>

#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "wifi/captive_portal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define WIFI_SSID_MAX_LEN 32
#define WIFI_PWD_MAX_LEN 64
#define WIFI_RETRY_CONNECT_COUNT 10  // ADDED: Maximum number of retries for STA connection or AP start
#define WIFI_RSSI_THRESHOLD -70      // RSSI threshold for connection quality monitoring

// WiFi Disconnect Reason Code Macros
// Standard IEEE 802.11 reason codes
#define WIFI_DISCONNECT_REASON_UNSPECIFIED 1
#define WIFI_DISCONNECT_REASON_AUTH_EXPIRE 2
#define WIFI_DISCONNECT_REASON_ASSOC_EXPIRE 3
#define WIFI_DISCONNECT_REASON_DISASSOC_INACTIVITY 4
#define WIFI_DISCONNECT_REASON_ASSOC_TOOMANY 5
#define WIFI_DISCONNECT_REASON_CLASS2_FRAME_FROM_NONAUTH_STA 6
#define WIFI_DISCONNECT_REASON_CLASS3_FRAME_FROM_NONASSOC_STA 7
#define WIFI_DISCONNECT_REASON_ASSOC_LEAVE 8
#define WIFI_DISCONNECT_REASON_ASSOC_NOT_AUTHED 9
#define WIFI_DISCONNECT_REASON_DISASSOC_PWRCAP_BAD 10
#define WIFI_DISCONNECT_REASON_DISASSOC_SUPCHAN_BAD 11
#define WIFI_DISCONNECT_REASON_BSS_TRANSITION_DISASSOC 12
#define WIFI_DISCONNECT_REASON_IE_INVALID 13
#define WIFI_DISCONNECT_REASON_MIC_FAILURE 14
#define WIFI_DISCONNECT_REASON_4WAY_HANDSHAKE_TIMEOUT 15
#define WIFI_DISCONNECT_REASON_GROUP_KEY_UPDATE_TIMEOUT 16
#define WIFI_DISCONNECT_REASON_IE_IN_4WAY_DIFFERS 17
#define WIFI_DISCONNECT_REASON_GROUP_CIPHER_INVALID 18
#define WIFI_DISCONNECT_REASON_PAIRWISE_CIPHER_INVALID 19
#define WIFI_DISCONNECT_REASON_AKMP_INVALID 20
#define WIFI_DISCONNECT_REASON_UNSUPP_RSN_IE_VERSION 21
#define WIFI_DISCONNECT_REASON_INVALID_RSN_IE_CAP 22
#define WIFI_DISCONNECT_REASON_IEEE_802_1X_AUTH_FAILED 23
#define WIFI_DISCONNECT_REASON_CIPHER_SUITE_REJECTED 24
#define WIFI_DISCONNECT_REASON_TDLS_PEER_UNREACHABLE 25
#define WIFI_DISCONNECT_REASON_TDLS_UNSPECIFIED 26
#define WIFI_DISCONNECT_REASON_SSP_REQUESTED_DISASSOC 27
#define WIFI_DISCONNECT_REASON_NO_SSP_ROAMING_AGREEMENT 28
#define WIFI_DISCONNECT_REASON_BAD_CIPHER_OR_AKM 29
#define WIFI_DISCONNECT_REASON_NOT_AUTHORIZED_THIS_LOCATION 30
#define WIFI_DISCONNECT_REASON_SERVICE_CHANGE_PRECLUDES_TS 31
#define WIFI_DISCONNECT_REASON_UNSPECIFIED_QOS 32
#define WIFI_DISCONNECT_REASON_NOT_ENOUGH_BANDWIDTH 33
#define WIFI_DISCONNECT_REASON_MISSING_ACKS 34
#define WIFI_DISCONNECT_REASON_EXCEEDED_TXOP 35
#define WIFI_DISCONNECT_REASON_STA_LEAVING 36
#define WIFI_DISCONNECT_REASON_END_BA 37
#define WIFI_DISCONNECT_REASON_UNKNOWN_BA 38
#define WIFI_DISCONNECT_REASON_TIMEOUT 39
#define WIFI_DISCONNECT_REASON_PEER_INITIATED 46
#define WIFI_DISCONNECT_REASON_AP_INITIATED 47
#define WIFI_DISCONNECT_REASON_INVALID_FT_ACTION_FRAME_COUNT 48
#define WIFI_DISCONNECT_REASON_INVALID_PMKID 49
#define WIFI_DISCONNECT_REASON_INVALID_MDE 50
#define WIFI_DISCONNECT_REASON_INVALID_FTE 51
#define WIFI_DISCONNECT_REASON_TX_LINK_EST_FAILED 67
#define WIFI_DISCONNECT_REASON_ALTERATIVE_CHANNEL_OCCUPIED 68

// Espressif-specific reason codes (200+)
#define WIFI_DISCONNECT_REASON_BEACON_TIMEOUT 200
#define WIFI_DISCONNECT_REASON_NO_AP_FOUND 201
#define WIFI_DISCONNECT_REASON_AUTH_FAIL 202
#define WIFI_DISCONNECT_REASON_ASSOC_FAIL 203
#define WIFI_DISCONNECT_REASON_HANDSHAKE_TIMEOUT 204
#define WIFI_DISCONNECT_REASON_CONNECTION_FAIL 205
#define WIFI_DISCONNECT_REASON_AP_TSF_RESET 206
#define WIFI_DISCONNECT_REASON_ROAMING 207
#define WIFI_DISCONNECT_REASON_ASSOC_COMEBACK_TIME_TOO_LONG 208
#define WIFI_DISCONNECT_REASON_SA_QUERY_TIMEOUT 209
#define WIFI_DISCONNECT_REASON_NO_AP_FOUND_SECURITY 210
#define WIFI_DISCONNECT_REASON_NO_AP_FOUND_AUTHMODE 211
#define WIFI_DISCONNECT_REASON_NO_AP_FOUND_RSSI 212

// WiFi Disconnect Reason Code Descriptions
#define WIFI_DISCONNECT_REASON_STR_UNSPECIFIED "UNSPECIFIED - Internal failure (memory, TX failure, or remote side)"
#define WIFI_DISCONNECT_REASON_STR_AUTH_EXPIRE "AUTH_EXPIRE - Previous authentication no longer valid"
#define WIFI_DISCONNECT_REASON_STR_ASSOC_EXPIRE "ASSOC_EXPIRE - Association timed out"
#define WIFI_DISCONNECT_REASON_STR_DISASSOC_INACTIVITY "DISASSOC_INACTIVITY - Disassociation due to inactivity"
#define WIFI_DISCONNECT_REASON_STR_ASSOC_TOOMANY "ASSOC_TOOMANY - AP unable to handle all associated stations"
#define WIFI_DISCONNECT_REASON_STR_CLASS2_FRAME_FROM_NONAUTH_STA "CLASS2_FRAME_FROM_NONAUTH_STA - Class-2 frame from non-authenticated station"
#define WIFI_DISCONNECT_REASON_STR_CLASS3_FRAME_FROM_NONASSOC_STA "CLASS3_FRAME_FROM_NONASSOC_STA - Class-3 frame from non-associated station"
#define WIFI_DISCONNECT_REASON_STR_ASSOC_LEAVE "ASSOC_LEAVE - Station leaving BSS"
#define WIFI_DISCONNECT_REASON_STR_ASSOC_NOT_AUTHED "ASSOC_NOT_AUTHED - Station not authenticated"
#define WIFI_DISCONNECT_REASON_STR_DISASSOC_PWRCAP_BAD "DISASSOC_PWRCAP_BAD - Power capability unacceptable"
#define WIFI_DISCONNECT_REASON_STR_DISASSOC_SUPCHAN_BAD "DISASSOC_SUPCHAN_BAD - Supported channels unacceptable"
#define WIFI_DISCONNECT_REASON_STR_BSS_TRANSITION_DISASSOC "BSS_TRANSITION_DISASSOC - AP wants station to move to another AP"
#define WIFI_DISCONNECT_REASON_STR_IE_INVALID "IE_INVALID - Invalid element content"
#define WIFI_DISCONNECT_REASON_STR_MIC_FAILURE "MIC_FAILURE - Message integrity code failure"
#define WIFI_DISCONNECT_REASON_STR_4WAY_HANDSHAKE_TIMEOUT "4WAY_HANDSHAKE_TIMEOUT - Four-way handshake timeout (wrong password)"
#define WIFI_DISCONNECT_REASON_STR_GROUP_KEY_UPDATE_TIMEOUT "GROUP_KEY_UPDATE_TIMEOUT - Group-Key handshake timeout"
#define WIFI_DISCONNECT_REASON_STR_IE_IN_4WAY_DIFFERS "IE_IN_4WAY_DIFFERS - Four-way handshake IE differs from association"
#define WIFI_DISCONNECT_REASON_STR_GROUP_CIPHER_INVALID "GROUP_CIPHER_INVALID - Invalid group cipher"
#define WIFI_DISCONNECT_REASON_STR_PAIRWISE_CIPHER_INVALID "PAIRWISE_CIPHER_INVALID - Invalid pairwise cipher"
#define WIFI_DISCONNECT_REASON_STR_AKMP_INVALID "AKMP_INVALID - Invalid AKMP"
#define WIFI_DISCONNECT_REASON_STR_UNSUPP_RSN_IE_VERSION "UNSUPP_RSN_IE_VERSION - Unsupported RSN IE version"
#define WIFI_DISCONNECT_REASON_STR_INVALID_RSN_IE_CAP "INVALID_RSN_IE_CAP - Invalid RSNE capabilities"
#define WIFI_DISCONNECT_REASON_STR_IEEE_802_1X_AUTH_FAILED "IEEE_802_1X_AUTH_FAILED - IEEE 802.1X authentication failed"
#define WIFI_DISCONNECT_REASON_STR_CIPHER_SUITE_REJECTED "CIPHER_SUITE_REJECTED - Cipher suite rejected due to security policies"
#define WIFI_DISCONNECT_REASON_STR_TDLS_PEER_UNREACHABLE "TDLS_PEER_UNREACHABLE - TDLS peer unreachable"
#define WIFI_DISCONNECT_REASON_STR_TDLS_UNSPECIFIED "TDLS_UNSPECIFIED - TDLS teardown for unspecified reason"
#define WIFI_DISCONNECT_REASON_STR_SSP_REQUESTED_DISASSOC "SSP_REQUESTED_DISASSOC - Session terminated by SSP request"
#define WIFI_DISCONNECT_REASON_STR_NO_SSP_ROAMING_AGREEMENT "NO_SSP_ROAMING_AGREEMENT - Lack of SSP roaming agreement"
#define WIFI_DISCONNECT_REASON_STR_BAD_CIPHER_OR_AKM "BAD_CIPHER_OR_AKM - Requested service rejected due to cipher/AKM requirements"
#define WIFI_DISCONNECT_REASON_STR_NOT_AUTHORIZED_THIS_LOCATION "NOT_AUTHORIZED_THIS_LOCATION - Service not authorized in this location"
#define WIFI_DISCONNECT_REASON_STR_SERVICE_CHANGE_PRECLUDES_TS "SERVICE_CHANGE_PRECLUDES_TS - QoS bandwidth insufficient"
#define WIFI_DISCONNECT_REASON_STR_UNSPECIFIED_QOS "UNSPECIFIED_QOS - Unspecified QoS-related reason"
#define WIFI_DISCONNECT_REASON_STR_NOT_ENOUGH_BANDWIDTH "NOT_ENOUGH_BANDWIDTH - QoS AP lacks sufficient bandwidth"
#define WIFI_DISCONNECT_REASON_STR_MISSING_ACKS "MISSING_ACKS - Excessive unacknowledged frames"
#define WIFI_DISCONNECT_REASON_STR_EXCEEDED_TXOP "EXCEEDED_TXOP - Station transmitting outside TXOP limits"
#define WIFI_DISCONNECT_REASON_STR_STA_LEAVING "STA_LEAVING - Station leaving BSS or resetting"
#define WIFI_DISCONNECT_REASON_STR_END_BA "END_BA - Station no longer using stream/session"
#define WIFI_DISCONNECT_REASON_STR_UNKNOWN_BA "UNKNOWN_BA - Frames using incomplete mechanism"
#define WIFI_DISCONNECT_REASON_STR_TIMEOUT "TIMEOUT - Requested from peer due to timeout"
#define WIFI_DISCONNECT_REASON_STR_PEER_INITIATED "PEER_INITIATED - Authorized access limit reached"
#define WIFI_DISCONNECT_REASON_STR_AP_INITIATED "AP_INITIATED - External service requirements"
#define WIFI_DISCONNECT_REASON_STR_INVALID_FT_ACTION_FRAME_COUNT "INVALID_FT_ACTION_FRAME_COUNT - Invalid FT Action frame count"
#define WIFI_DISCONNECT_REASON_STR_INVALID_PMKID "INVALID_PMKID - Invalid pairwise master key identifier"
#define WIFI_DISCONNECT_REASON_STR_INVALID_MDE "INVALID_MDE - Invalid MDE"
#define WIFI_DISCONNECT_REASON_STR_INVALID_FTE "INVALID_FTE - Invalid FTE"
#define WIFI_DISCONNECT_REASON_STR_TX_LINK_EST_FAILED "TX_LINK_EST_FAILED - Transmission link establishment failed"
#define WIFI_DISCONNECT_REASON_STR_ALTERATIVE_CHANNEL_OCCUPIED "ALTERATIVE_CHANNEL_OCCUPIED - Alternative channel occupied"
#define WIFI_DISCONNECT_REASON_STR_BEACON_TIMEOUT "BEACON_TIMEOUT - Station lost N beacons continuously"
#define WIFI_DISCONNECT_REASON_STR_NO_AP_FOUND "NO_AP_FOUND - Station failed to scan target AP (wrong password possible)"
#define WIFI_DISCONNECT_REASON_STR_AUTH_FAIL "AUTH_FAIL - Authentication failed (not timeout)"
#define WIFI_DISCONNECT_REASON_STR_ASSOC_FAIL "ASSOC_FAIL - Association failed (not inactivity/too many)"
#define WIFI_DISCONNECT_REASON_STR_HANDSHAKE_TIMEOUT "HANDSHAKE_TIMEOUT - Four-way handshake failed (wrong password)"
#define WIFI_DISCONNECT_REASON_STR_CONNECTION_FAIL "CONNECTION_FAIL - Connection to AP failed"
#define WIFI_DISCONNECT_REASON_STR_AP_TSF_RESET "AP_TSF_RESET - AP's TSF reset caused disconnection"
#define WIFI_DISCONNECT_REASON_STR_ROAMING "ROAMING - Station roaming to another AP"
#define WIFI_DISCONNECT_REASON_STR_ASSOC_COMEBACK_TIME_TOO_LONG "ASSOC_COMEBACK_TIME_TOO_LONG - Association comeback time too high"
#define WIFI_DISCONNECT_REASON_STR_SA_QUERY_TIMEOUT "SA_QUERY_TIMEOUT - Security Association query timeout"
#define WIFI_DISCONNECT_REASON_STR_NO_AP_FOUND_SECURITY "NO_AP_FOUND_SECURITY - AP found but security config incompatible"
#define WIFI_DISCONNECT_REASON_STR_NO_AP_FOUND_AUTHMODE "NO_AP_FOUND_AUTHMODE - AP found but authmode threshold not met"
#define WIFI_DISCONNECT_REASON_STR_NO_AP_FOUND_RSSI "NO_AP_FOUND_RSSI - AP found but RSSI threshold not met"
#define WIFI_DISCONNECT_REASON_STR_UNKNOWN "UNKNOWN - Unknown disconnect reason"

// Macro to get disconnect reason string from reason code
#define WIFI_DISCONNECT_REASON_TO_STRING(reason)                                                                                     \
    ((reason) == WIFI_DISCONNECT_REASON_UNSPECIFIED                      ? WIFI_DISCONNECT_REASON_STR_UNSPECIFIED                    \
     : (reason) == WIFI_DISCONNECT_REASON_AUTH_EXPIRE                    ? WIFI_DISCONNECT_REASON_STR_AUTH_EXPIRE                    \
     : (reason) == WIFI_DISCONNECT_REASON_ASSOC_EXPIRE                   ? WIFI_DISCONNECT_REASON_STR_ASSOC_EXPIRE                   \
     : (reason) == WIFI_DISCONNECT_REASON_DISASSOC_INACTIVITY            ? WIFI_DISCONNECT_REASON_STR_DISASSOC_INACTIVITY            \
     : (reason) == WIFI_DISCONNECT_REASON_ASSOC_TOOMANY                  ? WIFI_DISCONNECT_REASON_STR_ASSOC_TOOMANY                  \
     : (reason) == WIFI_DISCONNECT_REASON_CLASS2_FRAME_FROM_NONAUTH_STA  ? WIFI_DISCONNECT_REASON_STR_CLASS2_FRAME_FROM_NONAUTH_STA  \
     : (reason) == WIFI_DISCONNECT_REASON_CLASS3_FRAME_FROM_NONASSOC_STA ? WIFI_DISCONNECT_REASON_STR_CLASS3_FRAME_FROM_NONASSOC_STA \
     : (reason) == WIFI_DISCONNECT_REASON_ASSOC_LEAVE                    ? WIFI_DISCONNECT_REASON_STR_ASSOC_LEAVE                    \
     : (reason) == WIFI_DISCONNECT_REASON_ASSOC_NOT_AUTHED               ? WIFI_DISCONNECT_REASON_STR_ASSOC_NOT_AUTHED               \
     : (reason) == WIFI_DISCONNECT_REASON_DISASSOC_PWRCAP_BAD            ? WIFI_DISCONNECT_REASON_STR_DISASSOC_PWRCAP_BAD            \
     : (reason) == WIFI_DISCONNECT_REASON_DISASSOC_SUPCHAN_BAD           ? WIFI_DISCONNECT_REASON_STR_DISASSOC_SUPCHAN_BAD           \
     : (reason) == WIFI_DISCONNECT_REASON_BSS_TRANSITION_DISASSOC        ? WIFI_DISCONNECT_REASON_STR_BSS_TRANSITION_DISASSOC        \
     : (reason) == WIFI_DISCONNECT_REASON_IE_INVALID                     ? WIFI_DISCONNECT_REASON_STR_IE_INVALID                     \
     : (reason) == WIFI_DISCONNECT_REASON_MIC_FAILURE                    ? WIFI_DISCONNECT_REASON_STR_MIC_FAILURE                    \
     : (reason) == WIFI_DISCONNECT_REASON_4WAY_HANDSHAKE_TIMEOUT         ? WIFI_DISCONNECT_REASON_STR_4WAY_HANDSHAKE_TIMEOUT         \
     : (reason) == WIFI_DISCONNECT_REASON_GROUP_KEY_UPDATE_TIMEOUT       ? WIFI_DISCONNECT_REASON_STR_GROUP_KEY_UPDATE_TIMEOUT       \
     : (reason) == WIFI_DISCONNECT_REASON_IE_IN_4WAY_DIFFERS             ? WIFI_DISCONNECT_REASON_STR_IE_IN_4WAY_DIFFERS             \
     : (reason) == WIFI_DISCONNECT_REASON_GROUP_CIPHER_INVALID           ? WIFI_DISCONNECT_REASON_STR_GROUP_CIPHER_INVALID           \
     : (reason) == WIFI_DISCONNECT_REASON_PAIRWISE_CIPHER_INVALID        ? WIFI_DISCONNECT_REASON_STR_PAIRWISE_CIPHER_INVALID        \
     : (reason) == WIFI_DISCONNECT_REASON_AKMP_INVALID                   ? WIFI_DISCONNECT_REASON_STR_AKMP_INVALID                   \
     : (reason) == WIFI_DISCONNECT_REASON_UNSUPP_RSN_IE_VERSION          ? WIFI_DISCONNECT_REASON_STR_UNSUPP_RSN_IE_VERSION          \
     : (reason) == WIFI_DISCONNECT_REASON_INVALID_RSN_IE_CAP             ? WIFI_DISCONNECT_REASON_STR_INVALID_RSN_IE_CAP             \
     : (reason) == WIFI_DISCONNECT_REASON_IEEE_802_1X_AUTH_FAILED        ? WIFI_DISCONNECT_REASON_STR_IEEE_802_1X_AUTH_FAILED        \
     : (reason) == WIFI_DISCONNECT_REASON_CIPHER_SUITE_REJECTED          ? WIFI_DISCONNECT_REASON_STR_CIPHER_SUITE_REJECTED          \
     : (reason) == WIFI_DISCONNECT_REASON_TDLS_PEER_UNREACHABLE          ? WIFI_DISCONNECT_REASON_STR_TDLS_PEER_UNREACHABLE          \
     : (reason) == WIFI_DISCONNECT_REASON_TDLS_UNSPECIFIED               ? WIFI_DISCONNECT_REASON_STR_TDLS_UNSPECIFIED               \
     : (reason) == WIFI_DISCONNECT_REASON_SSP_REQUESTED_DISASSOC         ? WIFI_DISCONNECT_REASON_STR_SSP_REQUESTED_DISASSOC         \
     : (reason) == WIFI_DISCONNECT_REASON_NO_SSP_ROAMING_AGREEMENT       ? WIFI_DISCONNECT_REASON_STR_NO_SSP_ROAMING_AGREEMENT       \
     : (reason) == WIFI_DISCONNECT_REASON_BAD_CIPHER_OR_AKM              ? WIFI_DISCONNECT_REASON_STR_BAD_CIPHER_OR_AKM              \
     : (reason) == WIFI_DISCONNECT_REASON_NOT_AUTHORIZED_THIS_LOCATION   ? WIFI_DISCONNECT_REASON_STR_NOT_AUTHORIZED_THIS_LOCATION   \
     : (reason) == WIFI_DISCONNECT_REASON_SERVICE_CHANGE_PRECLUDES_TS    ? WIFI_DISCONNECT_REASON_STR_SERVICE_CHANGE_PRECLUDES_TS    \
     : (reason) == WIFI_DISCONNECT_REASON_UNSPECIFIED_QOS                ? WIFI_DISCONNECT_REASON_STR_UNSPECIFIED_QOS                \
     : (reason) == WIFI_DISCONNECT_REASON_NOT_ENOUGH_BANDWIDTH           ? WIFI_DISCONNECT_REASON_STR_NOT_ENOUGH_BANDWIDTH           \
     : (reason) == WIFI_DISCONNECT_REASON_MISSING_ACKS                   ? WIFI_DISCONNECT_REASON_STR_MISSING_ACKS                   \
     : (reason) == WIFI_DISCONNECT_REASON_EXCEEDED_TXOP                  ? WIFI_DISCONNECT_REASON_STR_EXCEEDED_TXOP                  \
     : (reason) == WIFI_DISCONNECT_REASON_STA_LEAVING                    ? WIFI_DISCONNECT_REASON_STR_STA_LEAVING                    \
     : (reason) == WIFI_DISCONNECT_REASON_END_BA                         ? WIFI_DISCONNECT_REASON_STR_END_BA                         \
     : (reason) == WIFI_DISCONNECT_REASON_UNKNOWN_BA                     ? WIFI_DISCONNECT_REASON_STR_UNKNOWN_BA                     \
     : (reason) == WIFI_DISCONNECT_REASON_TIMEOUT                        ? WIFI_DISCONNECT_REASON_STR_TIMEOUT                        \
     : (reason) == WIFI_DISCONNECT_REASON_PEER_INITIATED                 ? WIFI_DISCONNECT_REASON_STR_PEER_INITIATED                 \
     : (reason) == WIFI_DISCONNECT_REASON_AP_INITIATED                   ? WIFI_DISCONNECT_REASON_STR_AP_INITIATED                   \
     : (reason) == WIFI_DISCONNECT_REASON_INVALID_FT_ACTION_FRAME_COUNT  ? WIFI_DISCONNECT_REASON_STR_INVALID_FT_ACTION_FRAME_COUNT  \
     : (reason) == WIFI_DISCONNECT_REASON_INVALID_PMKID                  ? WIFI_DISCONNECT_REASON_STR_INVALID_PMKID                  \
     : (reason) == WIFI_DISCONNECT_REASON_INVALID_MDE                    ? WIFI_DISCONNECT_REASON_STR_INVALID_MDE                    \
     : (reason) == WIFI_DISCONNECT_REASON_INVALID_FTE                    ? WIFI_DISCONNECT_REASON_STR_INVALID_FTE                    \
     : (reason) == WIFI_DISCONNECT_REASON_TX_LINK_EST_FAILED             ? WIFI_DISCONNECT_REASON_STR_TX_LINK_EST_FAILED             \
     : (reason) == WIFI_DISCONNECT_REASON_ALTERATIVE_CHANNEL_OCCUPIED    ? WIFI_DISCONNECT_REASON_STR_ALTERATIVE_CHANNEL_OCCUPIED    \
     : (reason) == WIFI_DISCONNECT_REASON_BEACON_TIMEOUT                 ? WIFI_DISCONNECT_REASON_STR_BEACON_TIMEOUT                 \
     : (reason) == WIFI_DISCONNECT_REASON_NO_AP_FOUND                    ? WIFI_DISCONNECT_REASON_STR_NO_AP_FOUND                    \
     : (reason) == WIFI_DISCONNECT_REASON_AUTH_FAIL                      ? WIFI_DISCONNECT_REASON_STR_AUTH_FAIL                      \
     : (reason) == WIFI_DISCONNECT_REASON_ASSOC_FAIL                     ? WIFI_DISCONNECT_REASON_STR_ASSOC_FAIL                     \
     : (reason) == WIFI_DISCONNECT_REASON_HANDSHAKE_TIMEOUT              ? WIFI_DISCONNECT_REASON_STR_HANDSHAKE_TIMEOUT              \
     : (reason) == WIFI_DISCONNECT_REASON_CONNECTION_FAIL                ? WIFI_DISCONNECT_REASON_STR_CONNECTION_FAIL                \
     : (reason) == WIFI_DISCONNECT_REASON_AP_TSF_RESET                   ? WIFI_DISCONNECT_REASON_STR_AP_TSF_RESET                   \
     : (reason) == WIFI_DISCONNECT_REASON_ROAMING                        ? WIFI_DISCONNECT_REASON_STR_ROAMING                        \
     : (reason) == WIFI_DISCONNECT_REASON_ASSOC_COMEBACK_TIME_TOO_LONG   ? WIFI_DISCONNECT_REASON_STR_ASSOC_COMEBACK_TIME_TOO_LONG   \
     : (reason) == WIFI_DISCONNECT_REASON_SA_QUERY_TIMEOUT               ? WIFI_DISCONNECT_REASON_STR_SA_QUERY_TIMEOUT               \
     : (reason) == WIFI_DISCONNECT_REASON_NO_AP_FOUND_SECURITY           ? WIFI_DISCONNECT_REASON_STR_NO_AP_FOUND_SECURITY           \
     : (reason) == WIFI_DISCONNECT_REASON_NO_AP_FOUND_AUTHMODE           ? WIFI_DISCONNECT_REASON_STR_NO_AP_FOUND_AUTHMODE           \
     : (reason) == WIFI_DISCONNECT_REASON_NO_AP_FOUND_RSSI               ? WIFI_DISCONNECT_REASON_STR_NO_AP_FOUND_RSSI               \
                                                                         : WIFI_DISCONNECT_REASON_STR_UNKNOWN)

typedef enum {
    WIFI_MODE_STATE_STOPPED = 0,
    WIFI_MODE_STATE_STARTED,
    WIFI_MODE_STATE_CONNECTING,
    WIFI_MODE_STATE_CONNECTED,
    WIFI_MODE_STATE_DISCONNECTED,
    WIFI_MODE_STATE_FAILED,
} wifi_mode_state_e;

typedef enum {
    WIFI_TX_POWER_MIN = 0,     // 0 dBm
    WIFI_TX_POWER_LOW = 1,     // 5 dBm
    WIFI_TX_POWER_MEDIUM = 2,  // 10 dBm (default)
    WIFI_TX_POWER_HIGH = 3,    // 15 dBm
    WIFI_TX_POWER_MAX = 4      // 20 dBm
} wifi_tx_power_level_t;

typedef struct wifi_s {
    // Configuration loaded from settings
    bool sta_enabled;
    char sta_ssid[WIFI_SSID_MAX_LEN];
    char sta_pwd[WIFI_PWD_MAX_LEN];
    bool ap_enabled;
    char ap_ssid[WIFI_SSID_MAX_LEN];
    char ap_pwd[WIFI_PWD_MAX_LEN];
    uint8_t ap_channel;
    bool espnow_enabled;
    bool start_ap_if_sta_failed;  // ADDED: New setting to control AP failover
    bool captive_portal_enabled;
    void *captive_portal_http_server;  // HTTP server instance for captive portal

    captive_portal_t captive_portal;

    // ESP-NOW MAC address is now configured per logical port; no global setting

    // Internal state
    wifi_mode_state_e sta_state;
    wifi_mode_state_e ap_state;
    uint32_t ip;
    int sta_retry_count;             // ADDED: Counter for STA connection retries
    int8_t last_rssi;                // ADDED: Last known RSSI value
    uint32_t connection_start_time;  // ADDED: Connection start timestamp
    uint8_t reset_attempts;          // ADDED: Counter for WiFi stack reset attempts
    bool restart_needed;             // ADDED: Flag to indicate WiFi restart is needed

    /* removed: legacy observer channel */
    wifi_tx_power_level_t tx_power_level;
    int8_t tx_power_dbm;
} wifi_t;

/**
 * @brief Initializes the WiFi module, drivers, and event loops.
 * Performs one-time setup and loads the initial configuration.
 * @param wifi Pointer to the wifi_t instance.
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t wifi_init(wifi_t *wifi);

/**
 * @brief Starts WiFi services (STA and/or AP) based on loaded configuration.
 * @param wifi Pointer to the wifi_t instance.
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t wifi_start(wifi_t *wifi);

/**
 * @brief Stops WiFi services (STA and AP). Does not affect ESP-NOW.
 * @param wifi Pointer to the wifi_t instance.
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t wifi_stop(wifi_t *wifi);

/**
 * @brief Initializes and starts the ESP-NOW protocol.
 * @param wifi Pointer to the wifi_t instance.
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t wifi_start_espnow(wifi_t *wifi);

/**
 * @brief Deinitializes the ESP-NOW protocol.
 * @param wifi Pointer to the wifi_t instance.
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t wifi_stop_espnow(wifi_t *wifi);

/**
 * @brief Sends data via ESP-NOW to the configured broadcast address.
 * @param wifi Pointer to the wifi_t instance.
 * @param data Pointer to the data to send.
 * @param len Length of the data to send.
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t wifi_send_espnow(wifi_t *wifi, const uint8_t broadcast_addr[6], const void *data, size_t len);

// ESP-NOW peer management helpers
esp_err_t wifi_ensure_espnow_peer(wifi_t *wifi, const uint8_t peer_mac[6]);
esp_err_t wifi_remove_espnow_peer(wifi_t *wifi, const uint8_t peer_mac[6]);
esp_err_t wifi_get_device_mac_current(wifi_interface_t ifx, uint8_t out[6]);

esp_err_t wifi_start_sta(wifi_t *wifi);
esp_err_t wifi_start_ap(wifi_t *wifi);
esp_err_t wifi_stop_sta(wifi_t *wifi);
esp_err_t wifi_stop_ap(wifi_t *wifi);

// WiFi TX Power Control
esp_err_t wifi_set_tx_power_level(wifi_t *wifi, wifi_tx_power_level_t level);
wifi_tx_power_level_t wifi_get_tx_power_level(wifi_t *wifi);
int8_t wifi_get_rssi(wifi_t *wifi);

/**
 * @brief Loads WiFi configuration from settings into the wifi_t structure.
 * This is an internal function used for configuration reloading.
 * @param wifi Pointer to the wifi_t instance.
 */
void wifi_load_config(wifi_t *wifi);

// WiFi reset and recovery
#define WIFI_MAX_RESET_ATTEMPTS 3

// WiFi reset and recovery functions
esp_err_t wifi_reset_stack(wifi_t *wifi);
esp_err_t wifi_force_reconnect(wifi_t *wifi);
bool wifi_should_reset_on_timeout(wifi_t *wifi, uint32_t timeout_seconds);

// ADDED: Channel coordination functions
/**
 * @brief Determines the optimal AP channel for APSTA mode to avoid conflicts
 * @param wifi Pointer to the wifi_t instance
 * @return The channel number to use for AP (0 for auto, 1-13 for fixed)
 */
uint8_t wifi_get_optimal_ap_channel(wifi_t *wifi);

/**
 * @brief Checks if a channel is suitable for STA to avoid AP conflicts
 * @param wifi Pointer to the wifi_t instance
 * @param channel The channel to check
 * @return true if channel is suitable, false if it conflicts with AP
 */
bool wifi_is_channel_suitable_for_sta(wifi_t *wifi, uint8_t channel);

// Captive portal HTTP server integration
void wifi_set_captive_portal_http_server(wifi_t *wifi, void *http_server);

#ifdef __cplusplus
}
#endif

#endif  // defined(USE_WIFI)
