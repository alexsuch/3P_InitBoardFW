#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "app_state.h"

typedef enum {
    APP_CMD_NONE,
    APP_CMD_SET_MODE_IDLE,
    APP_CMD_SET_MODE_LOGGING,
    APP_CMD_SET_MODE_ERROR,
    APP_CMD_RECONFIGURE_WIFI,
    APP_CMD_ONLINE_MODE_START,
    APP_CMD_ONLINE_MODE_STOP,
    APP_CMD_WEB_SERVER_START,
    APP_CMD_WEB_SERVER_STOP,
    APP_CMD_STOP_LOGGING_AND_FREE_MEMORY,
    APP_CMD_REBOOT_TO_BOOT,
    APP_CMD_REBOOT,
    APP_CMD_START_USB_FILE_SERVER,
    APP_CMD_STOP_USB_FILE_SERVER,
} app_command_id_e;

typedef struct {
    app_command_id_e id;
    union {
        uint8_t reserved;  // Placeholder to satisfy C compilers when no payload is required
    } payload;

} app_command_t;

typedef enum {
    APP_EVENT_NONE,
    APP_EVENT_WIFI_STA_CONNECTING,
    APP_EVENT_WIFI_STA_CONNECTED,
    APP_EVENT_WIFI_STA_DISCONNECTED,
    APP_EVENT_WIFI_STA_CONNECT_FAILED,
    APP_EVENT_WIFI_AP_STARTED,
    APP_EVENT_WIFI_AP_STOPPED,
    APP_EVENT_WIFI_DISCONNECTED,
} app_event_id_e;

typedef struct {
    app_event_id_e id;

    union {
        int error_code;
        bool success_status;
        struct {
            uint32_t ip;
        } wifi_connected;
    } payload;
} app_event_t;
