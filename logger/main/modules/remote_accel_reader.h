#pragma once

#include <stdbool.h>

void remote_accel_reader_init(void);

// Restart the STM32 and re-run the config handshake (CMD 42) before starting a new logging session.
// This call is blocking (includes ~100 ms delay while STM32 reboots).
void remote_accel_reader_restart_session(void);

#include "logger_frame_link.h"

// Returns true only when STM32 acknowledged CMD45 phase transition (INT high)
// and the full config payload was transmitted successfully.
bool remote_accel_reader_send_config(const logger_config_t *cfg);

// Request a fresh config read (CMD 42) without resetting STM32.
// Useful after sending a new config (CMD 45) so the log-file header captures both
// writable fields (from Web/NVS) and read-only/runtime/register snapshot fields (from STM32).
void remote_accel_reader_request_config_refresh(void);
