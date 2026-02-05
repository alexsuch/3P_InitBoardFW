#pragma once

void remote_accel_reader_init(void);

// Restart the STM32 and re-run the config handshake (CMD 42) before starting a new logging session.
// This call is blocking (includes ~100 ms delay while STM32 reboots).
void remote_accel_reader_restart_session(void);

