# Logger (ESP32) — STM32 Frame Receiver

This firmware runs on ESP32-S3 and records binary `logger_frame_t` frames produced by the STM32 firmware over an SPI link.
It does **not** talk to the IMU directly and does not perform any sensor processing.

## Key Settings

- STM32 link SPI clock: `logger/include/target.h` → `LINK_SPI_FREQ_HZ`
- Link pins / board mapping: `logger/include/target.h` (`LINK_SPI_*`, `LINK_INT_GPIO`, `STM_RESET_GPIO`)

## Windows USB Drivers (Zadig)

Some boards expose multiple USB interfaces. On Windows you may need Zadig to install the correct driver per interface.

### ESP32-S3
- For "USB JTAG/serial debug unit (Interface 0)": keep `USB Serial (CDC)` (COM port).
- For "USB JTAG/serial debug unit (Interface 2)": install `WinUSB` (or `libusbK`).

## Build / Flash (PlatformIO)

- Build: `pio run -e esp32s3_logger` (or your target environment)
- Flash: `pio run -e esp32s3_logger -t upload`
- Monitor: `pio device monitor`
