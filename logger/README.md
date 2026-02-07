# Logger (ESP32-S3) - STM32 Frame Receiver

This firmware runs on ESP32-S3 and records binary `logger_frame_t` frames produced by the STM32 firmware over an SPI link.
It does **not** talk to the IMU directly and does not perform any sensor processing.

## Control & Indication

### Button Commands

| Press Type          | Action                                  |
| :------------------ | :-------------------------------------- |
| **Double Press**    | Toggle **Logging** (Start/Stop)         |
| **Triple Press**    | Toggle **RPC Server** (WiFi + JSON-RPC) |
| **Quadruple Press** | **Reboot** Device                       |
| **Long Press** (1s) | Toggle **Online Mode** (WiFi / USB MSC) |

### Indication Legend

| Mode              | RGB LED        | External LED     | Description                             |
| :---------------- | :------------- | :--------------- | :-------------------------------------- |
| **Idle**          | Green (Solid)  | On (Solid)       | System ready, waiting for commands      |
| **Logging**       | Orange (Solid) | Blink (1 Hz)     | Recording data to SD card               |
| **Online (WiFi)** | Deep Violet    | 3 Blinks + Pause | WiFi Access Point / Station active      |
| **Download**      | Blue (Solid)   | 2 Blinks + Pause | USB File Download / Web Download active |
| **Error**         | Red (Solid)    | Fast Blink       | System error (check logs)               |

## Key Settings

- STM32 link SPI clock: `logger/include/target.h` -> `LINK_SPI_FREQ_HZ`
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

## Web UI Build

The `web-ui/` folder contains a Preact-based configuration interface for WiFi settings.

### Build Modes

| Command           | Mode        | Output                         | Use Case                        |
| ----------------- | ----------- | ------------------------------ | ------------------------------- |
| `pnpm build`      | Development | `dist/`                        | Testing with device at local IP |
| `pnpm build:prod` | Production  | `../main/assets/index.html.gz` | Embedding in firmware           |

### Environment Files

- `.env` - Development settings with `VITE_API_BASE_URL=http://192.168.4.1`
- `.env.prod` - Production settings with empty API URL (relative paths)

### Development Workflow

```bash
cd logger/web-ui
pnpm install

# Development build - connect to device over WiFi
pnpm build

# Production build for embedding
pnpm build:prod
pnpm embed  # Generate C arrays in main/generated_assets.c
```

### How It Works

1. **Development mode** (`pnpm build`): Uses `.env` with `VITE_API_BASE_URL` pointing to the device IP. Run `pnpm dev` and access the UI in browser while the device serves the RPC API.

2. **Production mode** (`pnpm build:prod`): Creates a single-file gzipped HTML (`index.html.gz`) that gets embedded into the firmware. The `embed` script converts the asset to C arrays.
