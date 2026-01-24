# 3P Logger Binary to CSV Converter

Converts binary log files from the 3P Logger to human-readable CSV format.

## Building

### Windows (Visual Studio)

Open **Developer Command Prompt for VS** and run:

```cmd
cd log-converter
cl /O2 /W4 log_converter.c log_parser.c csv_writer.c
```

### Windows (MinGW/GCC)

```cmd
gcc -Wall -Wextra -O2 -std=c99 -o log_converter.exe log_converter.c log_parser.c csv_writer.c
```

### Linux/macOS

```bash
make
```

Or directly:

```bash
gcc -Wall -Wextra -O2 -std=c99 -o log_converter log_converter.c log_parser.c csv_writer.c
```

## Usage

```
log_converter <input> <output_folder>

Arguments:
  input          - Binary log file (.dat) OR folder containing .dat files
  output_folder  - Destination folder for converted CSV files
```

### Examples

Convert a single file:

```cmd
./log_converter.exe data_0.dat ./output
```

Convert all `.dat` files in a folder:

```cmd
./log_converter.exe ./logs ./output
```

## Output Files

For each input file, creates a subfolder with 4 CSV files:

| File                 | Description                         |
| -------------------- | ----------------------------------- |
| `config.csv`         | Logger configuration parameters     |
| `adc_data.csv`       | ADC piezo sensor data (100 kHz)     |
| `imu_data.csv`       | IMU gyro/accel data with timestamps |
| `mavlink_events.csv` | MAVLink telemetry events            |

## Timestamps

- Firmware stores timestamps as a 32-bit TIM6 tick counter at `config.adc_sample_rate_khz` (e.g. 100 kHz → 10 µs per tick).
- CSV outputs use `timestamp_us` / `frame_timestamp_us` in **microseconds** (printed as 64-bit integers).
- `adc_timestamp` is captured in the ADC DMA callback, so it is closer to the end of a 256-sample block; the converter reconstructs per-sample timestamps from this value and enforces non-decreasing timestamps across blocks.

## Binary Format

- **Header**: 64 bytes (`log_config_t`)
- **Frames**: 852 bytes each (`log_frame_t`)
  - 256 ADC samples (512 bytes)
  - 0-20 IMU samples (320 bytes)
  - MAVLink data (10 bytes)
  - CRC (2 bytes)
