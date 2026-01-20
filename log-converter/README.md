# 3P Logger Binary to CSV Converter

Converts binary log files from the 3P Logger to human-readable CSV format.

## Building

### Windows (Visual Studio)

Open **Developer Command Prompt for VS** and run:

```cmd
cd converter
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
log_converter data_0.dat ./output
```

Convert all `.dat` files in a folder:

```cmd
log_converter ./logs ./output
```

## Output Files

For each input file, creates a subfolder with 4 CSV files:

| File                 | Description                         |
| -------------------- | ----------------------------------- |
| `config.csv`         | Logger configuration parameters     |
| `adc_data.csv`       | ADC piezo sensor data (100 kHz)     |
| `imu_data.csv`       | IMU gyro/accel data with timestamps |
| `mavlink_events.csv` | MAVLink telemetry events            |

## Binary Format

- **Header**: 32 bytes (`logger_config_t`)
- **Frames**: 852 bytes each (`logger_frame_t`)
  - 256 ADC samples (512 bytes)
  - 0-20 IMU samples (320 bytes)
  - MAVLink data (10 bytes)
  - CRC (2 bytes)
