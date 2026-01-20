#pragma once

/**
 * @file logger_frame_link.h
 * @brief Phase 5 SPI Communication - LogFrame_t structure for STM32→ESP32 transmission
 *
 * Defines the 640-byte LogFrame_t structure with ADC+IMU merged data.
 * This replaces the previous 50-byte accel_link_frame_t for enhanced logging capability.
 *
 * Frame Layout (852 bytes):
 *   - magic (2B):          0x5A5A
 *   - n_imu (2B):          Number of valid IMU samples (0-20)
 *   - adc_timestamp (4B):  Timestamp of first ADC sample (TIM6 @ 100 kHz)
 *   - adc[256] (512B):     ADC samples at 100 kHz (256 × int16_t)
 *   - imu[20] (320B):      IMU raw samples with per-sample timestamps (20 × 16B, only first n_imu valid)
 *   - mavlink_log (10B):   MAVLink event flags and telemetry (event_flags, speed, altitude)
 *   - crc16 (2B):          CRC16 over entire frame (polynomial 0xA001)
 *   Total: 852 bytes
 */

#include <stddef.h>
#include <stdint.h>

#define LOGGER_FRAME_MAGIC 0x5A5A
#define LOGGER_FRAME_SIZE_BYTES 852u
#define LOGGER_IMU_BLOCK_SIZE 20
#define LOGGER_ADC_BLOCK_SIZE 256

/**
 * @brief IMU raw sample structure (16 bytes)
 * Contains raw gyro and accelerometer readings from LSM6DS3 with timestamp
 */
typedef struct __attribute__((packed)) {
    uint8_t data[12];    // Raw 12 bytes from SPI (gx, gy, gz, ax, ay, az as int16_t each)
    uint32_t timestamp;  // Timestamp of this IMU sample (TIM6 @ 100 kHz, 10 µs resolution, 4 bytes)
} logger_imu_raw_sample_t;

_Static_assert(sizeof(logger_imu_raw_sample_t) == 16, "IMU sample must be 16 bytes");

/**
 * @brief MAVLink event types (from STM32 firmware)
 *
 * Events logged as bitmask in mavlink_log_data_t.event_flags
 * Bit N corresponds to event value 0x0N (e.g., bit 5 = MAVLINK_EVT_AUTOPILOT_ARMED)
 */
typedef enum {
    MAVLINK_EVT_NO_EVT = 0x00,                     // 0 - No event
    MAVLINK_EVT_COMMAND_IGNITION = 0x01,           // 1 - Ignition command received
    MAVLINK_EVT_AUTOPILOT_CONNECTED = 0x02,        // 2 - Autopilot connection established
    MAVLINK_EVT_AUTOPILOT_DISCONNECTED = 0x03,     // 3 - Autopilot connection lost
    MAVLINK_EVT_AUTOPILOT_HEARTBEAT = 0x04,        // 4 - Autopilot HEARTBEAT received
    MAVLINK_EVT_AUTOPILOT_ARMED = 0x05,            // 5 - Autopilot changed to ARMED state
    MAVLINK_EVT_AUTOPILOT_DISARMED = 0x06,         // 6 - Autopilot changed to DISARMED state
    MAVLINK_EVT_AUTOPILOT_PREARM_ENABLED = 0x07,   // 7 - Autopilot PREARM enabled
    MAVLINK_EVT_AUTOPILOT_PREARM_DISABLED = 0x08,  // 8 - Autopilot PREARM disabled
    MAVLINK_EVT_VFR_HUD_RECEIVED = 0x09,           // 9 - VFR_HUD data received
    MAVLINK_EVT_SPEED_RECEIVED = 0x0A,             // 10 - Speed data received
    MAVLINK_EVT_ALTITUDE_RECEIVED = 0x0B,          // 11 - Altitude data received
    MAVLINK_EVT_AUTOPILOT_CHARGE = 0x0C,           // 12 - Charge command received
    MAVLINK_EVT_AUTOPILOT_DISCHARGE = 0x0D,        // 13 - Discharge command received
    MAVLINK_EVT_AUTOPILOT_FLYING = 0x0E,           // 14 - Autopilot FLYING state
    MAVLINK_EVT_AUTOPILOT_LANDED = 0x0F            // 15 - Autopilot LANDED state
} mavlink_event_t;

/**
 * @brief MAVLink event log data structure (embedded in logger_frame_t)
 *
 * Contains MAVLink event flags (32-bit bitmask) and telemetry data (speed/altitude).
 * Event flags accumulate during frame window, then reset after frame assembly.
 * Speed and altitude are only updated when corresponding events occur, otherwise remain zero.
 *
 * Layout (10 bytes total):
 *   Offset 0-3:  event_flags (uint32_t bitmask, bit N = event N from mavlink_event_t)
 *   Offset 4-5:  speed_ms (uint16_t groundspeed in m/s, 0 if no MAVLINK_EVT_SPEED_RECEIVED)
 *   Offset 6-9:  altitude_m (int32_t altitude in meters, 0 if no MAVLINK_EVT_ALTITUDE_RECEIVED)
 */
typedef struct __attribute__((packed)) {
    uint32_t event_flags;  // Bitmask of mavlink_event_t events (bit N = event 0x0N)
    uint16_t speed_ms;     // Groundspeed in m/s (0 = no data this frame)
    int32_t altitude_m;    // Altitude in meters (0 = no data this frame)
} mavlink_log_data_t;

_Static_assert(sizeof(mavlink_log_data_t) == 10, "mavlink_log_data_t must be 10 bytes");

/**
 * @brief Complete logger frame (Phase 4 ADC+IMU merged with CRC16)
 *
 * Transmitted from STM32 SPI2 slave to ESP32 SPI master via DMA.
 * Frame rate: ~390 Hz (one per 2.56 ms ADC block)
 *
 * Size: 852 bytes
 *   - magic (2B): 0x5A5A
 *   - n_imu (2B): number of valid IMU samples (0-20)
 *   - adc_timestamp (4B): timestamp of first ADC sample
 *   - adc[256] (512B): ADC data block (256 × int16_t)
 *   - imu[20] (320B): IMU data block with per-sample timestamps (20 × 16B)
 *   - mavlink_log (10B): MAVLink event flags + telemetry
 *   - crc16 (2B): CRC checksum
 */
typedef struct __attribute__((packed)) {
    uint16_t magic;                                      // 0x5A5A (2B)
    uint16_t n_imu;                                      // Valid IMU sample count (0-20) (2B)
    uint32_t adc_timestamp;                              // Timestamp of first ADC sample (4B)
    int16_t adc[LOGGER_ADC_BLOCK_SIZE];                  // ADC buffer (512B)
    logger_imu_raw_sample_t imu[LOGGER_IMU_BLOCK_SIZE];  // IMU buffer (320B)
    mavlink_log_data_t mavlink_log;                      // MAVLink event flags + telemetry (10B)
    uint16_t crc16;                                      // CRC over entire frame (2B)
} logger_frame_t;

_Static_assert(sizeof(logger_frame_t) == LOGGER_FRAME_SIZE_BYTES, "logger_frame_t must be 852 bytes");

/* ============================================================================
 * IMU CONFIGURATION STRUCTURE (Phase 5 SPI - Embedded in logger_config_t)
 * ============================================================================
 *
 * Extended IMU configuration with actual hardware register snapshots.
 * Embedded as nested structure within logger_config_t (offset 8-30).
 *
 * Layout (23 bytes total):
 *   Offset 0-3:   Sensor presence and identification (accel_present, gyro_present, chip_id, pad)
 *   Offset 4-7:   Output Data Rates (accel_odr_hz, gyro_odr_hz)
 *   Offset 8-11:  Full-scale ranges (accel_range_g, alignment byte, gyro_range_dps as uint16_t)
 *   Offset 12-16: Control register snapshots (CTRL1_XL, CTRL2_G, CTRL3_C, CTRL7_G, CTRL4_C)
 *   Offset 17-22: Reserved for future expansion (6 bytes)
 */
typedef struct __attribute__((packed)) {
    // Offset 0-3: Presence and identification (4 bytes)
    uint8_t accel_present;  // 1 if accelerometer enabled (ODR > 0), 0 otherwise
    uint8_t gyro_present;   // 1 if gyroscope enabled (ODR > 0), 0 otherwise
    uint8_t chip_id;        // WHO_AM_I register value (0x69 for LSM6DS3)
    uint8_t reserved_pad;   // Padding for alignment

    // Offset 4-7: Output Data Rates (4 bytes)
    uint16_t accel_odr_hz;  // Decoded from CTRL1_XL[7:4] (0-6664 Hz)
    uint16_t gyro_odr_hz;   // Decoded from CTRL2_G[7:4] (0-6664 Hz)

    // Offset 8-11: Full-scale ranges (4 bytes)
    uint8_t accel_range_g;    // Decoded from CTRL1_XL[3:2]: 2, 4, 8, or 16 G
    uint8_t reserved_align;   // Alignment byte for uint16_t gyro_range_dps
    uint16_t gyro_range_dps;  // Decoded from CTRL2_G[3:2]: 245, 500, 1000, or 2000 DPS

    // Offset 12-16: Control register snapshots (5 bytes)
    uint8_t reserved0;  // CTRL1_XL (0x10) snapshot - Accel ODR + FS
    uint8_t reserved1;  // CTRL2_G (0x11) snapshot - Gyro ODR + FS
    uint8_t reserved2;  // CTRL3_C (0x12) snapshot - Common control (BDU, IF_INC)
    uint8_t reserved3;  // CTRL7_G (0x16) snapshot - Gyro HP mode
    uint8_t reserved4;  // CTRL4_C (0x13) snapshot - DRDY control

    // Offset 17-22: Reserved for future expansion (6 bytes)
    uint8_t reserved5;   // Future: Additional sensor parameters
    uint8_t reserved6;   // Future: LIS2DH12 or other IMU support
    uint8_t reserved7;   // Future: Expansion slot
    uint8_t reserved8;   // Future: Expansion slot
    uint8_t reserved9;   // Future: Expansion slot
    uint8_t reserved10;  // Future: Expansion slot
} imu_config_t;

_Static_assert(sizeof(imu_config_t) == 23, "imu_config_t must be exactly 23 bytes");

/* ============================================================================
 * LOGGER CONFIGURATION (Phase 5 SPI Protocol - Command 0x2A)
 * ============================================================================
 *
 * Device configuration structure sent to ESP32 master via command 42 (0x2A).
 * Defines ADC and IMU parameters for proper frame interpretation.
 * Also written as a 64-byte header at the beginning of each log file.
 *
 * Size: Fixed 64 bytes (padded to 640-byte DMA frame with zeros on STM32)
 * Magic: 0xCAFE (for validation)
 * Version: Split into major.minor (1B each, extensible for future protocol versions)
 * Layout: 2B magic + 1B major + 1B minor + 4B ADC params + 23B IMU config + 1B MAVLink flag + 32B reserved = 64 bytes
 */
#define LOGGER_CONFIG_MAGIC 0xCAFE
#define LOGGER_CONFIG_VERSION_MAJOR 0
#define LOGGER_CONFIG_VERSION_MINOR 1

#define LOGGER_CONFIG_SIZE_BYTES 64u

/* Logger Configuration Version History:
 * v0.1 (2026-01-15): Initial MAVLink integration
 *   - Added mavlink_log_data_t (10 bytes) to logger_frame_t
 *   - Frame size: 852 bytes (was 842 bytes)
 *   - Added mavlink_logging_enabled flag to logger_config_t
 *   - imu_config_t reduced to 23 bytes (was 24 bytes)
 *   - MAVLink event logging: event_flags (32-bit), speed_ms, altitude_m
 */

typedef struct __attribute__((packed)) {
    uint16_t magic;           // 0xCAFE — validity check (2B)
    uint8_t version_major;    // Major version (1B)
    uint8_t version_minor;    // Minor version (1B)

    // --- Core ADC parameters (4B) ---
    uint16_t adc_sample_rate_khz;  // Sampling frequency kHz (2B)
    uint16_t adc_block_size;       // Samples per frame (2B)

    // --- Extended IMU configuration (23B) ---
    imu_config_t imu_config;  // Nested IMU config structure

    // --- MAVLink logging control (1B) ---
    uint8_t mavlink_logging_enabled;  // 1 = MAVLink event logging enabled, 0 = disabled

    // --- Reserved bytes for future expansion (32B) ---
    uint8_t reserved[32];  // Reserved for future use, ensures 64-byte total size
} logger_config_t;


_Static_assert(sizeof(logger_config_t) == LOGGER_CONFIG_SIZE_BYTES, "logger_config_t must be exactly 64 bytes");
