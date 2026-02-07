#pragma once

/**
 * @file logger_frame_link.h
 * @brief Phase 5 SPI Communication - LogFrame_t structure for STM32->ESP32 transmission
 *
 * Defines the 852-byte LogFrame_t structure with ADC+IMU merged data.
 * This replaces the previous 50-byte legacy link frame for enhanced logging capability.
 *
 * Frame Layout (852 bytes):
 *   - magic (2B):          0x5A5A
 *   - n_imu (2B):          Number of valid IMU samples (0-20)
 *   - adc_timestamp (4B):  ADC block reference timestamp (100 kHz ticks, captured on STM32 in DMA callback)
 *   - adc[256] (512B):     ADC samples at 100 kHz (256 x int16_t)
 *   - imu[20] (320B):      IMU raw samples with per-sample timestamps (20 x 16B, only first n_imu valid)
 *   - mavlink_log (10B):   MAVLink event flags and telemetry (event_flags, speed, altitude)
 *   - checksum8 (1B):      Frame checksum (CRC8 or SUM8, selected on STM32 build)
 *   - checksum_pad (1B):   Reserved (0)
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
 * Contains raw IMU readings as provided by STM32 (12 bytes + timestamp)
 */
typedef struct __attribute__((packed)) {
    uint8_t data[12];    // Raw 12 bytes from SPI (gx, gy, gz, ax, ay, az as int16_t each)
    uint32_t timestamp;  // Timestamp of this IMU sample (100 kHz ticks, captured on STM32 when sample is received)
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
 * @brief Complete logger frame (Phase 4 ADC+IMU merged with checksum)
 *
 * Transmitted from STM32 SPI2 slave to ESP32 SPI master via DMA.
 * Frame rate: ~390 Hz (one per 2.56 ms ADC block)
 *
 * Size: 852 bytes
 *   - magic (2B): 0x5A5A
 *   - n_imu (2B): number of valid IMU samples (0-20)
 *   - adc_timestamp (4B): ADC block reference timestamp (captured in DMA callback)
 *   - adc[256] (512B): ADC data block (256 x int16_t)
 *   - imu[20] (320B): IMU data block with per-sample timestamps (20 x 16B)
 *   - mavlink_log (10B): MAVLink event flags + telemetry
 *   - checksum8 (1B): checksum (CRC8 or SUM8)
 *   - checksum_pad (1B): reserved (0)
 */
typedef struct __attribute__((packed)) {
    uint16_t magic;                                      // 0x5A5A (2B)
    uint16_t n_imu;                                      // Valid IMU sample count (0-20) (2B)
    uint32_t adc_timestamp;                              // ADC block reference timestamp (100 kHz tick counter) (4B)
    int16_t adc[LOGGER_ADC_BLOCK_SIZE];                  // ADC buffer (512B)
    logger_imu_raw_sample_t imu[LOGGER_IMU_BLOCK_SIZE];  // IMU buffer (320B)
    mavlink_log_data_t mavlink_log;                      // MAVLink event flags + telemetry (10B)
    uint8_t checksum8;                                   // Checksum over payload (1B)
    uint8_t checksum_pad;                                // Reserved/padding (1B)
} logger_frame_t;

_Static_assert(sizeof(logger_frame_t) == LOGGER_FRAME_SIZE_BYTES, "logger_frame_t must be 852 bytes");

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
 * Version: Major.Minor (Firmware Version of STM32)
 *
 * Layout (64 bytes total):
 *   Header (4B)
 *   ADC parameters (4B)
 *   Accelerometer parameters (11B) - formerly part of imu_config_t
 *   Gyroscope parameters (11B) - formerly part of imu_config_t
 *   Runtime state (4B)
 *   Register snapshots (9B)
 *   Reserved (21B)
 */
#define LOGGER_CONFIG_MAGIC 0xCAFE

#define LOGGER_CONFIG_SIZE_BYTES 64u

/* Config source values */
#define LOGGER_CONFIG_SOURCE_DEFAULT 0   // Using hardcoded defaults
#define LOGGER_CONFIG_SOURCE_NVS 1       // Loaded from flash/NVS
#define LOGGER_CONFIG_SOURCE_RECEIVED 2  // Received from ESP32 via CMD 45

typedef struct __attribute__((packed)) {
    /* Header (4B) */
    uint16_t magic;         // 0xCAFE — validity check
    uint8_t version_major;  // STM32 Firmware Major Version
    uint8_t version_minor;  // STM32 Firmware Minor Version

    /* ADC parameters (4B) */
    uint16_t adc_sample_rate_khz;  // Sampling frequency kHz (1-200, default: 100)
    uint16_t adc_block_size;       // Samples per frame (default: 256)

    /* Accelerometer parameters (11B) */
    uint8_t accel_enable;     // 1=enabled, 0=disabled
    uint8_t accel_range_g;    // Full scale: 2, 4, 8, or 16 G
    uint16_t accel_odr_hz;    // Output data rate: 0-6664 Hz
    uint16_t accel_bw_hz;     // Anti-aliasing BW: 50, 100, 200, 400 Hz
    uint8_t accel_lpf2_en;    // LPF2 enable (CTRL8_XL[7])
    uint8_t accel_hp_en;      // HP filter enable (CTRL8_XL[2])
    uint8_t accel_hp_cutoff;  // HP cutoff index 0-3 (CTRL8_XL[6:5])
    uint8_t accel_hm_mode;    // CTRL6_C.XL_HM_MODE: 0=high-performance, 1=normal/low-power path
    uint8_t checksum_algo;    // 1=CRC8, 2=SUM8, 3=CRC8_HW

    /* Gyroscope parameters (11B) */
    uint8_t gyro_enable;              // 1=enabled, 0=disabled
    uint8_t gyro_lpf1_en;             // Gyro LPF1 enable (CTRL4_C.LPF1_SEL_G)
    uint16_t gyro_odr_hz;             // Output data rate: 0-6660 Hz (chip-specific limits apply)
    uint16_t gyro_range_dps;          // Full scale: 125, 245, 500, 1000, 2000 DPS
    uint8_t gyro_lpf1_bw;             // Gyro LPF1 bandwidth index 0-3 (CTRL6_C.FTYPE[1:0])
    uint8_t gyro_hp_en;               // HP filter enable (CTRL7_G[6])
    uint8_t gyro_hp_cutoff;           // HP cutoff index 0-3 (CTRL7_G[5:4])
    uint8_t gyro_hm_mode;             // CTRL7_G.G_HM_MODE: 0=high-performance, 1=normal/low-power path
    uint8_t mavlink_logging_enabled;  // 1=MAVLink event logging enabled

    /* Runtime state (read-only, 4B) */
    uint8_t logging_active;  // 1=currently logging, 0=idle
    uint8_t config_source;   // LOGGER_CONFIG_SOURCE_*
    uint8_t chip_id;         // WHO_AM_I (0x69 for LSM6DS3)
    uint8_t reserved_state;

    /* Register snapshots (9B) */
    uint8_t ctrl1_xl;  // CTRL1_XL actual value
    uint8_t ctrl2_g;   // CTRL2_G actual value
    uint8_t ctrl3_c;   // CTRL3_C actual value
    uint8_t ctrl4_c;   // CTRL4_C actual value
    uint8_t ctrl6_c;   // CTRL6_C actual value
    uint8_t ctrl7_g;   // CTRL7_G actual value
    uint8_t ctrl8_xl;  // CTRL8_XL actual value
    uint8_t ctrl9_xl;  // CTRL9_XL actual value
    uint8_t ctrl10_c;  // CTRL10_C actual value

    /* Reserved for future expansion (21B) */
    uint8_t reserved[21];
} logger_config_t;

_Static_assert(sizeof(logger_config_t) == LOGGER_CONFIG_SIZE_BYTES, "logger_config_t must be exactly 64 bytes");
