/**
 * @file log_parser.h
 * @brief Binary log file parser for 3P Logger data
 *
 * Defines data structures matching the firmware logger format and
 * provides functions to parse binary log files.
 */

#ifndef LOG_PARSER_H
#define LOG_PARSER_H

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * CONSTANTS
 * ============================================================================
 */

#define LOG_CONFIG_MAGIC 0xCAFE
#define LOG_FRAME_MAGIC 0x5A5A
#define LOG_CONFIG_SIZE 64 // v0.1: 64-byte config header
#define LOG_FRAME_SIZE 852
#define LOG_ADC_BLOCK_SIZE 256
#define LOG_IMU_BLOCK_SIZE 20
#define LOG_IMU_RAW_DATA_SIZE 12

/* ============================================================================
 * DATA STRUCTURES (mirroring firmware definitions)
 * ============================================================================
 */

/**
 * @brief IMU configuration structure (23 bytes, packed)
 */
typedef struct __attribute__((packed)) {
  uint8_t accel_present; // 1 if accelerometer enabled
  uint8_t gyro_present;  // 1 if gyroscope enabled
  uint8_t chip_id;       // WHO_AM_I register (0x69 for LSM6DS3)
  uint8_t reserved_pad;  // Padding

  uint16_t accel_odr_hz; // Accelerometer ODR in Hz
  uint16_t gyro_odr_hz;  // Gyroscope ODR in Hz

  uint8_t accel_range_g;   // Accel range: 2, 4, 8, or 16 G
  uint8_t reserved_align;  // Alignment padding
  uint16_t gyro_range_dps; // Gyro range: 245, 500, 1000, 2000 DPS

  uint8_t reserved[11]; // Reserved for future use
} log_imu_config_t;

/**
 * @brief Logger configuration header (64 bytes, packed)
 * Written at the start of each binary log file
 *
 * TODO: Add backward compatibility for older versions when needed.
 *       Currently MVP version - only 64-byte config supported.
 *       Future versions should read magic+version first, then
 *       handle different config sizes based on version.
 */
typedef struct __attribute__((packed)) {
  uint16_t magic;        // 0xCAFE - validity check
  uint8_t version_major; // Major version
  uint8_t version_minor; // Minor version

  uint16_t adc_sample_rate_khz; // ADC sampling rate in kHz
  uint16_t adc_block_size;      // Samples per frame (256)

  log_imu_config_t imu_config; // IMU configuration (23 bytes)

  uint8_t mavlink_logging_enabled; // 1 = MAVLink logging enabled

  uint8_t reserved[32]; // reserved[0]=checksum algorithm id; remaining reserved for future use
} log_config_t;

/**
 * @brief IMU raw sample with timestamp (16 bytes, packed)
 */
typedef struct __attribute__((packed)) {
  uint8_t data[LOG_IMU_RAW_DATA_SIZE]; // Raw SPI data: gx,gy,gz,ax,ay,az (6 x
                                       // int16)
  uint32_t timestamp;                  // Timestamp @ 100kHz (10us resolution)
} log_imu_sample_t;

/**
 * @brief MAVLink event log data (10 bytes, packed)
 */
typedef struct __attribute__((packed)) {
  uint32_t event_flags; // Bitmask of MAVLink events
  uint16_t speed_ms;    // Groundspeed in m/s
  int32_t altitude_m;   // Altitude in meters
} log_mavlink_data_t;

/**
 * @brief Complete logger frame (852 bytes, packed)
 */
typedef struct __attribute__((packed)) {
  uint16_t magic;         // 0x5A5A - frame marker
  uint16_t n_imu;         // Number of valid IMU samples (0-20)
  uint32_t adc_timestamp; // Timestamp of the first ADC sample in this block (100 kHz tick counter)

  int16_t adc[LOG_ADC_BLOCK_SIZE]; // ADC samples (256 x int16 = 512 bytes)
  log_imu_sample_t imu[LOG_IMU_BLOCK_SIZE]; // IMU samples (20 x 16 = 320 bytes)
  log_mavlink_data_t mavlink;               // MAVLink data (10 bytes)

  uint8_t checksum8;     // Checksum (CRC8 or SUM8, depends on firmware build)
  uint8_t checksum_pad;  // Reserved/padding (0)
} log_frame_t;

/* ============================================================================
 * PARSER RESULT CODES
 * ============================================================================
 */

typedef enum {
  PARSE_OK = 0,
  PARSE_ERROR_FILE,
  PARSE_ERROR_READ,
  PARSE_ERROR_MAGIC,
  PARSE_ERROR_CRC,
  PARSE_ERROR_EOF
} parse_result_t;

/* ============================================================================
 * FUNCTION PROTOTYPES
 * ============================================================================
 */

/**
 * @brief Parse configuration header from binary log file
 * @param fp File pointer (positioned at start of file)
 * @param config Output configuration structure
 * @return PARSE_OK on success, error code otherwise
 */
parse_result_t parse_config(FILE *fp, log_config_t *config);

/**
 * @brief Parse a single frame from binary log file
 * @param fp File pointer (positioned after config or previous frame)
 * @param frame Output frame structure
 * @return PARSE_OK on success, PARSE_ERROR_EOF at end of file, error code
 * otherwise
 */
parse_result_t parse_frame(FILE *fp, log_frame_t *frame);

/**
 * @brief Validate configuration magic number
 * @param config Configuration to validate
 * @return true if magic is valid (0xCAFE)
 */
bool validate_config_magic(const log_config_t *config);

/**
 * @brief Validate frame magic number
 * @param frame Frame to validate
 * @return true if magic is valid (0x5A5A)
 */
bool validate_frame_magic(const log_frame_t *frame);

/**
 * @brief Extract IMU data from raw bytes
 * @param sample IMU sample containing raw data
 * @param gx Output gyroscope X (raw int16)
 * @param gy Output gyroscope Y (raw int16)
 * @param gz Output gyroscope Z (raw int16)
 * @param ax Output accelerometer X (raw int16)
 * @param ay Output accelerometer Y (raw int16)
 * @param az Output accelerometer Z (raw int16)
 */
void extract_imu_data(const log_imu_sample_t *sample, int16_t *gx, int16_t *gy,
                      int16_t *gz, int16_t *ax, int16_t *ay, int16_t *az);

/**
 * @brief Get file size in bytes
 * @param fp File pointer
 * @return File size or -1 on error
 */
long get_file_size(FILE *fp);

/**
 * @brief Calculate expected frame count from file size
 * @param file_size Total file size in bytes
 * @return Expected number of frames
 */
size_t calculate_frame_count(long file_size);

#ifdef __cplusplus
}
#endif

#endif /* LOG_PARSER_H */
