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
 * @brief Logger configuration header (64 bytes, packed)
 * Written at the start of each binary log file
 */
typedef struct __attribute__((packed)) {
  uint16_t magic;        // 0xCAFE - validity check
  uint8_t version_major; // Major version
  uint8_t version_minor; // Minor version

  uint16_t adc_sample_rate_khz; // ADC sampling rate in kHz
  uint16_t adc_block_size;      // Samples per frame (256)

  uint8_t accel_enable;     // 1 if accelerometer enabled
  uint8_t accel_range_g;    // 2, 4, 8, 16 g
  uint16_t accel_odr_hz;    // 0..6660 Hz
  uint16_t accel_bw_hz;     // 50/100/200/400 Hz
  uint8_t accel_lpf2_en;    // CTRL8_XL.LPF2_XL_EN
  uint8_t accel_hp_en;      // CTRL8_XL.HP_SLOPE_XL_EN
  uint8_t accel_hp_cutoff;  // CTRL8_XL.HPCF_XL[1:0]
  uint8_t accel_hm_mode;    // CTRL6_C.XL_HM_MODE
  uint8_t checksum_algo;    // 1=CRC8, 2=SUM8, 3=CRC8_HW

  uint8_t gyro_enable;      // 1 if gyroscope enabled
  uint8_t gyro_lpf1_en;     // Gyro LPF1 enable (CTRL4_C.LPF1_SEL_G)
  uint16_t gyro_odr_hz;     // 0..6660 Hz
  uint16_t gyro_range_dps;  // 125/245/500/1000/2000 dps
  uint8_t gyro_lpf1_bw;     // Gyro LPF1 bandwidth index (CTRL6_C.FTYPE[1:0])
  uint8_t gyro_hp_en;       // CTRL7_G.HP_G_EN
  uint8_t gyro_hp_cutoff;   // CTRL7_G.HPCF_G[1:0]
  uint8_t gyro_hm_mode;     // CTRL7_G.G_HM_MODE
  uint8_t mavlink_logging_enabled;

  uint8_t logging_active;
  uint8_t config_source;
  uint8_t chip_id;
  uint8_t reserved_state;

  uint8_t ctrl1_xl;
  uint8_t ctrl2_g;
  uint8_t ctrl3_c;
  uint8_t ctrl4_c;
  uint8_t ctrl6_c;
  uint8_t ctrl7_g;
  uint8_t ctrl8_xl;
  uint8_t ctrl9_xl;
  uint8_t ctrl10_c;

  uint8_t reserved[21];
} log_config_t;

_Static_assert(sizeof(log_config_t) == LOG_CONFIG_SIZE,
               "log_config_t must match 64-byte header");

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
