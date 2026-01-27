/**
 * @file csv_writer.h
 * @brief CSV export functions for converted log data
 */

#ifndef CSV_WRITER_H
#define CSV_WRITER_H

#include "log_parser.h"
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * RESULT CODES
 * ============================================================================
 */

typedef enum {
  CSV_OK = 0,
  CSV_ERROR_FILE,
  CSV_ERROR_WRITE,
  CSV_ERROR_PARAM
} csv_result_t;

/* ============================================================================
 * FUNCTION PROTOTYPES
 * ============================================================================
 */

/**
 * @brief Write configuration to CSV file
 * @param filepath Output file path
 * @param config Configuration data to write
 * @return CSV_OK on success
 */
csv_result_t write_config_csv(const char *filepath, const log_config_t *config);

/**
 * @brief Write ADC data to CSV file
 * @param filepath Output file path
 * @param frames Array of frames containing ADC data
 * @param frame_count Number of frames
 * @param adc_sample_rate_khz ADC sample rate for timestamp calculation
 * @return CSV_OK on success
 */
csv_result_t write_adc_csv(const char *filepath, const log_frame_t *frames,
                           size_t frame_count, uint16_t adc_sample_rate_khz,
                           const uint8_t *frame_ok);

/**
 * @brief Write IMU data to CSV file
 * @param filepath Output file path
 * @param frames Array of frames containing IMU data
 * @param frame_count Number of frames
 * @param adc_sample_rate_khz Logger tick rate in kHz (from config)
 * @return CSV_OK on success
 */
csv_result_t write_imu_csv(const char *filepath, const log_frame_t *frames,
                           size_t frame_count, uint16_t adc_sample_rate_khz,
                           const uint8_t *frame_ok);

/**
 * @brief Write IMU anomaly report to CSV file
 *
 * Intended for debugging. Does not modify data; it flags:
 *  - duplicate samples (identical 12 raw bytes within a very small time delta)
 *  - large gaps between consecutive IMU samples
 *  - non-monotonic IMU timestamps (should not happen)
 *
 * @param filepath Output file path
 * @param frames Array of frames
 * @param frame_count Number of frames
 * @param adc_sample_rate_khz Logger tick rate in kHz (from config)
 * @param imu_odr_hz IMU ODR in Hz (from config.imu_config.*_odr_hz)
 * @param frame_ok Per-frame validity flags (0/1), may be NULL (treat as all OK)
 * @param out_anomaly_count Optional output: number of anomalies written
 * @return CSV_OK on success
 */
csv_result_t write_imu_anomalies_csv(const char *filepath,
                                     const log_frame_t *frames,
                                     size_t frame_count,
                                     uint16_t adc_sample_rate_khz,
                                     uint16_t imu_odr_hz,
                                     const uint8_t *frame_ok,
                                     size_t *out_anomaly_count);

/**
 * @brief Write MAVLink events to CSV file
 * @param filepath Output file path
 * @param frames Array of frames containing MAVLink data
 * @param frame_count Number of frames
 * @param adc_sample_rate_khz Logger tick rate in kHz (from config)
 * @return CSV_OK on success
 */
csv_result_t write_mavlink_csv(const char *filepath, const log_frame_t *frames,
                               size_t frame_count, uint16_t adc_sample_rate_khz,
                               const uint8_t *frame_ok);

/**
 * @brief Write per-frame status (magic/checksum) to CSV file
 * @param filepath Output file path
 * @param frames Array of frames
 * @param frame_magic_ok Per-frame magic validity flags (0/1), may be NULL
 * @param frame_checksum_ok Per-frame checksum validity flags (0/1), may be NULL
 * @param frame_checksum_calc Per-frame calculated checksum (8-bit), may be NULL
 * @param frame_count Number of frames
 * @param adc_sample_rate_khz Logger tick rate in kHz (from config)
 * @param checksum_algo_id Algorithm id from config.reserved[0]
 * @return CSV_OK on success
 */
csv_result_t write_frame_status_csv(const char *filepath, const log_frame_t *frames,
                                    const uint8_t *frame_magic_ok,
                                    const uint8_t *frame_checksum_ok,
                                    const uint8_t *frame_checksum_calc,
                                    size_t frame_count, uint16_t adc_sample_rate_khz,
                                    uint8_t checksum_algo_id);

/**
 * @brief Write timestamp anomaly report to CSV file
 *
 * This file is intended for debugging. It does not "fix" or "smooth" timestamps.
 * It highlights non-monotonic timestamps and unexpected frame-to-frame deltas.
 *
 * @param filepath Output file path
 * @param frames Array of frames
 * @param frame_count Number of frames
 * @param adc_sample_rate_khz Logger tick rate in kHz (from config)
 * @param adc_block_size Expected ADC block size in ticks (from config.adc_block_size)
 * @param frame_magic_ok Per-frame magic validity flags (0/1), may be NULL
 * @param frame_checksum_ok Per-frame checksum validity flags (0/1), may be NULL
 * @param out_anomaly_count Optional output: number of anomalies written
 * @return CSV_OK on success
 */
csv_result_t write_timestamp_anomalies_csv(const char *filepath,
                                           const log_frame_t *frames,
                                           size_t frame_count,
                                           uint16_t adc_sample_rate_khz,
                                           uint16_t adc_block_size,
                                           const uint8_t *frame_magic_ok,
                                           const uint8_t *frame_checksum_ok,
                                           size_t *out_anomaly_count);

/**
 * @brief Create output directory if it doesn't exist
 * @param dirpath Directory path to create
 * @return 0 on success, -1 on failure
 */
int create_directory(const char *dirpath);

#ifdef __cplusplus
}
#endif

#endif /* CSV_WRITER_H */
