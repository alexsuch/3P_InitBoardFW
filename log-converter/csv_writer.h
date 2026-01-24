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
                           size_t frame_count, uint16_t adc_sample_rate_khz);

/**
 * @brief Write IMU data to CSV file
 * @param filepath Output file path
 * @param frames Array of frames containing IMU data
 * @param frame_count Number of frames
 * @param adc_sample_rate_khz Logger TIM6 tick rate in kHz (from config)
 * @return CSV_OK on success
 */
csv_result_t write_imu_csv(const char *filepath, const log_frame_t *frames,
                           size_t frame_count, uint16_t adc_sample_rate_khz);

/**
 * @brief Write MAVLink events to CSV file
 * @param filepath Output file path
 * @param frames Array of frames containing MAVLink data
 * @param frame_count Number of frames
 * @param adc_sample_rate_khz Logger TIM6 tick rate in kHz (from config)
 * @return CSV_OK on success
 */
csv_result_t write_mavlink_csv(const char *filepath, const log_frame_t *frames,
                               size_t frame_count, uint16_t adc_sample_rate_khz);

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
