/**
 * @file log_parser.c
 * @brief Binary log file parser implementation
 */

#include "log_parser.h"
#include <string.h>

parse_result_t parse_config(FILE *fp, log_config_t *config) {
  if (!fp || !config) {
    return PARSE_ERROR_FILE;
  }

  size_t read = fread(config, 1, sizeof(log_config_t), fp);
  if (read != sizeof(log_config_t)) {
    return PARSE_ERROR_READ;
  }

  if (!validate_config_magic(config)) {
    return PARSE_ERROR_MAGIC;
  }

  return PARSE_OK;
}

parse_result_t parse_frame(FILE *fp, log_frame_t *frame) {
  if (!fp || !frame) {
    return PARSE_ERROR_FILE;
  }

  size_t read = fread(frame, 1, sizeof(log_frame_t), fp);
  if (read == 0) {
    return PARSE_ERROR_EOF;
  }
  if (read != sizeof(log_frame_t)) {
    return PARSE_ERROR_READ;
  }

  if (!validate_frame_magic(frame)) {
    return PARSE_ERROR_MAGIC;
  }

  return PARSE_OK;
}

bool validate_config_magic(const log_config_t *config) {
  return config && config->magic == LOG_CONFIG_MAGIC;
}

bool validate_frame_magic(const log_frame_t *frame) {
  return frame && frame->magic == LOG_FRAME_MAGIC;
}

void extract_imu_data(const log_imu_sample_t *sample, int16_t *gx, int16_t *gy,
                      int16_t *gz, int16_t *ax, int16_t *ay, int16_t *az) {
  if (!sample)
    return;

  // IMU data is stored in little-endian format:
  // bytes 0-1: gx, bytes 2-3: gy, bytes 4-5: gz
  // bytes 6-7: ax, bytes 8-9: ay, bytes 10-11: az
  const uint8_t *d = sample->data;

  if (gx)
    *gx = (int16_t)(d[0] | (d[1] << 8));
  if (gy)
    *gy = (int16_t)(d[2] | (d[3] << 8));
  if (gz)
    *gz = (int16_t)(d[4] | (d[5] << 8));
  if (ax)
    *ax = (int16_t)(d[6] | (d[7] << 8));
  if (ay)
    *ay = (int16_t)(d[8] | (d[9] << 8));
  if (az)
    *az = (int16_t)(d[10] | (d[11] << 8));
}

long get_file_size(FILE *fp) {
  if (!fp)
    return -1;

  long current = ftell(fp);
  if (current < 0)
    return -1;

  if (fseek(fp, 0, SEEK_END) != 0)
    return -1;
  long size = ftell(fp);

  // Restore original position
  fseek(fp, current, SEEK_SET);
  return size;
}

size_t calculate_frame_count(long file_size) {
  if (file_size <= (long)sizeof(log_config_t)) {
    return 0;
  }
  return (size_t)((file_size - sizeof(log_config_t)) / sizeof(log_frame_t));
}
