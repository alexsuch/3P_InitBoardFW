/**
 * @file csv_writer.c
 * @brief CSV export implementation
 */

#include "csv_writer.h"
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>

#ifdef _WIN32
#include <direct.h>
#endif

csv_result_t write_config_csv(const char *filepath,
                              const log_config_t *config) {
  if (!filepath || !config) {
    return CSV_ERROR_PARAM;
  }

  FILE *fp = fopen(filepath, "w");
  if (!fp) {
    return CSV_ERROR_FILE;
  }

  // Write header comment
  fprintf(fp, "# Logger Configuration\n");
  fprintf(fp, "parameter,value\n");

  // Write configuration parameters
  fprintf(fp, "magic,0x%04X\n", config->magic);
  fprintf(fp, "version_major,%u\n", config->version_major);
  fprintf(fp, "version_minor,%u\n", config->version_minor);
  fprintf(fp, "adc_sample_rate_khz,%u\n", config->adc_sample_rate_khz);
  fprintf(fp, "adc_block_size,%u\n", config->adc_block_size);

  // IMU configuration
  fprintf(fp, "accel_present,%u\n", config->imu_config.accel_present);
  fprintf(fp, "gyro_present,%u\n", config->imu_config.gyro_present);
  fprintf(fp, "chip_id,0x%02X\n", config->imu_config.chip_id);
  fprintf(fp, "accel_odr_hz,%u\n", config->imu_config.accel_odr_hz);
  fprintf(fp, "gyro_odr_hz,%u\n", config->imu_config.gyro_odr_hz);
  fprintf(fp, "accel_range_g,%u\n", config->imu_config.accel_range_g);
  fprintf(fp, "gyro_range_dps,%u\n", config->imu_config.gyro_range_dps);

  // MAVLink
  fprintf(fp, "mavlink_logging_enabled,%u\n", config->mavlink_logging_enabled);

  fclose(fp);
  return CSV_OK;
}

csv_result_t write_adc_csv(const char *filepath, const log_frame_t *frames,
                           size_t frame_count, uint16_t adc_sample_rate_khz) {
  if (!filepath || !frames || frame_count == 0) {
    return CSV_ERROR_PARAM;
  }

  FILE *fp = fopen(filepath, "w");
  if (!fp) {
    return CSV_ERROR_FILE;
  }

  // Calculate sample period in microseconds
  // For 100 kHz -> 10 us per sample
  uint32_t sample_period_us = 1000 / adc_sample_rate_khz;

  // Write header
  fprintf(fp, "# ADC Piezo Sensor Data\n");
  fprintf(fp, "# Sample Rate: %u kHz (%u us per sample)\n", adc_sample_rate_khz,
          sample_period_us);
  fprintf(fp, "timestamp_us,adc_value\n");

  // Write ADC samples from all frames
  for (size_t f = 0; f < frame_count; f++) {
    const log_frame_t *frame = &frames[f];
    uint32_t base_timestamp = frame->adc_timestamp;

    for (int s = 0; s < LOG_ADC_BLOCK_SIZE; s++) {
      // Each sample is sample_period_us apart (10us @ 100kHz)
      uint32_t timestamp = base_timestamp + (s * sample_period_us);
      fprintf(fp, "%u,%d\n", timestamp, frame->adc[s]);
    }
  }

  fclose(fp);
  return CSV_OK;
}

csv_result_t write_imu_csv(const char *filepath, const log_frame_t *frames,
                           size_t frame_count) {
  if (!filepath || !frames || frame_count == 0) {
    return CSV_ERROR_PARAM;
  }

  FILE *fp = fopen(filepath, "w");
  if (!fp) {
    return CSV_ERROR_FILE;
  }

  // Write header
  fprintf(fp, "# IMU Data (LSM6DS3)\n");
  fprintf(fp, "# Gyro: raw int16, Accel: raw int16\n");
  fprintf(fp, "timestamp_us,gx,gy,gz,ax,ay,az\n");

  // Write IMU samples from all frames
  for (size_t f = 0; f < frame_count; f++) {
    const log_frame_t *frame = &frames[f];

    // Only process valid IMU samples
    uint16_t imu_count = frame->n_imu;
    if (imu_count > LOG_IMU_BLOCK_SIZE) {
      imu_count = LOG_IMU_BLOCK_SIZE;
    }

    for (uint16_t i = 0; i < imu_count; i++) {
      const log_imu_sample_t *sample = &frame->imu[i];

      int16_t gx, gy, gz, ax, ay, az;
      extract_imu_data(sample, &gx, &gy, &gz, &ax, &ay, &az);

      fprintf(fp, "%u,%d,%d,%d,%d,%d,%d\n", sample->timestamp, gx, gy, gz, ax,
              ay, az);
    }
  }

  fclose(fp);
  return CSV_OK;
}

csv_result_t write_mavlink_csv(const char *filepath, const log_frame_t *frames,
                               size_t frame_count) {
  if (!filepath || !frames || frame_count == 0) {
    return CSV_ERROR_PARAM;
  }

  FILE *fp = fopen(filepath, "w");
  if (!fp) {
    return CSV_ERROR_FILE;
  }

  // Write header
  fprintf(fp, "# MAVLink Telemetry Events\n");
  fprintf(fp, "frame_timestamp_us,event_flags,speed_ms,altitude_m\n");

  // Write MAVLink data from each frame
  for (size_t f = 0; f < frame_count; f++) {
    const log_frame_t *frame = &frames[f];
    const log_mavlink_data_t *mav = &frame->mavlink;

    // Only write if there are events
    if (mav->event_flags != 0 || mav->speed_ms != 0 || mav->altitude_m != 0) {
      fprintf(fp, "%u,0x%08X,%u,%d\n", frame->adc_timestamp, mav->event_flags,
              mav->speed_ms, mav->altitude_m);
    }
  }

  fclose(fp);
  return CSV_OK;
}

int create_directory(const char *dirpath) {
  if (!dirpath) {
    return -1;
  }

  struct stat st = {0};
  if (stat(dirpath, &st) == -1) {
#ifdef _WIN32
    return _mkdir(dirpath);
#else
    return mkdir(dirpath, 0755);
#endif
  }
  return 0; // Directory already exists
}
