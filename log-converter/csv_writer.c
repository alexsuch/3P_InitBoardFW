/**
 * @file csv_writer.c
 * @brief CSV export implementation
 */

#include "csv_writer.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>


#ifdef _WIN32
#include <direct.h>
#endif

static uint64_t ticks_to_us_u64(uint64_t ticks, uint16_t tick_rate_khz) {
    if (tick_rate_khz == 0) {
        return 0;
    }

    // Firmware timestamps are TIM6 "ticks" at (tick_rate_khz * 1000) Hz.
    // Convert ticks -> microseconds:
    //   1 tick = 1e6 / (tick_rate_khz * 1000) us = 1000 / tick_rate_khz us
    return (ticks * 1000ULL) / (uint64_t)tick_rate_khz;
}

csv_result_t write_config_csv(const char* filepath, const log_config_t* config) {
    if (!filepath || !config) {
        return CSV_ERROR_PARAM;
    }

    FILE* fp = fopen(filepath, "w");
    if (!fp) {
        return CSV_ERROR_FILE;
    }

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

csv_result_t write_adc_csv(const char* filepath, const log_frame_t* frames, size_t frame_count, uint16_t adc_sample_rate_khz) {
    if (!filepath || !frames || frame_count == 0) {
        return CSV_ERROR_PARAM;
    }
    if (adc_sample_rate_khz == 0) {
        return CSV_ERROR_PARAM;
    }

    FILE* fp = fopen(filepath, "w");
    if (!fp) {
        return CSV_ERROR_FILE;
    }

    // Write header

    fprintf(fp, "timestamp_us,adc_value\n");

    // Write ADC samples from all frames
    bool have_prev_start = false;
    uint64_t prev_block_start_ticks = 0;
    for (size_t f = 0; f < frame_count; f++) {
        const log_frame_t* frame = &frames[f];

        // Firmware captures adc_timestamp from a 100 kHz TIM6 tick in the ADC DMA callback.
        // In practice this is closer to the *end* of the 256-sample block, not the first sample.
        // Convert it to an estimated "first sample" timestamp by subtracting (block_size - 1).
        uint64_t block_end_ticks = (uint64_t)frame->adc_timestamp;
        uint64_t block_start_ticks = (block_end_ticks >= (LOG_ADC_BLOCK_SIZE - 1))
                                         ? (block_end_ticks - (LOG_ADC_BLOCK_SIZE - 1))
                                         : 0;

        // Enforce non-decreasing block starts even if the captured end-timestamp jitters by ±1 tick.
        if (have_prev_start) {
            uint64_t expected_start = prev_block_start_ticks + (uint64_t)LOG_ADC_BLOCK_SIZE;
            if (block_start_ticks < expected_start) {
                block_start_ticks = expected_start;
            }
        }
        have_prev_start = true;
        prev_block_start_ticks = block_start_ticks;

        for (int s = 0; s < LOG_ADC_BLOCK_SIZE; s++) {
            // Keep arithmetic in firmware ticks, then scale to microseconds for output.
            uint64_t ticks = block_start_ticks + (uint64_t)s;
            uint64_t timestamp_us = ticks_to_us_u64(ticks, adc_sample_rate_khz);
            fprintf(fp, "%" PRIu64 ",%d\n", timestamp_us, frame->adc[s]);
        }
    }

    fclose(fp);
    return CSV_OK;
}

csv_result_t write_imu_csv(const char* filepath, const log_frame_t* frames, size_t frame_count, uint16_t adc_sample_rate_khz) {
    if (!filepath || !frames || frame_count == 0) {
        return CSV_ERROR_PARAM;
    }

    FILE* fp = fopen(filepath, "w");
    if (!fp) {
        return CSV_ERROR_FILE;
    }

    // Write header
    // Write header

    fprintf(fp, "timestamp_us,gx,gy,gz,ax,ay,az\n");

    // Write IMU samples from all frames
    for (size_t f = 0; f < frame_count; f++) {
        const log_frame_t* frame = &frames[f];

        // Only process valid IMU samples
        uint16_t imu_count = frame->n_imu;
        if (imu_count > LOG_IMU_BLOCK_SIZE) {
            imu_count = LOG_IMU_BLOCK_SIZE;
        }

        for (uint16_t i = 0; i < imu_count; i++) {
            const log_imu_sample_t* sample = &frame->imu[i];

            int16_t gx, gy, gz, ax, ay, az;
            extract_imu_data(sample, &gx, &gy, &gz, &ax, &ay, &az);

            uint64_t timestamp_us = ticks_to_us_u64((uint64_t)sample->timestamp, adc_sample_rate_khz);
            fprintf(fp, "%" PRIu64 ",%d,%d,%d,%d,%d,%d\n", timestamp_us, gx, gy, gz, ax, ay, az);
        }
    }

    fclose(fp);
    return CSV_OK;
}

csv_result_t write_mavlink_csv(const char* filepath, const log_frame_t* frames, size_t frame_count, uint16_t adc_sample_rate_khz) {
    if (!filepath || !frames || frame_count == 0) {
        return CSV_ERROR_PARAM;
    }

    FILE* fp = fopen(filepath, "w");
    if (!fp) {
        return CSV_ERROR_FILE;
    }

    // Write header
    // Write header

    fprintf(fp, "frame_timestamp_us,event_flags,speed_ms,altitude_m\n");

    // Write MAVLink data from each frame
    for (size_t f = 0; f < frame_count; f++) {
        const log_frame_t* frame = &frames[f];
        const log_mavlink_data_t* mav = &frame->mavlink;

        // Only write if there are events
        if (mav->event_flags != 0 || mav->speed_ms != 0 || mav->altitude_m != 0) {
            uint64_t frame_timestamp_us = ticks_to_us_u64((uint64_t)frame->adc_timestamp, adc_sample_rate_khz);
            fprintf(fp, "%" PRIu64 ",0x%08X,%u,%d\n", frame_timestamp_us, mav->event_flags, mav->speed_ms, mav->altitude_m);
        }
    }

    fclose(fp);
    return CSV_OK;
}

int create_directory(const char* dirpath) {
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
    return 0;  // Directory already exists
}
