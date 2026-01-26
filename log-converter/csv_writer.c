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

    // Firmware timestamps are "ticks" at (tick_rate_khz * 1000) Hz.
    // Convert ticks -> microseconds:
    //   1 tick = 1e6 / (tick_rate_khz * 1000) us = 1000 / tick_rate_khz us
    return (ticks * 1000ULL) / (uint64_t)tick_rate_khz;
}

static const char* checksum_algo_name(uint8_t algo_id) {
    switch (algo_id) {
        case 1:
            return "CRC8_SW";
        case 2:
            return "SUM8";
        case 3:
            return "CRC8_HW";
        default:
            return "UNKNOWN";
    }
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

    // Checksum algorithm id (exported by firmware in reserved[0])
    fprintf(fp, "checksum_algo_id,%u\n", config->reserved[0]);
    fprintf(fp, "checksum_algo_name,%s\n", checksum_algo_name(config->reserved[0]));

    fclose(fp);
    return CSV_OK;
}

csv_result_t write_adc_csv(const char* filepath, const log_frame_t* frames, size_t frame_count, uint16_t adc_sample_rate_khz,
                           const uint8_t* frame_ok) {
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
        const bool ok = frame_ok ? (frame_ok[f] != 0) : true;

        // Firmware provides adc_timestamp as the timestamp of the first ADC sample in the block.
        uint64_t block_start_ticks = (uint64_t)frame->adc_timestamp;

        // Enforce non-decreasing block starts even if the captured end-timestamp jitters by ±1 tick.
        if (have_prev_start) {
            uint64_t expected_start = prev_block_start_ticks + (uint64_t)LOG_ADC_BLOCK_SIZE;
            if (!ok) {
                block_start_ticks = expected_start;
            } else if (block_start_ticks < expected_start) {
                block_start_ticks = expected_start;
            }
        }
        have_prev_start = true;
        prev_block_start_ticks = block_start_ticks;

        for (int s = 0; s < LOG_ADC_BLOCK_SIZE; s++) {
            // Keep arithmetic in firmware ticks, then scale to microseconds for output.
            uint64_t ticks = block_start_ticks + (uint64_t)s;
            uint64_t timestamp_us = ticks_to_us_u64(ticks, adc_sample_rate_khz);
            const int16_t adc_val = ok ? frame->adc[s] : 0;
            fprintf(fp, "%" PRIu64 ",%d\n", timestamp_us, adc_val);
        }
    }

    fclose(fp);
    return CSV_OK;
}

csv_result_t write_imu_csv(const char* filepath, const log_frame_t* frames, size_t frame_count, uint16_t adc_sample_rate_khz,
                           const uint8_t* frame_ok) {
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
        const bool ok = frame_ok ? (frame_ok[f] != 0) : true;
        if (!ok) {
            continue;
        }

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

csv_result_t write_mavlink_csv(const char* filepath, const log_frame_t* frames, size_t frame_count, uint16_t adc_sample_rate_khz,
                               const uint8_t* frame_ok) {
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
        const bool ok = frame_ok ? (frame_ok[f] != 0) : true;
        if (!ok) {
            continue;
        }
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

csv_result_t write_frame_status_csv(const char* filepath, const log_frame_t* frames, const uint8_t* frame_magic_ok,
                                    const uint8_t* frame_checksum_ok, const uint8_t* frame_checksum_calc, size_t frame_count,
                                    uint16_t adc_sample_rate_khz, uint8_t checksum_algo_id) {
    if (!filepath || !frames || frame_count == 0) {
        return CSV_ERROR_PARAM;
    }

    FILE* fp = fopen(filepath, "w");
    if (!fp) {
        return CSV_ERROR_FILE;
    }

    fprintf(fp, "frame_index,adc_timestamp_ticks,frame_timestamp_us,delta_ticks,delta_us,magic,magic_ok,checksum_stored,checksum_calc,checksum_ok,checksum_algo_id,checksum_algo\n");

    for (size_t f = 0; f < frame_count; f++) {
        const log_frame_t* frame = &frames[f];
        const uint8_t magic_ok_v = frame_magic_ok ? frame_magic_ok[f] : (frame->magic == LOG_FRAME_MAGIC);
        const uint8_t checksum_ok_v = frame_checksum_ok ? frame_checksum_ok[f] : 0;
        const uint8_t checksum_calc_v = frame_checksum_calc ? frame_checksum_calc[f] : 0;
        const uint64_t frame_ts_us = ticks_to_us_u64((uint64_t)frame->adc_timestamp, adc_sample_rate_khz);
        const uint32_t delta_ticks = (f == 0) ? 0u : (uint32_t)(frame->adc_timestamp - frames[f - 1].adc_timestamp);
        const uint64_t delta_us = ticks_to_us_u64((uint64_t)delta_ticks, adc_sample_rate_khz);

        fprintf(fp, "%zu,%u,%" PRIu64 ",%u,%" PRIu64 ",0x%04X,%u,0x%02X,0x%02X,%u,%u,%s\n", f, (unsigned)frame->adc_timestamp, frame_ts_us,
                (unsigned)delta_ticks, delta_us, (unsigned)frame->magic, (unsigned)magic_ok_v, (unsigned)frame->checksum8,
                (unsigned)checksum_calc_v, (unsigned)checksum_ok_v, (unsigned)checksum_algo_id, checksum_algo_name(checksum_algo_id));
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
