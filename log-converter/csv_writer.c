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

    // Accelerometer configuration
    fprintf(fp, "accel_enable,%u\n", config->accel_enable);
    fprintf(fp, "accel_range_g,%u\n", config->accel_range_g);
    fprintf(fp, "accel_odr_hz,%u\n", config->accel_odr_hz);
    fprintf(fp, "accel_bw_hz,%u\n", config->accel_bw_hz);
    fprintf(fp, "accel_lpf2_en,%u\n", config->accel_lpf2_en);
    fprintf(fp, "accel_hp_en,%u\n", config->accel_hp_en);
    fprintf(fp, "accel_hp_cutoff,%u\n", config->accel_hp_cutoff);
    fprintf(fp, "accel_hm_mode,%u\n", config->accel_hm_mode);

    // Gyroscope configuration
    fprintf(fp, "gyro_enable,%u\n", config->gyro_enable);
    fprintf(fp, "gyro_range_dps,%u\n", config->gyro_range_dps);
    fprintf(fp, "gyro_odr_hz,%u\n", config->gyro_odr_hz);
    fprintf(fp, "gyro_hp_en,%u\n", config->gyro_hp_en);
    fprintf(fp, "gyro_hp_cutoff,%u\n", config->gyro_hp_cutoff);
    fprintf(fp, "gyro_lpf1_en,%u\n", config->gyro_lpf1_en);
    fprintf(fp, "gyro_lpf1_bw,%u\n", config->gyro_lpf1_bw);
    fprintf(fp, "gyro_hm_mode,%u\n", config->gyro_hm_mode);

    // Runtime/system info
    fprintf(fp, "logging_active,%u\n", config->logging_active);
    fprintf(fp, "config_source,%u\n", config->config_source);
    fprintf(fp, "chip_id,0x%02X\n", config->chip_id);
    fprintf(fp, "mavlink_logging_enabled,%u\n", config->mavlink_logging_enabled);

    // Register snapshots
    fprintf(fp, "ctrl1_xl,0x%02X\n", config->ctrl1_xl);
    fprintf(fp, "ctrl2_g,0x%02X\n", config->ctrl2_g);
    fprintf(fp, "ctrl3_c,0x%02X\n", config->ctrl3_c);
    fprintf(fp, "ctrl4_c,0x%02X\n", config->ctrl4_c);
    fprintf(fp, "ctrl6_c,0x%02X\n", config->ctrl6_c);
    fprintf(fp, "ctrl7_g,0x%02X\n", config->ctrl7_g);
    fprintf(fp, "ctrl8_xl,0x%02X\n", config->ctrl8_xl);
    fprintf(fp, "ctrl9_xl,0x%02X\n", config->ctrl9_xl);
    fprintf(fp, "ctrl10_c,0x%02X\n", config->ctrl10_c);

    // Checksum algorithm id (explicit field in current firmware layout)
    fprintf(fp, "checksum_algo_id,%u\n", config->checksum_algo);
    fprintf(fp, "checksum_algo_name,%s\n", checksum_algo_name(config->checksum_algo));

    // Dump all reserved bytes so no configuration information is lost.
    for (size_t i = 0; i < sizeof(config->reserved); i++) {
        fprintf(fp, "reserved_%u,%u\n", (unsigned)i, (unsigned)config->reserved[i]);
    }

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
    for (size_t f = 0; f < frame_count; f++) {
        const log_frame_t* frame = &frames[f];
        const bool ok = frame_ok ? (frame_ok[f] != 0) : true;

        // Firmware provides adc_timestamp as the timestamp of the first ADC sample in the block.
        const uint64_t block_start_ticks = (uint64_t)frame->adc_timestamp;

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

csv_result_t write_imu_anomalies_csv(const char* filepath,
                                     const log_frame_t* frames,
                                     size_t frame_count,
                                     uint16_t adc_sample_rate_khz,
                                     uint16_t imu_odr_hz,
                                     const uint8_t* frame_ok,
                                     size_t* out_anomaly_count) {
    if (out_anomaly_count) {
        *out_anomaly_count = 0;
    }
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

    // expected ticks per IMU sample at current ODR (ticks are at adc_sample_rate_khz * 1000 Hz)
    uint32_t expected_ticks = 0;
    if (imu_odr_hz != 0) {
        const uint32_t tick_hz = (uint32_t)adc_sample_rate_khz * 1000u;
        expected_ticks = (tick_hz + (uint32_t)imu_odr_hz / 2u) / (uint32_t)imu_odr_hz;  // rounded
        if (expected_ticks == 0) {
            expected_ticks = 1;
        }
    }

    // Heuristics tuned to catch real duplicates without flagging valid steady-state data:
    // - duplicate: identical raw bytes within <= 5 ticks (at 100 kHz tick = 50 us)
    // - gap: delta > 5 * expected period (if expected is known)
    const uint32_t dup_max_ticks = 5u;
    const uint32_t gap_mult = 5u;

    fprintf(fp,
            "sample_index,anomaly,prev_ts_ticks,ts_ticks,delta_ticks,prev_ts_us,ts_us,delta_us,prev_frame,prev_sample,frame,sample,n_imu,expected_ticks\n");

    uint8_t have_prev = 0;
    uint32_t prev_ts = 0;
    uint8_t prev_raw[LOG_IMU_RAW_DATA_SIZE] = {0};
    size_t prev_frame = 0;
    uint16_t prev_sample = 0;

    size_t sample_index = 0;
    size_t anomalies = 0;

    for (size_t f = 0; f < frame_count; f++) {
        const log_frame_t* frame = &frames[f];
        const bool ok = frame_ok ? (frame_ok[f] != 0) : true;
        if (!ok) {
            continue;
        }

        uint16_t imu_count = frame->n_imu;
        if (imu_count > LOG_IMU_BLOCK_SIZE) {
            imu_count = LOG_IMU_BLOCK_SIZE;
        }

        for (uint16_t i = 0; i < imu_count; i++) {
            const log_imu_sample_t* s = &frame->imu[i];
            const uint32_t ts = s->timestamp;

            if (have_prev) {
                const int32_t delta_ticks_i = (int32_t)(ts - prev_ts);
                const int64_t delta_us_i = (adc_sample_rate_khz == 0)
                                               ? 0
                                               : (((int64_t)delta_ticks_i * 1000LL) / (int64_t)adc_sample_rate_khz);
                const uint64_t prev_us = ticks_to_us_u64((uint64_t)prev_ts, adc_sample_rate_khz);
                const uint64_t cur_us = ticks_to_us_u64((uint64_t)ts, adc_sample_rate_khz);

                if (ts < prev_ts) {
                    fprintf(fp,
                            "%zu,IMU_TS_BACKWARDS,%u,%u,%ld,%" PRIu64 ",%" PRIu64 ",%lld,%zu,%u,%zu,%u,%u,%u\n",
                            sample_index, (unsigned)prev_ts, (unsigned)ts, (long)delta_ticks_i, prev_us, cur_us,
                            (long long)delta_us_i, prev_frame, (unsigned)prev_sample, f, (unsigned)i, (unsigned)frame->n_imu,
                            (unsigned)expected_ticks);
                    anomalies++;
                } else {
                    const uint32_t delta_u = (uint32_t)delta_ticks_i;
                    const bool raw_equal = (memcmp(prev_raw, s->data, LOG_IMU_RAW_DATA_SIZE) == 0);

                    if (raw_equal && delta_u <= dup_max_ticks) {
                        fprintf(fp,
                                "%zu,IMU_DUPLICATE_RAW,%u,%u,%u,%" PRIu64 ",%" PRIu64 ",%lld,%zu,%u,%zu,%u,%u,%u\n",
                                sample_index, (unsigned)prev_ts, (unsigned)ts, (unsigned)delta_u, prev_us, cur_us,
                                (long long)delta_us_i, prev_frame, (unsigned)prev_sample, f, (unsigned)i, (unsigned)frame->n_imu,
                                (unsigned)expected_ticks);
                        anomalies++;
                    }

                    if (expected_ticks != 0 && delta_u > (expected_ticks * gap_mult)) {
                        fprintf(fp,
                                "%zu,IMU_GAP,%u,%u,%u,%" PRIu64 ",%" PRIu64 ",%lld,%zu,%u,%zu,%u,%u,%u\n",
                                sample_index, (unsigned)prev_ts, (unsigned)ts, (unsigned)delta_u, prev_us, cur_us,
                                (long long)delta_us_i, prev_frame, (unsigned)prev_sample, f, (unsigned)i, (unsigned)frame->n_imu,
                                (unsigned)expected_ticks);
                        anomalies++;
                    }
                }
            }

            memcpy(prev_raw, s->data, LOG_IMU_RAW_DATA_SIZE);
            prev_ts = ts;
            prev_frame = f;
            prev_sample = i;
            have_prev = 1;
            sample_index++;
        }
    }

    if (out_anomaly_count) {
        *out_anomaly_count = anomalies;
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

    bool any_bad = false;
    for (size_t f = 0; f < frame_count; f++) {
        const log_frame_t* frame = &frames[f];
        const uint8_t magic_ok_v = frame_magic_ok ? frame_magic_ok[f] : (frame->magic == LOG_FRAME_MAGIC);
        const uint8_t checksum_ok_v = frame_checksum_ok ? frame_checksum_ok[f] : 0;
        if (!magic_ok_v || !checksum_ok_v) {
            any_bad = true;
            break;
        }
    }

    // If everything is OK, do not create the file at all.
    if (!any_bad) {
        return CSV_OK;
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

        // Only write problematic frames.
        if (magic_ok_v && checksum_ok_v) {
            continue;
        }

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

csv_result_t write_timestamp_anomalies_csv(const char* filepath, const log_frame_t* frames, size_t frame_count,
                                           uint16_t adc_sample_rate_khz, uint16_t adc_block_size, const uint8_t* frame_magic_ok,
                                           const uint8_t* frame_checksum_ok, size_t* out_anomaly_count) {
    if (out_anomaly_count) {
        *out_anomaly_count = 0;
    }
    if (!filepath || !frames || frame_count == 0) {
        return CSV_ERROR_PARAM;
    }

    FILE* fp = fopen(filepath, "w");
    if (!fp) {
        return CSV_ERROR_FILE;
    }

    fprintf(fp,
            "frame_index,anomaly,prev_adc_ticks,adc_ticks,delta_ticks,expected_delta_ticks,prev_adc_us,adc_us,delta_us,n_imu,imu_min_ticks,imu_max_ticks,prev_imu_ticks,magic_ok,checksum_ok\n");

    bool have_prev_adc = false;
    uint32_t prev_adc = 0;

    bool have_prev_imu = false;
    uint32_t prev_imu = 0;

    size_t anomalies = 0;

    for (size_t f = 0; f < frame_count; f++) {
        const log_frame_t* frame = &frames[f];
        const uint8_t magic_ok_v = frame_magic_ok ? frame_magic_ok[f] : (frame->magic == LOG_FRAME_MAGIC);
        const uint8_t checksum_ok_v = frame_checksum_ok ? frame_checksum_ok[f] : 0;

        const uint32_t adc_ts = frame->adc_timestamp;
        const uint64_t adc_us = ticks_to_us_u64((uint64_t)adc_ts, adc_sample_rate_khz);

        if (have_prev_adc) {
            const int32_t delta_ticks = (int32_t)(adc_ts - prev_adc);
            const int64_t delta_us =
                (adc_sample_rate_khz == 0) ? 0 : (((int64_t)delta_ticks * 1000LL) / (int64_t)adc_sample_rate_khz);

            const uint64_t prev_adc_us = ticks_to_us_u64((uint64_t)prev_adc, adc_sample_rate_khz);

            if (adc_ts < prev_adc) {
                fprintf(fp, "%zu,ADC_TS_BACKWARDS,%u,%u,%ld,%u,%" PRIu64 ",%" PRIu64 ",%lld,%u,,,,%u,%u\n", f, (unsigned)prev_adc,
                        (unsigned)adc_ts, (long)delta_ticks, (unsigned)adc_block_size, prev_adc_us, adc_us, (long long)delta_us,
                        (unsigned)frame->n_imu, (unsigned)magic_ok_v, (unsigned)checksum_ok_v);
                anomalies++;
            }

            if (adc_block_size != 0 && delta_ticks > 0) {
                const uint32_t delta_u = (uint32_t)delta_ticks;
                const uint32_t rem = delta_u % (uint32_t)adc_block_size;
                // Allow small ±1 tick jitter (common when timestamps are captured in DMA callbacks).
                const bool ok_jitter = (rem == 0u) || (rem == 1u) || (rem == ((uint32_t)adc_block_size - 1u));
                if (!ok_jitter) {
                fprintf(fp, "%zu,ADC_DELTA_NOT_MULTIPLE_OF_BLOCK,%u,%u,%ld,%u,%" PRIu64 ",%" PRIu64 ",%lld,%u,,,,%u,%u\n", f,
                        (unsigned)prev_adc, (unsigned)adc_ts, (long)delta_ticks, (unsigned)adc_block_size, prev_adc_us, adc_us,
                        (long long)delta_us, (unsigned)frame->n_imu, (unsigned)magic_ok_v, (unsigned)checksum_ok_v);
                anomalies++;
                }
            }

            if (adc_ts <= 10u) {
                // Helpful hint for session resets where ticks restart near zero.
                fprintf(fp, "%zu,SESSION_RESET_NEAR_ZERO,%u,%u,%ld,%u,%" PRIu64 ",%" PRIu64 ",%lld,%u,,,,%u,%u\n", f, (unsigned)prev_adc,
                        (unsigned)adc_ts, (long)delta_ticks, (unsigned)adc_block_size, prev_adc_us, adc_us, (long long)delta_us,
                        (unsigned)frame->n_imu, (unsigned)magic_ok_v, (unsigned)checksum_ok_v);
                anomalies++;
            }
        }

        // IMU anomalies: only attempt if the frame itself is valid and contains IMU samples.
        if (magic_ok_v && checksum_ok_v) {
            uint16_t imu_count = frame->n_imu;
            if (imu_count > LOG_IMU_BLOCK_SIZE) {
                imu_count = LOG_IMU_BLOCK_SIZE;
            }

            if (imu_count > 0) {
                uint32_t imu_min = frame->imu[0].timestamp;
                uint32_t imu_max = frame->imu[0].timestamp;
                uint32_t prev_in_frame = frame->imu[0].timestamp;
                bool imu_in_frame_nonmonotonic = false;

                for (uint16_t i = 0; i < imu_count; i++) {
                    const uint32_t ts = frame->imu[i].timestamp;
                    if (ts < imu_min) imu_min = ts;
                    if (ts > imu_max) imu_max = ts;
                    if (i > 0 && ts < prev_in_frame) {
                        imu_in_frame_nonmonotonic = true;
                    }
                    prev_in_frame = ts;
                }

                if (have_prev_imu && imu_min < prev_imu) {
                    const uint64_t prev_adc_us = ticks_to_us_u64((uint64_t)prev_adc, adc_sample_rate_khz);
                    fprintf(fp,
                            "%zu,IMU_TS_BACKWARDS,%u,%u,,%u,%" PRIu64 ",%" PRIu64 ",,%u,%u,%u,%u,%u,%u\n", f, (unsigned)prev_adc,
                            (unsigned)adc_ts, (unsigned)adc_block_size, prev_adc_us, adc_us, (unsigned)frame->n_imu,
                            (unsigned)imu_min, (unsigned)imu_max, (unsigned)prev_imu, (unsigned)magic_ok_v, (unsigned)checksum_ok_v);
                    anomalies++;
                }

                if (imu_in_frame_nonmonotonic) {
                    const uint64_t prev_adc_us = ticks_to_us_u64((uint64_t)prev_adc, adc_sample_rate_khz);
                    fprintf(fp, "%zu,IMU_TS_NON_MONOTONIC_IN_FRAME,%u,%u,,%u,%" PRIu64 ",%" PRIu64 ",,%u,%u,%u,%u,%u,%u\n", f,
                            (unsigned)prev_adc, (unsigned)adc_ts, (unsigned)adc_block_size, prev_adc_us, adc_us, (unsigned)frame->n_imu,
                            (unsigned)imu_min, (unsigned)imu_max, (unsigned)prev_imu, (unsigned)magic_ok_v, (unsigned)checksum_ok_v);
                    anomalies++;
                }

                prev_imu = imu_max;
                have_prev_imu = true;
            }
        }

        prev_adc = adc_ts;
        have_prev_adc = true;
    }

    if (out_anomaly_count) {
        *out_anomaly_count = anomalies;
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
