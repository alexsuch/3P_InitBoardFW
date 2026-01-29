#include <emscripten.h>
#include <stdint.h>
#include <string.h>

// Constants matching JavaScript version
#define HEADER_SIZE 64
#define FRAME_SIZE 852
#define LOG_CONFIG_MAGIC 0xCAFE
#define LOG_FRAME_MAGIC 0x5A5A
#define ADC_BLOCK_SIZE 256
#define IMU_BLOCK_SIZE 20

// Result structure for binary parsing
typedef struct {
    uint32_t adc_count;
    uint32_t imu_count;
    uint16_t adc_rate_khz;
} ParseResult;

// Helper: Read uint16 little-endian
static inline uint16_t read_u16_le(const uint8_t* data) { return (uint16_t)data[0] | ((uint16_t)data[1] << 8); }

// Helper: Read int16 little-endian
static inline int16_t read_i16_le(const uint8_t* data) { return (int16_t)(data[0] | (data[1] << 8)); }

// Helper: Read uint32 little-endian
static inline uint32_t read_u32_le(const uint8_t* data) {
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8) | ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
}

/**
 * Parse binary log file and extract ADC/IMU data.
 *
 * @param buffer        Input binary buffer
 * @param buffer_len    Length of input buffer
 * @param max_frames    Maximum frames to parse (0 = unlimited)
 * @param adc_time      Output: ADC timestamps (pre-allocated)
 * @param adc_val       Output: ADC values (pre-allocated)
 * @param imu_time      Output: IMU timestamps (pre-allocated)
 * @param imu_gx/gy/gz  Output: Gyro data (pre-allocated)
 * @param imu_ax/ay/az  Output: Accel data (pre-allocated)
 * @param out_adc_count Output: Number of ADC samples written
 * @param out_imu_count Output: Number of IMU samples written
 * @return ADC rate in kHz, or 0 on error
 */
EMSCRIPTEN_KEEPALIVE
uint32_t parse_binary_log(const uint8_t* buffer, uint32_t buffer_len, uint32_t max_frames, double* adc_time, double* adc_val, double* imu_time, double* imu_gx,
    double* imu_gy, double* imu_gz, double* imu_ax, double* imu_ay, double* imu_az, uint32_t* out_adc_count, uint32_t* out_imu_count) {
    if (buffer_len < HEADER_SIZE) {
        *out_adc_count = 0;
        *out_imu_count = 0;
        return 0;
    }

    // Read header
    uint16_t magic = read_u16_le(buffer);
    if (magic != LOG_CONFIG_MAGIC) {
        *out_adc_count = 0;
        *out_imu_count = 0;
        return 0;
    }

    uint16_t adc_rate_khz = read_u16_le(buffer + 4);
    double sample_period_us = 1000.0 / (adc_rate_khz > 0 ? adc_rate_khz : 100);

    uint32_t adc_idx = 0;
    uint32_t imu_idx = 0;
    uint32_t offset = HEADER_SIZE;
    uint32_t frame_count = 0;

    while (offset + FRAME_SIZE <= buffer_len) {
        // Check frame limit
        if (max_frames > 0 && frame_count >= max_frames) {
            break;
        }

        const uint8_t* frame = buffer + offset;

        uint16_t f_magic = read_u16_le(frame);
        if (f_magic != LOG_FRAME_MAGIC) {
            offset += FRAME_SIZE;
            continue;
        }

        uint16_t n_imu = read_u16_le(frame + 2);
        uint32_t adc_base_time = read_u32_le(frame + 4);

        // Extract ADC data (256 samples per frame)
        const uint8_t* adc_data = frame + 8;
        for (uint32_t i = 0; i < ADC_BLOCK_SIZE; i++) {
            adc_time[adc_idx] = (double)(adc_base_time + i) * sample_period_us;
            adc_val[adc_idx] = (double)read_i16_le(adc_data + i * 2);
            adc_idx++;
        }

        // Extract IMU data (up to 20 samples per frame)
        const uint8_t* imu_data = frame + 520;
        uint32_t valid_imu = n_imu < IMU_BLOCK_SIZE ? n_imu : IMU_BLOCK_SIZE;

        for (uint32_t i = 0; i < valid_imu; i++) {
            const uint8_t* sample = imu_data + i * 16;

            imu_gx[imu_idx] = (double)read_i16_le(sample + 0);
            imu_gy[imu_idx] = (double)read_i16_le(sample + 2);
            imu_gz[imu_idx] = (double)read_i16_le(sample + 4);
            imu_ax[imu_idx] = (double)read_i16_le(sample + 6);
            imu_ay[imu_idx] = (double)read_i16_le(sample + 8);
            imu_az[imu_idx] = (double)read_i16_le(sample + 10);

            uint32_t ts = read_u32_le(sample + 12);
            imu_time[imu_idx] = (double)ts * sample_period_us;

            imu_idx++;
        }

        offset += FRAME_SIZE;
        frame_count++;
    }

    *out_adc_count = adc_idx;
    *out_imu_count = imu_idx;
    return adc_rate_khz;
}

/**
 * Calculate anomalies (time gaps > threshold).
 *
 * @param x_data          Input: timestamp array
 * @param length          Length of x_data
 * @param threshold_us    Time gap threshold in microseconds
 * @param anomaly_indices Output: indices where anomalies occur
 * @param anomaly_diffs   Output: actual time differences
 * @param max_anomalies   Maximum anomalies to return
 * @return Number of anomalies found
 */
EMSCRIPTEN_KEEPALIVE
uint32_t calculate_anomalies(
    const double* x_data, uint32_t length, double threshold_us, uint32_t* anomaly_indices, double* anomaly_diffs, uint32_t max_anomalies) {
    uint32_t count = 0;

    for (uint32_t i = 1; i < length && count < max_anomalies; i++) {
        double diff = x_data[i] - x_data[i - 1];
        if (diff > threshold_us) {
            anomaly_indices[count] = i;
            anomaly_diffs[count] = diff;
            count++;
        }
    }

    return count;
}

/**
 * Interpolate source data to master time base using sample-and-hold.
 *
 * @param src_x      Source timestamps
 * @param src_y      Source values
 * @param src_len    Length of source arrays
 * @param master_x   Master timestamps
 * @param master_len Length of master array
 * @param result     Output: interpolated values
 */
EMSCRIPTEN_KEEPALIVE
void interpolate_to_master(const double* src_x, const double* src_y, uint32_t src_len, const double* master_x, uint32_t master_len, double* result) {
    if (src_len == 0) {
        for (uint32_t i = 0; i < master_len; i++) {
            result[i] = 0.0 / 0.0;  // NaN
        }
        return;
    }

    double src_first = src_x[0];
    double src_last = src_x[src_len - 1];

    for (uint32_t i = 0; i < master_len; i++) {
        double t = master_x[i];

        if (t < src_first || t > src_last) {
            result[i] = 0.0 / 0.0;  // NaN
            continue;
        }

        // Binary search for position
        uint32_t lo = 0, hi = src_len - 1;
        while (lo < hi) {
            uint32_t mid = (lo + hi + 1) >> 1;
            if (src_x[mid] <= t) {
                lo = mid;
            } else {
                hi = mid - 1;
            }
        }

        // Sample-and-hold
        result[i] = src_y[lo];
    }
}

// Keep the simple hello/add functions for testing
EMSCRIPTEN_KEEPALIVE
const char* hello_world() { return "Hello from Wasm!"; }

EMSCRIPTEN_KEEPALIVE
int add(int a, int b) { return a + b; }
