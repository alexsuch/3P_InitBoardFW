/**
 * @file log_converter.c
 * @brief Logger Binary to CSV Converter - Main Application
 *
 * Converts binary log files from 3P Logger to human-readable CSV format.
 *
 * Usage:
 *   log_converter <input> <output_folder>
 *
 * Arguments:
 *   input          - Binary log file (.dat) OR folder containing .dat files
 *   output_folder  - Destination folder for converted CSV files
 *
 * Output Structure:
 *   For each input .dat file, creates: <output>/<filename>/
 *     - config.csv       - Logger configuration
 *     - adc_data.csv     - ADC piezo sensor data
 *     - imu_data.csv     - IMU gyro/accel data
 *     - mavlink_events.csv - MAVLink telemetry events
 */

#include "csv_writer.h"
#include "log_parser.h"
#include <dirent.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

#ifdef _WIN32
#include <direct.h>
#include <windows.h>
#define PATH_SEPARATOR '\\'
#define mkdir(path, mode) _mkdir(path)
#else
#include <unistd.h>
#define PATH_SEPARATOR '/'
#endif

/* ============================================================================
 * CONFIGURATION
 * ============================================================================
 */

#define MAX_PATH_LEN 1024
#define MAX_FRAMES 100000 // Max frames to process per file

/* ============================================================================
 * HELPER FUNCTIONS
 * ============================================================================
 */

/**
 * @brief Check if path is a directory
 */
static int is_directory(const char *path) {
  struct stat st;
  if (stat(path, &st) != 0) {
    return 0;
  }
  return S_ISDIR(st.st_mode);
}

/**
 * @brief Check if path is a file
 */
static int is_file(const char *path) {
  struct stat st;
  if (stat(path, &st) != 0) {
    return 0;
  }
  return S_ISREG(st.st_mode);
}

/**
 * @brief Extract filename without extension from path
 */
static void get_basename(const char *path, char *basename, size_t size) {
  const char *filename = strrchr(path, PATH_SEPARATOR);
  if (!filename) {
    filename = strrchr(path, '/'); // Handle mixed separators
  }
  if (!filename) {
    filename = path;
  } else {
    filename++; // Skip separator
  }

  strncpy(basename, filename, size - 1);
  basename[size - 1] = '\0';

  // Remove extension
  char *dot = strrchr(basename, '.');
  if (dot) {
    *dot = '\0';
  }
}

/**
 * @brief Check if filename ends with .dat (case insensitive)
 */
static int is_dat_file(const char *filename) {
  size_t len = strlen(filename);
  if (len < 4)
    return 0;

  const char *ext = filename + len - 4;
  return (strcasecmp(ext, ".dat") == 0);
}

/**
 * @brief Create nested directories
 */
static int create_directories(const char *path) {
  char tmp[MAX_PATH_LEN];
  char *p = NULL;
  size_t len;

  snprintf(tmp, sizeof(tmp), "%s", path);
  len = strlen(tmp);

  // Remove trailing separator
  if (tmp[len - 1] == PATH_SEPARATOR || tmp[len - 1] == '/') {
    tmp[len - 1] = '\0';
  }

  // Create each directory in path
  for (p = tmp + 1; *p; p++) {
    if (*p == PATH_SEPARATOR || *p == '/') {
      *p = '\0';
      create_directory(tmp);
      *p = PATH_SEPARATOR;
    }
  }

  return create_directory(tmp);
}

/* ============================================================================
 * CONVERSION LOGIC
 * ============================================================================
 */

/**
 * @brief Convert a single binary log file to CSV files
 */
static int convert_file(const char *input_path, const char *output_dir) {
  printf("Converting: %s\n", input_path);

  // Open input file
  FILE *fp = fopen(input_path, "rb");
  if (!fp) {
    fprintf(stderr, "Error: Cannot open file: %s\n", input_path);
    return -1;
  }

  // Get file size and calculate frame count
  long file_size = get_file_size(fp);
  if (file_size < 0) {
    fprintf(stderr, "Error: Cannot get file size\n");
    fclose(fp);
    return -1;
  }

  size_t expected_frames = calculate_frame_count(file_size);
  printf("  File size: %ld bytes, expected frames: %zu\n", file_size,
         expected_frames);

  // Read configuration
  log_config_t config;
  parse_result_t result = parse_config(fp, &config);
  if (result != PARSE_OK) {
    fprintf(stderr, "Error: Failed to parse config (error %d)\n", result);
    fclose(fp);
    return -1;
  }

  printf("  Config: magic=0x%04X, version=%u.%u, ADC=%ukHz, block=%u\n",
         config.magic, config.version_major, config.version_minor,
         config.adc_sample_rate_khz, config.adc_block_size);

  // Allocate frame buffer
  size_t frame_count = expected_frames;
  if (frame_count > MAX_FRAMES) {
    fprintf(stderr, "Warning: Limiting to %d frames\n", MAX_FRAMES);
    frame_count = MAX_FRAMES;
  }

  log_frame_t *frames =
      (log_frame_t *)malloc(frame_count * sizeof(log_frame_t));
  if (!frames) {
    fprintf(stderr, "Error: Out of memory (need %zu bytes)\n",
            frame_count * sizeof(log_frame_t));
    fclose(fp);
    return -1;
  }

  // Read all frames
  size_t frames_read = 0;
  size_t frames_invalid = 0;
  for (size_t i = 0; i < frame_count; i++) {
    result = parse_frame(fp, &frames[frames_read]);
    if (result == PARSE_ERROR_EOF) {
      break;
    }
    if (result != PARSE_OK) {
      frames_invalid++;
      continue;
    }
    frames_read++;
  }

  fclose(fp);

  printf("  Frames read: %zu, invalid: %zu\n", frames_read, frames_invalid);

  if (frames_read == 0) {
    fprintf(stderr, "Error: No valid frames found\n");
    free(frames);
    return -1;
  }

  // Create output directory
  char basename[256];
  get_basename(input_path, basename, sizeof(basename));

  char output_path[MAX_PATH_LEN];
  snprintf(output_path, sizeof(output_path), "%s%c%s", output_dir,
           PATH_SEPARATOR, basename);

  if (create_directories(output_path) != 0) {
    fprintf(stderr, "Error: Cannot create output directory: %s\n", output_path);
    free(frames);
    return -1;
  }

  // Write CSV files - use larger buffer to avoid truncation warning
  // MAX_PATH_LEN for output_path + 1 for separator + up to 20 for filename
  char filepath[MAX_PATH_LEN + 32];
  csv_result_t csv_result;

  // config.csv
  snprintf(filepath, sizeof(filepath), "%s%cconfig.csv", output_path,
           PATH_SEPARATOR);
  csv_result = write_config_csv(filepath, &config);
  if (csv_result != CSV_OK) {
    fprintf(stderr, "Error: Failed to write config.csv\n");
  } else {
    printf("  Written: config.csv\n");
  }

  // adc_data.csv
  snprintf(filepath, sizeof(filepath), "%s%cadc_data.csv", output_path,
           PATH_SEPARATOR);
  csv_result =
      write_adc_csv(filepath, frames, frames_read, config.adc_sample_rate_khz);
  if (csv_result != CSV_OK) {
    fprintf(stderr, "Error: Failed to write adc_data.csv\n");
  } else {
    printf("  Written: adc_data.csv (%zu samples)\n",
           frames_read * LOG_ADC_BLOCK_SIZE);
  }

  // imu_data.csv
  snprintf(filepath, sizeof(filepath), "%s%cimu_data.csv", output_path,
           PATH_SEPARATOR);
  csv_result = write_imu_csv(filepath, frames, frames_read, config.adc_sample_rate_khz);
  if (csv_result != CSV_OK) {
    fprintf(stderr, "Error: Failed to write imu_data.csv\n");
  } else {
    // Count total IMU samples
    size_t imu_count = 0;
    for (size_t i = 0; i < frames_read; i++) {
      imu_count += frames[i].n_imu;
    }
    printf("  Written: imu_data.csv (%zu samples)\n", imu_count);
  }

  // mavlink_events.csv
  snprintf(filepath, sizeof(filepath), "%s%cmavlink_events.csv", output_path,
           PATH_SEPARATOR);
  csv_result = write_mavlink_csv(filepath, frames, frames_read, config.adc_sample_rate_khz);
  if (csv_result != CSV_OK) {
    fprintf(stderr, "Error: Failed to write mavlink_events.csv\n");
  } else {
    printf("  Written: mavlink_events.csv\n");
  }

  free(frames);
  printf("  Conversion complete: %s\n\n", output_path);

  return 0;
}

/**
 * @brief Process all .dat files in a directory
 */
static int convert_directory(const char *input_dir, const char *output_dir) {
  DIR *dir = opendir(input_dir);
  if (!dir) {
    fprintf(stderr, "Error: Cannot open directory: %s\n", input_dir);
    return -1;
  }

  int files_converted = 0;
  int files_failed = 0;
  struct dirent *entry;

  while ((entry = readdir(dir)) != NULL) {
    if (!is_dat_file(entry->d_name)) {
      continue;
    }

    char filepath[MAX_PATH_LEN];
    snprintf(filepath, sizeof(filepath), "%s%c%s", input_dir, PATH_SEPARATOR,
             entry->d_name);

    if (convert_file(filepath, output_dir) == 0) {
      files_converted++;
    } else {
      files_failed++;
    }
  }

  closedir(dir);

  printf("Summary: %d files converted, %d failed\n", files_converted,
         files_failed);
  return (files_failed > 0) ? -1 : 0;
}

/* ============================================================================
 * MAIN
 * ============================================================================
 */

static void print_usage(const char *program) {
  printf("Usage: %s <input> <output_folder>\n\n", program);
  printf("Arguments:\n");
  printf("  input          - Binary log file (.dat) OR folder containing .dat "
         "files\n");
  printf("  output_folder  - Destination folder for converted CSV files\n\n");
  printf("Examples:\n");
  printf("  %s data_0.dat ./output\n", program);
  printf("  %s ./logs ./output\n", program);
}

int main(int argc, char *argv[]) {
  printf("3P Logger Binary to CSV Converter v1.0\n");
  printf("======================================\n\n");

  if (argc != 3) {
    print_usage(argv[0]);
    return 1;
  }

  const char *input_path = argv[1];
  const char *output_dir = argv[2];

  // Create output directory
  if (create_directories(output_dir) != 0) {
    fprintf(stderr, "Error: Cannot create output directory: %s\n", output_dir);
    return 1;
  }

  int result;
  if (is_directory(input_path)) {
    printf("Processing directory: %s\n\n", input_path);
    result = convert_directory(input_path, output_dir);
  } else if (is_file(input_path)) {
    result = convert_file(input_path, output_dir);
  } else {
    fprintf(stderr, "Error: Input path does not exist: %s\n", input_path);
    return 1;
  }

  return (result == 0) ? 0 : 1;
}
