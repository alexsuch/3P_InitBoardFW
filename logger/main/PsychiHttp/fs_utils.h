#ifndef FS_UTILS_H
#define FS_UTILS_H

#include "CustomString.h"
#include <cstdio>
#include <sys/stat.h>

namespace PsychicHttp
{

    // Forward declaration
    class ESPFile;

    /**
     * File system abstraction for ESP-IDF
     * Replaces Arduino's fs::FS and fs::File
     * Simplified wrapper around standard C file operations
     */
    class ESPFileSystem
    {
    public:
        ESPFileSystem() = default;

        /**
         * Check if file exists
         * @param path File path
         * @return true if file exists
         */
        bool exists(const String &path) const;

        /**
         * Get file size
         * @param path File path
         * @return File size in bytes, -1 if error
         */
        long size(const String &path) const;

        /**
         * Check if path is a directory
         * @param path Path to check
         * @return true if directory
         */
        bool isDirectory(const String &path) const;

        /**
         * Open file for reading
         * @param path File path
         * @param mode File mode (e.g., "r", "w", "rb", "wb") - for compatibility
         * @return ESPFile object
         */
        ESPFile open(const String &path, const String &mode = "rb") const;

        /**
         * Read file content as string
         * @param path File path
         * @return File content as string
         */
        String readFile(const String &path) const;

        /**
         * Get file modification time
         * @param path File path
         * @return Modification time as string
         */
        String getLastModified(const String &path) const;
    };

    /**
     * File wrapper class (replaces Arduino's File)
     */
    class ESPFile
    {
    private:
        FILE *_file;
        String _path;
        bool _isDirectory;

    public:
        ESPFile();
        ESPFile(FILE *file, const String &path, bool isDirectory = false);
        ~ESPFile();

        ESPFile(const ESPFile &) = delete;
        ESPFile &operator=(const ESPFile &) = delete;

        ESPFile(ESPFile&& other) noexcept;
        ESPFile& operator=(ESPFile&& other) noexcept;
        /**
         * Check if file is valid
         * @return true if file is open
         */
        operator bool() const;

        /**
         * Read bytes from file
         * @param buffer Buffer to read into
         * @param size Number of bytes to read
         * @return Number of bytes read
         */
        size_t read(uint8_t *buffer, size_t size);

        /**
         * Read bytes from file (compatibility method)
         * @param buffer Buffer to read into
         * @param size Number of bytes to read
         * @return Number of bytes read
         */
        size_t readBytes(char *buffer, size_t size);

        /**
         * Read string from file
         * @return File content as string
         */
        String readString();

        /**
         * Get file size
         * @return File size in bytes
         */
        long size() const;

        /**
         * Check if file is directory
         * @return true if directory
         */
        bool isDirectory() const;

        /**
         * Get file path
         * @return File path
         */
        const String &path() const;

        /**
         * Get file name (compatibility method)
         * @return File name
         */
        const char *name() const;

        /**
         * Close file
         */
        void close();
    };

} // namespace PsychicHttp

#endif // FS_UTILS_H