#include "fs_utils.h"
#include <cstring>
#include <ctime>

namespace PsychicHttp {

bool ESPFileSystem::exists(const String& path) const {
    struct stat st;
    return stat(path.c_str(), &st) == 0;
}

long ESPFileSystem::size(const String& path) const {
    struct stat st;
    if (stat(path.c_str(), &st) == 0) {
        return st.st_size;
    }
    return -1;
}

bool ESPFileSystem::isDirectory(const String& path) const {
    struct stat st;
    if (stat(path.c_str(), &st) == 0) {
        return S_ISDIR(st.st_mode);
    }
    return false;
}

ESPFile ESPFileSystem::open(const String& path, const String& mode) const {
    FILE* file = fopen(path.c_str(), mode.c_str());
    return ESPFile(file, path, false);
}

String ESPFileSystem::readFile(const String& path) const {
    ESPFile file = open(path);
    if (!file) {
        return "";
    }
    
    return file.readString();
}

String ESPFileSystem::getLastModified(const String& path) const {
    struct stat st;
    if (stat(path.c_str(), &st) == 0) {
        std::time_t time = st.st_mtime;
        std::tm* tm = std::gmtime(&time);

        char buf[40];
        if (std::strftime(buf, sizeof(buf), "%a, %d %b %Y %H:%M:%S GMT", tm)) {
            return String(buf);
        }
        return "";
    }
    return "";
}

// ESPFile implementation
ESPFile::ESPFile() : _file(nullptr), _isDirectory(false) {}

ESPFile::ESPFile(FILE* file, const String& path, bool isDirectory) 
    : _file(file), _path(path), _isDirectory(isDirectory) {}

ESPFile::~ESPFile() {
    close();
}
ESPFile::ESPFile(ESPFile&& other) noexcept 
    : _file(other._file), _path(std::move(other._path)), _isDirectory(other._isDirectory) {
    //Nullify pointer in the source object, so its destructor doesn't close the file
    other._file = nullptr;
}

// Operator move assignment
ESPFile& ESPFile::operator=(ESPFile&& other) noexcept {
    if (this != &other) {
        close(); // Close our own file if it was open
        
        // Move resources from other object
        _file = other._file;
        _path = std::move(other._path);
        _isDirectory = other._isDirectory;

        // Prevent double close
        other._file = nullptr;
    }
    return *this;
}

ESPFile::operator bool() const {
    return _file != nullptr;
}

size_t ESPFile::read(uint8_t* buffer, size_t size) {
    if (!_file) {
        return 0;
    }
    return fread(buffer, 1, size, _file);
}

size_t ESPFile::readBytes(char* buffer, size_t size) {
    if (!_file) {
        return 0;
    }
    return fread(buffer, 1, size, _file);
}

String ESPFile::readString() {
    if (!_file) {
        return "";
    }
    
    // Get current position
    long currentPos = ftell(_file);
    
    // Get file size
    fseek(_file, 0, SEEK_END);
    long fileSize = ftell(_file);
    fseek(_file, currentPos, SEEK_SET);
    
    // Calculate remaining bytes
    long remainingBytes = fileSize - currentPos;
    if (remainingBytes <= 0) {
        return "";
    }
    
    // Read remaining content
    String content;
    content.resize(remainingBytes);
    size_t bytesRead = fread(&content[0], 1, remainingBytes, _file);
    
    if (bytesRead != static_cast<size_t>(remainingBytes)) {
        content.resize(bytesRead);
    }
    
    return content;
}

long ESPFile::size() const {
    if (!_file) {
        return 0;
    }
    
    long currentPos = ftell(_file);
    fseek(const_cast<FILE*>(_file), 0, SEEK_END);
    long fileSize = ftell(const_cast<FILE*>(_file));
    fseek(const_cast<FILE*>(_file), currentPos, SEEK_SET);
    
    return fileSize;
}

bool ESPFile::isDirectory() const {
    return _isDirectory;
}

const String& ESPFile::path() const {
    return _path;
}

const char* ESPFile::name() const {
    // Extract filename from path
    size_t lastSlash = _path.find_last_of('/');
    if (lastSlash != std::string::npos) {
        return _path.c_str() + lastSlash + 1;
    }
    return _path.c_str();
}

void ESPFile::close() {
    if (_file) {
        fclose(_file);
        _file = nullptr;
    }
}

} // namespace PsychicHttp 