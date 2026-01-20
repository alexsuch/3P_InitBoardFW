#ifndef CUSTOM_STRING_H
#define CUSTOM_STRING_H

// ESP-IDF compatible includes
#include <cctype>
#include <cstring>
#include <string>

// Custom String class extending std::string for Arduino compatibility
class String : public std::string {
   public:
    // Default constructors
    String() : std::string() {}
    String(const std::string& str) : std::string(str) {}
    String(const char* str) : std::string(str ? str : "") {}
    String(const char* str, size_t len) : std::string(str, len) {}
    String(char c) : std::string(1, c) {}
    String(size_t count, char ch) : std::string(count, ch) {}
    String(uint32_t value) : std::string(std::to_string(value)) {}
    String(int value) : std::string(std::to_string(value)) {}
    String(long value) : std::string(std::to_string(value)) {}
    // Note: unsigned long is same as uint32_t on ESP32, so we skip it to avoid conflict

    // Arduino String compatibility methods
    bool endsWith(const String& suffix) const {
        if (suffix.length() > this->length()) return false;
        return this->substr(this->length() - suffix.length()) == suffix;
    }

    bool endsWith(const char* suffix) const { return endsWith(String(suffix)); }

    bool startsWith(const String& prefix) const {
        if (prefix.length() > this->length()) return false;
        return this->substr(0, prefix.length()) == prefix;
    }

    bool startsWith(const char* prefix) const { return startsWith(String(prefix)); }

    bool equals(const String& str) const { return *this == str; }

    bool equals(const char* str) const { return *this == String(str); }

    bool equalsIgnoreCase(const String& str) const {
        if (this->length() != str.length()) return false;
        for (size_t i = 0; i < this->length(); i++) {
            if (tolower((*this)[i]) != tolower(str[i])) return false;
        }
        return true;
    }

    bool equalsIgnoreCase(const char* str) const { return equalsIgnoreCase(String(str)); }

    int lastIndexOf(char ch) const {
        size_t pos = this->rfind(ch);
        return pos == std::string::npos ? -1 : static_cast<int>(pos);
    }

    int lastIndexOf(const String& str) const {
        size_t pos = this->rfind(str);
        return pos == std::string::npos ? -1 : static_cast<int>(pos);
    }

    int lastIndexOf(const char* str) const { return lastIndexOf(String(str)); }

    // Arduino String compatibility: indexOf
    int indexOf(char ch, size_t from = 0) const {
        size_t pos = this->find(ch, from);
        return pos == std::string::npos ? -1 : static_cast<int>(pos);
    }
    int indexOf(const String& str, size_t from = 0) const {
        size_t pos = this->find(str, from);
        return pos == std::string::npos ? -1 : static_cast<int>(pos);
    }
    int indexOf(const char* str, size_t from = 0) const { return indexOf(String(str), from); }

    // Arduino String compatibility: substring
    String substring(size_t beginIndex, size_t endIndex = std::string::npos) const {
        if (beginIndex > this->length()) return String("");
        if (endIndex == std::string::npos || endIndex > this->length()) endIndex = this->length();
        if (endIndex <= beginIndex) return String("");
        return this->substr(beginIndex, endIndex - beginIndex);
    }

    String& concat(const String& str) {
        this->append(str);
        return *this;
    }

    String& concat(const char* str) {
        this->append(str ? str : "");
        return *this;
    }

    String& concat(char c) {
        this->push_back(c);
        return *this;
    }

    String& concat(int value) {
        this->append(std::to_string(value));
        return *this;
    }

    String& concat(unsigned int value) {
        this->append(std::to_string(value));
        return *this;
    }

    String& concat(long value) {
        this->append(std::to_string(value));
        return *this;
    }

    // Note: unsigned long concat removed to avoid conflict with uint32_t

    // Operator overloads for concatenation
    String& operator+=(const String& str) { return concat(str); }

    String& operator+=(const char* str) { return concat(str); }

    String& operator+=(char c) { return concat(c); }

    String& operator+=(int value) { return concat(value); }

    String& operator+=(unsigned int value) { return concat(value); }

    String& operator+=(long value) { return concat(value); }

    // Note: unsigned long operator+= removed to avoid conflict with uint32_t

    // Replace all occurrences of a character with another character
    void replace(char find, char replaceWith) {
        for (size_t i = 0; i < this->length(); ++i) {
            if ((*this)[i] == find) {
                (*this)[i] = replaceWith;
            }
        }
    }

    // Replace all occurrences of a substring with another substring
    void replace(const String& find, const String& replace) {
        std::string& self = *this;
        std::string sfind = find;
        std::string sreplace = replace;
        size_t pos = 0;
        while ((pos = self.find(sfind, pos)) != std::string::npos) {
            self.replace(pos, sfind.length(), sreplace);
            pos += sreplace.length();
        }
    }

    // Replace all occurrences of a C-string with another C-string
    void replace(const char* find, const char* replaceWith) {
        std::string& self = *this;
        std::string sfind = find ? find : "";
        std::string sreplace = replaceWith ? replaceWith : "";
        size_t pos = 0;
        while ((pos = self.find(sfind, pos)) != std::string::npos) {
            self.replace(pos, sfind.length(), sreplace);
            pos += sreplace.length();
        }
    }

    // ArduinoJson compatibility: write method for serialization
    size_t write(uint8_t c) {
        this->push_back(static_cast<char>(c));
        return 1;
    }

    size_t write(const uint8_t* buffer, size_t size) {
        this->append(reinterpret_cast<const char*>(buffer), size);
        return size;
    }

    size_t write(const char* str) {
        if (str == nullptr) return 0;
        // Use string length instead of strlen to avoid include issues
        size_t len = 0;
        while (str[len] != '\0') len++;
        this->append(str, len);
        return len;
    }
};

#endif  // STRING_H