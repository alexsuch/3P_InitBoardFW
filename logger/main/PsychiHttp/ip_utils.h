#ifndef IP_UTILS_H
#define IP_UTILS_H

#include "CustomString.h"
#include <cstdint>

namespace PsychicHttp {

/**
 * IP Address wrapper for ESP-IDF (replaces Arduino IPAddress)
 */
class IPAddress {
private:
    uint8_t _address[4];
    bool _valid;

public:
    IPAddress();
    IPAddress(uint8_t first, uint8_t second, uint8_t third, uint8_t fourth);
    IPAddress(uint32_t address);
    
    /**
     * Parse IP address from string
     * @param address IP address string (e.g., "192.168.1.1")
     * @return true if parsing successful
     */
    bool fromString(const String& address);
    bool fromString(const char* address);
    
    /**
     * Convert to string
     * @return IP address as string
     */
    String toString() const;
    
    /**
     * Get octet at index
     * @param index Octet index (0-3)
     * @return Octet value
     */
    uint8_t operator[](int index) const;
    
    /**
     * Check if IP address is valid
     * @return true if valid
     */
    bool isValid() const;
    
    /**
     * Convert to 32-bit integer
     * @return IP address as 32-bit integer
     */
    uint32_t toInt() const;
    
    /**
     * Equality comparison
     */
    bool operator==(const IPAddress& other) const;
    bool operator!=(const IPAddress& other) const;
};

} // namespace PsychicHttp

#endif // IP_UTILS_H 