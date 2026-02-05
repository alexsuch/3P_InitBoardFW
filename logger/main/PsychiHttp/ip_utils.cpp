#include "ip_utils.h"
#include "CustomString.h"
#include <cstring>
#include <lwip/inet.h>

namespace PsychicHttp {

IPAddress::IPAddress() : _valid(false) {
    memset(_address, 0, sizeof(_address));
}

IPAddress::IPAddress(uint8_t first, uint8_t second, uint8_t third, uint8_t fourth) : _valid(true) {
    _address[0] = first;
    _address[1] = second;
    _address[2] = third;
    _address[3] = fourth;
}

IPAddress::IPAddress(uint32_t address) : _valid(true) {
    _address[0] = (address >> 24) & 0xFF;
    _address[1] = (address >> 16) & 0xFF;
    _address[2] = (address >> 8) & 0xFF;
    _address[3] = address & 0xFF;
}

bool IPAddress::fromString(const String& address) {
    return fromString(address.c_str());
}

bool IPAddress::fromString(const char* address) {
    if (!address) {
        _valid = false;
        return false;
    }
    
    struct in_addr addr;
    if (inet_aton(address, &addr) == 1) {
        _address[0] = (addr.s_addr >> 24) & 0xFF;
        _address[1] = (addr.s_addr >> 16) & 0xFF;
        _address[2] = (addr.s_addr >> 8) & 0xFF;
        _address[3] = addr.s_addr & 0xFF;
        _valid = true;
        return true;
    }
    
    _valid = false;
    return false;
}

String IPAddress::toString() const {
    if (!_valid) {
        return "0.0.0.0";
    }
    char buf[16];
    snprintf(buf, sizeof(buf), "%u.%u.%u.%u",
             _address[0], _address[1], _address[2], _address[3]);
    return String(buf);
}

uint8_t IPAddress::operator[](int index) const {
    if (index >= 0 && index < 4) {
        return _address[index];
    }
    return 0;
}

bool IPAddress::isValid() const {
    return _valid;
}

uint32_t IPAddress::toInt() const {
    if (!_valid) {
        return 0;
    }
    
    return (static_cast<uint32_t>(_address[0]) << 24) |
           (static_cast<uint32_t>(_address[1]) << 16) |
           (static_cast<uint32_t>(_address[2]) << 8) |
           static_cast<uint32_t>(_address[3]);
}

bool IPAddress::operator==(const IPAddress& other) const {
    if (!_valid || !other._valid) {
        return false;
    }
    
    return memcmp(_address, other._address, sizeof(_address)) == 0;
}

bool IPAddress::operator!=(const IPAddress& other) const {
    return !(*this == other);
}

} // namespace PsychicHttp 