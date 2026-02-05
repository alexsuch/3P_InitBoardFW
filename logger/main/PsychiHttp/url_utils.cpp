#include "url_utils.h"
#include "CustomString.h"
#include <cstring>
#include <sstream>
#include <iomanip>

namespace PsychicHttp {

String urlEncode(const String &value) {
    String escaped;
    
    for (size_t i = 0; i < value.length(); i++) {
        char c = value[i];
        if (isalnum(static_cast<unsigned char>(c)) || c == '-' || c == '_' || c == '.' || c == '~') {
            escaped += c;
        } else {
            char hex[4];
            snprintf(hex, sizeof(hex), "%%%02X", static_cast<unsigned char>(c));
            escaped += hex;
        }
    }
    return escaped;
}

String urlDecode(const String &value) {
    String result;
    size_t i = 0;
    
    while (i < value.length()) {
        if (value[i] == '%' && i + 2 < value.length()) {
            String hex = value.substring(i + 1, i + 3);
            char decoded_char = static_cast<char>(strtol(hex.c_str(), nullptr, 16));
            result += decoded_char;
            i += 3;
        } else if (value[i] == '+') {
            result += ' ';
            i++;
        } else {
            result += value[i];
            i++;
        }
    }
    return result;
}

String urlDecode(const char* encoded) {
    if (encoded == nullptr) {
        return "";
    }
    return urlDecode(String(encoded));
}

} // namespace PsychicHttp 