#ifndef URL_UTILS_H
#define URL_UTILS_H

#include "CustomString.h"

namespace PsychicHttp {

/**
 * URL encode a string
 * @param value The string to encode
 * @return URL encoded string
 */
String urlEncode(const String &value);

/**
 * URL decode a string
 * @param value The string to decode
 * @return URL decoded string
 */
String urlDecode(const String &value);

/**
 * URL decode a C-style string (legacy compatibility)
 * @param encoded The C-style string to decode
 * @return URL decoded string
 */
String urlDecode(const char* encoded);

} // namespace PsychicHttp

#endif // URL_UTILS_H 