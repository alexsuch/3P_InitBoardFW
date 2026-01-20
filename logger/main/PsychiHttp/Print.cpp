#include "Print.h"
#include <string>
#include <cstring>

#include <cmath>

size_t Print::write(const uint8_t *buffer, size_t size) {
    size_t n = 0;
    while (size--) {
        if (write(*buffer++)) n++;
        else break;
    }
    return n;
}

size_t Print::write(const char *str) {
    if (str == nullptr) return 0;
    return write((const uint8_t *)str, strlen(str));
}

size_t Print::print(const char *str) {
    return write(str);
}

size_t Print::print(const String &str) {
    return write(str.c_str());
}

size_t Print::println(const char *str) {
    size_t n = print(str);
    n += println();
    return n;
}

size_t Print::println(const String &str) {
    size_t n = print(str);
    n += println();
    return n;
}

size_t Print::println() {
    return write('\r') + write('\n');
}

size_t Print::print(int n, int base) {
    return print((long)n, base);
}

size_t Print::print(unsigned int n, int base) {
    return print((unsigned long)n, base);
}

size_t Print::print(long n, int base) {
    if (base == 0) {
        return write(n);
    } else if (base == 10) {
        if (n < 0) {
            int t = print('-');
            n = -n;
            return printNumber(n, 10) + t;
        }
        return printNumber(n, 10);
    } else {
        return printNumber(n, base);
    }
}

size_t Print::print(unsigned long n, int base) {
    if (base == 0) return write(n);
    else return printNumber(n, base);
}

size_t Print::print(double n, int digits) {
    return printFloat(n, digits);
}

size_t Print::println(int n, int base) {
    size_t len = print(n, base);
    len += println();
    return len;
}

size_t Print::println(unsigned int n, int base) {
    size_t len = print(n, base);
    len += println();
    return len;
}

size_t Print::println(long n, int base) {
    size_t len = print(n, base);
    len += println();
    return len;
}

size_t Print::println(unsigned long n, int base) {
    size_t len = print(n, base);
    len += println();
    return len;
}

size_t Print::println(double n, int digits) {
    size_t len = print(n, digits);
    len += println();
    return len;
}

size_t Print::printNumber(unsigned long n, uint8_t base) {
    char buf[8 * sizeof(long) + 1]; // Assumes 8-bit chars plus zero byte.
    char *str = &buf[sizeof(buf) - 1];

    *str = '\0';

    // prevent crash if called with base == 1
    if (base < 2) base = 10;

    do {
        char c = n % base;
        n /= base;

        *--str = c < 10 ? c + '0' : c + 'A' - 10;
    } while(n);

    return write(str);
}

size_t Print::printFloat(double number, uint8_t digits) {
    size_t n = 0;

    if (std::isnan(number)) return print("nan");
    if (std::isinf(number)) return print("inf");
    if (number > 4294967040.0) return print("ovf");  // constant determined empirically
    if (number < -4294967040.0) return print("ovf");  // constant determined empirically

    // Handle negative numbers
    if (number < 0.0) {
        n += print('-');
        number = -number;
    }

    // Round correctly so that print(1.999, 2) prints as "2.00"
    double rounding = 0.5;
    for (uint8_t i = 0; i < digits; ++i)
        rounding /= 10.0;

    number += rounding;

    // Extract the integer part of the number and print it
    unsigned long int_part = (unsigned long)number;
    double remainder = number - (double)int_part;
    n += print(int_part);

    // Print the decimal point, but only if there are digits beyond
    if (digits > 0) {
        n += print('.');
    }

    // Extract digits from the remainder one at a time
    while (digits-- > 0) {
        remainder *= 10.0;
        unsigned int toPrint = (unsigned int)(remainder);
        n += print(toPrint);
        remainder -= toPrint;
    }

    return n;
}

// Stream implementation
size_t Stream::readBytes(char *buffer, size_t length) {
    size_t count = 0;
    while (count < length) {
        int c = timedRead();
        if (c < 0) {
            setReadError();
            break;
        }
        *buffer++ = (char)c;
        count++;
    }
    return count;
}

size_t Stream::readBytes(uint8_t *buffer, size_t length) {
    return readBytes((char *)buffer, length);
}

// Helper functions for Stream
int Stream::timedRead() {
    int c;
    unsigned long startMillis = millis();
    do {
        c = read();
        if (c >= 0) return c;
        yield();
    } while(millis() - startMillis < 2000);
    return -1;     // -1 indicates timeout
}

void Stream::setReadError() {
    // Implementation for read error handling
    // Could set a flag or handle error state
}

// Global millis() function (simple implementation)
unsigned long millis() {
    // This would need to be implemented based on your system
    // For now, return a simple counter
    return xTaskGetTickCount() * portTICK_PERIOD_MS;    
}

// Global yield() function
void yield() {
    // Allow other tasks to run
    // This is a simple implementation
} 