#ifndef Print_h
#define Print_h

#include <stdint.h>
#include <stddef.h>
#include "PsychicCore.h"

// Simple Print interface to replace Arduino's Print class
class Print {
public:
    virtual ~Print() = default;
    
    // Core write methods
    virtual size_t write(uint8_t c) = 0;
    virtual size_t write(const uint8_t *buffer, size_t size);
    
    // Convenience methods
    size_t write(const char *str);
    size_t print(const char *str);
    size_t print(const String &str);
    size_t println(const char *str);
    size_t println(const String &str);
    size_t println();
    
    // Number printing methods
    size_t print(int n, int base = 10);
    size_t print(unsigned int n, int base = 10);
    size_t print(long n, int base = 10);
    size_t print(unsigned long n, int base = 10);
    size_t print(double n, int digits = 2);
    
    size_t println(int n, int base = 10);
    size_t println(unsigned int n, int base = 10);
    size_t println(long n, int base = 10);
    size_t println(unsigned long n, int base = 10);
    size_t println(double n, int digits = 2);
    
    // Virtual flush method
    virtual void flush() {}
    
    // Get write error status
    int getWriteError() { return write_error; }
    void clearWriteError() { write_error = 0; }
    
protected:
    int write_error = 0;
    
    // Helper methods for number conversion
    size_t printNumber(unsigned long n, uint8_t base);
    size_t printFloat(double number, uint8_t digits);
};

// Simple Stream interface to replace Arduino's Stream class
class Stream : public Print {
public:
    virtual ~Stream() = default;
    
    virtual int available() = 0;
    virtual int read() = 0;
    virtual int peek() = 0;
    
    // Convenience methods
    size_t readBytes(char *buffer, size_t length);
    size_t readBytes(uint8_t *buffer, size_t length);
    
    // Helper methods
    int timedRead();
    void setReadError();
};

// Global functions
unsigned long millis();
void yield();

#endif 