#include "atlas_log.h"
#include "FreeRTOS.h"
#include "stream_buffer.h"
#include "usart.h"
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern int _write(int file, char* ptr, int len);

void atlas_log(char const* format, ...)
{
    static char buffer[300];

    char* log_buf = buffer;
    size_t log_buf_len = sizeof(buffer);
    bool use_heap_buf = false;

    va_list args;

    va_start(args, format);
    size_t log_len = vsnprintf(NULL, 0, format, args) + 1UL;
    va_end(args);

    if (log_len > log_buf_len) {
        log_buf = malloc(log_len);
        if (!log_buf)
            return;
        log_buf_len = log_len;
        use_heap_buf = true;
    }

    va_start(args, format);
    vsnprintf(log_buf, log_buf_len, format, args);
    va_end(args);

    _write(0, log_buf, strlen(log_buf));

    if (use_heap_buf) {
        free(log_buf);
    }
}
