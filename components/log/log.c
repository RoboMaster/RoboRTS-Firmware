#include "log.h"

char *LOG_LEVEL_TAGS[6] = {"NULL", "ASSERT", "ERROR", "WARNING", "INFO", "DEBUG"};

uint8_t log_level = 5;

uint8_t get_global_log_level(void)
{
    return log_level;
}

void set_global_log_level(uint8_t level)
{
    log_level = level;
}

void log_printf(char *fmt, ...)
{
    char log_str[LOG_OUTPUT_MAX_LEN];
    int len = 0;
    va_list arg;
    va_start(arg, fmt);
    len += vsnprintf(log_str, LOG_OUTPUT_MAX_LEN - len, fmt, arg);
    va_end(arg);
    __log_output((uint8_t *)log_str, len);
}

int log_printf_to_buffer(char *buff, int size, char *fmt, ...)
{
    int len = 0;
    va_list arg;
    va_start(arg, fmt);
    len += vsnprintf(buff, size, fmt, arg);
    va_end(arg);
    return len;
}
