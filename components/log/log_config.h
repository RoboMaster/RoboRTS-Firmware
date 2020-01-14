#ifndef __LOG_CONFIG_H__
#define __LOG_CONFIG_H__

#include "sys.h"
#include "board.h"
/* config */
#define LOG_OUTPUT_MAX_LEN  128

#define LOG_TIMESTAMP_EN     1
#define LOG_FUNCTION_EN      0
#define LOG_FILE_LINE_EN     0

#define LOG_ASSERT_EN        1
#define LOG_ERROR_EN         1
#define LOG_WARINING_EN      1
#define LOG_INFO_EN          1
#define LOG_DEBUG_EN         1

#define __log_output(log_str, len) do{usart1_transmit((uint8_t*)log_str, len);}while(0)

#endif // __LOG_CONFIG_H__
