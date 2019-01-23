#include "ulog.h"
#include "drv_uart.h"

static struct ulog_backend console;

void ulog_console_backend_output(struct ulog_backend *backend, uint32_t level, const char *tag, int32_t is_raw,
        const char *log, size_t len)
{
    usart6_transmit((uint8_t *)log, len); 
}

int ulog_console_backend_init(void)
{
    console.output = ulog_console_backend_output;

    ulog_backend_register(&console, "console", TRUE);

    return 0;
}


