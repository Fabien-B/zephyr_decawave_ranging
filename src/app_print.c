#include <stdbool.h>
#include <stdio.h>

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>

#include "uart_protocol.h"
#include "uart_shell.h"
#include "app_print.h"

static int pdec_vprint_common(const char *fmt, va_list args)
{
    char buf[192];
    int len;
    const struct shell *sh;

    len = vsnprintf(buf, sizeof(buf), fmt, args);
    if (len <= 0) {
        return len;
    }

    if (k_is_in_isr()) {
        return len;
    }

    for (enum pdec_serial_port port = 0; port < PDEC_SERIAL_PORT_COUNT; ++port) {
        if (pdec_uart_protocol_get_mode(port) != PDEC_UART_MODE_ASCII) {
            continue;
        }
        sh = pdec_uart_shell_backend_get(port);
        if (sh != NULL && shell_ready(sh)) {
            shell_fprintf_normal(sh, "%s", buf);
        }
    }
    return len;
}

int pdec_vprintf(const char *fmt, va_list args)
{
    va_list copy;
    int ret;

    va_copy(copy, args);
    ret = pdec_vprint_common(fmt, copy);
    va_end(copy);

    return ret;
}

int pdec_printf(const char *fmt, ...)
{
    va_list args;
    int ret;

    va_start(args, fmt);
    ret = pdec_vprint_common(fmt, args);
    va_end(args);

    return ret;
}
