#pragma once

#include <stdint.h>
#include "uart_protocol.h"

struct shell;

int pdec_uart_shell_init(enum pdec_serial_port port);
int pdec_uart_shell_start(enum pdec_serial_port port);
int pdec_uart_shell_stop(enum pdec_serial_port port);
void pdec_uart_shell_feed_char(enum pdec_serial_port port, uint8_t c);
const struct shell *pdec_uart_shell_backend_get(enum pdec_serial_port port);
