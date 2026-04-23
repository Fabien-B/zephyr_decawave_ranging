#pragma once

#include <stdint.h>

struct shell;

int pdec_uart_shell_init(void);
int pdec_uart_shell_start(void);
int pdec_uart_shell_stop(void);
void pdec_uart_shell_feed_char(uint8_t c);
const struct shell *pdec_uart_shell_backend_get(void);
